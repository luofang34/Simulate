//! Forward-compatible 6-DoF dynamic 3D entities and conformal [`OverlayMesh`] builders.
//!
//! Uses upstream [`Placement3d`], [`Orientation3d`], [`AltitudeReference`], and
//! [`ViewSlotMask`] directly so that when `SCENE_3D_PASS` 3D traffic aircraft and
//! ground vehicle meshes are attached later, zero host placement or visibility
//! contracts need to change.

use cgmath::Matrix4;
use glam::{DMat3, DQuat, DVec3};
use maplibre::{
    coords::LatLon,
    headless::{AltitudeReference, Orientation3d, Placement3d, ViewSlotId, ViewSlotMask},
    overlay::{DepthProbe, MAX_PROBES, OverlayMesh, OverlayVertex},
    projection::body::Body,
};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_scenery::LandingRunwayGeometry;
use world::surveillance::{EmitterCategory, SurveillanceStore, TrackPhase};

use crate::pose::{ConformalFrameStamp, project_geodetic_to_screen};

mod runway;

/// Semantic class of a dynamic 3D entity for symbology and future 3D model selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DynamicEntityKind {
    /// Fixed-wing aircraft (traffic or chase target).
    FixedWing,
    /// Rotorcraft / eVTOL.
    Rotorcraft,
    /// Airport surface tug, fuel truck, or ground vehicle.
    GroundVehicle,
    /// Ownship exterior 3D airframe (hidden in cockpit OTW slot, visible in chase/external slots).
    OwnshipExterior,
}

/// Full 6-DoF dynamic entity ready for [`maplibre::render::draw_graph::node::SCENE_3D_PASS`]
/// and conformal [`OverlayMesh`] / [`DepthProbe`] rendering.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicEntity3D {
    /// Stable caller or track identifier.
    pub id: u64,
    /// Entity classification.
    pub kind: DynamicEntityKind,
    /// Full 6-DoF geodetic + quaternion placement and per-slot visibility mask.
    pub placement: Placement3d,
    /// Velocity in local East-North-Up coordinates (metres per second).
    pub velocity_enu_mps: DVec3,
    /// Nominal wingspan or vehicle length in metres (`> 0.0`).
    pub characteristic_span_m: f64,
}

impl DynamicEntity3D {
    /// Exact `f64` model-to-unit-sphere matrix for `SCENE_3D_PASS` instancing.
    #[must_use]
    pub fn model_to_unit_sphere(
        &self,
        body: &Body,
        ground_elevation_m: f64,
        ground_normal_enu: Option<cgmath::Vector3<f64>>,
    ) -> Matrix4<f64> {
        self.placement
            .model_to_unit_sphere(body, ground_elevation_m, ground_normal_enu)
    }

    /// Creates an ownship exterior entity hidden in `cockpit_otw_slot` and visible in all
    /// external / chase slots.
    #[must_use]
    pub fn ownship_exterior(
        id: u64,
        geodetic: &Geodetic,
        ned_from_body: DQuat,
        geoid_undulation_m: f64,
        cockpit_otw_slot: ViewSlotId,
        wingspan_m: f64,
    ) -> Self {
        let placement = Placement3d {
            position: LatLon::new(geodetic.lat_rad.to_degrees(), geodetic.lon_rad.to_degrees()),
            altitude_meters: geodetic.height_m - geoid_undulation_m,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: orientation_from_ned_body_quat(ned_from_body),
            view_mask: ViewSlotMask::ALL.without(cockpit_otw_slot),
        };
        Self {
            id,
            kind: DynamicEntityKind::OwnshipExterior,
            placement,
            velocity_enu_mps: DVec3::ZERO,
            characteristic_span_m: wingspan_m.max(1.0),
        }
    }
}

/// Converts an aerospace body-to-NED unit quaternion (`+X` forward, `+Y` right, `+Z` down
/// in `+X` North, `+Y` East, `+Z` Down) into `maplibre`'s [`Orientation3d`] (`+X` right,
/// `+Y` forward, `+Z` up in `+X` East, `+Y` North, `+Z` Up) with zero Euler gimbal lock.
#[must_use]
pub fn orientation_from_ned_body_quat(ned_from_body: DQuat) -> Orientation3d {
    if !ned_from_body.is_finite() {
        return Orientation3d::IDENTITY;
    }
    // Basis swap S (`det = +1`, `S = S^-1`): swaps X<->Y and negates Z.
    let swap = DMat3::from_cols(
        DVec3::new(0.0, 1.0, 0.0),
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(0.0, 0.0, -1.0),
    );
    let r_ned = DMat3::from_quat(ned_from_body.normalize());
    let r_enu_ml = swap * r_ned * swap;
    let q = DQuat::from_mat3(&r_enu_ml).normalize();
    Orientation3d::from_xyzw(q.x, q.y, q.z, q.w)
}

/// Extracts belief-state [`DynamicEntity3D`] targets from [`SurveillanceStore`],
/// excluding `ownship_shadow` tracks so ownship never ghosts on avionics underlays.
#[must_use]
pub fn entities_from_surveillance_belief(
    store: &SurveillanceStore,
    geoid_undulation_m: f64,
) -> Vec<DynamicEntity3D> {
    let mut out = Vec::with_capacity(store.tracks().len());
    for track in store.tracks() {
        if track.ownship_shadow {
            continue;
        }
        let heading_deg = track.heading_rad.unwrap_or(0.0).to_degrees();
        let orientation = Orientation3d::from_heading_pitch_roll_degrees(heading_deg, 0.0, 0.0);
        let altitude_reference = if track.on_ground {
            AltitudeReference::OnGroundSlopeAligned
        } else {
            AltitudeReference::AboveMsl
        };
        let altitude_meters = if track.on_ground {
            0.0
        } else {
            track.position.ellipsoidal_height_m - geoid_undulation_m
        };
        let kind = match track.category {
            EmitterCategory::Rotorcraft => DynamicEntityKind::Rotorcraft,
            EmitterCategory::SurfaceEmergency
            | EmitterCategory::SurfaceService
            | EmitterCategory::PointObstacle => DynamicEntityKind::GroundVehicle,
            _ => DynamicEntityKind::FixedWing,
        };
        let span = match track.phase {
            TrackPhase::Active => 12.0,
            TrackPhase::Coasting => 9.0,
        };
        out.push(DynamicEntity3D {
            id: track.id.0,
            kind,
            placement: Placement3d {
                position: LatLon::new(track.position.latitude_deg, track.position.longitude_deg),
                altitude_meters,
                altitude_reference,
                orientation,
                view_mask: ViewSlotMask::ALL,
            },
            velocity_enu_mps: DVec3::new(
                track.velocity_ned_m_s.east_m,
                track.velocity_ned_m_s.north_m,
                -track.velocity_ned_m_s.down_m,
            ),
            characteristic_span_m: span,
        });
    }
    out
}

/// Line-of-sight [`DepthProbe`]s at each entity visible in `slot`, with no drawing, for
/// views that must show only the physical scene.
#[must_use]
pub fn entity_depth_probes(
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    entities: &[DynamicEntity3D],
    slot: Option<ViewSlotId>,
) -> Vec<DepthProbe> {
    entities
        .iter()
        .filter(|entity| entity.placement.view_mask.matches_current(slot))
        .filter_map(|entity| {
            let point = Geodetic {
                lat_rad: entity.placement.position.latitude.to_radians(),
                lon_rad: entity.placement.position.longitude.to_radians(),
                height_m: entity.placement.altitude_meters + stamp.geoid_undulation_m,
            };
            project_geodetic_to_screen(stamp, &point, stamp.geoid_undulation_m, viewport_px)
        })
        .take(MAX_PROBES)
        .map(|(position, depth)| DepthProbe { position, depth })
        .collect()
}

/// Builds a terrain-depth-tested conformal [`OverlayMesh`] (runway outline + traffic
/// target marks) and line-of-sight [`DepthProbe`]s for `slot`. Ownship is never marked:
/// a conformal view is drawn from ownship.
#[must_use]
pub fn build_conformal_overlay(
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    runway: Option<&LandingRunwayGeometry>,
    entities: &[DynamicEntity3D],
    slot: Option<ViewSlotId>,
) -> (OverlayMesh, Vec<DepthProbe>) {
    build_conformal_overlay_with_dem(stamp, viewport_px, runway, entities, slot, &mut |_, _| None)
}

/// Builds a terrain-depth-tested conformal [`OverlayMesh`] conforming `runway`'s vertical
/// rail profile and grazing-angle depth clearance to `sample_dem_msl` (`(bilinear_msl, cell_max_msl)`).
#[must_use]
pub(crate) fn build_conformal_overlay_with_dem(
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    runway: Option<&LandingRunwayGeometry>,
    entities: &[DynamicEntity3D],
    slot: Option<ViewSlotId>,
    sample_dem_msl: &mut dyn FnMut(f64, f64) -> Option<(f64, f64)>,
) -> (OverlayMesh, Vec<DepthProbe>) {
    let mut mesh = OverlayMesh::default();
    let mut probes = Vec::new();

    if let Some(rwy) = runway {
        runway::append_runway_outline(
            &mut mesh,
            &mut probes,
            stamp,
            viewport_px,
            rwy,
            sample_dem_msl,
        );
    }
    for entity in entities {
        if entity.kind == DynamicEntityKind::OwnshipExterior
            || !entity.placement.view_mask.matches_current(slot)
        {
            continue;
        }
        let point = Geodetic {
            lat_rad: entity.placement.position.latitude.to_radians(),
            lon_rad: entity.placement.position.longitude.to_radians(),
            height_m: entity.placement.altitude_meters + stamp.geoid_undulation_m,
        };
        if let Some((px, depth)) =
            project_geodetic_to_screen(stamp, &point, stamp.geoid_undulation_m, viewport_px)
        {
            let color = match entity.kind {
                DynamicEntityKind::GroundVehicle => [0.85, 0.65, 0.15, 0.90],
                DynamicEntityKind::FixedWing
                | DynamicEntityKind::Rotorcraft
                | DynamicEntityKind::OwnshipExterior => [0.0, 0.92, 0.92, 0.92],
            };
            if entity.kind == DynamicEntityKind::FixedWing {
                append_airframe_wireframe(&mut mesh, stamp, viewport_px, entity, &point, color);
            }
            push_diamond(&mut mesh, px, depth, 6.0, color);
            if probes.len() < MAX_PROBES {
                probes.push(DepthProbe {
                    position: px,
                    depth,
                });
            }
        }
    }
    (mesh, probes)
}

fn append_airframe_wireframe(
    mesh: &mut OverlayMesh,
    stamp: &ConformalFrameStamp,
    viewport_px: [u32; 2],
    entity: &DynamicEntity3D,
    center_geo: &Geodetic,
    color: [f32; 4],
) {
    let [qx, qy, qz, qw] = entity.placement.orientation.enu_from_body;
    let q_enu = DQuat::from_xyzw(qx, qy, qz, qw).normalize();
    if !q_enu.is_finite() {
        return;
    }
    let span = entity.characteristic_span_m.max(2.0);
    let center_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(center_geo);
    let local = LocalFrame::at(center_geo);
    let project_body = |body_m: DVec3| -> Option<([f32; 2], f32)> {
        let enu = q_enu * body_m;
        let offset_ecef = local.east() * enu.x + local.north() * enu.y + local.up() * enu.z;
        let geo = BodyEllipsoid::WGS84.fixed_to_geodetic(center_ecef + offset_ecef);
        project_geodetic_to_screen(stamp, &geo, stamp.geoid_undulation_m, viewport_px)
    };
    let (Some(wing_l), Some(wing_r)) = (
        project_body(DVec3::new(-0.50 * span, 0.05 * span, 0.06 * span)),
        project_body(DVec3::new(0.50 * span, 0.05 * span, 0.06 * span)),
    ) else {
        return;
    };
    if (wing_r.0[0] - wing_l.0[0]).hypot(wing_r.0[1] - wing_l.0[1]) < 8.0 {
        return;
    }
    push_segment(mesh, wing_l.0, wing_l.1, wing_r.0, wing_r.1, 2.6, color);
    let segments = [
        (
            DVec3::new(0.0, 0.38 * span, 0.0),
            DVec3::new(0.0, -0.38 * span, 0.0),
        ),
        (
            DVec3::new(-0.17 * span, -0.35 * span, 0.02 * span),
            DVec3::new(0.17 * span, -0.35 * span, 0.02 * span),
        ),
        (
            DVec3::new(0.0, -0.35 * span, 0.0),
            DVec3::new(0.0, -0.39 * span, 0.14 * span),
        ),
    ];
    for (a_body, b_body) in segments {
        if let (Some(a), Some(b)) = (project_body(a_body), project_body(b_body)) {
            push_segment(mesh, a.0, a.1, b.0, b.1, 2.2, color);
        }
    }
}

fn push_diamond(
    mesh: &mut OverlayMesh,
    center: [f32; 2],
    depth: f32,
    radius_px: f32,
    color: [f32; 4],
) {
    let [cx, cy] = center;
    let top = [cx, cy - radius_px];
    let right = [cx + radius_px, cy];
    let bottom = [cx, cy + radius_px];
    let left = [cx - radius_px, cy];
    for (a, b) in [(top, right), (right, bottom), (bottom, left), (left, top)] {
        push_segment(mesh, a, depth, b, depth, 1.8, color);
    }
}

pub(super) fn push_segment(
    mesh: &mut OverlayMesh,
    a: [f32; 2],
    depth_a: f32,
    b: [f32; 2],
    depth_b: f32,
    width_px: f32,
    color: [f32; 4],
) {
    let [ax, ay] = a;
    let [bx, by] = b;
    let dx = bx - ax;
    let dy = by - ay;
    let len = dx.hypot(dy);
    if len <= 1e-3 {
        return;
    }
    let half = width_px * 0.5;
    let nx = -dy / len * half;
    let ny = dx / len * half;
    let [r, g, bl, al] = color;
    let premul = [r * al, g * al, bl * al, al];
    let Ok(base) = u32::try_from(mesh.vertices.len()) else {
        return;
    };
    mesh.vertices.extend_from_slice(&[
        OverlayVertex {
            position: [ax + nx, ay + ny],
            depth: depth_a,
            color: premul,
        },
        OverlayVertex {
            position: [ax - nx, ay - ny],
            depth: depth_a,
            color: premul,
        },
        OverlayVertex {
            position: [bx - nx, by - ny],
            depth: depth_b,
            color: premul,
        },
        OverlayVertex {
            position: [bx + nx, by + ny],
            depth: depth_b,
            color: premul,
        },
    ]);
    mesh.indices
        .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
}

pub(super) fn push_quad(
    mesh: &mut OverlayMesh,
    a_left: ([f32; 2], f32),
    a_right: ([f32; 2], f32),
    b_right: ([f32; 2], f32),
    b_left: ([f32; 2], f32),
    color: [f32; 4],
) {
    let [r, g, bl, al] = color;
    let premul = [r * al, g * al, bl * al, al];
    let Ok(base) = u32::try_from(mesh.vertices.len()) else {
        return;
    };
    mesh.vertices.extend_from_slice(&[
        OverlayVertex {
            position: a_left.0,
            depth: a_left.1,
            color: premul,
        },
        OverlayVertex {
            position: a_right.0,
            depth: a_right.1,
            color: premul,
        },
        OverlayVertex {
            position: b_right.0,
            depth: b_right.1,
            color: premul,
        },
        OverlayVertex {
            position: b_left.0,
            depth: b_left.1,
            color: premul,
        },
    ]);
    mesh.indices
        .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
}
