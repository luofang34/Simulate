//! Conformal screen HUD context: the window camera's projection relative to the airframe,
//! the surveyed landing runway, and belief traffic, all relative to ownship's estimated pose
//! on an explicit [`BodyEllipsoid`] and rendered [`ConformalSurface`].

use glam::{DMat3, DQuat, DVec3};
use sim_core::VehicleState;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, egm96_undulation_m};
use sim_indicate::{
    HudContact, HudContactDomain, HudContactIdentity, HudContext, HudProfile, HudProjection,
    HudRunway, IndicateQuat, Sig, SignalStatus, TargetPicture, VSpeedBugs, VSpeeds, WorldSource,
};
use sim_maplibre::{AltitudeReference, DynamicEntity3D, RenderBody, globe_ned_between_geodetic};
use sim_pack::ResolvedAirspeedEnvelope;
use sim_scenery::{CatalogVerticalDatum, LandingRunwayGeometry};

use super::HudView;

#[cfg(test)]
mod tests;

/// Geometric surface model of the window underlay that conformal HUD cues register against,
/// mirroring `Navigate`'s separation of `BodyTangentEnu` ([`Self::Ellipsoid`]) and
/// `SphericalEnu` ([`Self::GlobeSphere`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConformalSurface {
    /// Direct ellipsoidal mesh (`sim_render::GlobeMesh`), where world points sit at
    /// `physics.geodetic_to_fixed(point)` with zero spherical approximation.
    Ellipsoid {
        /// Reference ellipsoid of both navigation and the rendered mesh.
        physics: BodyEllipsoid,
    },
    /// Spherical globe renderer (`maplibre`), where navigation runs on `physics` while
    /// terrain and 3D overlays draw on `render_body`.
    GlobeSphere {
        /// Reference ellipsoid of the vehicle's body-fixed coordinates.
        physics: BodyEllipsoid,
        /// Mean-radius sphere of the conformal globe renderer.
        render_body: RenderBody,
    },
}

impl ConformalSurface {
    /// Earth `WGS84` navigation over `maplibre`'s `RenderBody::EARTH` sphere.
    #[cfg(test)]
    #[must_use]
    pub const fn earth_globe() -> Self {
        Self::GlobeSphere {
            physics: BodyEllipsoid::WGS84,
            render_body: RenderBody::EARTH,
        }
    }

    /// Direct ellipsoidal rendering on `physics` (`GlobeMesh` / `BodyTangentEnu`).
    #[cfg(test)]
    #[must_use]
    pub const fn ellipsoid(physics: BodyEllipsoid) -> Self {
        Self::Ellipsoid { physics }
    }

    /// Reference ellipsoid of the vehicle's body-fixed coordinates.
    #[must_use]
    pub const fn physics_ellipsoid(self) -> BodyEllipsoid {
        match self {
            Self::Ellipsoid { physics } | Self::GlobeSphere { physics, .. } => physics,
        }
    }
}

/// Rotation from the HUD's view axes (forward, right, down) into body FRD axes for a
/// window camera whose `+X` right, `+Y` up, `-Z` forward axes are `camera_to_ecef`.
pub(super) fn body_from_view(vehicle_attitude: DQuat, camera_to_ecef: DQuat) -> DQuat {
    let camera_from_view = DMat3::from_cols(DVec3::NEG_Z, DVec3::X, DVec3::NEG_Y);
    let body_from_camera = DMat3::from_quat(vehicle_attitude.inverse() * camera_to_ecef);
    DQuat::from_mat3(&(body_from_camera * camera_from_view)).normalize()
}

/// Maps a package-resolved [`ResolvedAirspeedEnvelope`] into shared `Indicate`
/// airspeed operating-range bands ([`VSpeeds`]) and reference speed bugs ([`VSpeedBugs`]).
pub(crate) fn indicate_v_speeds(
    envelope: Option<&ResolvedAirspeedEnvelope>,
) -> (Option<VSpeeds>, VSpeedBugs) {
    let Some(env) = envelope else {
        return (None, VSpeedBugs::default());
    };
    let speeds = VSpeeds {
        vs0_kt: env.vs0_kt,
        vs_kt: env.vs1_kt,
        vfe_kt: env.vfe_kt,
        vno_kt: env.vno_kt,
        vne_kt: env.vne_kt,
    };
    let bugs = VSpeedBugs {
        vr_kt: env.vr_kt,
        vx_kt: env.vx_kt,
        vy_kt: env.vy_kt,
        vg_kt: env.vg_kt,
        va_kt: env.va_kt,
        vref_kt: env.vref_kt,
    };
    (Some(speeds), bugs)
}

/// Everything the screen HUD needs besides aircraft state. The glass is placed in view by
/// the physical `airframe_attitude` and window `camera`; the runway, contacts, and rates
/// drawn on it come strictly from the navigation solution `vehicle` (offset to the pilot's
/// eye by `eye_body_lever_m`) and surveyed navdata `runway` (never live simulator DEM), so
/// estimation or navdata/raster discrepancies show faithfully while zero-error flight aligns
/// pixel-for-pixel through flare and rollout on any celestial body.
pub(super) fn hud_context(
    vehicle: &VehicleState,
    view: &HudView<'_>,
    runway: Option<&LandingRunwayGeometry>,
    airspeed_envelope: Option<&ResolvedAirspeedEnvelope>,
) -> HudContext {
    let q = body_from_view(view.airframe_attitude, view.camera.pose.orientation);
    let eye_fixed = vehicle.position_ecef + vehicle.attitude * view.eye_body_lever_m;
    let rates = vehicle.angular_rate_body;
    let (v_speeds, v_speed_bugs) = indicate_v_speeds(airspeed_envelope);
    HudContext {
        profile: HudProfile::Screen,
        projection: Sig::valid(HudProjection {
            vertical_fov_rad: view.camera.fov_y_rad as f32,
            body_from_view: IndicateQuat {
                w: q.w as f32,
                x: q.x as f32,
                y: q.y as f32,
                z: q.z as f32,
            },
            source: WorldSource::Synthetic,
        }),
        targets: target_picture(
            vehicle.position_ecef,
            eye_fixed,
            vehicle.velocity_ecef,
            view.traffic,
            view.conformal_surface,
            view.vertical_datum,
        ),
        body_rates_rps: Sig::valid([rates.x as f32, rates.y as f32, rates.z as f32]),
        approach: runway.map(|rwy| {
            hud_runway_for_vehicle(
                vehicle.position_ecef,
                eye_fixed,
                view.conformal_surface,
                view.vertical_datum,
                rwy,
            )
        }),
        v_speeds,
        v_speed_bugs,
        ..HudContext::default()
    }
}

/// Equipotential undulation (`N = h_ellipsoid - h_datum`) on `surface`.
///
/// Follows `Navigate` `ADR-0015`: Earth's EGM96 geoid undulation applies strictly on
/// `BodyEllipsoid::WGS84` when `OrthometricMslEgm96` is declared, and never leaks onto
/// ellipsoidal Earth catalogs or non-Earth bodies (`MOON_IAU`, `MARS_IAU`, etc.).
fn undulation_for_surface(
    surface: ConformalSurface,
    datum: Option<CatalogVerticalDatum>,
    geo: &Geodetic,
) -> f64 {
    if surface.physics_ellipsoid() != BodyEllipsoid::WGS84 {
        return 0.0;
    }
    match datum {
        Some(CatalogVerticalDatum::OrthometricMslEgm96) => {
            egm96_undulation_m(geo.lat_rad, geo.lon_rad)
        }
        Some(CatalogVerticalDatum::EllipsoidWgs84) | None => 0.0,
    }
}

struct ConformalObserver {
    eye_fixed: DVec3,
    eye_geo: Geodetic,
    eye_undul: f64,
    eye_local: LocalFrame,
    cg_local: LocalFrame,
    surface: ConformalSurface,
    datum: Option<CatalogVerticalDatum>,
}

impl ConformalObserver {
    fn new(
        vehicle_position_fixed: DVec3,
        eye_fixed: DVec3,
        surface: ConformalSurface,
        datum: Option<CatalogVerticalDatum>,
    ) -> Self {
        let physics = surface.physics_ellipsoid();
        let cg_local = LocalFrame::at(&physics.fixed_to_geodetic(vehicle_position_fixed));
        let eye_geo = physics.fixed_to_geodetic(eye_fixed);
        let eye_undul = undulation_for_surface(surface, datum, &eye_geo);
        let eye_local = LocalFrame::at(&eye_geo);
        Self {
            eye_fixed,
            eye_geo,
            eye_undul,
            eye_local,
            cg_local,
            surface,
            datum,
        }
    }

    fn ned_to(&self, point: &Geodetic) -> DVec3 {
        match self.surface {
            ConformalSurface::GlobeSphere {
                physics,
                render_body,
            } => {
                let pt_undul = undulation_for_surface(self.surface, self.datum, point);
                if let Some(ned_eye) = globe_ned_between_geodetic(
                    render_body,
                    &self.eye_geo,
                    self.eye_undul,
                    point,
                    pt_undul,
                ) {
                    self.cg_local
                        .ecef_to_ned_vector(self.eye_local.ned_to_ecef() * ned_eye)
                } else {
                    self.cg_local
                        .ecef_to_ned_vector(physics.geodetic_to_fixed(point) - self.eye_fixed)
                }
            }
            ConformalSurface::Ellipsoid { physics } => self
                .cg_local
                .ecef_to_ned_vector(physics.geodetic_to_fixed(point) - self.eye_fixed),
        }
    }
}

#[cfg(test)]
pub(crate) fn hud_runway(
    eye_fixed: DVec3,
    surface: ConformalSurface,
    datum: Option<CatalogVerticalDatum>,
    rwy: &LandingRunwayGeometry,
) -> HudRunway {
    hud_runway_for_vehicle(eye_fixed, eye_fixed, surface, datum, rwy)
}

/// Conformal [`HudRunway`] from `eye_fixed`, expressed in the local tangent NED basis of
/// `vehicle_position_fixed` (`StateFeed`'s attitude reference) so any non-zero pilot-eye
/// lever arm registers identically against [`ConformalSurface`] even across `±90°` poles.
pub(crate) fn hud_runway_for_vehicle(
    vehicle_position_fixed: DVec3,
    eye_fixed: DVec3,
    surface: ConformalSurface,
    datum: Option<CatalogVerticalDatum>,
    rwy: &LandingRunwayGeometry,
) -> HudRunway {
    let obs = ConformalObserver::new(vehicle_position_fixed, eye_fixed, surface, datum);
    let start_ned = obs.ned_to(&rwy.pavement_start);
    let thr_ned = obs.ned_to(&rwy.threshold);
    let far_ned = obs.ned_to(&rwy.far_end);
    let dn = far_ned.x - thr_ned.x;
    let de = far_ned.y - thr_ned.y;
    let horiz = dn.hypot(de);
    let scale = if rwy.landing_length_m > 1.0 && horiz > 1.0 {
        horiz / rwy.landing_length_m
    } else {
        1.0
    };
    let pre_horiz = (thr_ned.x - start_ned.x).hypot(thr_ned.y - start_ned.y);
    let pre_threshold_m = if rwy.displaced_threshold_m <= 0.0 {
        0.0
    } else if pre_horiz > 0.1 {
        pre_horiz
    } else {
        rwy.displaced_threshold_m * scale
    };
    let (course_rad, length_m) = if horiz > 1.0 {
        (
            de.atan2(dn).rem_euclid(std::f64::consts::TAU) as f32,
            horiz as f32,
        )
    } else {
        (rwy.true_bearing_rad as f32, rwy.landing_length_m as f32)
    };
    HudRunway {
        threshold_m: thr_ned.to_array().map(|v| v as f32),
        pre_threshold_m: pre_threshold_m.max(0.0) as f32,
        reciprocal_displaced_m: (rwy.reciprocal_displaced_m.max(0.0) * scale) as f32,
        unpaved: rwy.surface.is_unpaved(),
        course_rad,
        length_m,
        width_m: rwy.width_m as f32,
        far_down_offset_m: (far_ned.z - thr_ned.z) as f32,
        glide_path_rad: rwy.glidepath_rad as f32,
        status: SignalStatus::Valid,
    }
}

fn target_picture(
    vehicle_position_fixed: DVec3,
    eye_fixed: DVec3,
    own_velocity_fixed: DVec3,
    traffic: &[DynamicEntity3D],
    surface: ConformalSurface,
    datum: Option<CatalogVerticalDatum>,
) -> TargetPicture {
    let obs = ConformalObserver::new(vehicle_position_fixed, eye_fixed, surface, datum);
    let mut picture = TargetPicture::default();
    for entity in traffic {
        let lat_rad = entity.placement.position.latitude.to_radians();
        let lon_rad = entity.placement.position.longitude.to_radians();
        let zero_geo = Geodetic {
            lat_rad,
            lon_rad,
            height_m: 0.0,
        };
        let undulation_m = match entity.placement.altitude_reference {
            AltitudeReference::AboveMsl => undulation_for_surface(surface, datum, &zero_geo),
            AltitudeReference::AboveGround | AltitudeReference::OnGroundSlopeAligned => 0.0,
        };
        let position = Geodetic {
            lat_rad,
            lon_rad,
            height_m: entity.placement.altitude_meters + undulation_m,
        };
        let ent_local = LocalFrame::at(&position);
        let v = entity.velocity_enu_mps;
        let ent_vel_fixed = ent_local.east() * v.x + ent_local.north() * v.y + ent_local.up() * v.z;
        let velocity_ned = obs
            .cg_local
            .ecef_to_ned_vector(ent_vel_fixed - own_velocity_fixed);
        let rel_ned = obs.ned_to(&position);
        let contact = HudContact {
            relative_position_m: rel_ned.to_array().map(|c| c as f32),
            relative_velocity_mps: velocity_ned.to_array().map(|c| c as f32),
            identity: HudContactIdentity::Unknown,
            domain: HudContactDomain::Air,
            status: SignalStatus::Valid,
        };
        if picture.push(contact).is_none() {
            break;
        }
    }
    picture
}
