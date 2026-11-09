//! Curvature-matched 3D groomed snow skiway mesh for polar runways (`|lat| > 85.05°`)
//! beyond Web Mercator orthoimagery coverage (such as Amundsen-Scott `NZSP RWY 02T`).

use glam::{DMat4, DQuat};
use maplibre::{
    coords::LatLon,
    headless::{AltitudeReference, Orientation3d, Placement3d, ViewSlotMask},
    projection::body::Body,
};
use sim_scenery::LandingRunwayGeometry;

use crate::{
    ConformalFrameStamp, PartRole, PbrMaterial, UsdAircraftAsset, UsdMeshPart, UsdMeshVertex,
    globe_ned_between_geodetic,
};

const SEGMENT_STEP_M: f64 = 30.0;
const SHOULDER_EXTRA_HALF_W_M: f64 = 12.0;
const OVERRUN_M: f64 = 40.0;
const FLAG_STEP_M: f64 = 120.0;

/// Quantized spherical dimensions keying a cached polar skiway mesh in [`super::AircraftLibrary`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct PolarSkiwayKey {
    pub(crate) pre_threshold_cm: u32,
    pub(crate) primary_length_cm: u32,
    pub(crate) after_threshold_cm: u32,
    pub(crate) width_cm: u32,
    pub(crate) slope_mm_per_km: i32,
    pub(crate) curve_radius_m: u32,
}

impl PolarSkiwayKey {
    pub(crate) fn from_runway(runway: &LandingRunwayGeometry, render_body: Body) -> Self {
        let (_, horiz_m, pre_m) = spherical_skiway_span(runway, render_body);
        let after_m = horiz_m.clamp(200.0, 12_000.0);
        let scale = if runway.landing_length_m > 1.0 {
            after_m / runway.landing_length_m
        } else {
            1.0
        };
        let recip_m = (runway.reciprocal_displaced_m.max(0.0) * scale).min(after_m - 60.0);
        let primary_m = (after_m - recip_m.max(0.0)).clamp(60.0, after_m);
        let wid_m = runway.width_m.clamp(12.0, 300.0);
        let dh = runway.far_elevation_msl_m - runway.threshold_elevation_msl_m;
        let slope_mm_per_km = ((dh / after_m) * 1_000_000.0)
            .round()
            .clamp(-200_000.0, 200_000.0) as i32;
        let curve_r = (render_body.radius_meters + runway.threshold_elevation_msl_m)
            .round()
            .clamp(100_000.0, 100_000_000.0) as u32;
        Self {
            pre_threshold_cm: (pre_m.clamp(0.0, 6_000.0) * 100.0).round() as u32,
            primary_length_cm: (primary_m * 100.0).round() as u32,
            after_threshold_cm: (after_m * 100.0).round() as u32,
            width_cm: (wid_m * 100.0).round() as u32,
            slope_mm_per_km,
            curve_radius_m: curve_r,
        }
    }
}

/// Spherical `(bearing_rad, threshold_to_far_m, pre_threshold_m)` of `runway` in `threshold`'s
/// local sphere tangent frame on `render_body`, matching [`globe_ned_between_geodetic`].
fn spherical_skiway_span(runway: &LandingRunwayGeometry, render_body: Body) -> (f64, f64, f64) {
    let thr_undul = runway.threshold.height_m - runway.threshold_elevation_msl_m;
    let far_undul = runway.far_end.height_m - runway.far_elevation_msl_m;
    let (bearing_rad, horiz_m) = if let Some(far_ned) = globe_ned_between_geodetic(
        render_body,
        &runway.threshold,
        thr_undul,
        &runway.far_end,
        far_undul,
    ) && far_ned.x.hypot(far_ned.y) > 1.0
    {
        let h = far_ned.x.hypot(far_ned.y);
        (far_ned.y.atan2(far_ned.x), h)
    } else {
        (runway.true_bearing_rad, runway.landing_length_m)
    };
    let pre_m = if runway.displaced_threshold_m <= 0.0 {
        0.0
    } else {
        let start_undul = runway.pavement_start.height_m - runway.start_elevation_msl_m;
        globe_ned_between_geodetic(
            render_body,
            &runway.threshold,
            thr_undul,
            &runway.pavement_start,
            start_undul,
        )
        .map_or(runway.displaced_threshold_m, |n| n.x.hypot(n.y))
    };
    (bearing_rad, horiz_m, pre_m)
}

/// Threshold placement of `runway` on `stamp.render_body`'s MSL sphere (`+X` right, `+Y`
/// down-runway, `+Z` up), using the exact spherical chord bearing to `far_end` and surveyed
/// threshold MSL elevation (independent of ownship's geoid undulation).
pub(crate) fn skiway_placement(
    runway: &LandingRunwayGeometry,
    stamp: &ConformalFrameStamp,
) -> Placement3d {
    let (bearing_rad, _, _) = spherical_skiway_span(runway, stamp.render_body);
    let q = DQuat::from_rotation_z(-bearing_rad);
    Placement3d {
        position: LatLon::new(runway.threshold.lat_deg(), runway.threshold.lon_deg()),
        altitude_meters: runway.threshold_elevation_msl_m,
        altitude_reference: AltitudeReference::AboveMsl,
        orientation: Orientation3d {
            enu_from_body: [q.x, q.y, q.z, q.w],
        },
        view_mask: ViewSlotMask::ALL,
    }
}

#[derive(Clone, Copy)]
struct SkiwayCurve {
    slope: f64,
    radius_m: f64,
}

impl SkiwayCurve {
    /// Longitudinal grade plus spherical sagitta `-(x^2 + y^2) / (2 R)` so the entire skiway
    /// stays at constant `z_above` clearance above `render_body`'s spherical polar cap.
    fn z(self, x: f64, y: f64, z_above: f64) -> f32 {
        (z_above + self.slope * y - 0.5 * (x * x + y * y) / self.radius_m) as f32
    }
}

/// Builds a curvature-dropped groomed polar skiway asset with compacted firn deck, snow
/// threshold/aiming/centreline markings, and international-orange bamboo edge flags.
pub(crate) fn build_polar_skiway(key: PolarSkiwayKey) -> UsdAircraftAsset {
    let pre_m = f64::from(key.pre_threshold_cm) * 0.01;
    let primary_m = f64::from(key.primary_length_cm) * 0.01;
    let after_m = f64::from(key.after_threshold_cm) * 0.01;
    let half_w = f64::from(key.width_cm) * 0.005;
    let curve = SkiwayCurve {
        slope: f64::from(key.slope_mm_per_km) * 1e-6,
        radius_m: f64::from(key.curve_radius_m).max(100_000.0),
    };

    let mut shoulder = QuadAcc::new(curve);
    append_ribbon(
        &mut shoulder,
        -(half_w + SHOULDER_EXTRA_HALF_W_M),
        half_w + SHOULDER_EXTRA_HALF_W_M,
        -(pre_m + OVERRUN_M),
        after_m + OVERRUN_M,
        0.10,
    );

    let mut deck = QuadAcc::new(curve);
    append_ribbon(&mut deck, -half_w, half_w, -pre_m, after_m, 0.20);

    let mut marks = QuadAcc::new(curve);
    append_markings(&mut marks, primary_m, half_w);

    let mut flags = QuadAcc::new(curve);
    append_edge_flags(&mut flags, curve, -pre_m, after_m, half_w);

    UsdAircraftAsset::from_parts(vec![
        shoulder.into_part(
            "/PolarSkiway/Shoulder",
            PbrMaterial {
                diffuse_rgb: [0.68, 0.75, 0.84],
                emissive_rgb: [0.0, 0.0, 0.0],
                metallic: 0.0,
                roughness: 0.88,
                opacity: 1.0,
            },
            false,
        ),
        deck.into_part(
            "/PolarSkiway/GroomedDeck",
            PbrMaterial {
                diffuse_rgb: [0.43, 0.53, 0.66],
                emissive_rgb: [0.0, 0.0, 0.0],
                metallic: 0.02,
                roughness: 0.72,
                opacity: 1.0,
            },
            false,
        ),
        marks.into_part(
            "/PolarSkiway/SnowMarkings",
            PbrMaterial {
                diffuse_rgb: [0.96, 0.98, 1.0],
                emissive_rgb: [0.04, 0.05, 0.06],
                metallic: 0.0,
                roughness: 0.55,
                opacity: 1.0,
            },
            false,
        ),
        flags.into_part(
            "/PolarSkiway/OrangeFlags",
            PbrMaterial {
                diffuse_rgb: [0.96, 0.34, 0.06],
                emissive_rgb: [0.16, 0.05, 0.01],
                metallic: 0.0,
                roughness: 0.55,
                opacity: 1.0,
            },
            true,
        ),
    ])
}

struct QuadAcc {
    curve: SkiwayCurve,
    vertices: Vec<UsdMeshVertex>,
    indices: Vec<u32>,
}

impl QuadAcc {
    fn new(curve: SkiwayCurve) -> Self {
        Self {
            curve,
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }

    fn push_quad(&mut self, pts: [[f32; 3]; 4], normal: [f32; 3]) {
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let uvs = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        for (position, uv) in pts.into_iter().zip(uvs) {
            self.vertices.push(UsdMeshVertex {
                position,
                normal,
                uv,
            });
        }
        self.indices.extend_from_slice(&[
            base,
            base.wrapping_add(1),
            base.wrapping_add(2),
            base,
            base.wrapping_add(2),
            base.wrapping_add(3),
        ]);
    }

    fn push_ground_rect(&mut self, x0: f64, x1: f64, y0: f64, y1: f64, z_above: f64) {
        let c = self.curve;
        self.push_quad(
            [
                [x0 as f32, y0 as f32, c.z(x0, y0, z_above)],
                [x1 as f32, y0 as f32, c.z(x1, y0, z_above)],
                [x1 as f32, y1 as f32, c.z(x1, y1, z_above)],
                [x0 as f32, y1 as f32, c.z(x0, y1, z_above)],
            ],
            [0.0, 0.0, 1.0],
        );
    }

    fn into_part(self, prim_path: &str, material: PbrMaterial, double_sided: bool) -> UsdMeshPart {
        UsdMeshPart {
            prim_path: prim_path.to_owned(),
            role: PartRole::CabinExterior,
            material,
            double_sided,
            rest_to_aircraft: DMat4::IDENTITY,
            custom_hinge_axis: None,
            needle_channel: None,
            vertices: self.vertices,
            indices: self.indices,
        }
    }
}

fn append_ribbon(acc: &mut QuadAcc, x0: f64, x1: f64, y_start: f64, y_end: f64, z_above: f64) {
    let span = (y_end - y_start).max(1.0);
    let steps = ((span / SEGMENT_STEP_M).ceil() as u32).max(1);
    for i in 0..steps {
        let y0 = y_start + span * (f64::from(i) / f64::from(steps));
        let y1 = y_start + span * (f64::from(i.wrapping_add(1)) / f64::from(steps));
        acc.push_ground_rect(x0, x1, y0, y1, z_above);
    }
}

fn append_markings(acc: &mut QuadAcc, length_m: f64, half_w: f64) {
    let z = 0.26;
    let inset = (half_w - 2.0).max(4.0);
    acc.push_ground_rect(-inset, inset, 0.5, 5.5, z);
    if length_m > 120.0 {
        acc.push_ground_rect(-inset, inset, length_m - 5.5, length_m - 0.5, z);
    }
    let stripe_span = (half_w - 5.0).max(4.0);
    for k in 0..8_u32 {
        let t = (f64::from(k) + 0.5) / 8.0 * 2.0 - 1.0;
        let cx = t * stripe_span;
        if cx.abs() > 2.5 {
            acc.push_ground_rect(cx - 1.5, cx + 1.5, 12.0, 48.0, z);
        }
    }
    if length_m > 700.0 {
        let aim_x = (half_w * 0.42).max(5.0);
        for side in [-1.0, 1.0] {
            let cx = side * aim_x;
            acc.push_ground_rect(cx - 3.2, cx + 3.2, 285.0, 335.0, z);
        }
    }
    let mut y = 56.0;
    while y + 36.0 < length_m - 50.0 {
        acc.push_ground_rect(-1.0, 1.0, y, y + 36.0, z);
        y += 60.0;
    }
}

fn append_edge_flags(acc: &mut QuadAcc, curve: SkiwayCurve, y_start: f64, y_end: f64, half_w: f64) {
    for side in [-1.0_f64, 1.0_f64] {
        let bx = side * (half_w + 4.5);
        let z0 = curve.z(bx, 2.0, 0.15);
        let z1 = curve.z(bx, 2.0, 2.15);
        acc.push_quad(
            [
                [(bx - 1.8) as f32, 2.0, z0],
                [(bx + 1.8) as f32, 2.0, z0],
                [(bx + 1.8) as f32, 2.0, z1],
                [(bx - 1.8) as f32, 2.0, z1],
            ],
            [0.0, -1.0, 0.0],
        );
        let fx = side * (half_w + 3.0);
        let mut y = y_start;
        while y <= y_end + 1.0 {
            let z_lo = curve.z(fx, y, 1.10);
            let z_hi = curve.z(fx, y, 2.75);
            acc.push_quad(
                [
                    [fx as f32, (y - 0.85) as f32, z_lo],
                    [fx as f32, (y + 0.85) as f32, z_lo],
                    [fx as f32, (y + 0.85) as f32, z_hi],
                    [fx as f32, (y - 0.85) as f32, z_hi],
                ],
                [-side as f32, 0.0, 0.0],
            );
            y += FLAG_STEP_M;
        }
    }
}
