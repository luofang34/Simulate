//! Curvature-matched 3D groomed snow skiway mesh for polar runways (`|lat| > 85.05°`)
//! beyond Web Mercator orthoimagery coverage (such as Amundsen-Scott `NZSP RWY 02T`).

use flightbag::RunwayEndSpec;
use glam::{DMat4, DQuat};
use maplibre::{
    coords::LatLon,
    headless::{AltitudeReference, Orientation3d, Placement3d, ViewSlotMask},
    projection::body::Body,
};

use crate::{
    ConformalFrameStamp, PartRole, PbrMaterial, UsdAircraftAsset, UsdMeshPart, UsdMeshVertex,
};

const SEGMENT_STEP_M: f64 = 30.0;
const SHOULDER_EXTRA_HALF_W_M: f64 = 12.0;
const OVERRUN_M: f64 = 40.0;
const FLAG_STEP_M: f64 = 120.0;

/// Quantized dimensions keying a cached polar skiway mesh in [`super::AircraftLibrary`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct PolarSkiwayKey {
    pub(crate) length_dm: u32,
    pub(crate) width_dm: u32,
}

impl PolarSkiwayKey {
    pub(crate) fn from_runway(runway: &RunwayEndSpec) -> Self {
        let len = runway.length_m.clamp(200.0, 12_000.0);
        let wid = runway.width_m.clamp(12.0, 300.0);
        Self {
            length_dm: (len * 10.0).round() as u32,
            width_dm: (wid * 10.0).round() as u32,
        }
    }

    fn dimensions_m(self) -> (f64, f64) {
        (
            f64::from(self.length_dm) * 0.1,
            f64::from(self.width_dm) * 0.1,
        )
    }
}

/// Threshold placement of `runway` in `stamp`'s MSL sphere (`+X` right, `+Y` down-runway, `+Z` up).
pub(crate) fn skiway_placement(runway: &RunwayEndSpec, stamp: &ConformalFrameStamp) -> Placement3d {
    let q = DQuat::from_rotation_z(-runway.true_bearing_rad);
    Placement3d {
        position: LatLon::new(
            runway.threshold.latitude_deg,
            runway.threshold.longitude_deg,
        ),
        altitude_meters: runway.threshold.ellipsoidal_height_m - stamp.geoid_undulation_m,
        altitude_reference: AltitudeReference::AboveMsl,
        orientation: Orientation3d {
            enu_from_body: [q.x, q.y, q.z, q.w],
        },
        view_mask: ViewSlotMask::ALL,
    }
}

/// Builds a curvature-dropped groomed polar skiway asset with compacted firn deck, snow
/// threshold/aiming/centreline markings, and international-orange bamboo edge flags.
pub(crate) fn build_polar_skiway(key: PolarSkiwayKey) -> UsdAircraftAsset {
    let (length_m, width_m) = key.dimensions_m();
    let half_w = width_m * 0.5;
    let mut shoulder = QuadAcc::default();
    append_ribbon(
        &mut shoulder,
        -(half_w + SHOULDER_EXTRA_HALF_W_M),
        half_w + SHOULDER_EXTRA_HALF_W_M,
        -OVERRUN_M,
        length_m + OVERRUN_M,
        0.10,
    );

    let mut deck = QuadAcc::default();
    append_ribbon(&mut deck, -half_w, half_w, 0.0, length_m, 0.20);

    let mut marks = QuadAcc::default();
    append_markings(&mut marks, length_m, half_w);

    let mut flags = QuadAcc::default();
    append_edge_flags(&mut flags, length_m, half_w);

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

/// Drops `z_above` by the spherical sagitta `-(x^2 + y^2) / (2 R)` so the entire `3.6 km`
/// skiway stays at a constant clearance above `maplibre`'s spherical polar cap.
fn curved_z(x: f64, y: f64, z_above: f64) -> f32 {
    let r = Body::EARTH.radius_meters + 2_835.0;
    (z_above - 0.5 * (x * x + y * y) / r) as f32
}

#[derive(Default)]
struct QuadAcc {
    vertices: Vec<UsdMeshVertex>,
    indices: Vec<u32>,
}

impl QuadAcc {
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
        self.push_quad(
            [
                [x0 as f32, y0 as f32, curved_z(x0, y0, z_above)],
                [x1 as f32, y0 as f32, curved_z(x1, y0, z_above)],
                [x1 as f32, y1 as f32, curved_z(x1, y1, z_above)],
                [x0 as f32, y1 as f32, curved_z(x0, y1, z_above)],
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
    acc.push_ground_rect(-inset, inset, 3.0, 8.0, z);
    if length_m > 120.0 {
        acc.push_ground_rect(-inset, inset, length_m - 8.0, length_m - 3.0, z);
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

fn append_edge_flags(acc: &mut QuadAcc, length_m: f64, half_w: f64) {
    for side in [-1.0_f64, 1.0_f64] {
        let bx = side * (half_w + 4.5);
        let z0 = curved_z(bx, 2.0, 0.15);
        let z1 = curved_z(bx, 2.0, 2.15);
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
        let mut y = 30.0;
        while y <= length_m {
            let zb = curved_z(fx, y, 0.95);
            let zt = curved_z(fx, y, 2.35);
            let outer = (fx + side * 1.55) as f32;
            let inner = fx as f32;
            let yf = y as f32;
            acc.push_quad(
                [
                    [inner, yf, zb],
                    [outer, yf, zb],
                    [outer, yf, zt],
                    [inner, yf, zt],
                ],
                [0.0, -1.0, 0.0],
            );
            acc.push_quad(
                [
                    [inner, yf - 0.65, zb],
                    [inner, yf + 0.65, zb],
                    [inner, yf + 0.65, zt],
                    [inner, yf - 0.65, zt],
                ],
                [side as f32, 0.0, 0.0],
            );
            y += FLAG_STEP_M;
        }
    }
}
