//! Regression tests for 3D camera-eye frustum and near-plane clipping of SVS / XVS
//! runway surface fills, sidelines, threshold bars, and centreline stripes.

use glam::{DQuat, DVec3};
use sim_core::{CameraMount, SimInstant, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, egm96_undulation_m};
use sim_scenery::{AirportDatabase, CatalogVerticalDatum, LandingRunwayGeometry};

use crate::{
    DEFAULT_FAR_CLIP_M, DEFAULT_NEAR_CLIP_M, build_conformal_overlay,
    conformal_stamp_from_vehicle_on, default_render_body, symmetric_frustum,
};

fn pose_over_runway(
    rwy: &LandingRunwayGeometry,
    along_m: f64,
    agl_m: f64,
    pitch_deg: f64,
    roll_deg: f64,
) -> (VehicleState, f64) {
    let thr_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&rwy.threshold);
    let far_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&rwy.far_end);
    let local = LocalFrame::at(&rwy.threshold);
    let chord = far_ecef - thr_ecef;
    let fwd = (chord - local.up() * chord.dot(local.up())).normalize();
    let pos_ecef = thr_ecef + fwd * along_m + local.up() * agl_m;
    let geo = BodyEllipsoid::WGS84.fixed_to_geodetic(pos_ecef);
    let undul_m = egm96_undulation_m(geo.lat_rad, geo.lon_rad);
    let ned_to_ecef = LocalFrame::at(&geo).ned_to_ecef();
    let ecef_from_ned = DQuat::from_mat3(&ned_to_ecef);
    let ned_body = DQuat::from_rotation_z(rwy.true_bearing_rad)
        * DQuat::from_rotation_y(pitch_deg.to_radians())
        * DQuat::from_rotation_x(roll_deg.to_radians());
    let vehicle = VehicleState {
        position_ecef: pos_ecef,
        velocity_ecef: fwd * 60.0,
        attitude: (ecef_from_ned * ned_body).normalize(),
        angular_rate_body: DVec3::ZERO,
    };
    (vehicle, undul_m)
}

fn build_overflight_mesh(
    code: &str,
    designator: &str,
    along_m: f64,
    agl_m: f64,
    roll: f64,
    vp: [u32; 2],
) -> (
    maplibre::overlay::OverlayMesh,
    Vec<maplibre::overlay::DepthProbe>,
) {
    let db = AirportDatabase::builtin();
    let (_, rec) = db.by_code(code).expect("airport exists");
    let rwy = rec
        .landing_runway(designator, CatalogVerticalDatum::OrthometricMslEgm96)
        .expect("runway exists");
    let frustum = symmetric_frustum(
        45.0_f64.to_radians(),
        vp[0],
        vp[1],
        DEFAULT_NEAR_CLIP_M,
        DEFAULT_FAR_CLIP_M,
    )
    .expect("frustum");
    let (vehicle, undul_m) = pose_over_runway(&rwy, along_m, agl_m, -2.0, roll);
    let stamp = conformal_stamp_from_vehicle_on(
        SimInstant::START,
        BodyEllipsoid::WGS84,
        default_render_body(BodyEllipsoid::WGS84),
        &vehicle,
        &CameraMount::forward_nose(DVec3::ZERO),
        DVec3::ZERO,
        frustum,
        undul_m,
    )
    .expect("stamp");
    let _ = Geodetic::from_degrees(0.0, 0.0, 0.0);
    build_conformal_overlay(&stamp, vp, Some(&rwy), &[], None)
}

fn assert_no_foreground_cutoff(code: &str, designator: &str, along_m: f64, agl_m: f64, roll: f64) {
    let vp = [710_u32, 550_u32];
    let (mesh, probes) = build_overflight_mesh(code, designator, along_m, agl_m, roll, vp);
    assert!(!mesh.is_empty(), "{code} {designator} overlay empty");
    assert_eq!(
        probes.len(),
        1,
        "{code} {designator} must keep an in-viewport forward depth probe during overflight"
    );

    let w = vp[0] as f32;
    let h = vp[1] as f32;
    let cx = w * 0.5;
    // Surface fill (primary 0.56, displaced paved 0.30, turf 0.42) and sidelines (0.92 / 0.50).
    let is_fill =
        |a: f32| (a - 0.56).abs() < 0.04 || (a - 0.30).abs() < 0.04 || (a - 0.42).abs() < 0.04;
    let is_rail = |a: f32| (a - 0.92).abs() < 0.04 || (a - 0.50).abs() < 0.04;
    let fill_max_y = mesh
        .vertices
        .iter()
        .filter(|v| is_fill(v.color[3]))
        .map(|v| v.position[1])
        .fold(f32::NEG_INFINITY, f32::max);
    let left_exits_bezel = mesh
        .vertices
        .iter()
        .filter(|v| is_rail(v.color[3]) && v.position[0] < cx)
        .any(|v| v.position[1] >= h || v.position[0] <= 0.0);
    let right_exits_bezel = mesh
        .vertices
        .iter()
        .filter(|v| is_rail(v.color[3]) && v.position[0] > cx)
        .any(|v| v.position[1] >= h || v.position[0] >= w);

    assert!(
        fill_max_y >= h,
        "{code} {designator} surface fill truncated inside viewport: max_y={fill_max_y:.1} < {h}"
    );
    assert!(
        left_exits_bezel,
        "{code} {designator} left sideline truncated inside viewport [0..{w}, 0..{h}]"
    );
    assert!(
        right_exits_bezel,
        "{code} {designator} right sideline truncated inside viewport [0..{w}, 0..{h}]"
    );
    for v in &mesh.vertices {
        assert!(
            v.position[0].is_finite()
                && v.position[1].is_finite()
                && v.position[0].abs() < 5_000.0
                && v.position[1].abs() < 5_000.0,
            "unbounded projected coordinate: {:?}",
            v.position
        );
        assert!(
            (0.0..=1.0).contains(&v.depth),
            "invalid reversed-Z depth: {}",
            v.depth
        );
    }
}

#[test]
fn svs_runway_never_truncates_mid_viewport_during_banked_low_overflight_or_rollout() {
    // Reproduces the exact 120 ft (~31 m AGL), -10 deg left-banked mid-runway overflight
    // as well as flare/rollout right across the threshold at KEWR, KDOV, and NZSP.
    for (code, designator, along_m, agl_m, roll_deg) in [
        ("KEWR", "22L", 420.0, 31.0, -10.0),
        ("KEWR", "22L", 55.0, 8.0, 12.0),
        ("KEWR", "22L", -20.0, 4.0, -6.0),
        ("KDOV", "14", -300.0, 28.0, -11.0),
        ("KDOV", "14", 650.0, 32.0, 10.0),
        ("NZSP", "02T", 500.0, 31.0, -10.0),
        ("NZSP", "02T", 15.0, 2.2, 5.0),
    ] {
        assert_no_foreground_cutoff(code, designator, along_m, agl_m, roll_deg);
    }
}
