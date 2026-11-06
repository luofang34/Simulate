use glam::DVec3;
use sim_core::{CameraPoseEcef, SensorIntrinsics, VehicleState, VirtualCameraSpec};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_scenery::{AerodromeId, CatalogVerticalDatum, OsmAerodromeDiagram};

use super::scenario_virtual_cameras;
use crate::{CapsuleReentryRun, LunarLandingConfig, LunarLandingRun, ScenarioProfile};

const TOWER: usize = 3;
const APPROACH: usize = 4;

fn kewr() -> OsmAerodromeDiagram {
    OsmAerodromeDiagram::builtin(AerodromeId::NewarkKewr).unwrap()
}

/// Angle (`rad`) between the optical axis and the line of sight to `target`.
fn off_axis_rad(pose: &CameraPoseEcef, target: DVec3) -> f64 {
    let axis = pose.orientation * DVec3::NEG_Z;
    let sight = (target - pose.position_ecef).normalize();
    axis.cross(sight).length().atan2(axis.dot(sight))
}

/// Whether `target` is inside the frame vertically and horizontally.
fn in_frame(pose: &CameraPoseEcef, intrinsics: &SensorIntrinsics, target: DVec3) -> bool {
    let half_fov_y = 0.5 * VirtualCameraSpec::vertical_fov_rad(intrinsics);
    let half_fov_x = (0.5 * f64::from(intrinsics.width) / intrinsics.fx).atan();
    off_axis_rad(pose, target) < half_fov_y.min(half_fov_x)
}

fn up_at(body: &BodyEllipsoid, position_ecef: DVec3) -> DVec3 {
    LocalFrame::at(&body.fixed_to_geodetic(position_ecef)).up()
}

/// Ownship on the extended centreline `before_m` short of the primary
/// threshold, `agl_m` above the field, flying the final approach course.
fn on_final(diagram: &OsmAerodromeDiagram, before_m: f64, agl_m: f64) -> VehicleState {
    let rwy = diagram.primary_runway_end().unwrap();
    let field = Geodetic::from_degrees(
        rwy.threshold.latitude_deg,
        rwy.threshold.longitude_deg,
        diagram.field_elevation_m,
    );
    let (ellipsoidal, _) = CatalogVerticalDatum::OrthometricMslEgm96.heights(
        field.lat_rad,
        field.lon_rad,
        field.height_m,
    );
    let field = Geodetic {
        height_m: ellipsoidal.0,
        ..field
    };
    let frame = LocalFrame::at(&field);
    let (sin_b, cos_b) = rwy.true_bearing_rad.sin_cos();
    let along = frame.north() * cos_b + frame.east() * sin_b;
    let body = BodyEllipsoid::WGS84;
    let mut position = body.fixed_to_geodetic(body.geodetic_to_fixed(&field) - along * before_m);
    position.height_m = field.height_m + agl_m;
    VehicleState::level_flight(&position, rwy.true_bearing_rad, 60.0)
}

#[test]
fn pattern_trackers_frame_ownship_on_short_final_and_fifty_km_out() {
    let diagram = kewr();
    let (body, cameras) = scenario_virtual_cameras(
        Some(ScenarioProfile::AirportKewrToweredPattern),
        &diagram,
        CatalogVerticalDatum::OrthometricMslEgm96,
    );
    assert_eq!(body, BodyEllipsoid::WGS84);
    for (before_m, agl_m) in [(500.0, 27.0), (50_000.0, 900.0)] {
        let state = on_final(&diagram, before_m, agl_m);
        for index in [TOWER, APPROACH] {
            let camera = cameras.get(index).unwrap();
            let (pose, intrinsics) = camera
                .pose_and_intrinsics_on(&body, &state, DVec3::ZERO)
                .unwrap();
            let range = (pose.position_ecef - state.position_ecef).length();
            assert!(range < 18_000.0, "{} at {before_m} m: {range} m", camera.id);
            assert!(
                in_frame(&pose, &intrinsics, state.position_ecef),
                "{} at {before_m} m",
                camera.id
            );
            let look = pose.orientation * DVec3::NEG_Z;
            assert!(
                look.dot(up_at(&body, pose.position_ecef)) > -0.2,
                "{} at {before_m} m aims into the ground",
                camera.id
            );
        }
    }
}

#[test]
fn real_world_trackers_stand_on_the_ellipsoid_below_charted_msl() {
    let diagram = kewr();
    let tower_at = |datum| {
        let (_, cameras) = scenario_virtual_cameras(None, &diagram, datum);
        let near = on_final(&diagram, 500.0, 27.0);
        let (pose, _) = cameras
            .get(TOWER)
            .unwrap()
            .pose_and_intrinsics(&near, DVec3::ZERO)
            .unwrap();
        BodyEllipsoid::WGS84
            .fixed_to_geodetic(pose.position_ecef)
            .height_m
    };
    let charted = tower_at(CatalogVerticalDatum::EllipsoidWgs84);
    let raised = tower_at(CatalogVerticalDatum::OrthometricMslEgm96);
    assert!((charted - diagram.tower_viewpoint.height_m).abs() < 1e-6);
    // Newark's geoid lies about 33 m below the ellipsoid.
    assert!(
        (-36.0..-30.0).contains(&(raised - charted)),
        "{raised} vs {charted}"
    );
}

#[test]
fn lunar_cameras_stand_on_the_moon_and_frame_the_lander() {
    let (body, cameras) = scenario_virtual_cameras(
        Some(ScenarioProfile::Apollo11LunarLanding),
        &kewr(),
        CatalogVerticalDatum::OrthometricMslEgm96,
    );
    assert_eq!(body, BodyEllipsoid::MOON_IAU);
    let lander = LunarLandingRun::new(&LunarLandingConfig::default())
        .unwrap()
        .vehicle();
    let down = -up_at(&body, lander.position_ecef);
    for camera in &cameras {
        let (pose, intrinsics) = camera
            .pose_and_intrinsics_on(&body, &lander, DVec3::ZERO)
            .unwrap();
        let radius = pose.position_ecef.length();
        assert!(
            (radius - body.semi_major_m).abs() < 5_000.0,
            "{}: {radius} m from the Moon's centre",
            camera.id
        );
        let look = pose.orientation * DVec3::NEG_Z;
        match camera.id {
            "dps_nadir" => {
                assert!((look - lander.attitude * DVec3::NEG_X).length() < 1e-9);
                assert!(look.dot(down) > 0.5, "nadir looks at {}", look.dot(down));
            }
            "lpd_window" => assert!(look.dot(down) > 0.0, "the LPD view is above the horizon"),
            "tranquility_tracker" | "lunar_flyby" => assert!(
                in_frame(&pose, &intrinsics, lander.position_ecef),
                "{} loses the lander",
                camera.id
            ),
            _ => {}
        }
    }
}

#[test]
fn reentry_cameras_follow_the_capsule_on_earth() {
    let (body, cameras) = scenario_virtual_cameras(
        Some(ScenarioProfile::Apollo11CapsuleReentry),
        &kewr(),
        CatalogVerticalDatum::OrthometricMslEgm96,
    );
    assert_eq!(body, BodyEllipsoid::WGS84);
    let capsule = CapsuleReentryRun::new().unwrap().vehicle();
    for camera in cameras.iter().skip(TOWER) {
        let (pose, intrinsics) = camera
            .pose_and_intrinsics_on(&body, &capsule, DVec3::ZERO)
            .unwrap();
        let range = (pose.position_ecef - capsule.position_ecef).length();
        assert!(range < 40_000.0, "{}: {range} m", camera.id);
        assert!(
            in_frame(&pose, &intrinsics, capsule.position_ecef),
            "{} loses the capsule",
            camera.id
        );
    }
}
