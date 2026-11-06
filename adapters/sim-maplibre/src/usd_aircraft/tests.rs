use glam::{DMat3, DQuat, DVec3};
use sim_core::{AirframeArticulation, ControlInputs, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic};

use super::*;

#[test]
fn embedded_c172_composes_articulation_and_usda_export_roundtrips() {
    let asset = UsdAircraftAsset::embedded_c172();
    assert!(asset.is_ok(), "embedded c172.usdc failed: {asset:?}");
    let Some(asset) = asset.ok() else {
        return;
    };
    assert!(asset.parts().len() >= 18);
    assert!(asset.triangle_count() > 10_000);

    let origin = Geodetic {
        lat_rad: 40.6895_f64.to_radians(),
        lon_rad: (-74.1745_f64).to_radians(),
        height_m: 120.0,
    };
    let state = VehicleState {
        position_ecef: BodyEllipsoid::WGS84.geodetic_to_fixed(&origin),
        velocity_ecef: DVec3::new(0.0, 55.0, 0.0),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let art = AirframeArticulation::from_kinematic(
        &state,
        ControlInputs {
            roll: 0.25,
            pitch: 0.1,
            yaw: -0.15,
            throttle: 0.72,
        },
        4.2,
    );

    let ext_draws = asset.evaluate_draws(&art, SlotCameraRole::External);
    let hud_draws = asset.evaluate_draws(&art, SlotCameraRole::HudForward);
    let cockpit_draws = asset.evaluate_draws(&art, SlotCameraRole::CockpitInterior);
    let nadir_draws = asset.evaluate_draws(&art, SlotCameraRole::BellyNadir);
    assert!(ext_draws.len() > cockpit_draws.len());
    assert!(hud_draws.is_empty());
    assert!(!cockpit_draws.is_empty());
    assert!(nadir_draws.is_empty());

    let usda = export_scene_usda(&kewr_scene(origin, state, art));
    assert!(usda.is_ok(), "USDA scene export failed: {usda:?}");
    let Some(usda_text) = usda.ok() else {
        return;
    };
    assert!(usda_text.contains("sim:engineRpm"));
    assert!(usda_text.contains("c172.usdc"));

    let mut resolver = EmbeddedUsdResolver::single("c172.usdc", EMBEDDED_C172_USDC.into());
    resolver.insert("scene.usda", usda_text.into_bytes().into());
    let composed = openusd::usd::Stage::builder()
        .schema_registry(openusd_schemas::schema_registry())
        .resolver(resolver)
        .open("scene.usda");
    assert!(composed.is_ok(), "composed scene.usda failed: {composed:?}");
    if let Ok(stage) = composed {
        let spinner = stage
            .prim("/World/Ownship/Powerplant/PropellerHub/Spinner")
            .ok()
            .and_then(|p| p.type_name().ok().flatten());
        assert_eq!(spinner.as_ref().map(|t| t.as_str()), Some("Mesh"));
    }
}

fn kewr_scene(
    origin: Geodetic,
    state: VehicleState,
    art: AirframeArticulation,
) -> UsdSceneExportInput {
    UsdSceneExportInput {
        scenario_id: "KEWR".to_owned(),
        sim_time_s: 4.2,
        enu_origin: origin,
        c172_asset_path: "c172.usdc".to_owned(),
        ownship_state: state,
        ownship_articulation: art,
        traffic: Vec::new(),
        runways: vec![UsdRunwaySample {
            ident: "22L".to_owned(),
            threshold_geodetic: origin,
            opposite_geodetic: Geodetic {
                lat_rad: 40.6700_f64.to_radians(),
                lon_rad: (-74.1900_f64).to_radians(),
                height_m: 5.0,
            },
            width_m: 46.0,
        }],
        sun_direction_enu: [0.4, -0.3, 0.866],
        cameras: vec![UsdCameraSample {
            name: "Chase".to_owned(),
            eye_geodetic: origin,
            camera_to_enu_rotation: DMat3::IDENTITY,
            vfov_deg: 50.0,
            aspect: 16.0 / 9.0,
            clipping_range_m: [0.5, 80_000.0],
        }],
    }
}

#[test]
fn exported_ownship_axes_point_right_forward_and_up_in_the_stage_frame() {
    let origin = Geodetic::from_degrees(40.69, -74.17, 10.0);
    let frame = sim_geo::LocalFrame::at(&origin);
    // Level, heading east: forward is East, right is South, up is Up.
    let state = VehicleState::level_flight(&origin, std::f64::consts::FRAC_PI_2, 50.0);
    let q = export::aircraft_usd_to_enu_quat(&frame, state.attitude);
    let close = |a: DVec3, b: DVec3| (a - b).length() < 1e-6;
    assert!(close(q * DVec3::Y, DVec3::X), "{:?}", q * DVec3::Y);
    assert!(close(q * DVec3::X, DVec3::NEG_Y), "{:?}", q * DVec3::X);
    assert!(close(q * DVec3::Z, DVec3::Z), "{:?}", q * DVec3::Z);
}
