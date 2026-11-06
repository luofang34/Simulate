use std::time::Duration;

use glam::{DQuat, DVec3};
use sim_core::{CameraMount, SimInstant};

use super::{c172, sample_scenery_and_runtime, try_host_gpu};
use crate::{
    CelestialDisc, CelestialSkyInput, CelestialStar, CelestialTwilight, SceneryEndpoints,
    TruthSceneConfig, TruthSceneFrame, TruthSceneMap, TruthSlotMask, conformal_stamp_from_vehicle,
    symmetric_frustum, tests::sample_vehicle_at, truth_real_world_style,
};

fn apply_sample_sky(truth: &mut TruthSceneMap) {
    let star = CelestialStar {
        direction_ecef: DVec3::new(0.0, 0.6, 0.8),
        v_mag: 0.5,
        srgb: [1.0, 1.0, 0.9],
    };
    truth.set_celestial(&CelestialSkyInput {
        sun_direction_ecef: DVec3::new(0.8, 0.2, 0.5).normalize(),
        sun_angular_radius_deg: Some(0.27),
        sun_visible_fraction: 1.0,
        moon: Some(CelestialDisc {
            direction_ecef: DVec3::new(-0.5, 0.5, 0.7).normalize(),
            angular_radius_deg: 0.26,
        }),
        earth_shadow: None,
        twilight: CelestialTwilight {
            night_end_deg: -6.0,
            day_start_deg: 2.0,
            night_brightness: 0.35,
        },
        stars: Some(&[star]),
    });
}

#[test]
fn slot_mask_steps_only_requested_slots() {
    let Some(gpu) = try_host_gpu() else {
        return;
    };
    let (scenery, _, runtime) = sample_scenery_and_runtime(gpu);
    let mut truth = TruthSceneMap::new(
        &runtime,
        truth_real_world_style(&SceneryEndpoints::catalog(0, 0)).expect("truth style"),
        TruthSceneConfig {
            otw_size: [160, 90],
            sensor_size: [80, 60],
            ..TruthSceneConfig::default()
        },
        c172(),
    )
    .expect("truth map");
    assert_eq!(truth.supply_dem_synchronously(&scenery).expect("dem"), 1);
    truth.set_pixel_ratio(2.0);
    apply_sample_sky(&mut truth);

    let veh = sample_vehicle_at(39.0, -75.5, 600.0, DQuat::IDENTITY);
    let stamp = |w, h| {
        conformal_stamp_from_vehicle(
            SimInstant::START,
            &veh,
            &CameraMount::forward_nose(DVec3::ZERO),
            DVec3::ZERO,
            symmetric_frustum(1.0, w, h, 0.5, 250_000.0).expect("frustum"),
            0.0,
        )
        .expect("stamp")
    };
    let frame = |ms| TruthSceneFrame {
        timestamp: Duration::from_millis(ms),
        otw_stamp: Some(stamp(160, 90)),
        stereo_right_stamp: None,
        sensor_stamp: Some(stamp(80, 60)),
        extra_camera_slots: &[],
        entities: &[],
        aircraft: &[],
        ownship_camera_roles: &[],
        polar_runway: None,
    };

    truth
        .render_frame_slots(frame(16), TruthSlotMask::OTW_ONLY)
        .expect("otw frame");
    assert!(truth.last_otw_stamp().is_some());
    assert!(truth.last_sensor_stamp().is_none());

    let mut sensor_only = TruthSceneMap::new(
        &runtime,
        truth_real_world_style(&SceneryEndpoints::catalog(0, 0)).expect("truth style"),
        TruthSceneConfig::default(),
        c172(),
    )
    .expect("truth map");
    sensor_only
        .render_frame_slots(frame(16), TruthSlotMask::SENSOR_ONLY)
        .expect("sensor frame");
    assert!(sensor_only.last_otw_stamp().is_none());
    assert!(sensor_only.last_sensor_stamp().is_some());
}
