use std::f64::consts::FRAC_PI_2;

use glam::DVec3;
use sim_core::{CameraMount, VehicleState};
use sim_geo::{Geodetic, LocalFrame, SensorIntrinsics, ecef_to_geodetic};
use sim_pack::builtin_light_trainer_package;

use super::*;
use crate::{CalibrationLifecycle, PilotageBridgeError, package_camera_calibration};

fn sample_detent(calibration_id: u32, camera_id: u32, focal_px: f64) -> ZoomDetent {
    let intrinsics = SensorIntrinsics {
        width: 640,
        height: 480,
        fx: focal_px,
        fy: focal_px,
        cx: 320.0,
        cy: 240.0,
    };
    let mut pkg_cam = builtin_light_trainer_package()
        .unwrap()
        .camera_mounts()
        .unwrap()
        .remove(0);
    pkg_cam.intrinsics = intrinsics;
    let calibrated = package_camera_calibration(
        &pkg_cam,
        CalibrationLifecycle {
            calibration_id,
            camera_id,
            version: 1,
            effective_from_unix_ns: 0,
            effective_until_unix_ns: u64::MAX,
        },
    );
    ZoomDetent {
        intrinsics,
        calibrated,
    }
}

#[test]
fn length_delimited_envelopes_round_trip_and_omit_zero_proto3_scalars() {
    // Camera_id = 0 (FPV) must omit tag 6 per proto3 wire rules.
    let frame = BridgeFrame {
        width: 4,
        height: 2,
        pixel_format: PIXEL_FORMAT_RGB_INT8.to_owned(),
        sim_time_ns: 99_000,
        rgb: vec![128; 24],
        camera_id: CAMERA_ID_FPV,
    };
    let env = BridgeEnvelope {
        payload: Some(BridgePayload::Frame(frame)),
    };
    let framed = encode_length_delimited_envelope(&env);
    let (decoded, used) = decode_length_delimited_envelope(&framed).unwrap().unwrap();
    assert_eq!(used, framed.len());
    assert_eq!(decoded, env);

    // Partial slice returns Ok(None) without error.
    assert!(
        decode_length_delimited_envelope(&framed[..framed.len() - 3])
            .unwrap()
            .is_none()
    );

    // Camera command + ViewPose round-trip.
    let cmd = BridgeEnvelope {
        payload: Some(BridgePayload::CameraCommand(BridgeCameraCommand {
            mode: 2,
            pan_rad: 0.25,
            tilt_rad: -0.5,
            zoom_detent: 1,
        })),
    };
    let vp = BridgeEnvelope {
        payload: Some(BridgePayload::ViewPose(BridgeViewPose {
            sim_time_ns: 1_000_000,
            latitude_deg: 39.158,
            longitude_deg: -75.524,
            altitude_m: 150.0,
            vertical_datum: 3,
            q_w: 1.0,
            q_x: 0.0,
            q_y: 0.0,
            q_z: 0.0,
            gimbal: Some(BridgeGimbalPose {
                pan_rad: 0.1,
                tilt_rad: -0.2,
                zoom_detent: 1,
            }),
        })),
    };
    let mut concat = encode_length_delimited_envelope(&cmd);
    concat.extend_from_slice(&encode_length_delimited_envelope(&vp));

    let (d1, n1) = decode_length_delimited_envelope(&concat).unwrap().unwrap();
    assert_eq!(d1, cmd);
    let (d2, n2) = decode_length_delimited_envelope(&concat[n1..])
        .unwrap()
        .unwrap();
    assert_eq!(d2, vp);
    assert_eq!(n1 + n2, concat.len());
}

#[test]
fn vertical_datums_resolve_honestly_and_refuse_missing_surfaces() {
    let pose_msl = BridgeViewPose {
        sim_time_ns: 10,
        latitude_deg: 39.0,
        longitude_deg: -75.0,
        altitude_m: 100.0,
        vertical_datum: 2, // MSL
        q_w: 1.0,
        ..BridgeViewPose::default()
    };
    // EllipsoidalOnlyDatum refuses MSL rather than guessing geoid undulation = 0!
    assert!(matches!(
        vehicle_state_from_view_pose(&pose_msl, &EllipsoidalOnlyDatum),
        Err(PilotageBridgeError::UnsupportedVerticalDatum { datum: 2 })
    ));

    let surfaces = StaticDatumSurfaces {
        geoid_undulation_m: Some(-33.5),
        terrain_ellipsoid_height_m: Some(12.0),
    };
    let state_msl = vehicle_state_from_view_pose(&pose_msl, &surfaces).unwrap();
    let geo_msl = ecef_to_geodetic(state_msl.position_ecef);
    assert!((geo_msl.height_m - 66.5).abs() < 1.0e-3);

    let pose_agl = BridgeViewPose {
        vertical_datum: 3, // AGL
        altitude_m: 50.0,
        ..pose_msl
    };
    let state_agl = vehicle_state_from_view_pose(&pose_agl, &surfaces).unwrap();
    let geo_agl = ecef_to_geodetic(state_agl.position_ecef);
    assert!((geo_agl.height_m - 62.0).abs() < 1.0e-3);
}

#[test]
fn camera_pose_resolves_fpv_gimbal_zoom_detents_and_free_release() {
    let geo = Geodetic {
        lat_rad: 39.0_f64.to_radians(),
        lon_rad: (-75.0_f64).to_radians(),
        height_m: 500.0,
    };
    // Vehicle flying North, rolled 30 deg right: gimbal tilt must stay world-stabilized!
    let mut vehicle = VehicleState::level_flight(&geo, 0.0, 40.0);
    vehicle.attitude =
        (vehicle.attitude * glam::DQuat::from_rotation_x(30.0_f64.to_radians())).normalize();

    let fpv_mount = CameraMount::forward_nose(DVec3::new(1.0, 0.0, 0.0));
    let fpv_detent = sample_detent(101, CAMERA_ID_FPV, 500.0);
    let gimbal_detents = [
        sample_detent(201, CAMERA_ID_GIMBAL, 450.0),
        sample_detent(202, CAMERA_ID_GIMBAL, 900.0),
    ];

    let state = CameraViewState::default();
    let fpv = resolve_vehicle_camera_pose(
        &vehicle,
        DVec3::ZERO,
        &fpv_mount,
        &fpv_detent,
        DVec3::new(0.5, 0.0, 0.3),
        &gimbal_detents,
        &state,
    )
    .unwrap()
    .unwrap();
    assert_eq!(fpv.camera_id, CAMERA_ID_FPV);
    assert_eq!(fpv.calibration_id, 101);
}

#[test]
fn gimbal_horizon_stabilizes_through_airframe_roll_and_free_mode_releases() {
    let geo = Geodetic {
        lat_rad: 39.0_f64.to_radians(),
        lon_rad: (-75.0_f64).to_radians(),
        height_m: 500.0,
    };
    let mut vehicle = VehicleState::level_flight(&geo, 0.0, 40.0);
    vehicle.attitude =
        (vehicle.attitude * glam::DQuat::from_rotation_x(30.0_f64.to_radians())).normalize();

    let fpv_mount = CameraMount::forward_nose(DVec3::new(1.0, 0.0, 0.0));
    let fpv_detent = sample_detent(101, CAMERA_ID_FPV, 500.0);
    let gimbal_detents = [
        sample_detent(201, CAMERA_ID_GIMBAL, 450.0),
        sample_detent(202, CAMERA_ID_GIMBAL, 900.0),
    ];
    let mut state = CameraViewState::default();

    // Switch to Gimbal (mode=2), pan +90 deg East, tilt 0 (level horizon), zoom detent 1.
    state
        .apply_command(&BridgeCameraCommand {
            mode: 2,
            pan_rad: FRAC_PI_2 as f32,
            tilt_rad: 0.0,
            zoom_detent: 1,
        })
        .unwrap();
    // Delta command with mode=0 keeps Gimbal active while updating tilt!
    state
        .apply_command(&BridgeCameraCommand {
            mode: 0,
            pan_rad: FRAC_PI_2 as f32,
            tilt_rad: 0.0,
            zoom_detent: 1,
        })
        .unwrap();
    assert_eq!(state.mode, BridgeCameraMode::Gimbal);

    let gim = resolve_vehicle_camera_pose(
        &vehicle,
        DVec3::ZERO,
        &fpv_mount,
        &fpv_detent,
        DVec3::new(0.5, 0.0, 0.3),
        &gimbal_detents,
        &state,
    )
    .unwrap()
    .unwrap();
    assert_eq!(gim.camera_id, CAMERA_ID_GIMBAL);
    assert_eq!(gim.calibration_id, 202);
    assert_eq!(gim.intrinsics.fx, 900.0);

    // Renderer camera looks along local -Z with +Y up: verify optical forward points East
    // and camera +Y points local Up despite the 30-deg airframe bank!
    let local = LocalFrame::at(&ecef_to_geodetic(gim.pose.position_ecef));
    let cam_fwd_ecef = gim.pose.orientation * DVec3::NEG_Z;
    let cam_up_ecef = gim.pose.orientation * DVec3::Y;
    assert!((cam_fwd_ecef.dot(local.east()) - 1.0).abs() < 1.0e-5);
    assert!((cam_up_ecef.dot(local.up()) - 1.0).abs() < 1.0e-5);

    state.mode = BridgeCameraMode::Free;
    let released = resolve_vehicle_camera_pose(
        &vehicle,
        DVec3::ZERO,
        &fpv_mount,
        &fpv_detent,
        DVec3::ZERO,
        &gimbal_detents,
        &state,
    );
    assert!(released.unwrap().is_none());
}

#[test]
fn latest_frame_queue_drops_superseded_frames_and_converts_rgba() {
    let rgba = vec![10, 20, 30, 255, 40, 50, 60, 128];
    assert_eq!(
        rgba8_to_rgb_int8(&rgba).unwrap(),
        vec![10, 20, 30, 40, 50, 60]
    );

    let q = LatestFrameQueue::new();
    for idx in 0..5_u64 {
        assert!(q.submit(BridgeFrame {
            width: 1,
            height: 1,
            pixel_format: PIXEL_FORMAT_RGB_INT8.to_owned(),
            sim_time_ns: idx,
            rgb: vec![1, 2, 3],
            camera_id: 0,
        }));
    }
    assert_eq!(q.dropped_frames(), 4);
    let newest = q.try_take().unwrap();
    assert_eq!(newest.sim_time_ns, 4);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn tcp_sidecar_streams_frames_to_host_and_folds_camera_commands() {
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::thread;
    use std::time::{Duration, Instant};

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let host_thread = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        let cmd = BridgeEnvelope {
            payload: Some(BridgePayload::CameraCommand(BridgeCameraCommand {
                mode: 2,
                pan_rad: -0.35,
                tilt_rad: 0.15,
                zoom_detent: 1,
            })),
        };
        stream
            .write_all(&encode_length_delimited_envelope(&cmd))
            .unwrap();

        let mut buf = Vec::new();
        let mut chunk = [0_u8; 512];
        loop {
            let n = stream.read(&mut chunk).unwrap();
            assert!(n > 0);
            buf.extend_from_slice(&chunk[..n]);
            if let Some((env, _)) = decode_length_delimited_envelope(&buf).unwrap() {
                return env;
            }
        }
    });

    let sidecar = SimVideoSidecar::connect(addr).unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    while sidecar.latest_camera_command().is_none() {
        assert!(Instant::now() < deadline);
        thread::sleep(Duration::from_millis(2));
    }
    let state = sidecar.effective_view_state();
    assert_eq!(state.mode, BridgeCameraMode::Gimbal);
    assert_eq!(state.zoom_detent, 1);

    assert!(sidecar.submit_frame(BridgeFrame {
        width: 2,
        height: 1,
        pixel_format: PIXEL_FORMAT_RGB_INT8.to_owned(),
        sim_time_ns: 42_000_000,
        rgb: vec![255, 0, 0, 0, 255, 0],
        camera_id: CAMERA_ID_GIMBAL,
    }));

    let received = host_thread.join().unwrap();
    match received.payload {
        Some(BridgePayload::Frame(frame)) => {
            assert_eq!(frame.pixel_format, PIXEL_FORMAT_RGB_INT8);
            assert_eq!(frame.camera_id, CAMERA_ID_GIMBAL);
            assert_eq!(frame.sim_time_ns, 42_000_000);
            assert_eq!(frame.rgb, vec![255, 0, 0, 0, 255, 0]);
        }
        other => panic!("expected BridgeFrame, got {other:?}"),
    }
}
