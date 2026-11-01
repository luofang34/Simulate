use std::f64::consts::FRAC_PI_3;

use glam::{DMat3, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

use super::{VirtualCameraPlacement, VirtualCameraRigBlock, VirtualCameraSpec};
use crate::{Block, CAMERA_DEFAULT_PERIOD, CameraMount, EnuAnchor, VehicleState};

#[test]
fn tower_auto_track_points_optical_axis_at_aircraft_with_level_horizon_and_zoom() {
    let tower_geo = Geodetic::from_degrees(40.692_574, -74.182_761, 64.5);
    let ac_near = Geodetic::from_degrees(40.697_000, -74.182_761, 64.5);
    let ac_far = Geodetic::from_degrees(40.718_000, -74.182_761, 320.0);

    let spec = VirtualCameraSpec {
        id: "tower_ptz",
        title: "Tower PTZ",
        placement: VirtualCameraPlacement::WorldAutoTrack {
            position: tower_geo,
            lead_time_s: 0.0,
            auto_zoom_span_m: Some(50.0),
            min_fov_y_rad: 5.0_f64.to_radians(),
            max_fov_y_rad: 55.0_f64.to_radians(),
        },
        base_intrinsics: VirtualCameraSpec::pinhole_800x600(FRAC_PI_3),
        period: CAMERA_DEFAULT_PERIOD,
        depth_readout: true,
    };

    let state_near = VehicleState::level_flight(&ac_near, 0.0, 45.0);
    let (pose_near, intr_near) = spec
        .pose_and_intrinsics(&state_near, DVec3::ZERO)
        .unwrap_or_else(|_| unreachable_fallback());
    let tower_frame = LocalFrame::at(&tower_geo);
    let optical_fwd = pose_near.orientation * DVec3::NEG_Z;
    let cam_right = pose_near.orientation * DVec3::X;
    let cam_up = pose_near.orientation * DVec3::Y;

    assert!(optical_fwd.dot(tower_frame.north()) > 0.999);
    assert!(cam_right.dot(tower_frame.east()) > 0.999);
    assert!(cam_up.dot(tower_frame.up()) > 0.999);
    assert!((DMat3::from_quat(pose_near.orientation).determinant() - 1.0).abs() < 1e-9);

    let state_far = VehicleState::level_flight(&ac_far, 0.0, 45.0);
    let (_pose_far, intr_far) = spec
        .pose_and_intrinsics(&state_far, DVec3::ZERO)
        .unwrap_or_else(|_| unreachable_fallback());
    assert!(
        intr_far.fy > intr_near.fy,
        "telephoto focal length increases with slant range"
    );
}

#[test]
fn virtual_camera_rig_snapshots_and_replays_bit_identically() {
    let origin = Geodetic::from_degrees(40.524_5, -74.598_4, 26.0);
    let anchor = EnuAnchor::new(&BodyEllipsoid::WGS84, &origin);
    let specs = vec![
        VirtualCameraSpec {
            id: "nose_nav",
            title: "Nose Nav",
            placement: VirtualCameraPlacement::BodyMounted(CameraMount::forward_nose(DVec3::new(
                1.5, 0.0, 0.0,
            ))),
            base_intrinsics: VirtualCameraSpec::pinhole_800x600(50.0_f64.to_radians()),
            period: CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        },
        VirtualCameraSpec {
            id: "threshold_final",
            title: "Threshold Final",
            placement: VirtualCameraPlacement::WorldFixed {
                position: origin,
                azimuth_rad: 232.0_f64.to_radians(),
                elevation_rad: 3.0_f64.to_radians(),
                roll_rad: 0.0,
            },
            base_intrinsics: VirtualCameraSpec::pinhole_800x600(40.0_f64.to_radians()),
            period: CAMERA_DEFAULT_PERIOD,
            depth_readout: true,
        },
    ];
    let mut rig = VirtualCameraRigBlock::at_default_rate(specs, anchor)
        .unwrap_or_else(|_| unreachable_rig(anchor));
    let state = VehicleState::level_flight(&origin, 0.9, 42.0);
    assert!(rig.step(&(state, DVec3::ZERO)).is_ok());
    let snap = rig.snapshot();
    assert!(rig.step(&(state, DVec3::ZERO)).is_ok());
    let second = rig.output();
    rig.restore(&snap);
    assert!(rig.step(&(state, DVec3::ZERO)).is_ok());
    assert_eq!(rig.output(), second);
}

fn unreachable_fallback() -> (crate::CameraPoseEcef, crate::SensorIntrinsics) {
    (
        crate::CameraPoseEcef {
            position_ecef: DVec3::ZERO,
            orientation: glam::DQuat::IDENTITY,
        },
        VirtualCameraSpec::pinhole_800x600(1.0),
    )
}

fn unreachable_rig(anchor: EnuAnchor) -> VirtualCameraRigBlock {
    VirtualCameraRigBlock {
        specs: Vec::new(),
        anchor,
        period: CAMERA_DEFAULT_PERIOD,
        snapshot: super::VirtualCameraRigSnapshot {
            next_capture: crate::SimInstant::START,
            captures: Vec::new(),
        },
    }
}
