use glam::DVec3;
use pilotage_camera_calibration::{
    CALIBRATION_SCHEMA_VERSION, CalibrationError, SIM_FPV_CALIBRATION_HASH, sim_fpv_calibration,
    verify_sim_recovery,
};
use sim_core::{CameraMount, SensorIntrinsics, TickSpan};
use sim_pack::builtin_c172_package;

use super::*;

const NOW_UNIX_NS: u64 = 1_793_000_000_000_000_000;
const LIFECYCLE: CalibrationLifecycle = CalibrationLifecycle {
    calibration_id: 7,
    camera_id: 3,
    version: 1,
    effective_from_unix_ns: 1_577_836_800_000_000_000,
    effective_until_unix_ns: 2_051_222_400_000_000_000,
};

fn c172_camera() -> PackageCamera {
    let package = builtin_c172_package().unwrap();
    package.camera_mounts().unwrap().remove(0)
}

fn rotate(quat_wxyz: [f64; 4], vector: DVec3) -> DVec3 {
    let [w, x, y, z] = quat_wxyz;
    DQuat::from_xyzw(x, y, z, w) * vector
}

#[test]
fn the_c172s_camera_verifies_with_its_own_geometry() {
    let calibrated = package_camera_calibration(&c172_camera(), LIFECYCLE);
    calibrated.calibration.validate().unwrap();
    let model = calibrated.verified_model(NOW_UNIX_NS).unwrap();

    assert_eq!(model.calibration_id(), CalibrationId(7));
    assert_eq!(model.content_hash(), calibrated.recorded_hash);
    let (half_x, half_y) = model.half_fov_tangents();
    assert!((half_x - 400.0 / 600.0).abs() < 1e-12);
    assert!((half_y - 300.0 / 600.0).abs() < 1e-12);
    assert!(model.alignment_bound_rad() > 0.0);
}

/// The nose camera is the one Pilotage's own simulator publishes: its rotation
/// must take body axes to the same OpenCV axes, so the published artifact checks
/// the half-turn from Simulate's camera axes.
#[test]
fn a_nose_camera_matches_pilotages_published_body_to_camera_rotation() {
    let ours = package_camera_calibration(&c172_camera(), LIFECYCLE)
        .calibration
        .geometry
        .extrinsics
        .rotation_quat_wxyz;
    let published = sim_fpv_calibration().geometry.extrinsics.rotation_quat_wxyz;
    for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
        assert!((rotate(ours, axis) - rotate(published, axis)).length() < 1e-12);
    }
    assert!(
        (rotate(ours, DVec3::X) - DVec3::Z).length() < 1e-12,
        "forward"
    );
    assert!(
        (rotate(ours, DVec3::Y) - DVec3::X).length() < 1e-12,
        "right"
    );
    assert!((rotate(ours, DVec3::Z) - DVec3::Y).length() < 1e-12, "down");
}

#[test]
fn a_belly_camera_looks_down_in_opencv_axes() {
    let camera = PackageCamera {
        mount: CameraMount::nadir_belly(DVec3::new(0.5, 0.0, 1.0)),
        ..c172_camera()
    };
    let calibrated = package_camera_calibration(&camera, LIFECYCLE);
    let extrinsics = calibrated.calibration.geometry.extrinsics;
    assert_eq!(extrinsics.translation_body_m, [0.5, 0.0, 1.0]);
    assert!((rotate(extrinsics.rotation_quat_wxyz, DVec3::Z) - DVec3::Z).length() < 1e-12);
    calibrated.verified_model(NOW_UNIX_NS).unwrap();
}

#[test]
fn the_budget_covers_the_fit_it_records() {
    let calibration = package_camera_calibration(&c172_camera(), LIFECYCLE).calibration;
    let residuals = calibration.identity.residuals;
    assert!(
        residuals.max_px > 0.0 && residuals.max_px < 1.0,
        "{residuals:?}"
    );
    assert!(residuals.rms_px <= residuals.max_px);
    assert!(calibration.allowances.intrinsic_residual_px >= residuals.max_px);
}

/// The fit here must agree with Pilotage's own tool on Pilotage's own camera,
/// because that tool's report cannot be called on other intrinsics.
#[test]
fn the_fit_matches_pilotages_recovery_on_its_published_camera() {
    let report = verify_sim_recovery();
    let residuals = fit_residuals(&sim_fpv_calibration().geometry.intrinsics);
    assert!((residuals.rms_px - report.residual_rms_px).abs() < 1e-12);
    assert!((residuals.max_px - report.residual_max_px).abs() < 1e-12);
}

#[test]
fn a_changed_value_fails_the_recorded_hash() {
    let mut calibrated = package_camera_calibration(&c172_camera(), LIFECYCLE);
    calibrated.calibration.geometry.intrinsics.focal_x_px += 1.0;
    assert!(matches!(
        calibrated.verified_model(NOW_UNIX_NS),
        Err(CalibrationError::ContentHashMismatch { .. })
    ));
}

#[test]
fn an_expired_calibration_does_not_verify() {
    let calibrated = package_camera_calibration(&c172_camera(), LIFECYCLE);
    assert!(
        calibrated
            .verified_model(LIFECYCLE.effective_until_unix_ns)
            .is_err()
    );
}

#[test]
fn a_zero_sized_camera_is_refused_with_its_reason() {
    let camera = PackageCamera {
        intrinsics: SensorIntrinsics {
            width: 0,
            ..c172_camera().intrinsics
        },
        period: TickSpan::SECOND,
        ..c172_camera()
    };
    let calibrated = package_camera_calibration(&camera, LIFECYCLE);
    assert!(matches!(
        calibrated.verified_model(NOW_UNIX_NS),
        Err(CalibrationError::InvalidViewport { .. })
    ));
}

#[test]
fn the_artifact_is_distinct_from_pilotages_and_its_schema_is_current() {
    let calibrated = package_camera_calibration(&c172_camera(), LIFECYCLE);
    assert_ne!(calibrated.recorded_hash, SIM_FPV_CALIBRATION_HASH);
    assert_eq!(CALIBRATION_SCHEMA_VERSION, 2);
}
