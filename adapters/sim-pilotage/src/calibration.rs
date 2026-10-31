use glam::DQuat;
use indicate_frames::FrameKind;
use pilotage_calibration_id::CalibrationId;
use pilotage_camera_calibration::{
    AlignmentAllowances, BodyToCameraExtrinsics, Boresight, BrownConradyDistortion,
    CalibrationError, CalibrationIdentity, CalibrationVersion, CameraCalibration, CameraGeometry,
    DesignEye, EffectivePeriod, OpticalConvention, PinholeIntrinsics, ProvenanceSource, Residuals,
    SyntheticTarget, ToolVersion, ValidityStatus, VerifiedCameraModel, Viewport,
    recover_intrinsics,
};
use sim_pack::PackageCamera;

#[cfg(test)]
mod tests;

/// Half turn about camera `+X`: from Simulate's camera axes (right, up, back)
/// to the OpenCV axes Pilotage calibrates in (right, down, forward).
const OPENCV_FROM_CAMERA: DQuat = DQuat::from_xyzw(1.0, 0.0, 0.0, 0.0);
/// Target grid of the intrinsic fit, per side.
const TARGET_STEPS: u32 = 12;
/// Extent of the target grid on the normalised image plane.
const TARGET_HALF_WIDTH: f64 = 0.6;
const TARGET_HALF_HEIGHT: f64 = 0.45;
const TOOL_VERSION: ToolVersion = ToolVersion {
    major: 0,
    minor: 1,
    patch: 0,
};

/// Lifecycle of a calibration the package itself cannot state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationLifecycle {
    /// Routing identity video frames carry; never zero.
    pub calibration_id: u32,
    /// Physical camera this calibration describes.
    pub camera_id: u32,
    /// Content version, bumped with any calibrated value.
    pub version: u32,
    /// When the calibration takes effect, Unix nanoseconds.
    pub effective_from_unix_ns: u64,
    /// When it stops, Unix nanoseconds.
    pub effective_until_unix_ns: u64,
}

/// A package camera's calibration artifact with its recorded content hash.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibratedCamera {
    /// The artifact.
    pub calibration: CameraCalibration,
    /// The content hash published with it.
    pub recorded_hash: [u8; 32],
}

impl CalibratedCamera {
    /// Verifies the artifact at `now_unix_ns` and mints the only proof of that
    /// verification Pilotage accepts.
    pub fn verified_model(
        &self,
        now_unix_ns: u64,
    ) -> Result<VerifiedCameraModel, CalibrationError> {
        self.calibration
            .verified_camera_model(self.recorded_hash, now_unix_ns)
    }
}

/// `SyntheticTarget`s over the image, each pixel rounded to a whole pixel, the
/// only error a simulated camera's fit has.
fn fit_targets(intrinsics: &PinholeIntrinsics) -> Vec<SyntheticTarget> {
    let span =
        |step: u32, half: f64| -half + 2.0 * half * f64::from(step) / f64::from(TARGET_STEPS);
    (0..=TARGET_STEPS)
        .flat_map(|column| (0..=TARGET_STEPS).map(move |row| (column, row)))
        .map(|(column, row)| {
            let x = span(column, TARGET_HALF_WIDTH);
            let y = span(row, TARGET_HALF_HEIGHT);
            SyntheticTarget {
                point_camera_m: [x, y, 1.0],
                observed_px: [
                    (intrinsics.focal_x_px * x + intrinsics.principal_x_px).round(),
                    (intrinsics.focal_y_px * y + intrinsics.principal_y_px).round(),
                ],
            }
        })
        .collect()
}

/// The fit's residuals: run the recovery rather than declare a number, because
/// the budget must cover what the fit actually left.
fn fit_residuals(intrinsics: &PinholeIntrinsics) -> Residuals {
    let targets = fit_targets(intrinsics);
    let recovered = recover_intrinsics(&targets);
    let squares: Vec<f64> = targets
        .iter()
        .map(|target| {
            let [x, y, z] = target.point_camera_m;
            let [u, v] = target.observed_px;
            let du = recovered.focal_x_px * x / z + recovered.principal_x_px - u;
            let dv = recovered.focal_y_px * y / z + recovered.principal_y_px - v;
            du * du + dv * dv
        })
        .collect();
    let sum: f64 = squares.iter().sum();
    let largest = squares.iter().copied().fold(0.0, f64::max);
    Residuals {
        rms_px: (sum / squares.len() as f64).sqrt(),
        max_px: largest.sqrt(),
    }
}

/// The artifact Pilotage's conformal path needs for `camera`.
///
/// Rotation moves from Simulate's right, up, back camera axes into OpenCV's right,
/// down, forward. The lever arm stays in body axes, measured from the package's
/// root as the camera's own offset is. The principal point keeps Navigate's and
/// OpenCV's convention of pixel centres at integers.
pub fn package_camera_calibration(
    camera: &PackageCamera,
    lifecycle: CalibrationLifecycle,
) -> CalibratedCamera {
    let intrinsics = PinholeIntrinsics {
        focal_x_px: camera.intrinsics.fx,
        focal_y_px: camera.intrinsics.fy,
        principal_x_px: camera.intrinsics.cx,
        principal_y_px: camera.intrinsics.cy,
        skew_px: 0.0,
        convention: OpticalConvention::OpenCv,
    };
    let DQuat { x, y, z, w } = (OPENCV_FROM_CAMERA * camera.mount.body_to_camera).normalize();
    let residuals = fit_residuals(&intrinsics);
    let calibration = CameraCalibration {
        geometry: CameraGeometry {
            intrinsics,
            distortion: BrownConradyDistortion::NONE,
            viewport: Viewport {
                width_px: camera.intrinsics.width,
                height_px: camera.intrinsics.height,
            },
            extrinsics: BodyToCameraExtrinsics {
                translation_body_m: camera.mount.offset_body_m.to_array(),
                rotation_quat_wxyz: [w, x, y, z],
                from_kind: FrameKind::Body,
                to_kind: FrameKind::Installation,
            },
            design_eye: DesignEye {
                position_installation_m: [0.0, 0.0, 0.0],
            },
            boresight: Boresight {
                direction_camera: [0.0, 0.0, 1.0],
            },
        },
        identity: CalibrationIdentity {
            calibration_id: CalibrationId(lifecycle.calibration_id),
            camera_id: lifecycle.camera_id,
            version: CalibrationVersion(lifecycle.version),
            tool_version: TOOL_VERSION,
            effective: EffectivePeriod {
                start_unix_ns: lifecycle.effective_from_unix_ns,
                end_unix_ns: lifecycle.effective_until_unix_ns,
            },
            provenance: ProvenanceSource::SimSyntheticTool,
            residuals,
            status: ValidityStatus::Valid,
        },
        allowances: AlignmentAllowances::sim_defaults(residuals.max_px),
    };
    CalibratedCamera {
        recorded_hash: calibration.content_hash(),
        calibration,
    }
}
