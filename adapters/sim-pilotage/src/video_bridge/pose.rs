use glam::{DMat3, DQuat, DVec3};
use sim_core::{CameraMount, VehicleState};
use sim_geo::{Geodetic, LocalFrame, SensorIntrinsics, ecef_to_geodetic, geodetic_to_ecef};
use sim_render::CameraPose;

use crate::video_bridge::link::{CAMERA_ID_FPV, CAMERA_ID_GIMBAL};
use crate::video_bridge::wire::{BridgeCameraCommand, BridgeGimbalPose, BridgeViewPose};
use crate::{CalibratedCamera, PilotageBridgeError};

/// Active camera embodiment selected by `BridgeCameraCommand.mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BridgeCameraMode {
    /// Body-fixed forward view (`mode = 1`, `camera_id = 0`).
    #[default]
    Fpv,
    /// Heading-following, horizon-stabilized gimbal payload view (`mode = 2`, `camera_id = 2`).
    Gimbal,
    /// Producer releases the view to its local interactive operator (`mode = 3`).
    Free,
}

impl BridgeCameraMode {
    /// Parses the 1-based wire mode (`0` returns `None`, meaning unchanged).
    pub fn from_wire(mode: u32) -> Result<Option<Self>, PilotageBridgeError> {
        match mode {
            0 => Ok(None),
            1 => Ok(Some(Self::Fpv)),
            2 => Ok(Some(Self::Gimbal)),
            3 => Ok(Some(Self::Free)),
            other => Err(PilotageBridgeError::InvalidCameraView {
                detail: format!("unknown BridgeCameraCommand mode {other}"),
            }),
        }
    }

    /// 1-based wire encoding for `BridgeCameraCommand.mode`.
    #[must_use]
    pub const fn to_wire(self) -> u32 {
        match self {
            Self::Fpv => 1,
            Self::Gimbal => 2,
            Self::Free => 3,
        }
    }
}

/// Vertical datum code carried on [`BridgeViewPose`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerticalDatum {
    /// Height above the WGS84 reference ellipsoid (`1`).
    Ellipsoid,
    /// Orthometric height above mean sea level (`2`).
    Msl,
    /// Height above local terrain (`3`).
    Agl,
}

impl VerticalDatum {
    /// Parses a 1-based wire vertical datum code.
    pub const fn from_wire(code: u32) -> Result<Self, PilotageBridgeError> {
        match code {
            1 => Ok(Self::Ellipsoid),
            2 => Ok(Self::Msl),
            3 => Ok(Self::Agl),
            datum => Err(PilotageBridgeError::UnsupportedVerticalDatum { datum }),
        }
    }
}

/// Supplies geoid undulation and terrain height when resolving non-ellipsoidal
/// [`BridgeViewPose`] altitudes. Never guesses zero when a surface is unknown.
pub trait VerticalDatumResolver {
    /// Geoid height above the WGS84 ellipsoid (`N`, metres) at `(lat_rad, lon_rad)`.
    fn geoid_undulation_m(&self, lat_rad: f64, lon_rad: f64) -> Option<f64>;
    /// Terrain height above the WGS84 ellipsoid (`h_terrain`, metres) at `(lat_rad, lon_rad)`.
    fn terrain_ellipsoid_height_m(&self, lat_rad: f64, lon_rad: f64) -> Option<f64>;
}

/// Strict datum resolver accepting only [`VerticalDatum::Ellipsoid`].
#[derive(Debug, Clone, Copy, Default)]
pub struct EllipsoidalOnlyDatum;

impl VerticalDatumResolver for EllipsoidalOnlyDatum {
    fn geoid_undulation_m(&self, _lat_rad: f64, _lon_rad: f64) -> Option<f64> {
        None
    }

    fn terrain_ellipsoid_height_m(&self, _lat_rad: f64, _lon_rad: f64) -> Option<f64> {
        None
    }
}

/// Constant local geoid/terrain datum surfaces for flat-corridor or test scenes.
#[derive(Debug, Clone, Copy, Default)]
pub struct StaticDatumSurfaces {
    /// Optional geoid undulation (`m` above WGS84 ellipsoid).
    pub geoid_undulation_m: Option<f64>,
    /// Optional terrain ellipsoidal height (`m` above WGS84 ellipsoid).
    pub terrain_ellipsoid_height_m: Option<f64>,
}

impl VerticalDatumResolver for StaticDatumSurfaces {
    fn geoid_undulation_m(&self, _lat_rad: f64, _lon_rad: f64) -> Option<f64> {
        self.geoid_undulation_m
    }

    fn terrain_ellipsoid_height_m(&self, _lat_rad: f64, _lon_rad: f64) -> Option<f64> {
        self.terrain_ellipsoid_height_m
    }
}

/// Folded latest-value camera state maintained across delta commands.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CameraViewState {
    /// Active camera mode (`FPV` initially).
    pub mode: BridgeCameraMode,
    /// Gimbal pan (`rad`, positive right relative to body heading).
    pub pan_rad: f64,
    /// Gimbal tilt (`rad`, positive up, world-stabilized).
    pub tilt_rad: f64,
    /// Active zoom detent index.
    pub zoom_detent: usize,
}

impl CameraViewState {
    /// Applies an inbound `BridgeCameraCommand`, preserving the previous mode
    /// when `command.mode == 0` per `bridge.proto`.
    pub fn apply_command(
        &mut self,
        command: &BridgeCameraCommand,
    ) -> Result<(), PilotageBridgeError> {
        if !(command.pan_rad.is_finite() && command.tilt_rad.is_finite()) {
            return Err(PilotageBridgeError::InvalidCameraView {
                detail: "non-finite gimbal pan or tilt in BridgeCameraCommand".to_owned(),
            });
        }
        if let Some(next_mode) = BridgeCameraMode::from_wire(command.mode)? {
            self.mode = next_mode;
        }
        self.pan_rad = f64::from(command.pan_rad);
        self.tilt_rad = f64::from(command.tilt_rad);
        self.zoom_detent = command.zoom_detent as usize;
        Ok(())
    }

    /// Applies an optional `BridgeGimbalPose` carried inside a `BridgeViewPose`.
    pub fn apply_gimbal_pose(
        &mut self,
        gimbal: &BridgeGimbalPose,
    ) -> Result<(), PilotageBridgeError> {
        if !(gimbal.pan_rad.is_finite() && gimbal.tilt_rad.is_finite()) {
            return Err(PilotageBridgeError::InvalidCameraView {
                detail: "non-finite pan or tilt in BridgeGimbalPose".to_owned(),
            });
        }
        self.pan_rad = f64::from(gimbal.pan_rad);
        self.tilt_rad = f64::from(gimbal.tilt_rad);
        self.zoom_detent = gimbal.zoom_detent as usize;
        Ok(())
    }
}

/// One discrete zoom detent entry pairing optics with its published calibration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZoomDetent {
    /// Pinhole sensor intrinsics for this zoom step.
    pub intrinsics: SensorIntrinsics,
    /// Published `pilotage-camera-calibration` artifact for conformal overlay.
    pub calibrated: CalibratedCamera,
}

/// Resolved offscreen render camera pose and metadata for one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedCameraView {
    /// Renderer eye position and orientation in `f64` ECEF.
    pub pose: CameraPose,
    /// Pinhole optics to project with.
    pub intrinsics: SensorIntrinsics,
    /// Wire `BridgeFrame.camera_id` (`0` for FPV, `2` for gimbal).
    pub camera_id: u32,
    /// Published `CalibrationId` matching `intrinsics`.
    pub calibration_id: u32,
}

/// Converts an external [`BridgeViewPose`] into a [`VehicleState`] in `f64` ECEF.
pub fn vehicle_state_from_view_pose(
    view: &BridgeViewPose,
    datum: &dyn VerticalDatumResolver,
) -> Result<VehicleState, PilotageBridgeError> {
    if !(view.latitude_deg.is_finite()
        && view.longitude_deg.is_finite()
        && view.altitude_m.is_finite()
        && (-90.0..=90.0).contains(&view.latitude_deg)
        && (-180.0..=180.0).contains(&view.longitude_deg))
    {
        return Err(PilotageBridgeError::InvalidCameraView {
            detail: "invalid geodetic coordinates in BridgeViewPose".to_owned(),
        });
    }
    let lat_rad = view.latitude_deg.to_radians();
    let lon_rad = view.longitude_deg.to_radians();
    let height_m = match VerticalDatum::from_wire(view.vertical_datum)? {
        VerticalDatum::Ellipsoid => view.altitude_m,
        VerticalDatum::Msl => {
            let n = datum.geoid_undulation_m(lat_rad, lon_rad).ok_or(
                PilotageBridgeError::UnsupportedVerticalDatum {
                    datum: view.vertical_datum,
                },
            )?;
            view.altitude_m + n
        }
        VerticalDatum::Agl => {
            let h_terrain = datum.terrain_ellipsoid_height_m(lat_rad, lon_rad).ok_or(
                PilotageBridgeError::UnsupportedVerticalDatum {
                    datum: view.vertical_datum,
                },
            )?;
            h_terrain + view.altitude_m
        }
    };

    let q_frd_to_ned = DQuat::from_xyzw(view.q_x, view.q_y, view.q_z, view.q_w);
    let norm_sq = q_frd_to_ned.length_squared();
    if !norm_sq.is_finite() || norm_sq < 1.0e-12 {
        return Err(PilotageBridgeError::InvalidCameraView {
            detail: "degenerate attitude quaternion in BridgeViewPose".to_owned(),
        });
    }
    let geodetic = Geodetic {
        lat_rad,
        lon_rad,
        height_m,
    };
    let local = LocalFrame::at(&geodetic);
    let body_to_ecef = local.ned_to_ecef() * DMat3::from_quat(q_frd_to_ned.normalize());
    Ok(VehicleState {
        position_ecef: geodetic_to_ecef(&geodetic),
        velocity_ecef: DVec3::ZERO,
        attitude: DQuat::from_mat3(&body_to_ecef).normalize(),
        angular_rate_body: DVec3::ZERO,
    })
}

/// Resolves the active [`CameraPose`] and [`SensorIntrinsics`] for `vehicle` and
/// `view_state`. Returns `Ok(None)` in [`BridgeCameraMode::Free`].
pub fn resolve_vehicle_camera_pose(
    vehicle: &VehicleState,
    cg_body_m: DVec3,
    fpv_mount: &CameraMount,
    fpv_detent: &ZoomDetent,
    gimbal_offset_body_m: DVec3,
    gimbal_detents: &[ZoomDetent],
    view_state: &CameraViewState,
) -> Result<Option<ResolvedCameraView>, PilotageBridgeError> {
    match view_state.mode {
        BridgeCameraMode::Free => Ok(None),
        BridgeCameraMode::Fpv => {
            let cam_ecef = fpv_mount.pose(vehicle, cg_body_m);
            Ok(Some(ResolvedCameraView {
                pose: CameraPose {
                    position_ecef: cam_ecef.position_ecef,
                    orientation: cam_ecef.orientation,
                },
                intrinsics: fpv_detent.intrinsics,
                camera_id: CAMERA_ID_FPV,
                calibration_id: fpv_detent.calibrated.calibration.identity.calibration_id.0,
            }))
        }
        BridgeCameraMode::Gimbal => {
            let Some(detent) = gimbal_detents.get(view_state.zoom_detent) else {
                return Err(PilotageBridgeError::InvalidCameraView {
                    detail: format!(
                        "gimbal zoom_detent {} out of range (len {})",
                        view_state.zoom_detent,
                        gimbal_detents.len()
                    ),
                });
            };
            let eye_ecef =
                vehicle.position_ecef + vehicle.attitude * (gimbal_offset_body_m - cg_body_m);
            let local = LocalFrame::at(&ecef_to_geodetic(eye_ecef));
            let fwd_ned = local.ecef_to_ned_vector(vehicle.forward_ecef());
            let right_ned = local.ecef_to_ned_vector(vehicle.attitude * DVec3::Y);
            let body_heading = if fwd_ned.x.hypot(fwd_ned.y) > 1.0e-6 {
                fwd_ned.y.atan2(fwd_ned.x)
            } else {
                right_ned.y.atan2(right_ned.x) - std::f64::consts::FRAC_PI_2
            };

            let az = body_heading + view_state.pan_rad;
            let (sin_az, cos_az) = az.sin_cos();
            let (sin_tilt, cos_tilt) = view_state.tilt_rad.sin_cos();
            let horiz_fwd_ecef = local.north() * cos_az + local.east() * sin_az;
            let opt_fwd_ecef = horiz_fwd_ecef * cos_tilt + local.up() * sin_tilt;
            // Keep camera up well-defined even at exact nadir (-90 deg) or zenith (+90 deg).
            let ref_up_ecef = if cos_tilt.abs() > 1.0e-5 {
                local.up()
            } else if sin_tilt < 0.0 {
                horiz_fwd_ecef
            } else {
                -horiz_fwd_ecef
            };
            Ok(Some(ResolvedCameraView {
                pose: CameraPose::looking(eye_ecef, opt_fwd_ecef, ref_up_ecef),
                intrinsics: detent.intrinsics,
                camera_id: CAMERA_ID_GIMBAL,
                calibration_id: detent.calibrated.calibration.identity.calibration_id.0,
            }))
        }
    }
}
