use glam::{DMat3, DQuat, DVec3, EulerRot};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics};
use sim_spacetime::SensorMount;

use crate::{Block, BlockError, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};

#[cfg(test)]
mod tests;

const BLOCK: &str = "camera_sensor";
const DEFAULT_RATE_HZ: u64 = 30;
// A rate that does not divide the master clock would drift, so it is a build
// error rather than a runtime fallback.
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(DEFAULT_RATE_HZ));

/// Default capture period, 30 Hz.
pub const CAMERA_DEFAULT_PERIOD: TickSpan =
    TickSpan::from_ticks(TICKS_PER_SECOND / DEFAULT_RATE_HZ);

/// Turns renderer camera axes (`+Y` up, `-Z` forward) into optical axes (`+Y`
/// down, `+Z` forward) and back; a half turn about `+X`, so its own inverse.
const OPTICAL_FROM_RENDER: DMat3 = DMat3::from_diagonal(DVec3::new(1.0, -1.0, -1.0));

/// Where a camera sits on the airframe.
///
/// Camera axes here are the renderer's: `+X` right, `+Y` up, looking along
/// `-Z`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraMount {
    /// Camera position in the component tree's root axes, metres.
    pub offset_body_m: DVec3,
    /// Rotates body-axis components into camera-axis components.
    pub body_to_camera: DQuat,
}

impl CameraMount {
    /// Looks along body `+x` with the image's top towards body `-z`, the view
    /// out of the nose.
    pub fn forward_nose(offset_body_m: DVec3) -> Self {
        Self {
            offset_body_m,
            body_to_camera: nose_camera_to_body().inverse(),
        }
    }

    /// Looks along body `+z`, straight down in level flight, with the image's
    /// top towards the nose.
    pub fn nadir_belly(offset_body_m: DVec3) -> Self {
        let camera_to_body = DMat3::from_cols(DVec3::Y, DVec3::X, DVec3::NEG_Z);
        Self {
            offset_body_m,
            body_to_camera: DQuat::from_mat3(&camera_to_body).inverse(),
        }
    }

    /// The nose view turned by aerospace yaw, pitch, and roll about the body
    /// axes: yaw `+90°` looks out of the right window, and negative pitch
    /// tilts the view below the horizon.
    pub fn oblique(offset_body_m: DVec3, yaw_rad: f64, pitch_rad: f64, roll_rad: f64) -> Self {
        let turned = DQuat::from_euler(EulerRot::ZYX, yaw_rad, pitch_rad, roll_rad);
        Self {
            offset_body_m,
            body_to_camera: (turned * nose_camera_to_body()).normalize().inverse(),
        }
    }

    /// Looks along body-axis `forward_body` with the image's top as near
    /// `up_body` as that allows, for airframes whose axes are not an
    /// aircraft's, such as a lander thrusting along `+x`. A degenerate pair
    /// falls back to [`Self::forward_nose`]'s orientation.
    pub fn looking_along(offset_body_m: DVec3, forward_body: DVec3, up_body: DVec3) -> Self {
        let forward = forward_body.try_normalize();
        let right = forward.and_then(|f| f.cross(up_body).try_normalize());
        let camera_to_body = match (forward, right) {
            (Some(forward), Some(right)) => {
                DQuat::from_mat3(&DMat3::from_cols(right, right.cross(forward), -forward))
            }
            _ => nose_camera_to_body(),
        };
        Self {
            offset_body_m,
            body_to_camera: camera_to_body.normalize().inverse(),
        }
    }

    /// Where the camera is at `vehicle`, whose position is the centre of
    /// gravity at `cg_body_m` in the same axes as the mount's offset.
    pub fn pose(&self, vehicle: &VehicleState, cg_body_m: DVec3) -> CameraPoseEcef {
        let kin = self.to_sensor_mount().evaluate_world_kinematics(
            vehicle.position_ecef,
            vehicle.velocity_ecef,
            vehicle.attitude,
            vehicle.angular_rate_body,
            cg_body_m,
        );
        CameraPoseEcef {
            position_ecef: kin.position_ecef_or_world(),
            orientation: kin.q_mount_to_world,
        }
    }

    /// Converts this [`CameraMount`] into a unified [`sim_spacetime::SensorMount`].
    #[must_use]
    pub const fn to_sensor_mount(self) -> SensorMount {
        SensorMount {
            offset_body_m: self.offset_body_m,
            body_to_mount: self.body_to_camera,
        }
    }

    /// Creates a [`CameraMount`] from a unified [`sim_spacetime::SensorMount`].
    #[must_use]
    pub const fn from_sensor_mount(mount: SensorMount) -> Self {
        Self {
            offset_body_m: mount.offset_body_m,
            body_to_camera: mount.body_to_mount,
        }
    }
}

impl From<CameraMount> for SensorMount {
    fn from(mount: CameraMount) -> Self {
        mount.to_sensor_mount()
    }
}

impl From<SensorMount> for CameraMount {
    fn from(mount: SensorMount) -> Self {
        Self::from_sensor_mount(mount)
    }
}

trait MountedKinematicsPosExt {
    fn position_ecef_or_world(&self) -> DVec3;
}

impl MountedKinematicsPosExt for sim_spacetime::MountedKinematics {
    fn position_ecef_or_world(&self) -> DVec3 {
        self.position_world_m
    }
}

/// Camera axes in body axes for the view out of the nose.
fn nose_camera_to_body() -> DQuat {
    DQuat::from_mat3(&DMat3::from_cols(DVec3::Y, DVec3::NEG_Z, DVec3::NEG_X))
}

/// A camera's place in the body-fixed frame, in the renderer's convention:
/// `orientation` rotates camera components (`+X` right, `+Y` up, `-Z` forward)
/// into body-fixed components.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraPoseEcef {
    /// Eye position, ECEF metres.
    pub position_ecef: DVec3,
    /// Camera-to-ECEF rotation.
    pub orientation: DQuat,
}

/// The origin and axes of a catalog's local east-north-up frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnuAnchor {
    origin_ecef: DVec3,
    frame: LocalFrame,
}

impl EnuAnchor {
    /// The tangent frame at `origin` on `body`.
    pub fn new(body: &BodyEllipsoid, origin: &Geodetic) -> Self {
        Self {
            origin_ecef: body.geodetic_to_fixed(origin),
            frame: LocalFrame::at(origin),
        }
    }
}

/// A camera's place in `Navigate`'s convention, relative to an [`EnuAnchor`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavigateCameraPose {
    /// Eye position, ENU metres.
    pub eye_enu: [f64; 3],
    /// Row-major rotation from ENU components to camera components, with
    /// camera `+X` right, `+Y` down, `+Z` forward.
    pub world_to_cam: [f64; 9],
}

impl CameraPoseEcef {
    /// The same pose in `Navigate`'s convention.
    pub fn to_navigate(&self, anchor: &EnuAnchor) -> NavigateCameraPose {
        let eye = anchor
            .frame
            .ecef_to_enu_vector(self.position_ecef - anchor.origin_ecef);
        let render_from_ecef = DMat3::from_quat(self.orientation.inverse());
        let world_to_cam = OPTICAL_FROM_RENDER * render_from_ecef * anchor.frame.enu_to_ecef();
        NavigateCameraPose {
            eye_enu: eye.to_array(),
            world_to_cam: world_to_cam.transpose().to_cols_array(),
        }
    }
}

impl NavigateCameraPose {
    /// The same pose in the renderer's convention.
    pub fn to_ecef(&self, anchor: &EnuAnchor) -> CameraPoseEcef {
        let world_to_cam = DMat3::from_cols_array(&self.world_to_cam).transpose();
        let ecef_from_render =
            anchor.frame.enu_to_ecef() * world_to_cam.transpose() * OPTICAL_FROM_RENDER;
        CameraPoseEcef {
            position_ecef: anchor.origin_ecef
                + anchor.frame.enu_to_ecef() * DVec3::from_array(self.eye_enu),
            orientation: DQuat::from_mat3(&ecef_from_render).normalize(),
        }
    }
}

fn validate_intrinsics(intrinsics: &SensorIntrinsics) -> Result<(), BlockError> {
    if intrinsics.is_usable() {
        return Ok(());
    }
    Err(BlockError::InputOutOfRange {
        block: BLOCK,
        input: "intrinsics",
        detail: format!("{intrinsics:?}"),
    })
}

/// Fixed setup of a [`CameraSensorBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraSensorConfig {
    /// Where the camera sits.
    pub mount: CameraMount,
    /// The camera's optics.
    pub intrinsics: SensorIntrinsics,
    /// The catalog frame poses are reported in.
    pub anchor: EnuAnchor,
    /// Capture period; [`CAMERA_DEFAULT_PERIOD`] unless a test needs another.
    pub period: TickSpan,
}

/// What a [`CameraSensorBlock`] reads each step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraSensorInput {
    /// The vehicle at the capture instant.
    pub vehicle: VehicleState,
    /// Where the vehicle's centre of gravity is, in the mount's axes, so the
    /// camera stays fixed to the airframe as propellant burns.
    pub cg_body_m: DVec3,
}

/// One exposure: everything needed to render it and to compare a visual
/// estimate of it against the truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraFrameCapture {
    /// The capture instant.
    pub timestamp: SimInstant,
    /// True vehicle state.
    pub vehicle: VehicleState,
    /// True camera pose in the renderer's convention.
    pub pose: CameraPoseEcef,
    /// True eye position in the anchor frame, metres.
    pub eye_enu: [f64; 3],
    /// True anchor-to-camera rotation; see [`NavigateCameraPose::world_to_cam`].
    pub world_to_cam: [f64; 9],
    /// The camera's optics.
    pub intrinsics: SensorIntrinsics,
}

/// Complete state of a [`CameraSensorBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraSensorSnapshot {
    next_capture: SimInstant,
    latest: Option<CameraFrameCapture>,
}

/// A camera fixed to the airframe, taking one exposure each period.
///
/// Each step records the state in its input as the exposure at the block's
/// clock, then advances the clock, so the first capture is at
/// [`SimInstant::START`]. The output is `None` until then.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraSensorBlock {
    config: CameraSensorConfig,
    snapshot: CameraSensorSnapshot,
}

impl CameraSensorBlock {
    /// A camera with no exposures yet. Fails on unusable optics or a zero
    /// period.
    pub fn new(config: CameraSensorConfig) -> Result<Self, BlockError> {
        validate_intrinsics(&config.intrinsics)?;
        if config.period == TickSpan::ZERO {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "period",
                detail: "a zero period never advances".to_owned(),
            });
        }
        Ok(Self {
            config,
            snapshot: CameraSensorSnapshot {
                next_capture: SimInstant::START,
                latest: None,
            },
        })
    }
}

impl Block for CameraSensorBlock {
    type Input = CameraSensorInput;
    type Output = Option<CameraFrameCapture>;
    type Snapshot = CameraSensorSnapshot;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &CameraSensorInput) -> Result<(), BlockError> {
        let vehicle = input.vehicle;
        if !(vehicle.position_ecef.is_finite()
            && vehicle.attitude.is_finite()
            && input.cg_body_m.is_finite())
        {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "vehicle",
                detail: format!("{input:?} is not finite"),
            });
        }
        let pose = self.config.mount.pose(&vehicle, input.cg_body_m);
        let navigate = pose.to_navigate(&self.config.anchor);
        let timestamp = self.snapshot.next_capture;
        self.snapshot.latest = Some(CameraFrameCapture {
            timestamp,
            vehicle,
            pose,
            eye_enu: navigate.eye_enu,
            world_to_cam: navigate.world_to_cam,
            intrinsics: self.config.intrinsics,
        });
        self.snapshot.next_capture = timestamp.advanced(self.config.period);
        Ok(())
    }

    fn output(&self) -> Option<CameraFrameCapture> {
        self.snapshot.latest
    }

    fn snapshot(&self) -> CameraSensorSnapshot {
        self.snapshot
    }

    fn restore(&mut self, snapshot: &CameraSensorSnapshot) {
        self.snapshot = *snapshot;
    }
}
