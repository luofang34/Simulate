//! Configurable airframe-mounted, chase, fixed-ground, and auto-tracking PTZ
//! virtual cameras stepped deterministically on the master clock.

use glam::{DMat3, DQuat, DVec3};
use sim_geo::{Geodetic, LocalFrame, SensorIntrinsics, ecef_to_geodetic, geodetic_to_ecef};

use crate::{
    Block, BlockError, CAMERA_DEFAULT_PERIOD, CameraFrameCapture, CameraMount, CameraPoseEcef,
    EnuAnchor, SimInstant, TickSpan, VehicleState,
};

#[cfg(test)]
mod tests;

const BLOCK: &str = "virtual_camera_rig";

/// How a virtual camera is anchored and aimed in the simulation world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VirtualCameraPlacement {
    /// Rigidly or obliquely mounted on the airframe (`+X` forward, `+Y` right, `+Z` down).
    BodyMounted(CameraMount),
    /// Horizon-stabilized external chase eye trailing the vehicle's horizontal heading.
    ChaseOffset {
        /// Distance aft of the vehicle origin (`m`).
        back_m: f64,
        /// Distance right of the vehicle origin (`m`).
        right_m: f64,
        /// Distance above the vehicle origin along local zenith (`m`).
        up_m: f64,
        /// Elevation pitch relative to local horizon (`rad`, negative tilts nose down).
        pitch_rad: f64,
    },
    /// Stationary ground or runway-threshold camera fixed on WGS84.
    WorldFixed {
        /// Geodetic eye position on WGS84.
        position: Geodetic,
        /// True azimuth clockwise from north (`rad`).
        azimuth_rad: f64,
        /// Elevation angle above the local horizontal plane (`rad`).
        elevation_rad: f64,
        /// Roll angle clockwise around the optical axis (`rad`).
        roll_rad: f64,
    },
    /// Control-tower or range-tracking PTZ camera anchored on WGS84 and aiming at the vehicle.
    WorldAutoTrack {
        /// Geodetic tower cab or tripod eye position on WGS84.
        position: Geodetic,
        /// Velocity lead time (`s`) added to `vehicle.position_ecef` when aiming.
        lead_time_s: f64,
        /// Optional target scene span (`m`) at the vehicle range for automatic telephoto zoom.
        auto_zoom_span_m: Option<f64>,
        /// Narrowest vertical field of view (`rad`) allowed during telephoto zoom.
        min_fov_y_rad: f64,
        /// Widest vertical field of view (`rad`) allowed at close range.
        max_fov_y_rad: f64,
    },
}

/// Complete specification of one virtual camera channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VirtualCameraSpec {
    /// Stable machine identifier (e.g. `"nose_nav"`, `"tower_ptz"`, `"threshold_final"`).
    pub id: &'static str,
    /// Human-readable display label.
    pub title: &'static str,
    /// Mounting or world-space tracking rule.
    pub placement: VirtualCameraPlacement,
    /// Base pinhole intrinsics (`fx`/`fy` are scaled dynamically only in `WorldAutoTrack` with zoom).
    pub base_intrinsics: SensorIntrinsics,
    /// Exposure period on the master clock (typically [`CAMERA_DEFAULT_PERIOD`]).
    pub period: TickSpan,
    /// Whether an offscreen renderer should resolve `Depth32Float` range readouts for this camera.
    pub depth_readout: bool,
}

impl VirtualCameraSpec {
    /// Standard `800 × 600` pinhole intrinsics for a given vertical field of view (`rad`).
    #[must_use]
    pub fn pinhole_800x600(fov_y_rad: f64) -> SensorIntrinsics {
        let clamped_fov = fov_y_rad.clamp(0.02, 2.6);
        let fy = 300.0 / (0.5 * clamped_fov).tan();
        SensorIntrinsics {
            width: 800,
            height: 600,
            fx: fy,
            fy,
            cx: 399.5,
            cy: 299.5,
        }
    }

    /// Instantaneous vertical field of view (`rad`) of `intrinsics`.
    #[must_use]
    pub fn vertical_fov_rad(intrinsics: &SensorIntrinsics) -> f64 {
        2.0 * (0.5 * f64::from(intrinsics.height) / intrinsics.fy.max(1.0)).atan()
    }

    /// Evaluates the instantaneous ECEF camera pose and pinhole intrinsics for `vehicle`.
    pub fn pose_and_intrinsics(
        &self,
        vehicle: &VehicleState,
        cg_body_m: DVec3,
    ) -> Result<(CameraPoseEcef, SensorIntrinsics), BlockError> {
        if !self.base_intrinsics.is_usable() {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "base_intrinsics",
                detail: format!("{:?}", self.base_intrinsics),
            });
        }
        if !(vehicle.position_ecef.is_finite()
            && vehicle.velocity_ecef.is_finite()
            && vehicle.attitude.is_finite()
            && cg_body_m.is_finite())
        {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "vehicle",
                detail: "non-finite vehicle state".to_owned(),
            });
        }
        Ok(match self.placement {
            VirtualCameraPlacement::BodyMounted(mount) => {
                (mount.pose(vehicle, cg_body_m), self.base_intrinsics)
            }
            VirtualCameraPlacement::ChaseOffset {
                back_m,
                right_m,
                up_m,
                pitch_rad,
            } => (
                chase_pose(vehicle, back_m, right_m, up_m, pitch_rad),
                self.base_intrinsics,
            ),
            VirtualCameraPlacement::WorldFixed {
                position,
                azimuth_rad,
                elevation_rad,
                roll_rad,
            } => (
                world_fixed_pose(&position, azimuth_rad, elevation_rad, roll_rad),
                self.base_intrinsics,
            ),
            VirtualCameraPlacement::WorldAutoTrack {
                position,
                lead_time_s,
                auto_zoom_span_m,
                min_fov_y_rad,
                max_fov_y_rad,
            } => world_auto_track_pose(
                &position,
                vehicle,
                lead_time_s,
                &self.base_intrinsics,
                auto_zoom_span_m,
                min_fov_y_rad,
                max_fov_y_rad,
            ),
        })
    }

    /// Captures a stamped [`CameraFrameCapture`] relative to `anchor`.
    pub fn capture(
        &self,
        timestamp: SimInstant,
        vehicle: &VehicleState,
        cg_body_m: DVec3,
        anchor: &EnuAnchor,
    ) -> Result<CameraFrameCapture, BlockError> {
        let (pose, intrinsics) = self.pose_and_intrinsics(vehicle, cg_body_m)?;
        let nav = pose.to_navigate(anchor);
        Ok(CameraFrameCapture {
            timestamp,
            vehicle: *vehicle,
            pose,
            eye_enu: nav.eye_enu,
            world_to_cam: nav.world_to_cam,
            intrinsics,
        })
    }
}

/// Snapshot of a [`VirtualCameraRigBlock`].
#[derive(Debug, Clone, PartialEq)]
pub struct VirtualCameraRigSnapshot {
    next_capture: SimInstant,
    captures: Vec<CameraFrameCapture>,
}

/// Multi-camera sensor rig capturing synchronized exposures across body and ground cameras.
#[derive(Debug, Clone, PartialEq)]
pub struct VirtualCameraRigBlock {
    specs: Vec<VirtualCameraSpec>,
    anchor: EnuAnchor,
    period: TickSpan,
    snapshot: VirtualCameraRigSnapshot,
}

impl VirtualCameraRigBlock {
    /// Creates a multi-camera rig stepping at `period` (for example [`CAMERA_DEFAULT_PERIOD`]).
    pub fn new(
        specs: Vec<VirtualCameraSpec>,
        anchor: EnuAnchor,
        period: TickSpan,
    ) -> Result<Self, BlockError> {
        if period == TickSpan::ZERO {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "period",
                detail: "a zero period never advances".to_owned(),
            });
        }
        for spec in &specs {
            if !spec.base_intrinsics.is_usable() {
                return Err(BlockError::InputOutOfRange {
                    block: BLOCK,
                    input: spec.id,
                    detail: "unusable intrinsics".to_owned(),
                });
            }
        }
        Ok(Self {
            specs,
            anchor,
            period,
            snapshot: VirtualCameraRigSnapshot {
                next_capture: SimInstant::START,
                captures: Vec::new(),
            },
        })
    }

    /// Creates a `30 Hz` ([`CAMERA_DEFAULT_PERIOD`]) rig.
    pub fn at_default_rate(
        specs: Vec<VirtualCameraSpec>,
        anchor: EnuAnchor,
    ) -> Result<Self, BlockError> {
        Self::new(specs, anchor, CAMERA_DEFAULT_PERIOD)
    }

    /// Configured camera specifications in slot order.
    #[must_use]
    pub fn specs(&self) -> &[VirtualCameraSpec] {
        &self.specs
    }

    /// Latest exposure for camera `index`, if captured.
    #[must_use]
    pub fn capture_at(&self, index: usize) -> Option<&CameraFrameCapture> {
        self.snapshot.captures.get(index)
    }
}

impl Block for VirtualCameraRigBlock {
    type Input = (VehicleState, DVec3);
    type Output = Vec<CameraFrameCapture>;
    type Snapshot = VirtualCameraRigSnapshot;

    fn period(&self) -> TickSpan {
        self.period
    }

    fn step(&mut self, input: &Self::Input) -> Result<(), BlockError> {
        let (vehicle, cg_body_m) = *input;
        let timestamp = self.snapshot.next_capture;
        let mut captures = Vec::with_capacity(self.specs.len());
        for spec in &self.specs {
            captures.push(spec.capture(timestamp, &vehicle, cg_body_m, &self.anchor)?);
        }
        self.snapshot.captures = captures;
        self.snapshot.next_capture = timestamp.advanced(self.period);
        Ok(())
    }

    fn output(&self) -> Self::Output {
        self.snapshot.captures.clone()
    }

    fn snapshot(&self) -> Self::Snapshot {
        self.snapshot.clone()
    }

    fn restore(&mut self, snapshot: &Self::Snapshot) {
        self.snapshot.clone_from(snapshot);
    }
}

fn chase_pose(
    vehicle: &VehicleState,
    back_m: f64,
    right_m: f64,
    up_m: f64,
    pitch_rad: f64,
) -> CameraPoseEcef {
    let geo = ecef_to_geodetic(vehicle.position_ecef);
    let frame = LocalFrame::at(&geo);
    let zenith = frame.up();
    let body_fwd = vehicle.forward_ecef();
    let horiz = body_fwd - zenith * body_fwd.dot(zenith);
    let fwd_h = horiz.try_normalize().unwrap_or_else(|| frame.north());
    let right = fwd_h
        .cross(zenith)
        .try_normalize()
        .unwrap_or_else(|| frame.east());
    let position_ecef = vehicle.position_ecef - fwd_h * back_m + right * right_m + zenith * up_m;
    let (sin_p, cos_p) = pitch_rad.sin_cos();
    let forward = (fwd_h * cos_p + zenith * sin_p).normalize();
    let cam_up = right.cross(forward).normalize();
    let orientation = DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -forward)).normalize();
    CameraPoseEcef {
        position_ecef,
        orientation,
    }
}

fn world_fixed_pose(
    position: &Geodetic,
    azimuth_rad: f64,
    elevation_rad: f64,
    roll_rad: f64,
) -> CameraPoseEcef {
    let position_ecef = geodetic_to_ecef(position);
    let frame = LocalFrame::at(position);
    let (sin_az, cos_az) = azimuth_rad.sin_cos();
    let (sin_el, cos_el) = elevation_rad.sin_cos();
    let horiz = frame.north() * cos_az + frame.east() * sin_az;
    let zenith = frame.up();
    let forward = (horiz * cos_el + zenith * sin_el).normalize();
    let right0 = (frame.east() * cos_az - frame.north() * sin_az).normalize();
    let up0 = (-horiz * sin_el + zenith * cos_el).normalize();
    let (sin_r, cos_r) = roll_rad.sin_cos();
    let right = (right0 * cos_r + up0 * sin_r).normalize();
    let cam_up = (-right0 * sin_r + up0 * cos_r).normalize();
    let orientation = DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -forward)).normalize();
    CameraPoseEcef {
        position_ecef,
        orientation,
    }
}

fn world_auto_track_pose(
    position: &Geodetic,
    vehicle: &VehicleState,
    lead_time_s: f64,
    base: &SensorIntrinsics,
    auto_zoom_span_m: Option<f64>,
    min_fov_y_rad: f64,
    max_fov_y_rad: f64,
) -> (CameraPoseEcef, SensorIntrinsics) {
    let position_ecef = geodetic_to_ecef(position);
    let frame = LocalFrame::at(position);
    let zenith = frame.up();
    let aim_ecef = vehicle.position_ecef + vehicle.velocity_ecef * lead_time_s;
    let delta = aim_ecef - position_ecef;
    let range_m = delta.length().max(0.1);
    let forward = (delta / range_m)
        .try_normalize()
        .unwrap_or_else(|| frame.north());
    let right = forward
        .cross(zenith)
        .try_normalize()
        .unwrap_or_else(|| frame.east());
    let cam_up = right.cross(forward).normalize();
    let orientation = DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -forward)).normalize();

    let mut intrinsics = *base;
    if let Some(span_m) = auto_zoom_span_m
        && span_m > 0.0
    {
        let lo = min_fov_y_rad.clamp(0.01, 2.4);
        let hi = max_fov_y_rad.clamp(lo, 2.5);
        let desired_fov = (2.0 * (0.5 * span_m / range_m).atan()).clamp(lo, hi);
        let fy = 0.5 * f64::from(base.height) / (0.5 * desired_fov).tan();
        let aspect_scale = base.fx / base.fy.max(1e-6);
        intrinsics.fy = fy;
        intrinsics.fx = fy * aspect_scale;
    }
    (
        CameraPoseEcef {
            position_ecef,
            orientation,
        },
        intrinsics,
    )
}
