//! Configurable airframe-mounted, chase, fixed-ground, and auto-tracking PTZ
//! virtual cameras stepped deterministically on the master clock.

use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic, SensorIntrinsics};

use crate::{
    Block, BlockError, CAMERA_DEFAULT_PERIOD, CameraFrameCapture, CameraMount, CameraPoseEcef,
    EnuAnchor, SimInstant, TickSpan, VehicleState,
};

mod placement;
#[cfg(test)]
mod tests;

pub use placement::FlybyTracker;
use placement::{
    AutoZoom, chase_pose, flyby_pose, range_from, world_auto_track_pose, world_fixed_pose,
};

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
    /// Stationary ground or runway-threshold camera fixed on the body's
    /// ellipsoid (WGS84 unless the rig says otherwise).
    WorldFixed {
        /// Geodetic eye position on the body's ellipsoid.
        position: Geodetic,
        /// True azimuth clockwise from north (`rad`).
        azimuth_rad: f64,
        /// Elevation angle above the local horizontal plane (`rad`).
        elevation_rad: f64,
        /// Roll angle clockwise around the optical axis (`rad`).
        roll_rad: f64,
    },
    /// Control-tower or range-tracking PTZ camera anchored on the body's
    /// ellipsoid and aiming at the vehicle.
    WorldAutoTrack {
        /// Geodetic tower cab or tripod eye position on the body's ellipsoid.
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
    /// A ground or spatial observer near ownship that auto-tracks it as it
    /// flies past, anywhere on any body.
    FlybyAutoTrack(FlybyTracker),
    /// [`Self::WorldAutoTrack`] from a fixed site while the vehicle is within
    /// `max_range_m`, and [`Self::FlybyAutoTrack`] beyond, so a tower camera
    /// never stares across hundreds of kilometres of haze or through the
    /// body when the vehicle is far from its airport.
    RangeLimitedAutoTrack {
        /// Geodetic eye position on the body's ellipsoid.
        position: Geodetic,
        /// Velocity lead time (`s`) added to the vehicle position when aiming
        /// from the site.
        lead_time_s: f64,
        /// Optional target scene span (`m`) at the vehicle range for
        /// automatic telephoto zoom from the site.
        auto_zoom_span_m: Option<f64>,
        /// Narrowest vertical field of view (`rad`) from the site.
        min_fov_y_rad: f64,
        /// Widest vertical field of view (`rad`) from the site.
        max_fov_y_rad: f64,
        /// Slant range (`m`) beyond which `fallback` takes over.
        max_range_m: f64,
        /// The observer used beyond `max_range_m`.
        fallback: FlybyTracker,
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

    /// Evaluates the instantaneous ECEF camera pose and pinhole intrinsics for
    /// `vehicle` on WGS84.
    pub fn pose_and_intrinsics(
        &self,
        vehicle: &VehicleState,
        cg_body_m: DVec3,
    ) -> Result<(CameraPoseEcef, SensorIntrinsics), BlockError> {
        self.pose_and_intrinsics_on(&BodyEllipsoid::WGS84, vehicle, cg_body_m)
    }

    /// [`Self::pose_and_intrinsics`] for a vehicle flying over `body`, whose
    /// ellipsoid places world-anchored cameras and sets their local vertical.
    pub fn pose_and_intrinsics_on(
        &self,
        body: &BodyEllipsoid,
        vehicle: &VehicleState,
        cg_body_m: DVec3,
    ) -> Result<(CameraPoseEcef, SensorIntrinsics), BlockError> {
        self.validate_inputs(vehicle, cg_body_m)?;
        let base = &self.base_intrinsics;
        Ok(match self.placement {
            VirtualCameraPlacement::BodyMounted(mount) => (mount.pose(vehicle, cg_body_m), *base),
            VirtualCameraPlacement::ChaseOffset {
                back_m,
                right_m,
                up_m,
                pitch_rad,
            } => (
                chase_pose(body, vehicle, DVec3::new(back_m, right_m, up_m), pitch_rad),
                *base,
            ),
            VirtualCameraPlacement::WorldFixed {
                position,
                azimuth_rad,
                elevation_rad,
                roll_rad,
            } => (
                world_fixed_pose(body, &position, azimuth_rad, elevation_rad, roll_rad),
                *base,
            ),
            VirtualCameraPlacement::WorldAutoTrack {
                position,
                lead_time_s,
                auto_zoom_span_m,
                min_fov_y_rad,
                max_fov_y_rad,
            } => {
                let zoom = AutoZoom {
                    span_m: auto_zoom_span_m,
                    min_fov_y_rad,
                    max_fov_y_rad,
                };
                world_auto_track_pose(body, &position, vehicle, lead_time_s, base, zoom)
            }
            VirtualCameraPlacement::FlybyAutoTrack(tracker) => {
                tracker.validate()?;
                flyby_pose(body, &tracker, vehicle, base)
            }
            VirtualCameraPlacement::RangeLimitedAutoTrack {
                position,
                lead_time_s,
                auto_zoom_span_m,
                min_fov_y_rad,
                max_fov_y_rad,
                max_range_m,
                fallback,
            } => {
                // NaN compares false, so an unusable limit falls back too.
                if range_from(body, &position, vehicle) <= max_range_m {
                    let zoom = AutoZoom {
                        span_m: auto_zoom_span_m,
                        min_fov_y_rad,
                        max_fov_y_rad,
                    };
                    world_auto_track_pose(body, &position, vehicle, lead_time_s, base, zoom)
                } else {
                    fallback.validate()?;
                    flyby_pose(body, &fallback, vehicle, base)
                }
            }
        })
    }

    fn validate_inputs(&self, vehicle: &VehicleState, cg_body_m: DVec3) -> Result<(), BlockError> {
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
        Ok(())
    }

    /// Captures a stamped [`CameraFrameCapture`] relative to `anchor` on WGS84.
    pub fn capture(
        &self,
        timestamp: SimInstant,
        vehicle: &VehicleState,
        cg_body_m: DVec3,
        anchor: &EnuAnchor,
    ) -> Result<CameraFrameCapture, BlockError> {
        self.capture_on(&BodyEllipsoid::WGS84, timestamp, vehicle, cg_body_m, anchor)
    }

    /// [`Self::capture`] for a vehicle flying over `body`.
    pub fn capture_on(
        &self,
        body: &BodyEllipsoid,
        timestamp: SimInstant,
        vehicle: &VehicleState,
        cg_body_m: DVec3,
        anchor: &EnuAnchor,
    ) -> Result<CameraFrameCapture, BlockError> {
        let (pose, intrinsics) = self.pose_and_intrinsics_on(body, vehicle, cg_body_m)?;
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

/// Multi-camera sensor rig capturing synchronized exposures across body and
/// ground cameras, on WGS84 unless [`Self::with_ellipsoid`] names another body.
#[derive(Debug, Clone, PartialEq)]
pub struct VirtualCameraRigBlock {
    specs: Vec<VirtualCameraSpec>,
    anchor: EnuAnchor,
    body: BodyEllipsoid,
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
            body: BodyEllipsoid::WGS84,
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

    /// This rig over `body` instead of WGS84.
    #[must_use]
    pub fn with_ellipsoid(mut self, body: BodyEllipsoid) -> Self {
        self.body = body;
        self
    }

    /// Places world-anchored cameras on `body` from the next exposure on.
    pub fn set_ellipsoid(&mut self, body: BodyEllipsoid) {
        self.body = body;
    }

    /// The body world-anchored cameras are placed on.
    #[must_use]
    pub fn ellipsoid(&self) -> BodyEllipsoid {
        self.body
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
            captures.push(spec.capture_on(
                &self.body,
                timestamp,
                &vehicle,
                cg_body_m,
                &self.anchor,
            )?);
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
