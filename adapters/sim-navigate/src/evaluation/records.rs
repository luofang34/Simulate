use navigate_contract::{
    GeodeticPosition, NavigationSolution, NedVelocity, SourceId, SymmetricCov3,
};
use sim_core::VehicleState;
use sim_core::{CameraFrameCapture, CameraPoseEcef, NavigateCameraPose, SimInstant, TickSpan};

use super::NavigationOrigin;
use crate::NavigateBridgeError;

#[cfg(test)]
mod tests;

/// A value the simulator knows exactly. Only a scorer may read it beside an
/// [`Estimate`]; nothing onboard should ever hold one.
#[derive(Debug, Clone, PartialEq)]
pub struct Truth<T>(T);

impl<T> Truth<T> {
    /// Marks `value` as simulation truth.
    pub const fn new(value: T) -> Self {
        Self(value)
    }

    /// The true value.
    pub const fn get(&self) -> &T {
        &self.0
    }
}

/// A value a navigation means computed from its sensors.
#[derive(Debug, Clone, PartialEq)]
pub struct Estimate<T>(T);

impl<T> Estimate<T> {
    /// Marks `value` as computed by the system under test.
    pub const fn new(value: T) -> Self {
        Self(value)
    }

    /// The computed value.
    pub const fn get(&self) -> &T {
        &self.0
    }
}

/// Where a camera truly was at an exposure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraTruth {
    /// Pose in the renderer's convention, ECEF.
    pub pose_ecef: CameraPoseEcef,
    /// The same pose in `Navigate`'s convention, in the catalog anchor frame.
    pub pose_enu: NavigateCameraPose,
    /// Range to the terrain along the optical axis, metres; `None` when the
    /// axis meets sky.
    pub optical_axis_depth_m: Option<f64>,
}

/// The simulator's exact state at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimTruth {
    /// The instant.
    pub instant: SimInstant,
    /// Exact vehicle state; its position is the centre of gravity.
    pub vehicle: VehicleState,
    /// The camera at this instant, when one exposed.
    pub camera: Option<CameraTruth>,
}

impl SimTruth {
    /// The vehicle at `instant`, with no exposure.
    pub const fn vehicle(instant: SimInstant, vehicle: VehicleState) -> Self {
        Self {
            instant,
            vehicle,
            camera: None,
        }
    }

    /// The truth of one exposure, with the optical-axis range from its
    /// rendered depth, such as [`ImageTruth::optical_axis_depth_m`].
    pub fn from_capture(capture: &CameraFrameCapture, optical_axis_depth_m: Option<f64>) -> Self {
        Self {
            instant: capture.timestamp,
            vehicle: capture.vehicle,
            camera: Some(CameraTruth {
                pose_ecef: capture.pose,
                pose_enu: NavigateCameraPose {
                    eye_enu: capture.eye_enu,
                    world_to_cam: capture.world_to_cam,
                },
                optical_axis_depth_m,
            }),
        }
    }
}

/// Per-pixel ground truth of a rendered image, top row first.
#[derive(Debug, Clone, PartialEq)]
pub struct ImageTruth {
    /// Width, pixels.
    pub width: u32,
    /// Height, pixels.
    pub height: u32,
    /// `255` where the pixel sees terrain, `0` where it is occluded or sky.
    pub valid_mask: Vec<u8>,
    /// Range along the optical axis, metres; infinite for sky.
    pub depth_m: Vec<f32>,
}

impl ImageTruth {
    /// Truth from a reversed-Z, infinite-far depth buffer whose near plane
    /// is `near_m`, such as `sim_render::ApertureImage::depth`: depth `d`
    /// lies `near_m / d` along the axis, and `0` is sky. Fails when the
    /// buffer is not `width * height` long.
    pub fn from_reversed_z(
        width: u32,
        height: u32,
        depth: &[f32],
        near_m: f64,
    ) -> Result<Self, NavigateBridgeError> {
        let needed = (width as usize).saturating_mul(height as usize);
        if depth.len() != needed {
            return Err(NavigateBridgeError::FrameSize {
                what: "depth buffer",
                width,
                height,
                needed,
                actual: depth.len(),
            });
        }
        let depth_m: Vec<f32> = depth
            .iter()
            .map(|d| {
                if *d > 0.0 {
                    (near_m / f64::from(*d)) as f32
                } else {
                    f32::INFINITY
                }
            })
            .collect();
        let valid_mask = depth_m
            .iter()
            .map(|d| if d.is_finite() { 255 } else { 0 })
            .collect();
        Ok(Self {
            width,
            height,
            valid_mask,
            depth_m,
        })
    }

    /// The range at the pixel nearest the principal point `(cx, cy)`, or
    /// `None` for sky or a point outside the image.
    pub fn optical_axis_depth_m(&self, cx: f64, cy: f64) -> Option<f64> {
        let (x, y) = (cx.round(), cy.round());
        let inside =
            x >= 0.0 && y >= 0.0 && x < f64::from(self.width) && y < f64::from(self.height);
        if !inside {
            return None;
        }
        let index = (y as usize)
            .checked_mul(self.width as usize)?
            .checked_add(x as usize)?;
        let depth = f64::from(*self.depth_m.get(index)?);
        depth.is_finite().then_some(depth)
    }
}

/// The true image of one catalog star.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StarTruth {
    /// Catalog identifier.
    pub catalog_id: u32,
    /// True centroid, pixels, in `navigate_camera::Intrinsics`' convention.
    pub centroid_px: [f64; 2],
    /// Visual magnitude.
    pub v_magnitude: f64,
}

/// Ground truth of a star camera frame.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct StarFieldTruth {
    /// Every catalog star inside the frame.
    pub stars: Vec<StarTruth>,
}

/// Ground truth of a thermal frame, top row first.
#[derive(Debug, Clone, PartialEq)]
pub struct ThermalTruth {
    /// Width, pixels.
    pub width: u32,
    /// Height, pixels.
    pub height: u32,
    /// Scene temperature each pixel sees, kelvin.
    pub scene_temperature_k: Vec<f32>,
}

/// The ground truth of one sensor channel's observation. New sensor kinds
/// add a variant.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SensorTruth {
    /// A visible or near-infrared image.
    Image(ImageTruth),
    /// A star camera frame.
    StarField(StarFieldTruth),
    /// A long-wave infrared frame.
    Thermal(ThermalTruth),
}

/// One sensor observation's ground truth, keyed like the observation.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorTruthRecord {
    /// The instant the sensor observed.
    pub instant: SimInstant,
    /// The sensor, as Navigate knows it.
    pub source: SourceId,
    /// What it truly saw.
    pub truth: SensorTruth,
}

/// What one navigation means computed for one instant. Its compute cost is a
/// [`super::ComputeSample`] kept beside it, so the record stays a
/// deterministic function of the simulation.
#[derive(Debug, Clone, PartialEq)]
pub struct NavigationOutput {
    /// The means, alone or fused, that produced it.
    pub origin: NavigationOrigin,
    /// The publishing source.
    pub source: SourceId,
    /// The instant the output describes.
    pub valid_at: SimInstant,
    /// When the output became available on the master clock.
    pub delivered_at: SimInstant,
    /// Position of the body reference point (the centre of gravity).
    pub position: GeodeticPosition,
    /// Claimed position error covariance, north-east-down, m².
    pub position_cov: SymmetricCov3,
    /// Velocity, when the means estimates it.
    pub velocity: Option<NedVelocity>,
    /// Labels the means attached, such as a fusion eligibility.
    pub labels: Vec<String>,
}

impl NavigationOutput {
    /// A filter solution valid at `valid_at` and published at `delivered_at`.
    pub fn from_solution(
        origin: NavigationOrigin,
        source: SourceId,
        solution: &NavigationSolution,
        valid_at: SimInstant,
        delivered_at: SimInstant,
    ) -> Self {
        Self {
            origin,
            source,
            valid_at,
            delivered_at,
            position: solution.position,
            position_cov: solution.position_cov,
            velocity: Some(solution.velocity),
            labels: Vec::new(),
        }
    }

    /// Master-clock time from the instant described to delivery.
    pub const fn latency(&self) -> TickSpan {
        self.delivered_at.duration_since(self.valid_at)
    }
}
