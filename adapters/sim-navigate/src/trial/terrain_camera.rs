//! The terrain camera: a sensor that renders the simulated world at the
//! vehicle's true pose through a declared mount, pinhole intrinsics and
//! radiometric response, and hands the navigator only the frame, its
//! intrinsics, its mount and its mid-exposure time.
//!
//! Each frame is rendered from the world package at the true pose of the
//! middle of the exposure; motion during the exposure is not modelled.
//! Below the stated Sun elevation the scene contributes nothing and the
//! frame is sensor offset and noise alone.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_core::{CameraMount, SimInstant, VehicleState};
use sim_devices::WhiteNoise;
use sim_geo::{LocalFrame, SensorIntrinsics, UtcTime, ecef_to_geodetic};
use sim_scenery::{CatalogScenery, render_luma_view};

use super::TrialError;
use super::route::STEP_HZ;
use super::sky::sun_direction_ecef;
use crate::Truth;

#[cfg(test)]
mod tests;

/// Mixes a frame index into the seed so each frame draws its own noise.
const FRAME_SEED_STRIDE: u64 = 0x9E37_79B9_7F4A_7C15;

/// The spectral band a camera images in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CameraBand {
    /// Reflected sunlight.
    Visible,
    /// Long-wave infrared. Reserved: refused until thermal scenes render.
    Lwir,
}

/// Where the camera sits and where it looks.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraMountSpec {
    /// Optical centre in body axes (x forward, y right, z down), metres.
    pub offset_body_m: [f64; 3],
    /// Roll of the view about its own axis, degrees.
    pub roll_deg: f64,
    /// Pitch of the view from the nose, degrees; −90 looks straight down.
    pub pitch_deg: f64,
    /// Yaw of the view from the nose, degrees; +90 looks out of the right
    /// side.
    pub yaw_deg: f64,
}

/// Scene luma to recorded counts: `gain · 255 · (luma/255)^gamma + offset +
/// noise`, rounded and clamped to 8 bits.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RadiometrySpec {
    /// Scene gain.
    pub gain: f64,
    /// Dark offset, counts.
    pub offset_dn: f64,
    /// Response exponent.
    pub gamma: f64,
    /// White read noise, counts.
    pub noise_dn: f64,
}

/// The terrain camera of a request.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerrainCameraSpec {
    /// The band.
    pub band: CameraBand,
    /// Image width, pixels.
    pub width: u32,
    /// Image height, pixels.
    pub height: u32,
    /// Horizontal focal length, pixels.
    pub fx_px: f64,
    /// Vertical focal length, pixels.
    pub fy_px: f64,
    /// Principal point, horizontal, pixels.
    pub cx_px: f64,
    /// Principal point, vertical, pixels.
    pub cy_px: f64,
    /// The mount.
    pub mount: CameraMountSpec,
    /// Time between frames, seconds; a whole number of simulation steps.
    pub period_s: f64,
    /// Exposure, seconds; frames are stamped at its middle.
    pub exposure_s: f64,
    /// The sensor's response.
    pub radiometry: RadiometrySpec,
    /// Sun elevation below which the scene gives no light, degrees.
    pub min_sun_elevation_deg: f64,
}

impl TerrainCameraSpec {
    /// The pinhole intrinsics.
    pub fn intrinsics(&self) -> SensorIntrinsics {
        SensorIntrinsics {
            width: self.width,
            height: self.height,
            fx: self.fx_px,
            fy: self.fy_px,
            cx: self.cx_px,
            cy: self.cy_px,
        }
    }

    /// The mount as the simulator's camera mount.
    pub fn camera_mount(&self) -> CameraMount {
        let m = self.mount;
        CameraMount::oblique(
            DVec3::from_array(m.offset_body_m),
            m.yaw_deg.to_radians(),
            m.pitch_deg.to_radians(),
            m.roll_deg.to_radians(),
        )
    }

    /// Steps between frames, and from a frame's middle to its readout.
    pub fn schedule_steps(&self) -> Result<(usize, usize), TrialError> {
        let invalid = |detail: String| TrialError::InvalidRequest {
            field: "terrain_camera",
            detail,
        };
        let period = self.period_s * STEP_HZ as f64;
        if !(period >= 1.0 && (period - period.round()).abs() < 1e-9) {
            return Err(invalid(format!(
                "period {} s is not a whole number of steps",
                self.period_s
            )));
        }
        if !(self.exposure_s > 0.0 && self.exposure_s < self.period_s) {
            return Err(invalid(format!(
                "exposure {} s is not inside the period",
                self.exposure_s
            )));
        }
        let half = (0.5 * self.exposure_s * STEP_HZ as f64).ceil().max(1.0);
        Ok((period.round() as usize, half as usize))
    }

    /// Refuses a camera the trial cannot simulate as stated.
    pub fn validate(&self) -> Result<(), TrialError> {
        let invalid = |detail: String| TrialError::InvalidRequest {
            field: "terrain_camera",
            detail,
        };
        if self.band == CameraBand::Lwir {
            return Err(invalid("the lwir band is not rendered yet".into()));
        }
        if !self.intrinsics().is_usable() {
            return Err(invalid(format!("intrinsics {:?}", self.intrinsics())));
        }
        let r = self.radiometry;
        let response = [r.gain, r.offset_dn, r.gamma, r.noise_dn];
        if !(response.iter().all(|v| v.is_finite()) && r.gamma > 0.0 && r.noise_dn >= 0.0) {
            return Err(invalid(format!("radiometry {r:?}")));
        }
        let mount = self.mount;
        let angles = [mount.roll_deg, mount.pitch_deg, mount.yaw_deg];
        if !(angles
            .iter()
            .chain(&mount.offset_body_m)
            .all(|v| v.is_finite())
            && self.min_sun_elevation_deg.is_finite())
        {
            return Err(invalid(format!("mount {mount:?}")));
        }
        self.schedule_steps().map(|_| ())
    }
}

/// What the scene knew while rendering a frame. Scoring only.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainFrameTruth {
    /// The camera's true optical centre, ECEF metres.
    pub eye_ecef: DVec3,
    /// The camera's true axes (right, up, back) into ECEF.
    pub camera_to_ecef: DQuat,
    /// The Sun's elevation at the eye, degrees.
    pub sun_elevation_deg: f64,
}

/// One terrain camera frame.
#[derive(Debug, Clone, PartialEq)]
pub struct TerrainFrame {
    /// The step at the middle of the exposure, which the frame is stamped
    /// with.
    pub mid_step: usize,
    /// The step at which the frame is read out and delivered.
    pub end_step: usize,
    /// The instant of `mid_step`.
    pub mid: SimInstant,
    /// Width and height, pixels.
    pub size: (u32, u32),
    /// Grey counts, row-major, top row first, `width × height`.
    pub pixels: Vec<u8>,
    /// The scene's part.
    pub truth: Truth<TerrainFrameTruth>,
}

/// The terrain camera over a simulated world.
#[derive(Debug, Clone)]
pub struct TerrainCamera {
    world: Arc<CatalogScenery>,
    spec: TerrainCameraSpec,
    mount: CameraMount,
    seed: u64,
    start: UtcTime,
}

impl TerrainCamera {
    /// A camera of `spec` seeing `world`, its noise drawn from `seed`, on a
    /// run whose first instant is `start`.
    pub fn new(
        world: Arc<CatalogScenery>,
        spec: TerrainCameraSpec,
        seed: u64,
        start: UtcTime,
    ) -> Result<Self, TrialError> {
        spec.validate()?;
        Ok(Self {
            mount: spec.camera_mount(),
            world,
            spec,
            seed,
            start,
        })
    }

    /// The mid-exposure and readout steps of every frame along a flight of
    /// `len` steps: a frame every period from the first, each read out
    /// half an exposure after its middle and inside the flight.
    pub fn schedule(&self, len: usize) -> Result<Vec<(usize, usize)>, TrialError> {
        let (period, half) = self.spec.schedule_steps()?;
        Ok((period..len)
            .step_by(period)
            .map(|mid| (mid, mid.wrapping_add(half)))
            .filter(|(_, end)| *end < len)
            .collect())
    }

    /// Every frame along `truth`, each rendered at the true pose of its
    /// middle.
    pub fn render_along(
        &self,
        truth: &[Truth<(SimInstant, VehicleState)>],
    ) -> Result<Vec<TerrainFrame>, TrialError> {
        let mut frames = Vec::new();
        for (index, steps) in (1_u64..).zip(self.schedule(truth.len())?) {
            let Some((at, state)) = truth.get(steps.0).map(|sample| *sample.get()) else {
                continue;
            };
            frames.push(self.capture(index, steps, at, &state)?);
        }
        Ok(frames)
    }

    /// Frame `index` (from 1), with the vehicle truly in `state` at the
    /// middle of the exposure.
    pub fn capture(
        &self,
        index: u64,
        (mid_step, end_step): (usize, usize),
        mid: SimInstant,
        state: &VehicleState,
    ) -> Result<TerrainFrame, TrialError> {
        let pose = self.mount.pose(state, DVec3::ZERO);
        let (eye_ecef, camera_to_ecef) = (pose.position_ecef, pose.orientation);
        let sun = sun_direction_ecef(self.start, mid, eye_ecef)?;
        let up = LocalFrame::at(&ecef_to_geodetic(eye_ecef)).up();
        let sun_elevation_deg = sun.dot(up).clamp(-1.0, 1.0).asin().to_degrees();
        let lit = sun_elevation_deg >= self.spec.min_sun_elevation_deg;
        let rendered = render_luma_view(
            &self.world,
            eye_ecef,
            camera_to_ecef,
            &self.spec.intrinsics(),
        )
        .map_err(|error| TrialError::Maps {
            what: format!("world render: {error}"),
        })?;
        let mut noise = WhiteNoise::from_seed(self.seed ^ index.wrapping_mul(FRAME_SEED_STRIDE));
        let r = self.spec.radiometry;
        let pixels = rendered
            .pixels()
            .iter()
            .map(|luma| {
                let scene = if lit {
                    r.gain * 255.0 * (f64::from(*luma) / 255.0).powf(r.gamma)
                } else {
                    0.0
                };
                let counts = scene + r.offset_dn + noise.next_gaussian() * r.noise_dn;
                counts.round().clamp(0.0, 255.0) as u8
            })
            .collect();
        Ok(TerrainFrame {
            mid_step,
            end_step,
            mid,
            size: (self.spec.width, self.spec.height),
            pixels,
            truth: Truth::new(TerrainFrameTruth {
                eye_ecef,
                camera_to_ecef,
                sun_elevation_deg,
            }),
        })
    }
}
