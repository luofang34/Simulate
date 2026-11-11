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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CameraBand {
    /// Reflected sunlight.
    #[default]
    Visible,
    /// Long-wave infrared. Reserved: refused until thermal scenes render.
    Lwir,
}

/// Where the camera sits and where it looks. Every field defaults: a
/// camera at the centre of gravity looking straight down, image top toward
/// the nose.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
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

impl Default for CameraMountSpec {
    fn default() -> Self {
        Self {
            offset_body_m: [0.0; 3],
            roll_deg: 0.0,
            pitch_deg: -90.0,
            yaw_deg: 0.0,
        }
    }
}

/// Scene luma to recorded counts: `gain · 255 · (luma/255)^gamma + offset +
/// noise`, rounded and clamped to 8 bits. Every field defaults to an ideal
/// linear sensor with a little read noise: gain 1, no offset, gamma 1,
/// 2 counts of noise (under 1 % of the 8-bit range; an assumption, not a
/// measured sensor).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
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

impl Default for RadiometrySpec {
    fn default() -> Self {
        Self {
            gain: 1.0,
            offset_dn: 0.0,
            gamma: 1.0,
            noise_dn: 2.0,
        }
    }
}

/// A 2 ms exposure: a daylight exposure under which the image moves less
/// than a tenth of a metre at 60 m/s, far below any ground pixel the
/// navigator matches at. An engineering choice, not a datasheet value.
const fn default_exposure_s() -> f64 {
    0.002
}

/// The renderer gives no ground lighting below 5° of Sun elevation: a
/// boundary of the render model, which models no night-time or low-Sun
/// illumination, not of any camera.
const fn default_min_sun_elevation_deg() -> f64 {
    5.0
}

/// The terrain camera of a request. Only the geometry is required: the
/// image size, a focal length or a horizontal field of view, and the frame
/// period; everything else takes the documented defaults.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerrainCameraSpec {
    /// The band; visible by default.
    #[serde(default)]
    pub band: CameraBand,
    /// Image width, pixels.
    pub width: u32,
    /// Image height, pixels.
    pub height: u32,
    /// Horizontal focal length, pixels; or give `horizontal_fov_deg`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fx_px: Option<f64>,
    /// Horizontal field of view, degrees; or give `fx_px`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub horizontal_fov_deg: Option<f64>,
    /// Vertical focal length, pixels; square pixels by default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fy_px: Option<f64>,
    /// Principal point, horizontal, pixels, with pixel centres on integer
    /// coordinates; the image centre `(width − 1) / 2` by default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cx_px: Option<f64>,
    /// Principal point, vertical, pixels; `(height − 1) / 2` by default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cy_px: Option<f64>,
    /// The mount; straight down from the centre of gravity by default.
    #[serde(default)]
    pub mount: CameraMountSpec,
    /// Time between frames, seconds; a whole number of simulation steps.
    pub period_s: f64,
    /// Exposure, seconds; frames are stamped at its middle.
    #[serde(default = "default_exposure_s")]
    pub exposure_s: f64,
    /// The sensor's response.
    #[serde(default)]
    pub radiometry: RadiometrySpec,
    /// Sun elevation below which the scene gives no light, degrees.
    #[serde(default = "default_min_sun_elevation_deg")]
    pub min_sun_elevation_deg: f64,
}

impl TerrainCameraSpec {
    /// The pinhole intrinsics, defaults filled in. A field of view gives
    /// the focal length `(width / 2) / tan(fov / 2)`.
    pub fn intrinsics(&self) -> SensorIntrinsics {
        let from_fov = self
            .horizontal_fov_deg
            .map(|fov| 0.5 * f64::from(self.width) / (0.5 * fov.to_radians()).tan());
        let fx = self.fx_px.or(from_fov).unwrap_or(f64::NAN);
        SensorIntrinsics {
            width: self.width,
            height: self.height,
            fx,
            fy: self.fy_px.unwrap_or(fx),
            cx: self.cx_px.unwrap_or(0.5 * (f64::from(self.width) - 1.0)),
            cy: self.cy_px.unwrap_or(0.5 * (f64::from(self.height) - 1.0)),
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
        let fov_usable = self
            .horizontal_fov_deg
            .is_none_or(|fov| fov > 0.0 && fov < 180.0);
        if self.fx_px.is_some() == self.horizontal_fov_deg.is_some() || !fov_usable {
            return Err(invalid(
                "give exactly one of fx_px and horizontal_fov_deg, a field of view inside (0, 180)"
                    .into(),
            ));
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

/// One exposure the camera is about to make: where it looks from, how,
/// and when. A host that renders the world itself takes this, renders the
/// grey image, and hands it to [`TerrainExposer::develop`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainExposure {
    /// The frame's index in the run, from 1; it seeds the frame's noise.
    pub index: u64,
    /// The step at the middle of the exposure.
    pub mid_step: usize,
    /// The step at which the frame is read out and delivered.
    pub end_step: usize,
    /// The instant of `mid_step`.
    pub mid: SimInstant,
    /// The camera's true optical centre, ECEF metres.
    pub eye_ecef: DVec3,
    /// The camera's true axes (right, up, back) into ECEF.
    pub camera_to_ecef: DQuat,
    /// The Sun's elevation at the eye, degrees.
    pub sun_elevation_deg: f64,
    /// The pinhole intrinsics to render with.
    pub intrinsics: SensorIntrinsics,
}

/// The camera without the world: it places exposures and turns a rendered
/// grey image into the sensor's frame through its radiometric response and
/// noise. Whoever renders, the frame depends only on the image.
#[derive(Debug, Clone)]
pub struct TerrainExposer {
    spec: TerrainCameraSpec,
    mount: CameraMount,
    seed: u64,
    start: UtcTime,
}

impl TerrainExposer {
    /// A camera of `spec`, its noise drawn from `seed`, on a run whose
    /// first instant is `start`.
    pub fn new(spec: TerrainCameraSpec, seed: u64, start: UtcTime) -> Result<Self, TrialError> {
        spec.validate()?;
        Ok(Self {
            mount: spec.camera_mount(),
            spec,
            seed,
            start,
        })
    }

    /// The camera's description.
    pub const fn spec(&self) -> &TerrainCameraSpec {
        &self.spec
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

    /// Exposure `index` (from 1), with the vehicle truly in `state` at the
    /// middle of the exposure.
    pub fn exposure(
        &self,
        index: u64,
        (mid_step, end_step): (usize, usize),
        mid: SimInstant,
        state: &VehicleState,
    ) -> Result<TerrainExposure, TrialError> {
        let pose = self.mount.pose(state, DVec3::ZERO);
        let (eye_ecef, camera_to_ecef) = (pose.position_ecef, pose.orientation);
        let sun = sun_direction_ecef(self.start, mid, eye_ecef)?;
        let up = LocalFrame::at(&ecef_to_geodetic(eye_ecef)).up();
        Ok(TerrainExposure {
            index,
            mid_step,
            end_step,
            mid,
            eye_ecef,
            camera_to_ecef,
            sun_elevation_deg: sun.dot(up).clamp(-1.0, 1.0).asin().to_degrees(),
            intrinsics: self.spec.intrinsics(),
        })
    }

    /// The frame `exposure` yields from `luma`, the scene's grey image of
    /// exactly width × height bytes, row-major, top row first.
    pub fn develop(
        &self,
        exposure: &TerrainExposure,
        luma: &[u8],
    ) -> Result<TerrainFrame, TrialError> {
        let size = (self.spec.width, self.spec.height);
        let expected = (size.0 as usize).saturating_mul(size.1 as usize);
        if luma.len() != expected {
            return Err(TrialError::InvalidRequest {
                field: "terrain frame",
                detail: format!(
                    "{} bytes for a {}x{} image of {expected}",
                    luma.len(),
                    size.0,
                    size.1
                ),
            });
        }
        let lit = exposure.sun_elevation_deg >= self.spec.min_sun_elevation_deg;
        let mut noise =
            WhiteNoise::from_seed(self.seed ^ exposure.index.wrapping_mul(FRAME_SEED_STRIDE));
        let r = self.spec.radiometry;
        let pixels = luma
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
            mid_step: exposure.mid_step,
            end_step: exposure.end_step,
            mid: exposure.mid,
            size,
            pixels,
            truth: Truth::new(TerrainFrameTruth {
                eye_ecef: exposure.eye_ecef,
                camera_to_ecef: exposure.camera_to_ecef,
                sun_elevation_deg: exposure.sun_elevation_deg,
            }),
        })
    }
}

/// The terrain camera over a simulated world.
#[derive(Debug, Clone)]
pub struct TerrainCamera {
    world: Arc<CatalogScenery>,
    exposer: TerrainExposer,
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
        Ok(Self {
            world,
            exposer: TerrainExposer::new(spec, seed, start)?,
        })
    }

    /// The mid-exposure and readout steps of every frame along a flight of
    /// `len` steps; see [`TerrainExposer::schedule`].
    pub fn schedule(&self, len: usize) -> Result<Vec<(usize, usize)>, TrialError> {
        self.exposer.schedule(len)
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
    /// middle of the exposure, rendered from the world here.
    pub fn capture(
        &self,
        index: u64,
        steps: (usize, usize),
        mid: SimInstant,
        state: &VehicleState,
    ) -> Result<TerrainFrame, TrialError> {
        let exposure = self.exposer.exposure(index, steps, mid, state)?;
        let rendered = render_luma_view(
            &self.world,
            exposure.eye_ecef,
            exposure.camera_to_ecef,
            &exposure.intrinsics,
        )
        .map_err(|error| TrialError::Maps {
            what: format!("world render: {error}"),
        })?;
        self.exposer.develop(&exposure, rendered.pixels())
    }
}
