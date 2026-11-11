//! The simulated vehicle camera: Simulate's renderer over the world imagery
//! at the vehicle's true pose, through a declared sensor response. It is the
//! only place a true pose enters the visual chain.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_core::{SimInstant, VehicleState};
use sim_devices::WhiteNoise;
use sim_geo::{UtcTime, ecef_to_geodetic};
use sim_scenery::{CatalogScenery, render_luma_view};

use super::{CameraSpec, RadiometrySpec};
use crate::Truth;
use crate::trial::TrialError;
use crate::trial::sky::sun_elevation_deg;

/// Mixes a frame index into a seed so each frame draws its own noise.
const FRAME_SEED_STRIDE: u64 = 0x9E37_79B9_7F4A_7C15;

/// What the scene knew while rendering a frame. Scoring only.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraFrameTruth {
    /// The camera's true optical centre, ECEF metres.
    pub eye_ecef: DVec3,
    /// The camera's true axes (right, up, back) into ECEF.
    pub camera_to_ecef: DQuat,
    /// The Sun's elevation, degrees.
    pub sun_elevation_deg: f64,
    /// Height of the eye above the world's ground below it, metres.
    pub ground_distance_m: Option<f64>,
}

/// One frame of the vehicle camera.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraFrame {
    /// The simulation step it was captured and delivered at.
    pub step: usize,
    /// Its index in the capture stream, from 1.
    pub index: u64,
    /// Grey counts, row-major, top row first.
    pub pixels: Vec<u8>,
    /// The scene's part.
    pub truth: Truth<CameraFrameTruth>,
    /// Wall time of rendering the frame, seconds; not deterministic, and
    /// never in a result.
    pub render_s: f64,
}

/// The vehicle camera over the simulated world.
#[derive(Debug, Clone)]
pub struct LiveCamera {
    world: Arc<CatalogScenery>,
    spec: CameraSpec,
    radiometry: RadiometrySpec,
    seed: u64,
    start: UtcTime,
}

impl LiveCamera {
    /// A camera seeing `world`, its noise drawn from `seed`, on a run that
    /// started at `start`.
    pub fn new(
        world: Arc<CatalogScenery>,
        spec: CameraSpec,
        radiometry: RadiometrySpec,
        seed: u64,
        start: UtcTime,
    ) -> Self {
        Self {
            world,
            spec,
            radiometry,
            seed,
            start,
        }
    }

    /// Frame `index` with the vehicle truly in `state` at `at`.
    pub fn capture(
        &self,
        index: u64,
        step: usize,
        at: SimInstant,
        state: &VehicleState,
    ) -> Result<CameraFrame, TrialError> {
        let offset = DVec3::from_array(self.spec.mount_offset_body_m);
        let eye_ecef = state.position_ecef + state.attitude * offset;
        let camera_to_ecef = (state.attitude * CameraSpec::camera_to_body()).normalize();
        let started = web_time::Instant::now();
        let sun_elevation_deg = sun_elevation_deg(self.start, at, eye_ecef)?;
        let lit = sun_elevation_deg >= self.spec.min_sun_elevation_deg;
        let rendered = render_luma_view(
            &self.world,
            eye_ecef,
            camera_to_ecef,
            &self.spec.intrinsics(),
        )
        .map_err(|error| TrialError::Visual {
            what: format!("world render: {error}"),
        })?;
        let eye = ecef_to_geodetic(eye_ecef);
        let ground_distance_m = self
            .world
            .ellipsoidal_ground_height_m(&eye)
            .ok()
            .flatten()
            .map(|ground| eye.height_m - ground.0);
        let mut noise = WhiteNoise::from_seed(self.seed ^ index.wrapping_mul(FRAME_SEED_STRIDE));
        let r = self.radiometry;
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
        Ok(CameraFrame {
            step,
            index,
            render_s: started.elapsed().as_secs_f64(),
            pixels,
            truth: Truth::new(CameraFrameTruth {
                eye_ecef,
                camera_to_ecef,
                sun_elevation_deg,
                ground_distance_m,
            }),
        })
    }
}
