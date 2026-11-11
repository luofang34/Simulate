//! The star camera on a flight sensed as it is flown: a frame opens every
//! period, and is rendered and delivered once its exposure has closed.

use sim_core::{SimInstant, VehicleState};
use sim_render::StarCameraConfig;

use super::record::{SensorLog, star_scene};
use super::stars::{StarScene, exposure_steps, render_frame, star_camera_config};
use super::{TrialError, TrialRequest};
use crate::Truth;

/// The request's star camera, sensing step by step into a log.
#[derive(Debug, Clone)]
pub struct LiveStarCamera {
    scene: StarScene,
    config: StarCameraConfig,
    length: usize,
}

impl LiveStarCamera {
    /// The star camera `request` describes.
    pub fn new(request: &TrialRequest) -> Result<Self, TrialError> {
        let scene = star_scene(request)?;
        let config = star_camera_config();
        Ok(Self {
            length: exposure_steps(&config, scene.step_hz)?,
            scene,
            config,
        })
    }

    /// Senses `step`, the last of `truth`, into `log`: the middle of a frame
    /// that opens at it, and the frame that closes at it. A frame is taken
    /// only when `in_service` says the camera worked at its opening step.
    pub fn sense(
        &self,
        truth: &[Truth<(SimInstant, VehicleState)>],
        step: usize,
        log: &mut SensorLog,
        in_service: impl Fn(usize) -> bool,
    ) -> Result<(), TrialError> {
        let period = self.scene.period_steps.max(1);
        let opens =
            |open: usize| open >= 1 && (open - 1).is_multiple_of(period) && in_service(open);
        if opens(step) {
            log.star_mid_steps.push(step + self.length / 2);
        }
        if let Some(open) = step.checked_sub(self.length)
            && opens(open)
        {
            let frame = render_frame(truth, &self.scene, &self.config, open, self.length)?;
            log.stars.push(frame);
        }
        Ok(())
    }
}
