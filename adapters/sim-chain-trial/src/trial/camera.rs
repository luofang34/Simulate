//! The terrain camera, rendered by the host: exposures are placed at their
//! mid-exposure step from the true pose, handed out as requests, and their
//! frames developed here when the host returns the rendered image.

use std::collections::VecDeque;

use sim_core::VehicleState;
use sim_navigate::trial::{TerrainExposer, TerrainExposure, TerrainFrame};

use crate::{ChainTrialError, FrameRequest};

/// The camera and the exposures waiting for their images.
pub(super) struct HostCamera {
    exposer: TerrainExposer,
    /// Steps between frames, and from a frame's middle to its readout.
    period_steps: usize,
    half_steps: usize,
    next_index: u64,
    waiting: VecDeque<TerrainExposure>,
}

impl HostCamera {
    pub(super) fn new(exposer: TerrainExposer) -> Result<Self, ChainTrialError> {
        let (period_steps, half_steps) = exposer.spec().schedule_steps()?;
        Ok(Self {
            exposer,
            period_steps: period_steps.max(1),
            half_steps,
            next_index: 1,
            waiting: VecDeque::new(),
        })
    }

    /// The exposure whose middle is `step`, when one is: `state` is the
    /// vehicle's true state then and `out` whether the camera is out of
    /// service. An exposure is placed only when the camera works, and the
    /// index advances with every placed one.
    pub(super) fn expose(
        &mut self,
        step: usize,
        mid: sim_core::SimInstant,
        state: &VehicleState,
        out: bool,
    ) -> Result<Option<FrameRequest>, ChainTrialError> {
        if out || step == 0 || !step.is_multiple_of(self.period_steps) {
            return Ok(None);
        }
        let index = self.next_index;
        self.next_index = self.next_index.wrapping_add(1);
        let end = step.wrapping_add(self.half_steps);
        let exposure = self.exposer.exposure(index, (step, end), mid, state)?;
        self.waiting.push_back(exposure);
        Ok(Some(request(&exposure, self.half_steps)))
    }

    /// The oldest exposure still waiting whose readout is at or before
    /// `step`.
    pub(super) fn blocking(&self, step: usize) -> Option<FrameRequest> {
        self.waiting
            .front()
            .filter(|e| e.end_step <= step)
            .map(|e| request(e, self.half_steps))
    }

    /// The frame of exposure `index` from the host's grey image.
    pub(super) fn develop(
        &mut self,
        index: u64,
        luma: &[u8],
    ) -> Result<TerrainFrame, ChainTrialError> {
        // Frames are read out in order, so only the oldest may come back.
        let Some(exposure) = self.waiting.front().filter(|e| e.index == index).copied() else {
            return Err(ChainTrialError::UnexpectedFrame { index });
        };
        let frame = self.exposer.develop(&exposure, luma)?;
        self.waiting.pop_front();
        Ok(frame)
    }
}

fn request(exposure: &TerrainExposure, half_steps: usize) -> FrameRequest {
    let step_s = sim_navigate::trial::step_span().as_seconds();
    let mid_s = exposure.mid_step as f64 * step_s;
    let q = exposure.camera_to_ecef;
    let k = exposure.intrinsics;
    FrameRequest {
        index: exposure.index,
        mid_s,
        readout_s: mid_s + half_steps as f64 * step_s,
        eye_ecef: exposure.eye_ecef.to_array(),
        camera_to_ecef_wxyz: [q.w, q.x, q.y, q.z],
        size: (k.width, k.height),
        pinhole: [k.fx, k.fy, k.cx, k.cy],
        sun_elevation_deg: exposure.sun_elevation_deg,
    }
}
