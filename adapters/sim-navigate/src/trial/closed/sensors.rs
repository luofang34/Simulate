//! The sensors along a closed-loop flight, sensing each step as it is
//! flown.
//!
//! Every stream is consumed once per sample whatever the trajectory: the IMU
//! draws its fixed nine numbers per triad at every step, the receiver and
//! altimeter measure at every one of their sample steps even when a set
//! withholds them, and a star frame's noise is seeded by the camera seed and
//! its opening step. So the k-th draw of each seeded stream belongs to the
//! k-th sample, and two sets meet the same noise at the same route time
//! though they fly different paths.

use glam::DVec3;
use sim_core::{SimInstant, VehicleState};
use sim_render::StarCameraConfig;

use crate::trial::gnss::{BaroAltimeter, GnssReceiver};
use crate::trial::record::{Logged, SensorLog, period_steps, star_scene};
use crate::trial::stars::{StarScene, exposure_steps, render_frame, star_camera_config};
use crate::trial::{TrialError, TrialRequest};
use crate::{SimulatedImu, Truth};

struct LiveStars {
    scene: StarScene,
    config: StarCameraConfig,
    length: usize,
}

/// Every sensor of a request, sensing a flight step by step into a log.
pub(super) struct LiveSensors {
    imu: SimulatedImu,
    gnss: GnssReceiver,
    gnss_period: usize,
    baro: BaroAltimeter,
    baro_period: usize,
    stars: Option<LiveStars>,
}

impl LiveSensors {
    /// The request's sensors, rendering star frames only when `stars`.
    pub(super) fn new(request: &TrialRequest, stars: bool) -> Result<Self, TrialError> {
        let live_stars = if stars {
            let scene = star_scene(request)?;
            let config = star_camera_config();
            Some(LiveStars {
                length: exposure_steps(&config, scene.step_hz)?,
                scene,
                config,
            })
        } else {
            None
        };
        Ok(Self {
            imu: SimulatedImu::new(request.imu_grade.grade().spec(), request.seeds.imu)?,
            gnss: GnssReceiver::new(request.gnss, request.seeds.gnss)?,
            gnss_period: period_steps("gnss", request.gnss.rate_hz)?,
            baro: BaroAltimeter::new(request.baro, request.seeds.baro)?,
            baro_period: period_steps("baro", request.baro.rate_hz)?,
            stars: live_stars,
        })
    }

    /// Starts the IMU's first interval at step 0.
    pub(super) fn start(&mut self, at: SimInstant, state: &VehicleState) -> Result<(), TrialError> {
        self.imu.sense(at, state)?;
        Ok(())
    }

    /// Senses `step`, the last of `truth`, into `log`, and returns the body
    /// rate the IMU sensed over the step.
    pub(super) fn sense(
        &mut self,
        truth: &[Truth<(SimInstant, VehicleState)>],
        step: usize,
        log: &mut SensorLog,
    ) -> Result<DVec3, TrialError> {
        let Some((at, state)) = truth.get(step).map(|sample| *sample.get()) else {
            return Err(TrialError::InvalidRequest {
                field: "closed loop",
                detail: format!("no truth at step {step}"),
            });
        };
        log.instants.push(at);
        let mut rate = DVec3::ZERO;
        if let Some(increment) = self.imu.sense(at, &state)? {
            rate = increment.delta_theta_rad / increment.dt_s;
            log.imu.push(increment);
        }
        if step.is_multiple_of(self.gnss_period) {
            let reading = self.gnss.measure(&state);
            log.gnss.push(Logged { step, reading });
        }
        if step.is_multiple_of(self.baro_period) {
            let reading = self.baro.measure(&state);
            log.baro.push(Logged { step, reading });
        }
        if let Some(stars) = &self.stars {
            let period = stars.scene.period_steps.max(1);
            if step >= 1 && (step - 1).is_multiple_of(period) {
                log.star_mid_steps.push(step + stars.length / 2);
            }
            if let Some(open) = step.checked_sub(stars.length)
                && open >= 1
                && (open - 1).is_multiple_of(period)
            {
                let frame = render_frame(truth, &stars.scene, &stars.config, open, stars.length)?;
                log.stars.push(frame);
            }
        }
        Ok(rate)
    }
}
