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
use sim_devices::{AirDataSample, GnssReceiver};

use crate::trial::devices::{PitotProbe, StaticPort, receiver};
use crate::trial::live_stars::LiveStarCamera;
use crate::trial::record::{Logged, SensorLog, period_steps};
use crate::trial::route::step_seconds;
use crate::trial::{TrialError, TrialRequest};
use crate::{SimulatedImu, Truth};

/// Pitot-static air-data cadence (`24 Hz`, every 5th `120 Hz` step).
const PITOT_PERIOD_STEPS: usize = 5;

/// Direct sensor readings handed to the closed-loop autopilot at one step.
#[derive(Debug, Clone, Copy)]
pub(super) struct SensedAutopilotInputs {
    /// Body angular rate measured by the IMU over the last increment, rad/s.
    pub rate_body: DVec3,
    /// Latest simulated pitot-static air-data sample (`Pa`, `K`).
    pub air_data: Option<AirDataSample>,
}

/// Every sensor of a request, sensing a flight step by step into a log.
pub(super) struct LiveSensors {
    imu: SimulatedImu,
    gnss: GnssReceiver,
    gnss_period: usize,
    baro: StaticPort,
    baro_period: usize,
    pitot: PitotProbe,
    held_air_data: Option<AirDataSample>,
    stars: Option<LiveStarCamera>,
}

impl LiveSensors {
    /// The request's sensors, rendering star frames only when `stars`.
    pub(super) fn new(request: &TrialRequest, stars: bool) -> Result<Self, TrialError> {
        let live_stars = match stars {
            true => Some(LiveStarCamera::new(request)?),
            false => None,
        };
        Ok(Self {
            imu: SimulatedImu::new(request.imu_grade.grade().spec(), request.seeds.imu)?,
            gnss: receiver(request)?,
            gnss_period: period_steps("gnss", request.gnss.rate_hz)?,
            baro: StaticPort::new(request)?,
            baro_period: period_steps("baro", request.baro.rate_hz)?,
            pitot: PitotProbe::new(request)?,
            held_air_data: None,
            stars: live_stars,
        })
    }

    /// Starts the IMU's first interval and primes the pitot-static probe at step 0.
    pub(super) fn start(&mut self, at: SimInstant, state: &VehicleState) -> Result<(), TrialError> {
        self.imu.sense(at, state)?;
        let period_s = step_seconds(PITOT_PERIOD_STEPS);
        self.held_air_data = Some(self.pitot.sense(state, 0.0, period_s)?);
        Ok(())
    }

    /// Latest held pitot-static reading.
    pub(super) const fn held_air_data(&self) -> Option<AirDataSample> {
        self.held_air_data
    }

    /// Senses `step`, the last of `truth`, into `log`, and returns the IMU body
    /// rate and simulated pitot-static air data for the autopilot.
    pub(super) fn sense(
        &mut self,
        truth: &[Truth<(SimInstant, VehicleState)>],
        step: usize,
        log: &mut SensorLog,
    ) -> Result<SensedAutopilotInputs, TrialError> {
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
        if step.is_multiple_of(self.gnss_period)
            && let Some(reading) = self.gnss.measure(&state, step_seconds(self.gnss_period))
        {
            log.gnss.push(Logged { step, reading });
        }
        if step.is_multiple_of(self.baro_period) {
            let period_s = step_seconds(self.baro_period);
            let reading = self.baro.sense(&state, step_seconds(step), period_s)?;
            log.baro.push(Logged { step, reading });
        }
        if step.is_multiple_of(PITOT_PERIOD_STEPS) {
            let period_s = step_seconds(PITOT_PERIOD_STEPS);
            self.held_air_data = Some(self.pitot.sense(&state, step_seconds(step), period_s)?);
        }
        if let Some(stars) = &self.stars {
            stars.sense(truth, step, log, |_| true)?;
        }
        Ok(SensedAutopilotInputs {
            rate_body: rate,
            air_data: self.held_air_data,
        })
    }
}
