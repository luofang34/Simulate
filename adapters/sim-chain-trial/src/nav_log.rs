//! The navigation sets' view of the vehicle's own devices.
//!
//! The flight controller takes the suite's samples cycle by cycle; the
//! navigators take the same samples on their own step: the inertial
//! increments regrouped at the step boundaries, every receiver fix, and
//! every air-data sample, whose pressure each navigator reads as an
//! altitude itself. Nothing here reads truth except the truth record kept
//! for scoring.

use glam::DVec3;
use sim_core::{SimInstant, TickSpan, VehicleState};
use sim_devices::{AirDataSample, DeviceSamples, GnssReading, InertialIncrement};
use sim_navigate::trial::{
    LogPositions, LogWindow, Logged, SensorLog, StarFrame, TerrainFrame, step_span,
};
use sim_navigate::{AlignmentFix, Truth};

/// The log the sets replay, grown cycle by cycle, with truth at each step.
pub(crate) struct NavLog {
    pub(crate) log: SensorLog,
    /// The true state at each logged step: scoring, drawing, and the star
    /// camera's physics only.
    pub(crate) truth: LogWindow<Truth<(SimInstant, VehicleState)>>,
    step_ticks: u128,
    pending_imu: InertialIncrement,
    pending_gnss: Vec<Logged<GnssReading>>,
    pending_baro: Vec<Logged<AirDataSample>>,
}

impl NavLog {
    /// An empty log starting at `start`, the sets aligned by `alignment`.
    pub(crate) fn new(start: VehicleState, alignment: AlignmentFix) -> Self {
        Self {
            log: SensorLog {
                instants: vec![SimInstant::START].into(),
                alignment,
                imu: LogWindow::new(),
                gnss: LogWindow::new(),
                baro: LogWindow::new(),
                stars: LogWindow::new(),
                star_mid_steps: LogWindow::new(),
                terrain: LogWindow::new(),
            },
            truth: vec![Truth::new((SimInstant::START, start))].into(),
            step_ticks: step_span().ticks_u128(),
            pending_imu: empty_increment(),
            pending_gnss: Vec::new(),
            pending_baro: Vec::new(),
        }
    }

    /// Forgets what every reader has passed: the log before `positions`,
    /// the slowest set's, and the truth before `truth_from`, the earliest
    /// step the star camera or the records may still look at. A flight that
    /// holds for hours then keeps a log as long as its readers' lag, not as
    /// long as the flight.
    pub(crate) fn release_before(&mut self, positions: &LogPositions, truth_from: usize) {
        self.log.release_before(positions);
        self.truth.release_before(truth_from.min(positions.step));
    }

    /// What the log keeps, and roughly how many bytes that is.
    pub(crate) fn retained(&self) -> crate::Retained {
        let log = &self.log;
        let terrain_pixel_bytes = log.terrain.iter().map(|frame| frame.pixels.len()).sum();
        let star_bytes: usize = log
            .stars
            .iter()
            .map(|frame| size_of_val(frame.image.centroids.as_slice()))
            .sum();
        let entry_bytes = log.instants.kept() * size_of::<SimInstant>()
            + log.imu.kept() * size_of::<InertialIncrement>()
            + log.gnss.kept() * size_of::<Logged<GnssReading>>()
            + log.baro.kept() * size_of::<Logged<AirDataSample>>()
            + log.stars.kept() * size_of::<StarFrame>()
            + log.star_mid_steps.kept() * size_of::<usize>()
            + log.terrain.kept() * size_of::<TerrainFrame>()
            + self.truth.kept() * size_of::<Truth<(SimInstant, VehicleState)>>();
        crate::Retained {
            steps: log.instants.len(),
            steps_kept: log.instants.kept().max(self.truth.kept()),
            readings_kept: log.gnss.kept() + log.baro.kept(),
            frames_kept: log.stars.kept() + log.terrain.kept(),
            terrain_pixel_bytes,
            approximate_bytes: entry_bytes + terrain_pixel_bytes + star_bytes,
        }
    }

    /// The last step logged.
    pub(crate) fn last_step(&self) -> usize {
        self.log.imu.len()
    }

    /// One flight-controller cycle from `before` to `after`, the devices
    /// having delivered `samples` at its end. Returns the step it completed,
    /// if it crossed a step boundary.
    pub(crate) fn on_cycle(
        &mut self,
        (from, to): (SimInstant, SimInstant),
        (before, after): (&VehicleState, &VehicleState),
        samples: &DeviceSamples,
    ) -> Option<usize> {
        let next = self.last_step().wrapping_add(1);
        let boundary = self.step_ticks.saturating_mul(next as u128);
        let span = to.ticks_u128().saturating_sub(from.ticks_u128());
        let increment = samples.imu.unwrap_or_else(empty_increment);
        let completed = if to.ticks_u128() >= boundary && span > 0 {
            let fraction = boundary.saturating_sub(from.ticks_u128()) as f64 / span as f64;
            add(&mut self.pending_imu, &scaled(&increment, fraction));
            self.log.imu.push(self.pending_imu);
            let at = SimInstant::from_ticks_u128(boundary);
            self.log.instants.push(at);
            self.truth
                .push(Truth::new((at, interpolate(before, after, fraction))));
            self.pending_imu = scaled(&increment, 1.0 - fraction);
            Some(next)
        } else {
            add(&mut self.pending_imu, &increment);
            None
        };
        // A reading delivered at the cycle's end reaches the sets at the
        // first step at or after it.
        let due = to.ticks_u128().div_ceil(self.step_ticks.max(1)) as usize;
        if let Some(reading) = samples.gnss {
            self.pending_gnss.push(Logged { step: due, reading });
        }
        if let Some(reading) = samples.air_data {
            self.pending_baro.push(Logged { step: due, reading });
        }
        let logged = self.last_step();
        move_due(&mut self.pending_gnss, &mut self.log.gnss, logged);
        move_due(&mut self.pending_baro, &mut self.log.baro, logged);
        completed
    }

    /// Route time of `step`, seconds.
    pub(crate) fn seconds(&self, step: usize) -> f64 {
        TickSpan::from_ticks_u128(self.step_ticks.saturating_mul(step as u128)).as_seconds()
    }
}

fn empty_increment() -> InertialIncrement {
    InertialIncrement {
        dt_s: 0.0,
        delta_theta_rad: DVec3::ZERO,
        delta_velocity_m_s: DVec3::ZERO,
    }
}

fn scaled(increment: &InertialIncrement, fraction: f64) -> InertialIncrement {
    InertialIncrement {
        dt_s: increment.dt_s * fraction,
        delta_theta_rad: increment.delta_theta_rad * fraction,
        delta_velocity_m_s: increment.delta_velocity_m_s * fraction,
    }
}

fn add(sum: &mut InertialIncrement, increment: &InertialIncrement) {
    sum.dt_s += increment.dt_s;
    sum.delta_theta_rad += increment.delta_theta_rad;
    sum.delta_velocity_m_s += increment.delta_velocity_m_s;
}

/// The state a `fraction` of the way from `before` to `after`.
fn interpolate(before: &VehicleState, after: &VehicleState, fraction: f64) -> VehicleState {
    VehicleState {
        position_ecef: before.position_ecef.lerp(after.position_ecef, fraction),
        velocity_ecef: before.velocity_ecef.lerp(after.velocity_ecef, fraction),
        attitude: before.attitude.slerp(after.attitude, fraction),
        angular_rate_body: before
            .angular_rate_body
            .lerp(after.angular_rate_body, fraction),
    }
}

/// Moves the readings due at or before `logged` from `pending` to `log`.
fn move_due<T>(pending: &mut Vec<Logged<T>>, log: &mut LogWindow<Logged<T>>, logged: usize) {
    let ready = pending.iter().take_while(|r| r.step <= logged).count();
    log.extend(pending.drain(..ready));
}
