//! The navigation sets' view of the vehicle's own devices.
//!
//! The flight controller takes the suite's samples cycle by cycle; the
//! navigators take the same samples on their own step: the inertial
//! increments regrouped at the step boundaries, every receiver fix, and the
//! static port's pressure read as an altitude. Nothing here reads truth
//! except the truth record kept for scoring.

use glam::DVec3;
use navigate_contract::{GeodeticPosition, NedVelocity, SymmetricCov3};
use sim_core::{SimInstant, TickSpan, VehicleState};
use sim_devices::{DeviceSamples, GnssReading, InertialIncrement};
use sim_navigate::AlignmentFix;
use sim_navigate::trial::{BaroReading, GnssFix, Logged, SensorLog, step_span};

/// Sea-level standard temperature, kelvin.
const ISA_T0_K: f64 = 288.15;
/// Tropospheric lapse rate, kelvin per geopotential metre.
const ISA_LAPSE_K_M: f64 = 0.0065;
/// Sea-level standard pressure, pascals.
const ISA_P0_PA: f64 = 101_325.0;
/// `R·L/g0` of the standard troposphere.
const ISA_EXPONENT: f64 = 0.190_263;
/// Earth radius of the geopotential definition, metres.
const GEOPOTENTIAL_RADIUS_M: f64 = 6_356_766.0;

/// The log the sets replay, grown cycle by cycle, with truth at each step.
pub(crate) struct NavLog {
    pub(crate) log: SensorLog,
    /// The true state at each logged step: scoring and drawing only.
    pub(crate) truth: Vec<VehicleState>,
    step_ticks: u128,
    pending_imu: InertialIncrement,
    pending_gnss: Vec<Logged<GnssFix>>,
    pending_baro: Vec<Logged<BaroReading>>,
    /// Geoid height the altimeter adds to its pressure altitude, metres.
    undulation_m: f64,
}

impl NavLog {
    /// An empty log starting at `start`, the sets aligned by `alignment`.
    pub(crate) fn new(start: VehicleState, alignment: AlignmentFix, undulation_m: f64) -> Self {
        Self {
            log: SensorLog {
                instants: vec![SimInstant::START],
                alignment,
                imu: Vec::new(),
                gnss: Vec::new(),
                baro: Vec::new(),
                stars: Vec::new(),
                star_mid_steps: Vec::new(),
                terrain: Vec::new(),
            },
            truth: vec![start],
            step_ticks: step_span().ticks_u128(),
            pending_imu: empty_increment(),
            pending_gnss: Vec::new(),
            pending_baro: Vec::new(),
            undulation_m,
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
            self.log
                .instants
                .push(SimInstant::from_ticks_u128(boundary));
            self.truth.push(interpolate(before, after, fraction));
            self.pending_imu = scaled(&increment, 1.0 - fraction);
            Some(next)
        } else {
            add(&mut self.pending_imu, &increment);
            None
        };
        // A reading delivered at the cycle's end reaches the sets at the
        // first step at or after it.
        let due = to.ticks_u128().div_ceil(self.step_ticks.max(1)) as usize;
        if let Some(reading) = samples.gnss.as_ref() {
            self.pending_gnss.push(Logged {
                step: due,
                reading: gnss_fix(reading),
            });
        }
        if let Some(air) = samples.air_data.as_ref() {
            let altitude_m = pressure_altitude_m(air.static_pressure_pa) + self.undulation_m;
            self.pending_baro.push(Logged {
                step: due,
                reading: BaroReading { altitude_m },
            });
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
fn move_due<T>(pending: &mut Vec<Logged<T>>, log: &mut Vec<Logged<T>>, logged: usize) {
    let ready = pending.iter().take_while(|r| r.step <= logged).count();
    log.extend(pending.drain(..ready));
}

/// A receiver fix as the navigators take it, with the receiver's own
/// stated accuracy.
fn gnss_fix(reading: &GnssReading) -> GnssFix {
    let p = reading.position_sigma_ned_m * reading.position_sigma_ned_m;
    let v = reading.velocity_sigma_m_s * reading.velocity_sigma_m_s;
    let velocity = reading.velocity_ned_m_s;
    GnssFix {
        position: GeodeticPosition::new(
            reading.position.lat_rad,
            reading.position.lon_rad,
            reading.position.height_m,
        ),
        position_cov: SymmetricCov3::from_diagonal(p.x, p.y, p.z),
        velocity: NedVelocity::new(velocity.x, velocity.y, velocity.z),
        velocity_cov: SymmetricCov3::from_diagonal(v, v, v),
    }
}

/// The geometric height of `pressure_pa` in the standard troposphere: the
/// altimeter's own reading, made on board without knowing the weather.
fn pressure_altitude_m(pressure_pa: f64) -> f64 {
    let geopotential =
        ISA_T0_K / ISA_LAPSE_K_M * (1.0 - (pressure_pa / ISA_P0_PA).powf(ISA_EXPONENT));
    GEOPOTENTIAL_RADIUS_M * geopotential / (GEOPOTENTIAL_RADIUS_M - geopotential)
}
