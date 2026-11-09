//! Deterministic onboard sensors and navigation filter: what the flight controllers,
//! guidance host, and cockpit displays are allowed to read instead of simulation truth.
//!
//! Every error is a seeded Gauss–Markov process that does not depend on the truth it
//! corrupts, so the bus state is a handful of numbers that a rewind checkpoint captures
//! whole, and stepping it once per physics step reproduces the same errors on replay.

use glam::{DQuat, DVec3};
use sim_apollo::CockpitInstrumentSnapshot;
use sim_atmosphere::{AtmosphereField, FullColumnAtmosphere};
use sim_core::{SimInstant, SourceValidity, StampedReading, VehicleState};
use sim_geo::{BodyEllipsoid, LocalFrame, UtcTime, ecef_to_geodetic};
use sim_indicate::{AirDataReading, AirDataSample};

mod noise;
mod realism;
#[cfg(test)]
mod tests;

use noise::NoiseStream;
use realism::Budget;
pub use realism::SensorRealism;

/// Seed every flight's sensor errors start from.
const SENSOR_SEED: u64 = 0x0B5E_55ED_5EED_0001;
/// Static pressure below which a BMP390-class barometer and pitot transducer no longer
/// resolve anything (about 53 km MSL), pascals.
const STATIC_PORT_FLOOR_PA: f64 = 50.0;
/// Altimeter setting the air data reports, hPa.
const STANDARD_QNH_HPA: f64 = 1_013.25;

/// Sensor and navigation errors at one instant.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SensorErrors {
    /// Navigation position error, local north-east-down metres.
    pub position_ned_m: DVec3,
    /// Navigation velocity error, local north-east-down m/s.
    pub velocity_ned_m_s: DVec3,
    /// AHRS attitude error, body-axis rotation vector, radians.
    pub attitude_body_rad: DVec3,
    /// Gyro rate error, body axes, rad/s.
    pub rate_body_rad_s: DVec3,
    /// Accelerometer bias, body FRD axes, m/s².
    pub accel_frd_m_s2: DVec3,
    /// Static-port pressure error, pascals.
    pub static_pa: f64,
    /// Pitot airspeed error, m/s.
    pub pitot_m_s: f64,
}

impl SensorErrors {
    fn stationary(b: &Budget, noise: &mut NoiseStream) -> Self {
        let h = b.horizontal.stationary3(noise);
        Self {
            position_ned_m: DVec3::new(h.x, h.y, b.vertical.stationary(noise)),
            velocity_ned_m_s: b.velocity.stationary3(noise),
            attitude_body_rad: b.attitude.stationary3(noise),
            rate_body_rad_s: b.rate.stationary3(noise),
            accel_frd_m_s2: b.accel.stationary3(noise),
            static_pa: b.static_pa.stationary(noise),
            pitot_m_s: b.pitot_m_s.stationary(noise),
        }
    }

    fn step(self, b: &Budget, dt_s: f64, noise: &mut NoiseStream) -> Self {
        let p = self.position_ned_m;
        Self {
            position_ned_m: DVec3::new(
                b.horizontal.step(p.x, dt_s, noise),
                b.horizontal.step(p.y, dt_s, noise),
                b.vertical.step(p.z, dt_s, noise),
            ),
            velocity_ned_m_s: b.velocity.step3(self.velocity_ned_m_s, dt_s, noise),
            attitude_body_rad: b.attitude.step3(self.attitude_body_rad, dt_s, noise),
            rate_body_rad_s: b.rate.step3(self.rate_body_rad_s, dt_s, noise),
            accel_frd_m_s2: b.accel.step3(self.accel_frd_m_s2, dt_s, noise),
            static_pa: b.static_pa.step(self.static_pa, dt_s, noise),
            pitot_m_s: b.pitot_m_s.step(self.pitot_m_s, dt_s, noise),
        }
    }

    /// The errors a fraction `t` of the way from `self` to `to`.
    pub fn lerp(self, to: Self, t: f64) -> Self {
        if t >= 1.0 {
            return to;
        }
        let t = t.max(0.0);
        Self {
            position_ned_m: self.position_ned_m.lerp(to.position_ned_m, t),
            velocity_ned_m_s: self.velocity_ned_m_s.lerp(to.velocity_ned_m_s, t),
            attitude_body_rad: self.attitude_body_rad.lerp(to.attitude_body_rad, t),
            rate_body_rad_s: self.rate_body_rad_s.lerp(to.rate_body_rad_s, t),
            accel_frd_m_s2: self.accel_frd_m_s2.lerp(to.accel_frd_m_s2, t),
            static_pa: self.static_pa + (to.static_pa - self.static_pa) * t,
            pitot_m_s: self.pitot_m_s + (to.pitot_m_s - self.pitot_m_s) * t,
        }
    }

    /// The navigation solution these errors make of `truth` on `body`.
    pub fn estimate(&self, body: BodyEllipsoid, truth: &VehicleState) -> VehicleState {
        if *self == Self::default() {
            return *truth;
        }
        let ned_to_ecef =
            LocalFrame::at(&body.fixed_to_geodetic(truth.position_ecef)).ned_to_ecef();
        VehicleState {
            position_ecef: truth.position_ecef + ned_to_ecef * self.position_ned_m,
            velocity_ecef: truth.velocity_ecef + ned_to_ecef * self.velocity_ned_m_s,
            attitude: (truth.attitude * DQuat::from_scaled_axis(self.attitude_body_rad))
                .normalize(),
            angular_rate_body: truth.angular_rate_body + self.rate_body_rad_s,
        }
    }
}

/// The onboard sensors of one flight: the realism mode, the seeded noise stream, and the
/// errors at the previous and latest physics steps (display frames interpolate between
/// them by the same phase the truth is interpolated by).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OnboardSensorBus {
    realism: SensorRealism,
    noise: NoiseStream,
    previous: SensorErrors,
    current: SensorErrors,
    atmosphere: FullColumnAtmosphere,
}

impl OnboardSensorBus {
    /// A fresh bus whose errors start already settled at their stationary spread.
    pub fn new(realism: SensorRealism) -> Self {
        let mut noise = NoiseStream::new(SENSOR_SEED);
        let current = SensorErrors::stationary(&realism.budget(), &mut noise);
        Self {
            realism,
            noise,
            previous: current,
            current,
            atmosphere: FullColumnAtmosphere::new(),
        }
    }

    /// Active realism mode.
    pub const fn realism(&self) -> SensorRealism {
        self.realism
    }

    /// Advances every error process by one physics step of `dt_s` seconds.
    pub fn step(&mut self, dt_s: f64) {
        self.previous = self.current;
        self.current = self
            .current
            .step(&self.realism.budget(), dt_s, &mut self.noise);
    }

    /// Errors at the latest physics step.
    pub const fn current(&self) -> SensorErrors {
        self.current
    }

    /// Errors a display phase `alpha` past the previous physics step.
    pub fn errors_at(&self, alpha: f64) -> SensorErrors {
        self.previous.lerp(self.current, alpha)
    }

    /// Pitot-static air data sensed on `truth` at `at` (`utc` on the flight's
    /// clock): true airspeed from the velocity through the air mass (not over
    /// the ground), or `None` where the static port reads below the transducer floor.
    pub fn air_data(
        &self,
        at: SimInstant,
        utc: UtcTime,
        truth: &VehicleState,
        errors: &SensorErrors,
    ) -> Option<AirDataReading> {
        let geo = ecef_to_geodetic(truth.position_ecef);
        let air = self.atmosphere.sample(&geo, utc).ok()?;
        if air.pressure_pa < STATIC_PORT_FLOOR_PA {
            return None;
        }
        let wind_ecef = LocalFrame::at(&geo).ned_to_ecef() * air.wind_ned_m_s;
        let airspeed = (truth.velocity_ecef - wind_ecef).length() + errors.pitot_m_s;
        let sample = AirDataSample {
            true_airspeed_m_s: airspeed.max(0.0),
            density_kg_m3: air.density_kg_m3,
            static_pressure_pa: Some((air.pressure_pa + errors.static_pa).max(0.0)),
            altimeter_setting_hpa: Some(STANDARD_QNH_HPA),
            pec_indicated_airspeed_m_s: None,
        };
        Some(StampedReading::new(sample, at, SourceValidity::Valid))
    }
}

/// The accelerometers' reading of `truth_force` (body FRD specific force).
pub fn sensed_specific_force(truth_force: Option<DVec3>, errors: &SensorErrors) -> Option<DVec3> {
    truth_force.map(|force| force + errors.accel_frd_m_s2)
}

/// `cockpit` with its attitude and rate gyros reading the navigation solution `estimated`.
pub fn sensed_cockpit(
    mut cockpit: CockpitInstrumentSnapshot,
    estimated: &VehicleState,
) -> CockpitInstrumentSnapshot {
    cockpit.attitude.value = estimated.attitude;
    cockpit.pgns_attitude.value = estimated.attitude;
    cockpit.ags_attitude.value = estimated.attitude;
    cockpit.body_rate_rad_s.value = estimated.angular_rate_body;
    cockpit
}
