//! Proper time of clocks that move through Earth's gravity: what a GNSS
//! satellite's oscillator, a spacecraft's onboard clock or a ground station's
//! frequency standard actually counts, against Terrestrial Time.
//!
//! Rates follow the IAU 2000 metric to order `1/c²`, which is how GNSS
//! system time is defined. TT ticks at the rate of a clock on the rotating
//! geoid, so an ideal clock there reads TT, a GPS clock gains about 38.6 µs a
//! day, and a low orbit loses time. Only Earth's monopole and `J2` enter the
//! potential, which bounds the rate error near Earth to about `1e-14`.

mod barycentric;

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::scalar_math::sqrt;

pub use barycentric::{BarycentricClockModel, L_B, MassiveBody, tdb_minus_tt_position_term_s};

/// Speed of light in vacuum, m/s (exact).
pub const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

/// `1 - dTT/dTCG` (IAU 2000 Resolution B1.9, a defining constant): the
/// geoid's potential over `c²`, so that TT runs at a geoid clock's rate.
pub const L_G: f64 = 6.969_290_134e-10;

/// A body's gravity potential as a clock feels it: monopole plus the `J2`
/// oblateness term, with `z` along the body's rotation pole.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ClockPotential {
    /// Gravitational parameter `GM`, m³/s².
    pub gm_m3_s2: f64,
    /// Unnormalised second zonal harmonic.
    pub j2: f64,
    /// Reference radius `J2` is scaled by, m.
    pub reference_radius_m: f64,
}

impl ClockPotential {
    /// Earth: WGS84 `GM` and equatorial radius, EGM2008 `J2`.
    pub const EARTH: Self = Self {
        gm_m3_s2: 3.986_004_418e14,
        j2: 1.082_626_68e-3,
        reference_radius_m: 6_378_137.0,
    };

    /// Positive gravitational potential `U` at `position_m` (body-centred,
    /// `z` along the pole), m²/s². `None` at the centre.
    #[must_use]
    pub fn potential_m2_s2(&self, position_m: DVec3) -> Option<f64> {
        let r2 = position_m.length_squared();
        if r2.is_nan() || r2 <= 0.0 {
            return None;
        }
        let r = sqrt(r2);
        let sin2_lat = position_m.z * position_m.z / r2;
        let p2 = 1.5 * sin2_lat - 0.5;
        let scale2 = self.reference_radius_m * self.reference_radius_m / r2;
        Some(self.gm_m3_s2 / r * (1.0 - self.j2 * scale2 * p2))
    }

    /// `dτ/dTT - 1` for a clock at `position_m` moving at `velocity_m_s`, both
    /// in a non-rotating geocentric frame (GCRS axes; the pole tilt of date
    /// changes the `J2` term by under `1e-16`). Positive means the clock gains
    /// on TT. `None` at the centre.
    #[must_use]
    pub fn proper_rate_offset(&self, position_m: DVec3, velocity_m_s: DVec3) -> Option<f64> {
        let u = self.potential_m2_s2(position_m)?;
        let kinetic = 0.5 * velocity_m_s.length_squared();
        Some(L_G - (u + kinetic) / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S))
    }
}

/// The periodic relativistic clock term of an orbit, `-2 (r·v) / c²`
/// seconds: what IS-GPS-200 calls `Δt_r = F e √A sin E`. It is the part of
/// [`ClockPotential::proper_rate_offset`]'s integral that a constant rate
/// cannot absorb, zero on a circular orbit.
#[must_use]
pub fn orbit_periodic_clock_term_s(position_m: DVec3, velocity_m_s: DVec3) -> f64 {
    -2.0 * position_m.dot(velocity_m_s) / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S)
}

/// Accumulated proper time of one clock against TT, `τ - TT`, advanced on
/// the master clock. It is plain data, so it lives in a block's snapshot and
/// replays bit for bit.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ProperTimeClock {
    tau_minus_tt_s: f64,
}

impl ProperTimeClock {
    /// A clock that reads `tau_minus_tt_s` ahead of TT.
    #[must_use]
    pub const fn with_offset(tau_minus_tt_s: f64) -> Self {
        Self { tau_minus_tt_s }
    }

    /// `τ - TT`, seconds.
    #[must_use]
    pub const fn tau_minus_tt_s(self) -> f64 {
        self.tau_minus_tt_s
    }

    /// Advances by `dt_tt_s` of TT, with the trapezoid of the rate offsets at
    /// the step's ends. Take `dt_tt_s` from
    /// [`crate::SimEpoch::tt_j2000_seconds_at`], not from ticks: ticks count
    /// TDB, whose rate against TT swings by about `3e-10` over a year.
    pub fn advance(&mut self, dt_tt_s: f64, rate_offset_start: f64, rate_offset_end: f64) {
        self.tau_minus_tt_s += 0.5 * (rate_offset_start + rate_offset_end) * dt_tt_s;
    }
}

#[cfg(test)]
mod tests;
