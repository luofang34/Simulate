//! Gas kinetics of a sampled atmosphere: viscosity, mean free path, and the
//! Knudsen-number flow regime that decides whether continuum aerodynamics
//! still applies to a body of a given size.

use crate::AtmosphereSample;

#[cfg(test)]
mod tests;

/// Knudsen number below which the Navier–Stokes equations hold with no-slip walls.
const SLIP_KN: f64 = 0.01;
/// Knudsen number where slip-wall corrections stop being enough.
const TRANSITIONAL_KN: f64 = 0.1;
/// Knudsen number from which molecules reach the body without colliding first.
const FREE_MOLECULAR_KN: f64 = 10.0;

/// Aerodynamic flow regime, classified by Knudsen number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowRegime {
    /// `Kn < 0.01`: continuum flow, no-slip walls.
    Continuum,
    /// `0.01 ≤ Kn < 0.1`: continuum with velocity slip and temperature jump at walls.
    Slip,
    /// `0.1 ≤ Kn < 10`: neither continuum nor collisionless; DSMC territory.
    Transitional,
    /// `Kn ≥ 10`: collisionless free-molecular flow (and vacuum).
    FreeMolecular,
}

impl FlowRegime {
    /// The regime of a flow at Knudsen number `kn`; `NaN` counts as free-molecular.
    #[must_use]
    pub fn from_knudsen(kn: f64) -> Self {
        if kn < SLIP_KN {
            Self::Continuum
        } else if kn < TRANSITIONAL_KN {
            Self::Slip
        } else if kn < FREE_MOLECULAR_KN {
            Self::Transitional
        } else {
            Self::FreeMolecular
        }
    }
}

impl AtmosphereSample {
    /// Multi-species mixture dynamic viscosity from `mbd-core`, Pa·s; `0.0` where there is no gas.
    #[must_use]
    pub fn dynamic_viscosity_pa_s(&self) -> f64 {
        self.ambient_gas().dynamic_viscosity_pa_s
    }

    /// Multi-species kinetic-theory mean free path from `mbd-core`, metres;
    /// infinite in vacuum.
    #[must_use]
    pub fn mean_free_path_m(&self) -> f64 {
        let gas = self.ambient_gas();
        if gas.is_vacuum() || gas.mean_free_path_m <= 0.0 {
            f64::INFINITY
        } else {
            gas.mean_free_path_m
        }
    }

    /// Knudsen number `λ / L` for a body of characteristic length `L` metres;
    /// infinite for a non-positive or non-finite `L`.
    #[must_use]
    pub fn knudsen_number(&self, characteristic_length_m: f64) -> f64 {
        if !(characteristic_length_m.is_finite() && characteristic_length_m > 0.0) {
            return f64::INFINITY;
        }
        self.ambient_gas().knudsen_number(characteristic_length_m)
    }

    /// Flow regime around a body of characteristic length `L` metres.
    #[must_use]
    pub fn flow_regime(&self, characteristic_length_m: f64) -> FlowRegime {
        FlowRegime::from_knudsen(self.knudsen_number(characteristic_length_m))
    }
}
