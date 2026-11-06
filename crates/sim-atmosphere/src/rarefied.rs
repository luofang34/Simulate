//! Gas kinetics of a sampled atmosphere: viscosity, mean free path, and the
//! Knudsen-number flow regime that decides whether continuum aerodynamics
//! still applies to a body of a given size.

use crate::AtmosphereSample;

#[cfg(test)]
mod tests;

/// Sutherland reference viscosity of air, Pa·s.
const SUTHERLAND_MU0_PA_S: f64 = 1.716e-5;
/// Sutherland reference temperature, kelvin.
const SUTHERLAND_T0_K: f64 = 273.15;
/// Sutherland constant of air, kelvin.
const SUTHERLAND_S_K: f64 = 110.4;
/// Boltzmann constant, J/K (exact, SI 2019).
const BOLTZMANN_J_K: f64 = 1.380_649e-23;
/// Effective hard-sphere collision diameter of an air molecule, metres.
const AIR_MOLECULE_DIAMETER_M: f64 = 3.65e-10;

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
    /// Dynamic viscosity by Sutherland's law, Pa·s; `0.0` where there is no gas.
    #[must_use]
    pub fn dynamic_viscosity_pa_s(&self) -> f64 {
        let t = self.temperature_k;
        if self.density_kg_m3 <= 0.0 || self.pressure_pa <= 0.0 || t <= 0.0 {
            return 0.0;
        }
        SUTHERLAND_MU0_PA_S * (t / SUTHERLAND_T0_K).powf(1.5) * (SUTHERLAND_T0_K + SUTHERLAND_S_K)
            / (t + SUTHERLAND_S_K)
    }

    /// Hard-sphere kinetic-theory mean free path `k_B T / (√2 π d² p)`, metres;
    /// infinite in vacuum.
    #[must_use]
    pub fn mean_free_path_m(&self) -> f64 {
        if self.pressure_pa <= 0.0 {
            return f64::INFINITY;
        }
        BOLTZMANN_J_K * self.temperature_k
            / (std::f64::consts::SQRT_2
                * std::f64::consts::PI
                * AIR_MOLECULE_DIAMETER_M
                * AIR_MOLECULE_DIAMETER_M
                * self.pressure_pa)
    }

    /// Knudsen number `λ / L` for a body of characteristic length `L` metres;
    /// infinite for a non-positive or non-finite `L`.
    #[must_use]
    pub fn knudsen_number(&self, characteristic_length_m: f64) -> f64 {
        if !(characteristic_length_m.is_finite() && characteristic_length_m > 0.0) {
            return f64::INFINITY;
        }
        self.mean_free_path_m() / characteristic_length_m
    }

    /// Flow regime around a body of characteristic length `L` metres.
    #[must_use]
    pub fn flow_regime(&self, characteristic_length_m: f64) -> FlowRegime {
        FlowRegime::from_knudsen(self.knudsen_number(characteristic_length_m))
    }
}
