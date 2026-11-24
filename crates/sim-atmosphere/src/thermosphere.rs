//! Multi-species Earth VLEO/LEO thermosphere (`120 km .. 600 km`) driven by
//! a Bates-Walker temperature profile and species-by-species diffusive
//! equilibrium, capturing atomic oxygen (`O`) dominance for ABEP intakes and
//! Kapton AO erosion.

use glam::DVec3;
use mbd_core::{AmbientGas, GasSpecies, UNIVERSAL_GAS_CONSTANT_J_MOL_K};
use sim_geo::{Geodetic, UtcTime};

use crate::content::{feed_f64s, feed_kind};
use crate::{
    AtmosphereContent, AtmosphereError, AtmosphereField, AtmosphereSample, COSMIC_BACKGROUND_K,
    ReentryAtmosphere, VacuumAtmosphere,
};

#[cfg(test)]
mod tests;

const LOWER_TOP_M: f64 = 120_000.0;
const FADE_START_M: f64 = 550_000.0;
const VACUUM_FLOOR_M: f64 = 600_000.0;
const T_120_K: f64 = 360.0;
const S_PER_M: f64 = 1.5e-5;
const G_THERMO_M_S2: f64 = 9.0;

const SPECIES_ANCHORS_120KM: [(GasSpecies, f64); 5] = [
    (GasSpecies::N2, 3.1e17),
    (GasSpecies::O2, 4.4e16),
    (GasSpecies::AtomicO, 7.6e16),
    (GasSpecies::Ar, 1.9e15),
    (GasSpecies::He, 3.0e13),
];

/// Multi-species Earth atmosphere extending continuously up through the VLEO/LEO
/// thermosphere (`120 ..= 600 km`) using a solar-flux-parameterized
/// Bates-Walker profile with diffusive separation of `N2`, `O2`, `AtomicO`,
/// `Ar`, and `He`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VleoThermosphereAtmosphere {
    f107_sfu: f64,
}

impl Default for VleoThermosphereAtmosphere {
    fn default() -> Self {
        Self::new()
    }
}

impl VleoThermosphereAtmosphere {
    /// Moderate solar activity (`F10.7 = 140 sfu`).
    #[must_use]
    pub const fn new() -> Self {
        Self { f107_sfu: 140.0 }
    }

    /// Thermosphere parameterized by solar radio flux index `f107_sfu` (clamped to `65..=300 sfu`).
    #[must_use]
    pub const fn with_f107(f107_sfu: f64) -> Self {
        let clamped = if f107_sfu < 65.0 {
            65.0
        } else if f107_sfu > 300.0 {
            300.0
        } else {
            f107_sfu
        };
        Self { f107_sfu: clamped }
    }

    /// Active solar flux index $F_{10.7}$ (`sfu`).
    #[must_use]
    pub const fn f107_sfu(self) -> f64 {
        self.f107_sfu
    }
}

impl AtmosphereContent for VleoThermosphereAtmosphere {
    /// The solar flux, the profile's constants and anchors, and the model
    /// it hands over to below its floor and above its ceiling.
    fn visit_content(&self, f: &mut dyn FnMut(&[u8])) {
        feed_kind(f, "bates-walker-thermosphere");
        feed_f64s(
            f,
            &[
                self.f107_sfu,
                LOWER_TOP_M,
                FADE_START_M,
                VACUUM_FLOOR_M,
                T_120_K,
                S_PER_M,
                G_THERMO_M_S2,
            ],
        );
        let anchors = SPECIES_ANCHORS_120KM.map(|(_, density)| density);
        feed_f64s(f, &anchors);
        ReentryAtmosphere.visit_content(f);
        VacuumAtmosphere.visit_content(f);
    }
}

impl AtmosphereField for VleoThermosphereAtmosphere {
    fn sample(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let z_m = position.height_m;
        if !z_m.is_finite() || z_m < LOWER_TOP_M {
            return ReentryAtmosphere.sample(position, utc);
        }
        if z_m >= VACUUM_FLOOR_M {
            return Ok(VacuumAtmosphere::SAMPLE);
        }
        Ok(bates_walker_sample(z_m, self.f107_sfu))
    }
}

fn bates_walker_sample(height_m: f64, f107_sfu: f64) -> AtmosphereSample {
    let t_inf = (600.0 + 3.2 * (f107_sfu - 70.0)).clamp(650.0, 1_500.0);
    let dz = height_m - LOWER_TOP_M;
    let exp_s = (-S_PER_M * dz).exp();
    let t_k = t_inf - (t_inf - T_120_K) * exp_s;
    let t_ratio = T_120_K / t_k;

    let mut densities = [(GasSpecies::N2, 0.0_f64); 5];
    for (slot, &(species, n0)) in densities.iter_mut().zip(&SPECIES_ANCHORS_120KM) {
        let gamma_i = species.molar_mass_kg_mol() * G_THERMO_M_S2
            / (UNIVERSAL_GAS_CONSTANT_J_MOL_K * S_PER_M * t_inf);
        let n_i = n0 * t_ratio.powf(1.0 + gamma_i) * (-S_PER_M * gamma_i * dz).exp();
        *slot = (species, n_i);
    }

    let Ok(gas) = AmbientGas::from_partial_number_densities(t_k, &densities) else {
        return VacuumAtmosphere::SAMPLE;
    };
    let rem = fade_to_vacuum(height_m);
    AtmosphereSample {
        temperature_k: COSMIC_BACKGROUND_K + rem * (gas.temperature_k - COSMIC_BACKGROUND_K),
        pressure_pa: rem * gas.pressure_pa,
        density_kg_m3: rem * gas.density_kg_m3,
        speed_of_sound_m_s: rem * gas.sound_speed_m_s,
        wind_ned_m_s: DVec3::ZERO,
        composition: gas.composition,
    }
}

fn fade_to_vacuum(height_m: f64) -> f64 {
    let left = ((VACUUM_FLOOR_M - height_m) / (VACUUM_FLOOR_M - FADE_START_M)).clamp(0.0, 1.0);
    left * left * (3.0 - 2.0 * left)
}
