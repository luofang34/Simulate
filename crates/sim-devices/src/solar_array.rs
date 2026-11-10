//! A photovoltaic array, fed by the same sunlight sample that pushes on the
//! vessel and lights the scene: in orbit, on a planet's surface under its
//! atmosphere, or in an eclipse.

use glam::DVec3;
use sim_core::{Block, BlockError, TickSpan};
use sim_spacetime::SolarFluxSample;

const BLOCK: &str = "solar array";

/// One flat array.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolarArrayConfig {
    /// Cell area, m².
    pub area_m2: f64,
    /// Fraction of the incident sunlight delivered as electrical power at
    /// [`Self::reference_temperature_k`], in `[0, 1]`.
    pub efficiency: f64,
    /// Fractional change of power per kelvin above the reference
    /// temperature; negative for every practical cell.
    pub temperature_coefficient_per_k: f64,
    /// Cell temperature [`Self::efficiency`] is quoted at, K.
    pub reference_temperature_k: f64,
    /// How often the array's output is refreshed.
    pub period: TickSpan,
}

/// What the array is exposed to at one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolarArrayInput {
    /// Sunlight at the array, from the shared solar-flux model.
    pub sunlight: SolarFluxSample,
    /// Outward unit normal of the cell face, ICRF axes.
    pub normal_icrf: DVec3,
    /// Fraction of the direct beam an atmosphere passes, `1` in vacuum.
    pub direct_beam_fraction: f64,
    /// Cell temperature, K.
    pub cell_temperature_k: f64,
}

/// A solar array as a block: electrical power in watts, refreshed each
/// period.
#[derive(Debug, Clone, PartialEq)]
pub struct SolarArrayBlock {
    config: SolarArrayConfig,
    power_w: f64,
}

impl SolarArrayBlock {
    /// An array producing nothing until its first step.
    #[must_use]
    pub const fn new(config: SolarArrayConfig) -> Self {
        Self {
            config,
            power_w: 0.0,
        }
    }
}

impl Block for SolarArrayBlock {
    type Input = SolarArrayInput;
    type Output = f64;
    type Snapshot = f64;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &SolarArrayInput) -> Result<(), BlockError> {
        let beam = input.direct_beam_fraction;
        if !input.cell_temperature_k.is_finite() || !(0.0..=1.0).contains(&beam) {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "cell temperature or direct-beam fraction",
                detail: format!("{} K, {beam}", input.cell_temperature_k),
            });
        }
        let incident_w =
            input.sunlight.irradiance_on_w_m2(input.normal_icrf) * beam * self.config.area_m2;
        let warmth = input.cell_temperature_k - self.config.reference_temperature_k;
        let derating = (1.0 + self.config.temperature_coefficient_per_k * warmth).max(0.0);
        self.power_w = incident_w * self.config.efficiency * derating;
        Ok(())
    }

    fn output(&self) -> f64 {
        self.power_w
    }

    fn snapshot(&self) -> f64 {
        self.power_w
    }

    fn restore(&mut self, snapshot: &f64) {
        self.power_w = *snapshot;
    }
}

#[cfg(test)]
mod tests;
