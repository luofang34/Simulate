//! Antenna gain patterns.

use core::f64::consts::PI;

use glam::DVec3;
use sim_analysis_contract::{AntennaSpec, CosinePowerPatchSpec, ParabolicDishSpec};
use sim_spacetime::{SPEED_OF_LIGHT_M_S, acos_clamped, cos, sqrt};

use super::units::db_from_ratio;

/// Gain of an antenna toward a direction in its own frame (`+Z` boresight).
pub trait AntennaPattern: Send + Sync {
    /// Gain toward `direction_in_antenna_frame` (`dBi`); the vector need not be normalized.
    fn gain_dbi(&self, direction_in_antenna_frame: DVec3, carrier_frequency_hz: f64) -> f64;

    /// Largest gain over all directions (`dBi`).
    fn peak_gain_dbi(&self, carrier_frequency_hz: f64) -> f64;

    /// Whether the gain depends on where the antenna points, so a fixed mount needs attitude.
    fn requires_orientation(&self) -> bool {
        true
    }
}

fn off_axis_rad(direction: DVec3) -> f64 {
    let len = sqrt(direction.length_squared());
    if len <= 1.0e-15 {
        0.0
    } else {
        acos_clamped(direction.z / len)
    }
}

fn dish_peak_dbi(d: &ParabolicDishSpec, frequency_hz: f64) -> f64 {
    let wavelength_m = SPEED_OF_LIGHT_M_S / frequency_hz;
    let x = PI * d.diameter_m / wavelength_m;
    db_from_ratio(d.aperture_efficiency * x * x)
}

/// Half-power beamwidth $70\lambda/D$ degrees, the figure for a tapered circular aperture.
fn dish_half_power_beamwidth_rad(d: &ParabolicDishSpec, frequency_hz: f64) -> f64 {
    let wavelength_m = SPEED_OF_LIGHT_M_S / frequency_hz;
    (70.0 * wavelength_m / d.diameter_m).to_radians()
}

fn dish_gain_dbi(d: &ParabolicDishSpec, direction: DVec3, frequency_hz: f64) -> f64 {
    let peak = dish_peak_dbi(d, frequency_hz);
    let ratio = off_axis_rad(direction) / dish_half_power_beamwidth_rad(d, frequency_hz);
    (peak - 12.0 * ratio * ratio).max(peak - d.sidelobe_floor_below_peak_db)
}

fn patch_gain_dbi(p: &CosinePowerPatchSpec, direction: DVec3) -> f64 {
    let c = cos(off_axis_rad(direction));
    if c <= 0.0 {
        return p.back_hemisphere_floor_dbi;
    }
    let lobe = if p.cosine_exponent == 0.0 {
        p.peak_gain_dbi
    } else {
        p.peak_gain_dbi + p.cosine_exponent * db_from_ratio(c)
    };
    lobe.max(p.back_hemisphere_floor_dbi)
}

impl AntennaPattern for AntennaSpec {
    fn gain_dbi(&self, direction: DVec3, carrier_frequency_hz: f64) -> f64 {
        match self {
            Self::Isotropic => 0.0,
            Self::ParabolicDish(d) => dish_gain_dbi(d, direction, carrier_frequency_hz),
            Self::CosinePowerPatch(p) => patch_gain_dbi(p, direction),
        }
    }

    fn peak_gain_dbi(&self, carrier_frequency_hz: f64) -> f64 {
        match self {
            Self::Isotropic => 0.0,
            Self::ParabolicDish(d) => dish_peak_dbi(d, carrier_frequency_hz),
            Self::CosinePowerPatch(p) => p.peak_gain_dbi,
        }
    }

    fn requires_orientation(&self) -> bool {
        !matches!(self, Self::Isotropic)
    }
}
