use glam::DVec3;
use sim_geo::{Geodetic, UtcTime};

use crate::isa::{speed_of_sound_m_s, ussa76_at_geopotential};
use crate::{
    AtmosphereError, AtmosphereField, AtmosphereSample, COSMIC_BACKGROUND_K, VacuumAtmosphere,
    geometric_to_geopotential_m,
};

#[cfg(test)]
mod tests;

/// Geometric height where the layered model hands over to the table, metres.
const LAYERS_TOP_M: f64 = 86_000.0;
/// Geometric height where the fade to vacuum starts, metres.
const FADE_START_M: f64 = 180_000.0;
/// Geometric height from which the sample is exactly vacuum, metres.
const VACUUM_FLOOR_M: f64 = 200_000.0;

/// One tabulated state of the 1976 U.S. Standard Atmosphere.
struct Node {
    height_m: f64,
    temperature_k: f64,
    pressure_pa: f64,
    density_kg_m3: f64,
}

const fn node(height_km: f64, temperature_k: f64, pressure_pa: f64, density_kg_m3: f64) -> Node {
    Node {
        height_m: height_km * 1_000.0,
        temperature_k,
        pressure_pa,
        density_kg_m3,
    }
}

/// The thermosphere at geometric heights. Scale height grows fivefold across
/// it, which no single exponential can follow, so each interval decays with
/// its own. The first row is the top of the layered model, so the two meet
/// without a step.
const THERMOSPHERE: [Node; 17] = [
    node(86.0, 186.945_9, 3.733_795e-1, 6.957_811e-6),
    node(90.0, 186.867, 1.843_5e-1, 3.440_0e-6),
    node(95.0, 188.418, 7.577_5e-2, 1.387_3e-6),
    node(100.0, 195.081, 3.201_2e-2, 5.604_4e-7),
    node(105.0, 208.835, 1.442_3e-2, 2.332_5e-7),
    node(110.0, 240.0, 7.149_3e-3, 9.673_4e-8),
    node(115.0, 300.0, 4.003_7e-3, 4.279_4e-8),
    node(120.0, 360.0, 2.536_6e-3, 2.219_9e-8),
    node(125.0, 417.231, 1.736_0e-3, 1.291_8e-8),
    node(130.0, 469.268, 1.250_3e-3, 8.149_4e-9),
    node(140.0, 559.627, 7.202_9e-4, 3.831_3e-9),
    node(150.0, 634.392, 4.542_2e-4, 2.075_2e-9),
    node(160.0, 696.290, 3.039_4e-4, 1.233_6e-9),
    node(170.0, 747.566, 2.121_0e-4, 7.815_5e-10),
    node(180.0, 790.066, 1.527_2e-4, 5.194_0e-10),
    node(190.0, 825.312, 1.126_5e-4, 3.580_7e-10),
    node(200.0, 854.559, 8.473_6e-5, 2.540_7e-10),
];

/// A still atmosphere from sea level to space, for flight that crosses all of
/// it: the 1976 U.S. Standard Atmosphere to 86 km, its thermosphere table to
/// 180 km, then a fade reaching [`VacuumAtmosphere`] at 200 km and staying
/// there.
///
/// Ellipsoidal height is used as height above MSL. Inside the fade the fields
/// no longer satisfy the gas law; they only keep every quantity continuous, at
/// a density already four decades below the 86 km value.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ReentryAtmosphere;

impl AtmosphereField for ReentryAtmosphere {
    fn sample(
        &self,
        position: &Geodetic,
        _utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let height_m = position.height_m;
        if height_m >= VACUUM_FLOOR_M {
            return Ok(VacuumAtmosphere::SAMPLE);
        }
        if height_m >= LAYERS_TOP_M {
            return Ok(thermosphere_sample(height_m));
        }
        let state = ussa76_at_geopotential(geometric_to_geopotential_m(height_m))?;
        Ok(AtmosphereSample {
            temperature_k: state.temperature_k,
            pressure_pa: state.pressure_pa,
            density_kg_m3: state.density_kg_m3,
            speed_of_sound_m_s: state.speed_of_sound_m_s,
            wind_ned_m_s: DVec3::ZERO,
        })
    }
}

/// Interpolates within the table: pressure and density decay exponentially
/// between nodes and temperature runs linearly, then the fade applies.
fn thermosphere_sample(height_m: f64) -> AtmosphereSample {
    let interval = THERMOSPHERE
        .windows(2)
        .find(|pair| matches!(pair, [_, upper] if height_m < upper.height_m));
    let Some([lower, upper]) = interval else {
        return VacuumAtmosphere::SAMPLE;
    };
    let fraction = (height_m - lower.height_m) / (upper.height_m - lower.height_m);
    let temperature_k =
        lower.temperature_k + fraction * (upper.temperature_k - lower.temperature_k);
    let decay = |low: f64, high: f64| low * (high / low).powf(fraction);
    let remaining = fade_to_vacuum(height_m);
    AtmosphereSample {
        temperature_k: COSMIC_BACKGROUND_K + remaining * (temperature_k - COSMIC_BACKGROUND_K),
        pressure_pa: remaining * decay(lower.pressure_pa, upper.pressure_pa),
        density_kg_m3: remaining * decay(lower.density_kg_m3, upper.density_kg_m3),
        speed_of_sound_m_s: remaining * speed_of_sound_m_s(temperature_k),
        wind_ned_m_s: DVec3::ZERO,
    }
}

/// Smoothstep from one at [`FADE_START_M`] to zero at [`VACUUM_FLOOR_M`], with
/// zero slope at both ends.
fn fade_to_vacuum(height_m: f64) -> f64 {
    let left = ((VACUUM_FLOOR_M - height_m) / (VACUUM_FLOOR_M - FADE_START_M)).clamp(0.0, 1.0);
    left * left * (3.0 - 2.0 * left)
}
