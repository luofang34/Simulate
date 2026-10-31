use crate::AtmosphereError;

#[cfg(test)]
mod tests;

/// Specific gas constant for dry air, J/(kg·K) (ICAO Doc 7488).
const R_AIR: f64 = 287.052_87;
/// Standard gravity, m/s².
const G0: f64 = 9.806_65;
/// Ratio of specific heats for dry air.
const GAMMA: f64 = 1.4;
/// Earth radius used to convert geometric to geopotential height, metres.
const GEOPOTENTIAL_RADIUS_M: f64 = 6_356_766.0;

/// Lowest geopotential height the models accept, metres.
const FLOOR_M: f64 = -5_000.0;
/// Top of the ICAO standard atmosphere, metres.
const ICAO_CEILING_M: f64 = 32_000.0;
/// Geopotential height of the 86 km geometric top of the 1976 U.S. Standard
/// Atmosphere's layers, metres.
const USSA76_CEILING_M: f64 = 84_852.05;

/// One layer: base height, base temperature, base pressure, lapse rate.
struct Layer {
    base_m: f64,
    base_temperature_k: f64,
    base_pressure_pa: f64,
    lapse_k_per_m: f64,
}

/// The 1976 U.S. Standard Atmosphere, which agrees with ICAO up to 32 km.
const LAYERS: [Layer; 7] = [
    Layer {
        base_m: 0.0,
        base_temperature_k: 288.15,
        base_pressure_pa: 101_325.0,
        lapse_k_per_m: -0.006_5,
    },
    Layer {
        base_m: 11_000.0,
        base_temperature_k: 216.65,
        base_pressure_pa: 22_632.06,
        lapse_k_per_m: 0.0,
    },
    Layer {
        base_m: 20_000.0,
        base_temperature_k: 216.65,
        base_pressure_pa: 5_474.889,
        lapse_k_per_m: 0.001,
    },
    Layer {
        base_m: 32_000.0,
        base_temperature_k: 228.65,
        base_pressure_pa: 868.018_7,
        lapse_k_per_m: 0.002_8,
    },
    Layer {
        base_m: 47_000.0,
        base_temperature_k: 270.65,
        base_pressure_pa: 110.906_3,
        lapse_k_per_m: 0.0,
    },
    Layer {
        base_m: 51_000.0,
        base_temperature_k: 270.65,
        base_pressure_pa: 66.938_87,
        lapse_k_per_m: -0.002_8,
    },
    Layer {
        base_m: 71_000.0,
        base_temperature_k: 214.65,
        base_pressure_pa: 3.956_42,
        lapse_k_per_m: -0.002,
    },
];

/// Standard-atmosphere state at one height.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IsaState {
    /// Static air temperature, kelvin.
    pub temperature_k: f64,
    /// Static pressure, pascals.
    pub pressure_pa: f64,
    /// Air density, kg/m³.
    pub density_kg_m3: f64,
    /// Speed of sound, m/s.
    pub speed_of_sound_m_s: f64,
}

/// Converts a geometric height above MSL to geopotential height.
pub fn geometric_to_geopotential_m(geometric_m: f64) -> f64 {
    GEOPOTENTIAL_RADIUS_M * geometric_m / (GEOPOTENTIAL_RADIUS_M + geometric_m)
}

/// Speed of sound in dry air at `temperature_k`, m/s.
pub(crate) fn speed_of_sound_m_s(temperature_k: f64) -> f64 {
    (GAMMA * R_AIR * temperature_k).sqrt()
}

/// The ICAO standard atmosphere at `geopotential_m`, valid from −5 km to 32 km;
/// the troposphere lapse rate is extrapolated below sea level.
pub fn isa_at_geopotential(geopotential_m: f64) -> Result<IsaState, AtmosphereError> {
    state_below_ceiling(geopotential_m, ICAO_CEILING_M)
}

/// The 1976 U.S. Standard Atmosphere at `geopotential_m`, valid from −5 km to
/// 86 km geometric, with the same extrapolation below sea level.
pub(crate) fn ussa76_at_geopotential(geopotential_m: f64) -> Result<IsaState, AtmosphereError> {
    state_below_ceiling(geopotential_m, USSA76_CEILING_M)
}

fn state_below_ceiling(geopotential_m: f64, ceiling_m: f64) -> Result<IsaState, AtmosphereError> {
    if !(FLOOR_M..=ceiling_m).contains(&geopotential_m) {
        return Err(AtmosphereError::OutOfRange {
            geopotential_m,
            floor_m: FLOOR_M,
            ceiling_m,
        });
    }
    let layer = LAYERS
        .iter()
        .rev()
        .find(|layer| geopotential_m >= layer.base_m)
        .unwrap_or(&LAYERS[0]);
    let dh = geopotential_m - layer.base_m;
    let temperature_k = layer.base_temperature_k + layer.lapse_k_per_m * dh;
    let pressure_pa = if layer.lapse_k_per_m == 0.0 {
        layer.base_pressure_pa * (-G0 * dh / (R_AIR * layer.base_temperature_k)).exp()
    } else {
        let exponent = -G0 / (layer.lapse_k_per_m * R_AIR);
        layer.base_pressure_pa * (temperature_k / layer.base_temperature_k).powf(exponent)
    };
    Ok(IsaState {
        temperature_k,
        pressure_pa,
        density_kg_m3: pressure_pa / (R_AIR * temperature_k),
        speed_of_sound_m_s: speed_of_sound_m_s(temperature_k),
    })
}
