use sim_core::StampedReading;

#[cfg(test)]
mod tests;

/// ISA sea-level density, kg/m³.
const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;
/// ISA sea-level pressure, Pa.
const SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;
/// ISA sea-level temperature, K.
const SEA_LEVEL_TEMPERATURE_K: f64 = 288.15;
/// ISA sea-level pressure, hPa.
const STANDARD_SETTING_HPA: f64 = 1_013.25;
/// ISA tropospheric lapse rate, K/m.
const LAPSE_RATE_K_M: f64 = 0.0065;
/// `R L / g0` in the ISA barometric formula.
const PRESSURE_EXPONENT: f64 = 0.190_263;
/// ISA pressure at the tropopause, Pa.
const TROPOPAUSE_PRESSURE_PA: f64 = 22_632.1;

/// What a pitot-static system and its air-data computer sense.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirDataSample {
    /// True airspeed, m/s.
    pub true_airspeed_m_s: f64,
    /// Ambient density, kg/m³.
    pub density_kg_m3: f64,
    /// Static pressure at the port, Pa, from a sensor such as the BMP390.
    pub static_pressure_pa: Option<f64>,
    /// Altimeter setting the source applies, hPa.
    pub altimeter_setting_hpa: Option<f64>,
}

impl AirDataSample {
    /// Equivalent airspeed, m/s: the compressibility of a real pitot is not
    /// modelled, so this reads low of indicated above about 100 m/s at height.
    pub fn indicated_airspeed_m_s(&self) -> f64 {
        self.true_airspeed_m_s * (self.density_kg_m3 / SEA_LEVEL_DENSITY_KG_M3).sqrt()
    }
}

/// An air-data sample with its time and health.
pub type AirDataReading = StampedReading<AirDataSample>;

/// ISA pressure altitude of `pressure_pa`, metres, or `None` outside the
/// troposphere's formula. A BMP390's 30 kPa floor sits inside it.
pub fn pressure_altitude_m(pressure_pa: f64) -> Option<f64> {
    let in_range = pressure_pa.is_finite() && pressure_pa >= TROPOPAUSE_PRESSURE_PA;
    in_range.then(|| {
        let ratio = pressure_pa / SEA_LEVEL_PRESSURE_PA;
        SEA_LEVEL_TEMPERATURE_K / LAPSE_RATE_K_M * (1.0 - ratio.powf(PRESSURE_EXPONENT))
    })
}

/// The altitude an altimeter set to `setting_hpa` shows at `pressure_pa`,
/// metres: the setting shifts the whole pressure scale, as a Kollsman window
/// does, so a setting equal to the field's pressure reads zero there.
pub fn indicated_altitude_m(pressure_pa: f64, setting_hpa: f64) -> Option<f64> {
    (setting_hpa.is_finite() && setting_hpa > 0.0)
        .then(|| pressure_altitude_m(pressure_pa * STANDARD_SETTING_HPA / setting_hpa))
        .flatten()
}
