//! Continuous Mars $\text{CO}_2$ standard atmosphere from `-9 km` (Hellas basin) to space (`200 km`).

use glam::DVec3;
use sim_geo::{Geodetic, UtcTime};

use crate::{
    AtmosphereError, AtmosphereField, AtmosphereSample, COSMIC_BACKGROUND_K, VacuumAtmosphere,
};

#[cfg(test)]
mod tests;

/// Specific gas constant of Martian air (`95.32% CO2, 2.7% N2, 1.6% Ar`), J/(kg·K).
pub const MARS_GAS_CONSTANT_J_KG_K: f64 = 191.84;

/// Ratio of specific heats $\gamma = c_p / c_v$ for Martian $\text{CO}_2$-dominated air.
pub const MARS_GAMMA: f64 = 1.294;

/// Standard Martian surface gravity at datum, m/s².
const MARS_G0_M_S2: f64 = 3.720_76;

/// Volumetric mean radius of Mars used for geopotential height conversion, metres.
const MARS_GEOPOTENTIAL_RADIUS_M: f64 = 3_389_500.0;

/// Lowest supported geopotential height (below Hellas Planitia `-8,200 m`), metres.
const MARS_FLOOR_GEOPOTENTIAL_M: f64 = -9_000.0;

/// Geometric height where hydrostatic layers hand over to upper-atmosphere nodes, metres.
const MARS_LAYERS_TOP_M: f64 = 110_000.0;

/// Geometric height where smoothstep fade to vacuum starts, metres.
const MARS_FADE_START_M: f64 = 180_000.0;

/// Geometric height at and above which sample equals [`VacuumAtmosphere`], metres.
const MARS_VACUUM_FLOOR_M: f64 = 200_000.0;

/// Reference datum (`0 m`) temperature, kelvin.
const T_DATUM_K: f64 = 214.0;
/// Reference datum (`0 m`) static pressure, pascals.
const P_DATUM_PA: f64 = 610.0;

struct HydroLayer {
    base_h_m: f64,
    top_h_m: f64,
    lapse_k_per_m: f64,
}

const UPPER_LAYERS: [HydroLayer; 4] = [
    HydroLayer {
        base_h_m: 0.0,
        top_h_m: 7_000.0,
        lapse_k_per_m: -0.000_998,
    },
    HydroLayer {
        base_h_m: 7_000.0,
        top_h_m: 40_000.0,
        lapse_k_per_m: -0.001_45,
    },
    HydroLayer {
        base_h_m: 40_000.0,
        top_h_m: 70_000.0,
        lapse_k_per_m: 0.0,
    },
    HydroLayer {
        base_h_m: 70_000.0,
        top_h_m: 120_000.0,
        lapse_k_per_m: -0.000_75,
    },
];

const BASIN_LAPSE_K_PER_M: f64 = -0.002_22;

/// Continuous Mars atmosphere from `-9 km` below the `MARS_IAU` spheroid up to deep space.
///
/// Hydrostatic $\text{CO}_2$ layers govern `-9 km .. 110 km`, an exponential thermosphere
/// extends `110 km .. 180 km`, and a $C^1$ smoothstep fades cleanly into [`VacuumAtmosphere`]
/// at `200 km` so orbital aerobraking and entry integrators see zero discontinuities.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MarsAtmosphere {
    wind_ned_m_s: DVec3,
}

impl MarsAtmosphere {
    /// Still Martian air.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            wind_ned_m_s: DVec3::ZERO,
        }
    }

    /// Martian air moving with uniform `wind_ned_m_s` (toward, north-east-down, m/s).
    #[must_use]
    pub const fn with_wind(wind_ned_m_s: DVec3) -> Self {
        Self { wind_ned_m_s }
    }
}

impl AtmosphereField for MarsAtmosphere {
    fn sample(
        &self,
        position: &Geodetic,
        _utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let geom_m = position.height_m;
        if geom_m >= MARS_VACUUM_FLOOR_M {
            return Ok(VacuumAtmosphere::SAMPLE);
        }
        if geom_m >= MARS_LAYERS_TOP_M {
            return Ok(mars_thermosphere_sample(geom_m, self.wind_ned_m_s));
        }
        let h_gp = mars_geometric_to_geopotential_m(geom_m);
        if h_gp < MARS_FLOOR_GEOPOTENTIAL_M {
            return Err(AtmosphereError::OutOfRange {
                geopotential_m: h_gp,
                floor_m: MARS_FLOOR_GEOPOTENTIAL_M,
                ceiling_m: MARS_VACUUM_FLOOR_M,
            });
        }
        let (t_k, p_pa) = hydrostatic_tp_at_geopotential(h_gp);
        let rho = p_pa / (MARS_GAS_CONSTANT_J_KG_K * t_k);
        Ok(AtmosphereSample {
            temperature_k: t_k,
            pressure_pa: p_pa,
            density_kg_m3: rho,
            speed_of_sound_m_s: mars_speed_of_sound_m_s(t_k),
            wind_ned_m_s: self.wind_ned_m_s,
        })
    }
}

/// Converts Martian geometric altitude above the reference spheroid to geopotential metres.
#[must_use]
pub fn mars_geometric_to_geopotential_m(geometric_m: f64) -> f64 {
    MARS_GEOPOTENTIAL_RADIUS_M * geometric_m / (MARS_GEOPOTENTIAL_RADIUS_M + geometric_m)
}

fn mars_speed_of_sound_m_s(temperature_k: f64) -> f64 {
    (MARS_GAMMA * MARS_GAS_CONSTANT_J_KG_K * temperature_k.max(0.0)).sqrt()
}

fn advance_layer(t_base: f64, p_base: f64, dh: f64, lapse: f64) -> (f64, f64) {
    if lapse.abs() < 1.0e-12 {
        let p = p_base * (-MARS_G0_M_S2 * dh / (MARS_GAS_CONSTANT_J_KG_K * t_base)).exp();
        (t_base, p)
    } else {
        let t = t_base + lapse * dh;
        let exponent = -MARS_G0_M_S2 / (MARS_GAS_CONSTANT_J_KG_K * lapse);
        let p = p_base * (t / t_base).powf(exponent);
        (t, p)
    }
}

fn hydrostatic_tp_at_geopotential(h_gp_m: f64) -> (f64, f64) {
    if h_gp_m < 0.0 {
        return advance_layer(T_DATUM_K, P_DATUM_PA, h_gp_m, BASIN_LAPSE_K_PER_M);
    }
    let mut t_cur = T_DATUM_K;
    let mut p_cur = P_DATUM_PA;
    for layer in &UPPER_LAYERS {
        if h_gp_m <= layer.top_h_m {
            return advance_layer(t_cur, p_cur, h_gp_m - layer.base_h_m, layer.lapse_k_per_m);
        }
        let (t_top, p_top) = advance_layer(
            t_cur,
            p_cur,
            layer.top_h_m - layer.base_h_m,
            layer.lapse_k_per_m,
        );
        t_cur = t_top;
        p_cur = p_top;
    }
    (t_cur, p_cur)
}

fn mars_thermosphere_sample(geom_m: f64, wind_ned_m_s: DVec3) -> AtmosphereSample {
    let h_top_gp = mars_geometric_to_geopotential_m(MARS_LAYERS_TOP_M);
    let (t_110, p_110) = hydrostatic_tp_at_geopotential(h_top_gp);
    let dz = geom_m - MARS_LAYERS_TOP_M;
    // Warm smoothly toward a 195 K exobase while decaying with local CO2/O scale height (~9.5 km -> 13 km).
    let t_exo = 195.0;
    let t_k = t_exo - (t_exo - t_110) * (-dz / 28_000.0).exp();
    let mean_t = 0.5 * (t_110 + t_k);
    let g_local = MARS_G0_M_S2
        * (MARS_GEOPOTENTIAL_RADIUS_M
            / (MARS_GEOPOTENTIAL_RADIUS_M + 0.5 * (MARS_LAYERS_TOP_M + geom_m)))
            .powi(2);
    let scale_h = MARS_GAS_CONSTANT_J_KG_K * mean_t / g_local;
    let p_raw = p_110 * (-dz / scale_h).exp();
    let rho_raw = p_raw / (MARS_GAS_CONSTANT_J_KG_K * t_k);

    let rem = mars_fade_to_vacuum(geom_m);
    AtmosphereSample {
        temperature_k: COSMIC_BACKGROUND_K + rem * (t_k - COSMIC_BACKGROUND_K),
        pressure_pa: rem * p_raw,
        density_kg_m3: rem * rho_raw,
        speed_of_sound_m_s: rem * mars_speed_of_sound_m_s(t_k),
        wind_ned_m_s: rem * wind_ned_m_s,
    }
}

fn mars_fade_to_vacuum(geom_m: f64) -> f64 {
    let left = ((MARS_VACUUM_FLOOR_M - geom_m) / (MARS_VACUUM_FLOOR_M - MARS_FADE_START_M))
        .clamp(0.0, 1.0);
    left * left * (3.0 - 2.0 * left)
}
