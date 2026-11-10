//! The state a panel's mechanisms feed its dials: what every gauge shows,
//! as Indicate's input state and the engine and tank records beside it.

use indicate_instrument_state::{
    AirData, AircraftState, AltitudeClass, AltitudeDeclaration, BesideSamples, ClockSample,
    EngineGroup, EngineSample, EngineSamples, EstimateQuality, FreshnessPolicy, PanelData, Stamped,
    TankGroup, TankSample, TankSamples, ValidFlags, resolve,
};

/// One engine's gauge readings, SI; `None` where the engine has no such
/// gauge or it shows nothing.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EngineGauges {
    /// Manifold absolute pressure, Pa.
    pub manifold_pressure_pa: Option<f64>,
    /// Engine speed, rpm.
    pub rpm: Option<f64>,
    /// Oil temperature, K.
    pub oil_temperature_k: Option<f64>,
    /// Oil pressure above ambient, Pa.
    pub oil_pressure_pa: Option<f64>,
    /// Fuel pressure above ambient, Pa.
    pub fuel_pressure_pa: Option<f64>,
    /// Coolant temperature, K.
    pub coolant_temperature_k: Option<f64>,
    /// The engine's shaft angle in its own direction of rotation, degrees
    /// in `[0, 360)`, which a synchroscope compares between engines.
    pub shaft_angle_deg: Option<f64>,
}

/// What a panel's mechanisms show this instant, in SI: the values under
/// their needles after lag and instrument error, which the dials paint
/// without filtering again. `None` is an instrument with nothing to show.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GaugeIndications {
    /// The airspeed indicator, m/s.
    pub airspeed_m_s: Option<f64>,
    /// The rate-of-climb indicator, m/s.
    pub vertical_speed_m_s: Option<f64>,
    /// The altimeter, m above its setting's datum.
    pub altitude_m: Option<f64>,
    /// The altimeter's Kollsman setting, hPa.
    pub altimeter_setting_hpa: Option<f64>,
    /// Engine gauges in panel order, engine 1 first, one per engine.
    pub engines: Vec<EngineGauges>,
    /// Fuel-quantity gauges, one per tank in the aircraft's tank order, m³.
    pub tanks_m3: Vec<Option<f64>>,
    /// The flight state the gyro, turn and compass instruments read (attitude,
    /// rates, heading, magnetic variation, lateral force), as the avionics
    /// feed builds it; the gauges' own groups replace its air, altitude,
    /// engine, tank and clock groups.
    pub flight: Option<AircraftState>,
    /// The time of day the cockpit clock shows, s after midnight.
    pub clock_time_of_day_s: Option<f64>,
}

/// Indicate's state for a panel's gauges and the records beside it.
#[derive(Debug, Clone, PartialEq)]
pub struct GaugeState {
    /// The resolved state.
    pub data: PanelData,
    engines: Vec<EngineSample>,
    tanks: Vec<TankSample>,
}

impl GaugeState {
    /// The engine and tank records, as Indicate's dials read them.
    #[must_use]
    pub fn beside(&self) -> BesideSamples<'_> {
        BesideSamples {
            engines: EngineSamples::Slice(&self.engines),
            tanks: TankSamples::Slice(&self.tanks),
        }
    }
}

fn kelvin_to_c(k: f64) -> f32 {
    (k - 273.15) as f32
}

impl GaugeIndications {
    /// The gauges as Indicate's input state: air data, a barometric
    /// indicated altitude, and the engine and fuel tank group headers, each
    /// fresh this instant.
    #[must_use]
    pub fn state(&self) -> AircraftState {
        let fresh = |present: bool| if present { Some(0.0) } else { None };
        let air = AirData {
            ias_mps: self.airspeed_m_s.map(|v| v as f32),
            baro_setting_hpa: self.altimeter_setting_hpa.map(|v| v as f32),
            tas_mps: None,
            vsi_mps: self.vertical_speed_m_s.map(|v| v as f32),
        };
        let has_air =
            air.ias_mps.is_some() || air.vsi_mps.is_some() || air.baro_setting_hpa.is_some();
        let group = |count: usize| Stamped {
            data: (count > 0).then_some(EngineGroup {
                count: u16::try_from(count).unwrap_or(0),
                fault: None,
            }),
            age_ms: fresh(count > 0),
        };
        let tanks: Stamped<TankGroup> = group(self.tanks_m3.len());
        let base = self.flight.unwrap_or_default();
        let clock = self.clock_time_of_day_s.map(|s| ClockSample {
            time_of_day_s: s as f32,
        });
        AircraftState {
            clock: Stamped {
                age_ms: fresh(clock.is_some()),
                data: clock,
            },
            air: Stamped {
                data: has_air.then_some(air),
                age_ms: fresh(has_air),
            },
            altitude: AltitudeDeclaration {
                reference_class: AltitudeClass::BaroIndicated,
                sample_m: self.altitude_m.map(|m| m as f32),
                ..AltitudeDeclaration::default()
            },
            engine: group(self.engines.len()),
            fuel_tanks: tanks,
            quality: if self.flight.is_some() {
                base.quality
            } else {
                EstimateQuality::Good
            },
            valid: if self.flight.is_some() {
                base.valid
            } else {
                ValidFlags::default()
            },
            ..base
        }
    }

    /// The engine feed: one Indicate record per engine, beside the engine
    /// group's header. Every engine quantity Indicate's state carries goes
    /// through here and nowhere else.
    #[must_use]
    pub fn engine_samples(&self) -> Vec<EngineSample> {
        self.engines
            .iter()
            .map(|g| EngineSample {
                manifold_pressure_hpa: g.manifold_pressure_pa.map(|pa| (pa / 100.0) as f32),
                rpm: g.rpm.map(|rpm| rpm as f32),
                oil_temperature_c: g.oil_temperature_k.map(kelvin_to_c),
                oil_pressure_kpa: g.oil_pressure_pa.map(|pa| (pa / 1_000.0) as f32),
                fuel_pressure_kpa: g.fuel_pressure_pa.map(|pa| (pa / 1_000.0) as f32),
                coolant_temperature_c: g.coolant_temperature_k.map(kelvin_to_c),
                shaft_angle_deg: g.shaft_angle_deg.map(|deg| deg as f32),
                ..EngineSample::EMPTY
            })
            .collect()
    }

    /// The tank feed: one record per tank with the gauge's quantity, litres.
    #[must_use]
    pub fn tank_samples(&self) -> Vec<TankSample> {
        self.tanks_m3
            .iter()
            .map(|m3| TankSample {
                quantity_l: m3.map(|m3| (m3 * 1_000.0) as f32),
                ..TankSample::default()
            })
            .collect()
    }

    /// [`Self::state`] resolved as a panel reads it, with its records.
    #[must_use]
    pub fn resolve(&self) -> GaugeState {
        GaugeState {
            data: resolve(&self.state(), &FreshnessPolicy::default()),
            engines: self.engine_samples(),
            tanks: self.tank_samples(),
        }
    }
}
