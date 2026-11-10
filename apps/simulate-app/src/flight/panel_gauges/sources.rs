//! What each gauge is plumbed or wired to, the readings the aircraft gives
//! it, and where its indication goes in Indicate's state.

use mbd_airdata::impact_pressure_from_cas_pa;
use sim_core::{AirframeArticulation, EngineReadout};
use sim_indicate::{AirDataSample, AircraftState, GaugeIndications};
use syspack::{InstrumentMechanism, SourceBinding};

/// One pound-force per square inch, Pa.
const PSI_PA: f64 = 6_894.757_293_168;
/// Avgas, lb per US gallon.
const FUEL_LB_PER_GAL: f64 = 6.0;
/// One US gallon, m³.
const US_GALLON_M3: f64 = 0.003_785_411_784;

/// What a gauge reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Source {
    PitotStatic,
    ManifoldPressure(usize),
    Rpm(usize),
    OilTemperature(usize),
    OilPressure(usize),
    FuelTank(usize),
    /// A quantity the aircraft does not publish, such as coolant
    /// temperature, fuel pressure, suction or hydraulic pressure.
    Unpublished,
}

/// The fuel tanks a `system.fuel.<tank>` source may name, in the flight
/// model's tank order: the P-38G's main tanks forward of the front spar,
/// then its reserves aft.
pub(super) const TANKS: [&str; sim_core::FUEL_TANKS] =
    ["front_left", "front_right", "rear_left", "rear_right"];

fn fuel_tank(path: &str) -> Option<usize> {
    let tank = path.strip_prefix("fuel.")?;
    TANKS.iter().position(|t| *t == tank)
}

impl Source {
    pub(super) fn of(binding: SourceBinding) -> Self {
        match binding {
            SourceBinding::PitotStatic { .. } => Self::PitotStatic,
            SourceBinding::Engine { index, quantity } => {
                let engine = usize::from(index);
                match quantity.as_str() {
                    "manifold_pressure" => Self::ManifoldPressure(engine),
                    "rpm" => Self::Rpm(engine),
                    "oil_temperature" => Self::OilTemperature(engine),
                    "oil_pressure" => Self::OilPressure(engine),
                    // Fuel pressure and coolant temperature are not modelled.
                    _ => Self::Unpublished,
                }
            }
            SourceBinding::System { path } => {
                fuel_tank(&path).map_or(Self::Unpublished, Self::FuelTank)
            }
        }
    }
}

/// One engine's readings, SI.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineInputs {
    /// Manifold pressure, Pa.
    pub manifold_pressure_pa: f64,
    /// Crankshaft speed, rpm.
    pub rpm: f64,
    /// Oil temperature, K.
    pub oil_temperature_k: f64,
    /// Oil pressure above ambient, Pa.
    pub oil_pressure_pa: f64,
    /// The propeller shaft's angle in its own direction of rotation,
    /// degrees in `[0, 360)`, for a synchroscope.
    pub shaft_angle_deg: Option<f64>,
}

impl EngineInputs {
    fn of(readout: &EngineReadout) -> Self {
        Self {
            manifold_pressure_pa: readout.manifold_pressure_pa,
            rpm: readout.rpm,
            oil_temperature_k: (readout.oil_temp_deg_f - 32.0) * 5.0 / 9.0 + 273.15,
            oil_pressure_pa: readout.oil_pressure_psi * PSI_PA,
            shaft_angle_deg: None,
        }
    }
}

/// What the gauges read this step.
#[derive(Debug, Clone, PartialEq)]
pub struct GaugeInputs {
    /// Pitot total and sensed static pressure `[total, static]`, Pa; `None`
    /// where there is no air to sense.
    pub pitot_static_pa: Option<[f64; 2]>,
    /// The altimeter setting dialled into the Kollsman window, hPa: the one
    /// the head-up display's altitude also uses.
    pub altimeter_setting_hpa: Option<f64>,
    /// Each engine's readings, left first.
    pub engines: Vec<EngineInputs>,
    /// Fuel in each tank, m³; `None` past the last.
    pub fuel_tanks_m3: [Option<f64>; sim_core::FUEL_TANKS],
    /// The flight state the gyro, turn and compass instruments read, from
    /// the avionics feed.
    pub flight: Option<AircraftState>,
    /// The time of day the cockpit clock shows, s after midnight.
    pub clock_time_of_day_s: Option<f64>,
}

impl GaugeInputs {
    /// Readings from the sensed air data and the plant's engines. The total head
    /// reads true while the static holes sense the position error, so the
    /// airspeed indicator shows the same indicated airspeed as the head-up display.
    pub fn sense(air: Option<&AirDataSample>, art: &AirframeArticulation) -> Self {
        let pitot_static_pa = air.and_then(|air| {
            let static_pa = air.static_pressure_pa?;
            let total_pa = static_pa + impact_pressure_from_cas_pa(air.calibrated_airspeed_m_s());
            let sensed = total_pa - impact_pressure_from_cas_pa(air.indicated_airspeed_m_s());
            Some([total_pa, sensed])
        });
        // Each shaft's integrated phase runs with its rotation sense, so phase
        // times sense is the angle it has turned in its own direction.
        let shafts = [
            (art.propeller_phase_rad, art.propeller_sense),
            (art.propeller_right.phase_rad, art.propeller_right.sense),
        ];
        let engines = art
            .gauges
            .engines
            .iter()
            .zip(shafts)
            .map(|(readout, (phase, sense))| EngineInputs {
                shaft_angle_deg: Some(
                    (phase * sense)
                        .rem_euclid(std::f64::consts::TAU)
                        .to_degrees(),
                ),
                ..EngineInputs::of(readout)
            })
            .collect();
        Self {
            pitot_static_pa,
            altimeter_setting_hpa: air.and_then(|air| air.altimeter_setting_hpa),
            engines,
            fuel_tanks_m3: art
                .gauges
                .fuel_tanks_lbs
                .map(|lbs| lbs.map(|lbs| lbs / FUEL_LB_PER_GAL * US_GALLON_M3)),
            flight: None,
            clock_time_of_day_s: None,
        }
    }

    /// The SI reading `source` gives, other than the pitot-static system's.
    fn reading(&self, source: Source) -> Option<f64> {
        let engine = |e: usize| self.engines.get(e);
        match source {
            Source::ManifoldPressure(e) => engine(e).map(|e| e.manifold_pressure_pa),
            Source::Rpm(e) => engine(e).map(|e| e.rpm),
            Source::OilTemperature(e) => engine(e).map(|e| e.oil_temperature_k),
            Source::OilPressure(e) => engine(e).map(|e| e.oil_pressure_pa),
            Source::FuelTank(t) => self.fuel_tanks_m3.get(t).copied().flatten(),
            Source::PitotStatic | Source::Unpublished => None,
        }
    }
}

/// Settles `mechanism` on `inputs`, as after a jump in time.
pub(super) fn settle(mechanism: &mut InstrumentMechanism, source: Source, inputs: &GaugeInputs) {
    if let Some(hpa) = inputs.altimeter_setting_hpa
        && let InstrumentMechanism::Altimeter(alt) = mechanism
    {
        alt.set_kollsman(hpa * 100.0);
    }
    if let (Some([total, sensed]), Source::PitotStatic) = (inputs.pitot_static_pa, source) {
        match mechanism {
            InstrumentMechanism::Airspeed(asi) => asi.reset(total, sensed),
            InstrumentMechanism::Altimeter(alt) => alt.reset(sensed),
            InstrumentMechanism::VerticalSpeed(vsi) => vsi.reset(sensed),
            _ => {}
        }
        return;
    }
    let Some(si) = inputs.reading(source) else {
        return;
    };
    match mechanism {
        InstrumentMechanism::PressureGauge(g) => g.reset(si),
        InstrumentMechanism::Tachometer(t) => t.reset(si),
        InstrumentMechanism::Thermometer(t) => t.reset(si),
        InstrumentMechanism::FuelQuantity(f) => f.reset(si),
        _ => {}
    }
}

/// Steps `mechanism` and returns what its dial shows, dial units; `None`
/// when its source gives nothing.
pub(super) fn step(
    mechanism: &mut InstrumentMechanism,
    source: Source,
    dt_s: f64,
    inputs: &GaugeInputs,
) -> Option<f64> {
    if let Some(hpa) = inputs.altimeter_setting_hpa
        && let InstrumentMechanism::Altimeter(alt) = mechanism
    {
        alt.set_kollsman(hpa * 100.0);
    }
    if source == Source::PitotStatic {
        let [total, sensed] = inputs.pitot_static_pa?;
        return match mechanism {
            InstrumentMechanism::Airspeed(asi) => Some(asi.step(dt_s, total, sensed).indicated),
            InstrumentMechanism::Altimeter(alt) => Some(alt.step(dt_s, sensed).indicated),
            InstrumentMechanism::VerticalSpeed(vsi) => Some(vsi.step(dt_s, sensed).indicated),
            _ => None,
        };
    }
    let si = inputs.reading(source)?;
    Some(
        match mechanism {
            InstrumentMechanism::PressureGauge(g) => g.step(dt_s, si),
            InstrumentMechanism::Tachometer(t) => t.step(dt_s, si),
            InstrumentMechanism::Thermometer(t) => t.step(dt_s, si),
            InstrumentMechanism::FuelQuantity(f) => f.step(dt_s, si),
            _ => return None,
        }
        .indicated,
    )
}

/// Puts what a gauge shows, SI, in the lane of Indicate's state its source
/// feeds, if Indicate's state carries one.
pub(super) fn indicate(
    shown: &mut GaugeIndications,
    source: Source,
    si: f64,
    mechanism: &InstrumentMechanism,
) {
    match (mechanism, source) {
        (InstrumentMechanism::Airspeed(_), _) => shown.airspeed_m_s = Some(si),
        (InstrumentMechanism::VerticalSpeed(_), _) => shown.vertical_speed_m_s = Some(si),
        (InstrumentMechanism::Altimeter(alt), _) => {
            shown.altitude_m = Some(si);
            shown.altimeter_setting_hpa = Some(alt.kollsman_pa() / 100.0);
        }
        (InstrumentMechanism::PressureGauge(_), Source::ManifoldPressure(e)) => {
            if let Some(engine) = shown.engines.get_mut(e) {
                engine.manifold_pressure_pa = Some(si);
            }
        }
        (InstrumentMechanism::Tachometer(_), Source::Rpm(e)) => {
            if let Some(engine) = shown.engines.get_mut(e) {
                engine.rpm = Some(si);
            }
        }
        (InstrumentMechanism::Thermometer(_), Source::OilTemperature(e)) => {
            if let Some(engine) = shown.engines.get_mut(e) {
                engine.oil_temperature_k = Some(si);
            }
        }
        (InstrumentMechanism::PressureGauge(_), Source::OilPressure(e)) => {
            if let Some(engine) = shown.engines.get_mut(e) {
                engine.oil_pressure_pa = Some(si);
            }
        }
        (InstrumentMechanism::FuelQuantity(_), Source::FuelTank(t)) => {
            if let Some(tank) = shown.tanks_m3.get_mut(t) {
                *tank = Some(si);
            }
        }
        _ => {}
    }
}
