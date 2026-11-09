//! The flown aircraft's instrument-pack gauges as mechanisms: each pack
//! instrument's pressure capsules, aneroids, drag cups and second-order needle
//! (the pack's `NeedleSpec`) stepped on the readings it is plumbed to, with the
//! needle angles handed to the airframe model's panel.

use mbd_airdata::impact_pressure_from_cas_pa;
use sim_core::{AirframeArticulation, InstrumentNeedles};
use sim_indicate::AirDataSample;
use syspack::{InstrumentMechanism, InstrumentPackDocument, SourceBinding};
use tracing::warn;

#[cfg(test)]
mod tests;

/// What the gauges read this step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GaugeInputs {
    /// Pitot total and sensed static pressure `[total, static]`, Pa; `None`
    /// where there is no air to sense.
    pub pitot_static_pa: Option<[f64; 2]>,
    /// Manifold pressure per engine, Pa.
    pub manifold_pressure_pa: [f64; 2],
    /// Crankshaft speed per engine, rpm.
    pub engine_rpm: [f64; 2],
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
        let right = art.engine_right_gauges;
        Self {
            pitot_static_pa,
            manifold_pressure_pa: [art.manifold_pressure_pa, right.manifold_pressure_pa],
            engine_rpm: [art.engine_rpm, right.engine_rpm],
        }
    }
}

/// What a gauge is plumbed or wired to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    PitotStatic,
    ManifoldPressure(usize),
    Rpm(usize),
}

#[derive(Debug, Clone)]
struct Gauge {
    slot: usize,
    source: Source,
    mechanism: InstrumentMechanism,
}

/// The panel's gauges and their needles.
#[derive(Debug, Clone, Default)]
pub struct PanelGauges {
    gauges: Vec<Gauge>,
    needles: InstrumentNeedles,
}

fn source_of(binding: SourceBinding) -> Option<Source> {
    match binding {
        SourceBinding::PitotStatic { .. } => Some(Source::PitotStatic),
        SourceBinding::Engine { index, quantity } => {
            let engine = usize::from(index);
            match quantity.as_str() {
                "manifold_pressure" if engine < 2 => Some(Source::ManifoldPressure(engine)),
                "rpm" if engine < 2 => Some(Source::Rpm(engine)),
                _ => None,
            }
        }
        SourceBinding::System { .. } => None,
    }
}

impl PanelGauges {
    /// The gauges of `pack`'s layout; an instrument whose source the aircraft
    /// does not publish is left out and its needle stays at rest.
    pub fn for_pack(pack: Option<&InstrumentPackDocument>) -> Self {
        let Some(pack) = pack else {
            return Self::default();
        };
        let mut gauges = Vec::new();
        for (slot, place) in pack.layout.iter().enumerate() {
            let Some(spec) = pack.instrument(&place.instrument) else {
                continue;
            };
            let Some(source) = SourceBinding::parse(&spec.source).and_then(source_of) else {
                continue;
            };
            match spec.mechanism() {
                Ok(mechanism) => gauges.push(Gauge {
                    slot,
                    source,
                    mechanism,
                }),
                Err(err) => warn!(instrument = %spec.id, error = %err, "gauge has no mechanism"),
            }
        }
        Self {
            gauges,
            needles: InstrumentNeedles::default(),
        }
    }

    /// Settles every capsule and needle on `inputs`, as after a jump in time.
    pub fn settle(&mut self, inputs: &GaugeInputs) {
        for gauge in &mut self.gauges {
            gauge.settle(inputs);
        }
        self.step(0.0, inputs);
    }

    /// Advances every gauge `dt_s` on `inputs`.
    pub fn step(&mut self, dt_s: f64, inputs: &GaugeInputs) {
        for gauge in &mut self.gauges {
            gauge.step(dt_s, inputs, &mut self.needles);
        }
    }

    /// The needle angles, by `(layout slot, hand)`.
    #[must_use]
    pub const fn needles(&self) -> InstrumentNeedles {
        self.needles
    }
}

impl Gauge {
    fn settle(&mut self, inputs: &GaugeInputs) {
        let at = |pair: [f64; 2], i: usize| pair.get(i).copied().unwrap_or(0.0);
        match (&mut self.mechanism, self.source) {
            (InstrumentMechanism::Airspeed(asi), Source::PitotStatic) => {
                if let Some([total, sensed]) = inputs.pitot_static_pa {
                    asi.reset(total, sensed);
                }
            }
            (InstrumentMechanism::Altimeter(alt), Source::PitotStatic) => {
                if let Some([_, sensed]) = inputs.pitot_static_pa {
                    alt.reset(sensed);
                }
            }
            (InstrumentMechanism::VerticalSpeed(vsi), Source::PitotStatic) => {
                if let Some([_, sensed]) = inputs.pitot_static_pa {
                    vsi.reset(sensed);
                }
            }
            (InstrumentMechanism::PressureGauge(g), Source::ManifoldPressure(e)) => {
                g.reset(at(inputs.manifold_pressure_pa, e));
            }
            (InstrumentMechanism::Tachometer(t), Source::Rpm(e)) => {
                t.reset(at(inputs.engine_rpm, e));
            }
            _ => {}
        }
    }

    fn step(&mut self, dt_s: f64, inputs: &GaugeInputs, needles: &mut InstrumentNeedles) {
        let at = |pair: [f64; 2], i: usize| pair.get(i).copied().unwrap_or(0.0);
        let slot = self.slot;
        let angle = match (&mut self.mechanism, self.source) {
            (InstrumentMechanism::Airspeed(asi), Source::PitotStatic) => inputs
                .pitot_static_pa
                .map(|[total, sensed]| asi.step(dt_s, total, sensed).needle_angle_rad),
            (InstrumentMechanism::Altimeter(alt), Source::PitotStatic) => {
                if let Some([_, sensed]) = inputs.pitot_static_pa {
                    let r = alt.step(dt_s, sensed);
                    needles.set(slot, 1, r.thousands_hand_rad);
                    needles.set(slot, 2, r.ten_thousands_pointer_rad);
                    Some(r.hundreds_hand_rad)
                } else {
                    None
                }
            }
            (InstrumentMechanism::VerticalSpeed(vsi), Source::PitotStatic) => inputs
                .pitot_static_pa
                .map(|[_, sensed]| vsi.step(dt_s, sensed).needle_angle_rad),
            (InstrumentMechanism::PressureGauge(g), Source::ManifoldPressure(e)) => Some(
                g.step(dt_s, at(inputs.manifold_pressure_pa, e))
                    .needle_angle_rad,
            ),
            (InstrumentMechanism::Tachometer(t), Source::Rpm(e)) => {
                Some(t.step(dt_s, at(inputs.engine_rpm, e)).needle_angle_rad)
            }
            _ => None,
        };
        if let Some(angle) = angle {
            needles.set(slot, 0, angle);
        }
    }
}
