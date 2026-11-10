//! The flown aircraft's instrument-pack gauges as mechanisms and dials: each
//! pointer's capsule, aneroid, drag cup, bulb or float and its second-order
//! needle (the pack's `NeedleSpec`) stepped on the reading it is plumbed to.
//! What each mechanism then shows is fed to Indicate as the panel's state,
//! or handed to the dial directly where Indicate's state has no lane for it,
//! and Indicate's analog dials pose the pointers the airframe model draws, so
//! the mechanism owns lag and instrument error and Indicate owns the scale.

mod flight_state;
mod sources;

use mbd_airdata::DialUnit;
use sim_core::InstrumentNeedles;
use sim_indicate::{DialPose, DialSource, GaugeIndications, PanelDials};
use syspack::{InstrumentKindSpec, InstrumentMechanism, InstrumentPackDocument, SourceBinding};
use tracing::warn;

pub use sources::GaugeInputs;
use sources::Source;

#[cfg(test)]
mod tests;

#[derive(Debug, Clone)]
struct Gauge {
    slot: usize,
    pointer: usize,
    source: Source,
    unit: DialUnit,
    mechanism: InstrumentMechanism,
    /// What the dial shows, dial units; `None` with nothing to show.
    shown: Option<f64>,
}

/// The panel's gauges, the dials Indicate draws them on, and their pointers.
#[derive(Debug, Clone, Default)]
pub struct PanelGauges {
    gauges: Vec<Gauge>,
    dials: PanelDials,
    needles: InstrumentNeedles,
}

/// The unit a pointer's mechanism reads in; `None` for an instrument the
/// state feeds directly, which has no mechanism here.
fn dial_unit(kind: &InstrumentKindSpec) -> Option<DialUnit> {
    let units = match kind {
        InstrumentKindSpec::Airspeed { units, .. }
        | InstrumentKindSpec::Altimeter { units, .. }
        | InstrumentKindSpec::VerticalSpeed { units, .. }
        | InstrumentKindSpec::PressureGauge { units, .. }
        | InstrumentKindSpec::Tachometer { units, .. }
        | InstrumentKindSpec::Thermometer { units, .. }
        | InstrumentKindSpec::FuelQuantity { units, .. } => *units,
        InstrumentKindSpec::Direct { .. } => return None,
    };
    Some(DialUnit::from(units))
}

impl PanelGauges {
    /// The gauges of `pack`'s layout, one per pointer. A pointer whose
    /// source the aircraft does not publish keeps its mechanism at rest and
    /// its dial shows no value.
    pub fn for_pack(pack: Option<&InstrumentPackDocument>) -> Self {
        let Some(pack) = pack else {
            return Self::default();
        };
        let mut gauges = Vec::new();
        for (slot, place) in pack.layout.iter().enumerate() {
            let Some(spec) = pack.instrument(&place.instrument) else {
                continue;
            };
            let further = spec.pointers.iter().map(|p| (p.source.as_str(), &p.kind));
            let mechanisms = spec
                .mechanism()
                .and_then(|first| Ok((first, spec.pointer_mechanisms()?)));
            let (first, rest) = match mechanisms {
                Ok(built) => built,
                Err(err) => {
                    warn!(instrument = %spec.id, error = %err, "gauge has no mechanism");
                    continue;
                }
            };
            let pointers = std::iter::once((spec.source.as_str(), &spec.kind)).chain(further);
            for (pointer, ((source, kind), mechanism)) in
                pointers.zip(std::iter::once(first).chain(rest)).enumerate()
            {
                let Some(unit) = dial_unit(kind) else {
                    continue;
                };
                gauges.push(Gauge {
                    slot,
                    pointer,
                    source: SourceBinding::parse(source).map_or(Source::Unpublished, Source::of),
                    unit,
                    mechanism,
                    shown: None,
                });
            }
        }
        let (dials, refused) = PanelDials::from_pack(pack, &sources::TANKS);
        for err in refused {
            warn!(error = %err, "instrument has no dial");
        }
        Self {
            gauges,
            dials,
            needles: InstrumentNeedles::default(),
        }
    }

    /// Settles every capsule and needle on `inputs`, as after a jump in time.
    pub fn settle(&mut self, inputs: &GaugeInputs) {
        for gauge in &mut self.gauges {
            sources::settle(&mut gauge.mechanism, gauge.source, inputs);
        }
        self.step(0.0, inputs);
    }

    /// Advances every gauge `dt_s` on `inputs` and poses the dials on what
    /// the gauges then show.
    pub fn step(&mut self, dt_s: f64, inputs: &GaugeInputs) {
        let mut shown = GaugeIndications {
            engines: vec![Default::default(); inputs.engines.len()],
            tanks_m3: vec![None; inputs.fuel_tanks_m3.iter().flatten().count()],
            flight: inputs.flight,
            clock_time_of_day_s: inputs.clock_time_of_day_s,
            ..GaugeIndications::default()
        };
        // The synchroscope reads the shafts themselves, through no mechanism.
        for (shown, engine) in shown.engines.iter_mut().zip(&inputs.engines) {
            shown.shaft_angle_deg = engine.shaft_angle_deg;
        }
        for gauge in &mut self.gauges {
            gauge.shown = sources::step(&mut gauge.mechanism, gauge.source, dt_s, inputs);
            if let Some(value) = gauge.shown {
                sources::indicate(
                    &mut shown,
                    gauge.source,
                    gauge.unit.to_si(value),
                    &gauge.mechanism,
                );
            }
        }
        let state = shown.resolve();
        let gauges = &self.gauges;
        let direct = |slot: usize, pointer: usize| {
            gauges
                .iter()
                .find(|g| g.slot == slot && g.pointer == pointer)
                .and_then(|g| g.shown)
                .map(|v| v as f32)
        };
        let mut needles = InstrumentNeedles::default();
        for (slot, _, pose) in self.dials.poses(&state, direct) {
            let dial = self.dials.slot(slot).map(|d| &d.dial);
            pose_slot(&mut needles, slot, &pose, dial);
        }
        self.needles = needles;
    }

    /// The dials of the panel.
    #[cfg(test)]
    #[must_use]
    pub const fn dials(&self) -> &PanelDials {
        &self.dials
    }

    /// The pointer poses, by `(layout slot, hand)`.
    #[must_use]
    pub const fn needles(&self) -> InstrumentNeedles {
        self.needles
    }
}

/// Sets `slot`'s needles, wheels, cards and flag from `pose` of `dial`.
fn pose_slot(
    needles: &mut InstrumentNeedles,
    slot: usize,
    pose: &DialPose,
    dial: Option<&DialSource>,
) {
    for (hand, pointer) in pose.pointers.iter().enumerate() {
        needles.set(slot, hand, f64::from(pointer.angle_rad));
        needles.set_visible(slot, hand, pointer.visible);
    }
    // A card travels in face units; its face's radius makes that metres.
    let face_m = dial.map_or(0.0, |d| f64::from(d.face_radius_mm()) * 1e-3);
    for (card, posed) in pose.cards.iter().enumerate() {
        let travel_m = f64::from(posed.offset) * face_m;
        needles.set_card(
            slot,
            card,
            f64::from(posed.angle_rad),
            travel_m,
            posed.visible,
        );
    }
    // One drum per slot: a Kollsman window's wheels, or a heading card's
    // drum, which turns a tenth of a wheel's turn per digit as a wheel does.
    let tape = pose.tapes.first().and_then(|tape| {
        let turns = dial?.tape_turns(tape.value)?;
        Some((turns, tape.live))
    });
    let live = match (pose.windows.first(), tape) {
        (_, Some((turns, live))) => {
            needles.set_wheel(slot, 0, f64::from(turns) * 10.0);
            live
        }
        (Some(drum), None) => {
            for (wheel, position) in drum.wheels.iter().enumerate() {
                needles.set_wheel(slot, wheel, f64::from(*position));
            }
            drum.live
        }
        (None, None) => false,
    };
    needles.set_slot_state(slot, live, pose.flag);
}

impl super::Flight {
    /// What the panel's instruments sense now: the onboard air data, the
    /// airframe's articulation and the flight state the gyros and compass read.
    pub(super) fn gauge_inputs(&self) -> GaugeInputs {
        let air = self.onboard_sensors().air_data.map(|reading| reading.value);
        GaugeInputs::sense(air.as_ref(), &self.articulation()).with_flight(
            self.now(),
            self.vehicle_state(),
            self.specific_force_frd_m_s2(),
            self.world(),
        )
    }
}
