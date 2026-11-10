//! One pointer of a pack instrument, the primary or a further one: its
//! lane, its law, the scale it is printed on and its needles.

use indicate_instrument_analog::{
    AnalogDialSpec, DialChannel, DialInput, FailurePose, MarkArc, PointerShape, PointerStyle,
    Radial, ScalePoint, TickSet,
};
use syspack::{
    DialFaceSpec, DialUnitSpec, FailureSpec, InstrumentKindSpec, InstrumentSpec, MarkingSpec,
    PointerChannelSpec, PointerShapeSpec, SourceBinding,
};

use super::{indicate_unit, park_at_start, unlit};
use crate::dials::house;
use crate::dials::source::{Lane, Law, LawKind, PointerSource, RingSource};

/// One turn of each altimeter hand, feet: hundreds, thousands, ten thousands.
const ALTIMETER_TURNS: [f32; 3] = [1_000.0, 10_000.0, 100_000.0];
/// Height between pointers stacked on one pivot beyond the template's.
const STACK_STEP_MM: f32 = 0.5;

/// The engine lane a `engine[n].<quantity>` source names.
fn engine_input(engine: u16, quantity: &str) -> Option<DialInput> {
    Some(match quantity {
        "manifold_pressure" => DialInput::ManifoldPressure { engine },
        "rpm" => DialInput::EngineRpm { engine },
        "oil_temperature" => DialInput::OilTemperature { engine },
        "oil_pressure" => DialInput::OilPressure { engine },
        "fuel_pressure" => DialInput::FuelPressure { engine },
        "coolant_temperature" => DialInput::CoolantTemperature { engine },
        "cylinder_head_temperature" => DialInput::CylinderHeadTemperature { engine },
        "carburetor_air_temperature" => DialInput::CarburettorAirTemperature { engine },
        _ => return None,
    })
}

/// One pointer of an instrument as the pack describes it.
pub(super) struct Hand<'a> {
    source: &'a str,
    kind: &'a InstrumentKindSpec,
    pub(super) face: Option<&'a DialFaceSpec>,
    template: &'static AnalogDialSpec<'static>,
    print_template: bool,
    tanks: &'a [&'a str],
}

/// The scale a hand reads on: printed when it is the hand's own.
pub(super) struct HandRing {
    pub(super) printed: Option<RingSource>,
    center: [f32; 2],
    radius: f32,
    law: Law,
}

impl<'a> Hand<'a> {
    pub(super) fn primary(spec: &'a InstrumentSpec, tanks: &'a [&'a str]) -> Self {
        Self::new(&spec.source, &spec.kind, spec.dial.as_ref(), tanks)
    }

    pub(super) fn further(pointer: &'a PointerChannelSpec, tanks: &'a [&'a str]) -> Self {
        Self::new(&pointer.source, &pointer.kind, pointer.dial.as_ref(), tanks)
    }

    fn new(
        source: &'a str,
        kind: &'a InstrumentKindSpec,
        face: Option<&'a DialFaceSpec>,
        tanks: &'a [&'a str],
    ) -> Self {
        let (template, print_template) = house::template(kind, source);
        Self {
            source,
            kind,
            face,
            template,
            print_template,
            tanks,
        }
    }

    fn units(&self) -> Option<DialUnitSpec> {
        Some(match self.kind {
            InstrumentKindSpec::Airspeed { units, .. }
            | InstrumentKindSpec::Altimeter { units, .. }
            | InstrumentKindSpec::VerticalSpeed { units, .. }
            | InstrumentKindSpec::PressureGauge { units, .. }
            | InstrumentKindSpec::Tachometer { units, .. }
            | InstrumentKindSpec::Thermometer { units, .. }
            | InstrumentKindSpec::FuelQuantity { units, .. } => *units,
            InstrumentKindSpec::Direct { .. } => return None,
        })
    }

    /// The state lane Indicate reads this hand from, or a direct lane for
    /// a quantity its state does not carry.
    fn lane(&self) -> Lane {
        let input = match SourceBinding::parse(self.source) {
            Some(SourceBinding::PitotStatic { .. }) => match self.kind {
                InstrumentKindSpec::Airspeed { .. } => Some(DialInput::IndicatedAirspeed),
                InstrumentKindSpec::Altimeter { .. } => Some(DialInput::BaroAltitude),
                InstrumentKindSpec::VerticalSpeed { .. } => Some(DialInput::IndicatedVerticalSpeed),
                _ => None,
            },
            Some(SourceBinding::Engine { index, quantity }) => {
                engine_input(u16::from(index), &quantity)
            }
            Some(SourceBinding::System { path }) => path
                .strip_prefix("fuel.")
                .and_then(|tank| self.tanks.iter().position(|t| *t == tank))
                .and_then(|tank| u16::try_from(tank).ok())
                .map(|tank| DialInput::FuelQuantity { tank }),
            None => None,
        };
        input
            .zip(self.units().and_then(indicate_unit))
            .map(|(input, unit)| DialChannel { input, unit })
            .filter(DialChannel::units_agree)
            .map_or(Lane::Direct, Lane::State)
    }

    fn law(&self) -> Law {
        // Without a printed reference, a scale sits where the built-in gauge
        // of its kind puts it, or at twelve o'clock when there is none.
        let fallback = if self.print_template {
            house::reference_deg(self.template)
        } else {
            0.0
        };
        let reference_deg = self
            .face
            .and_then(|face| face.reference_deg)
            .map_or(fallback, |deg| deg as f32);
        let scale = match self.kind {
            InstrumentKindSpec::Altimeter { .. } => {
                return Law {
                    reference_deg,
                    kind: LawKind::Turns(ALTIMETER_TURNS[0]),
                };
            }
            InstrumentKindSpec::Airspeed { scale, .. }
            | InstrumentKindSpec::VerticalSpeed { scale, .. }
            | InstrumentKindSpec::PressureGauge { scale, .. }
            | InstrumentKindSpec::Tachometer { scale, .. }
            | InstrumentKindSpec::Thermometer { scale, .. }
            | InstrumentKindSpec::FuelQuantity { scale, .. } => scale,
            InstrumentKindSpec::Direct { .. } => {
                return Law {
                    reference_deg,
                    kind: LawKind::Points(Vec::new()),
                };
            }
        };
        Law {
            reference_deg,
            kind: LawKind::Points(
                scale
                    .iter()
                    .map(|p| ScalePoint {
                        value: p.value as f32,
                        angle_deg: p.angle_deg as f32,
                    })
                    .collect(),
            ),
        }
    }

    /// The graduated span, dial units.
    fn span(law: &Law) -> [f32; 2] {
        match &law.kind {
            LawKind::Points(points) => match (points.first(), points.last()) {
                (Some(first), Some(last)) => [first.value, last.value],
                _ => [0.0, 0.0],
            },
            LawKind::Turns(per_turn) => [0.0, *per_turn],
        }
    }

    fn ticks(&self, [lo, hi]: [f32; 2]) -> Vec<TickSet> {
        let (minor, major) = house::tick_geometry(self.template);
        match self.face.filter(|face| !face.graduations.is_empty()) {
            Some(face) => face
                .graduations
                .iter()
                .map(|g| TickSet {
                    from: g.from as f32,
                    to: g.to as f32,
                    every: g.every as f32,
                    ..if g.major { major } else { minor }
                })
                .collect(),
            None if self.print_template => house::ring(self.template)
                .map_or(&[][..], |ring| ring.ticks)
                .iter()
                .filter_map(|t| {
                    let (from, to) = (t.from.max(lo), t.to.min(hi));
                    (from <= to).then_some(TickSet { from, to, ..*t })
                })
                .collect(),
            None => Vec::new(),
        }
    }

    fn numerals(&self, [lo, hi]: [f32; 2]) -> Vec<(f32, String)> {
        match self.face.filter(|face| !face.numerals.is_empty()) {
            Some(face) => face
                .numerals
                .iter()
                .map(|n| (n.value as f32, n.text.clone()))
                .collect(),
            None if self.print_template => house::ring(self.template)
                .map_or(&[][..], |ring| ring.numerals)
                .iter()
                .filter(|n| (lo..=hi).contains(&n.value))
                .map(|n| (n.value, n.text.to_owned()))
                .collect(),
            None => Vec::new(),
        }
    }

    fn markings(&self, markings: &[MarkingSpec]) -> (Vec<MarkArc>, Vec<Radial>) {
        let [inner, outer] = house::arc_band(self.template);
        let [r_inner, r_outer, width] = house::radial_geometry(self.template);
        let mut arcs = Vec::new();
        let mut radials = Vec::new();
        for marking in markings {
            match *marking {
                MarkingSpec::Arc { from, to, colour } => arcs.push(MarkArc {
                    from: from as f32,
                    to: to as f32,
                    colour: house::mark_colour(colour),
                    inner,
                    outer,
                }),
                MarkingSpec::Radial { at, colour } => radials.push(Radial {
                    at: at as f32,
                    colour: house::mark_colour(colour),
                    inner: r_inner,
                    outer: r_outer,
                    width,
                }),
            }
        }
        (arcs, radials)
    }

    /// The hand's scale. The primary always prints its own; a further hand
    /// prints one only when its face places or graduates it, and otherwise
    /// reads the face's main scale.
    pub(super) fn ring(
        &self,
        markings: &[MarkingSpec],
        luminous: Option<bool>,
        primary: bool,
    ) -> HandRing {
        let face = self.face;
        let own = primary
            || face.is_some_and(|f| {
                f.ring_centre.is_some()
                    || f.ring_radius.is_some()
                    || !f.graduations.is_empty()
                    || !f.numerals.is_empty()
            });
        let center = face
            .and_then(|f| f.ring_centre)
            .map_or([0.0, 0.0], |c| c.map(|v| v as f32));
        let radius = face.and_then(|f| f.ring_radius).map_or(1.0, |r| r as f32);
        let law = self.law();
        let printed = own.then(|| {
            let span = Self::span(&law);
            let (arcs, radials) = self.markings(markings);
            let template_ring = house::ring(self.template);
            RingSource {
                center,
                radius,
                law: law.clone(),
                ticks: self.ticks(span),
                numerals: self.numerals(span),
                numeral_radius: template_ring.map_or(0.66, |r| r.numeral_radius),
                // Numerals are sized in face units; a sub-dial prints smaller.
                numeral_size: template_ring.map_or(0.14, |r| r.numeral_size) * radius.max(0.6),
                arcs,
                radials,
                paint: unlit(
                    template_ring.map_or(self.template.face, |r| r.paint),
                    luminous,
                ),
            }
        });
        HandRing {
            printed,
            center,
            radius,
            law,
        }
    }

    fn style(&self, base: PointerStyle, radius: f32) -> PointerStyle {
        let shape = |s: PointerShapeSpec| match s {
            PointerShapeSpec::Lance => PointerShape::Lance,
            PointerShapeSpec::Sword => PointerShape::Sword,
            PointerShapeSpec::Baton => PointerShape::Baton,
            PointerShapeSpec::Triangle => PointerShape::Triangle,
        };
        match self.face.and_then(|face| face.pointers.first()) {
            Some(p) => PointerStyle {
                shape: shape(p.shape),
                length: p.length as f32,
                width: p.width as f32,
                tail: p.tail as f32,
                hub_radius: base.hub_radius * radius,
                ..base
            },
            None => PointerStyle {
                length: base.length * radius,
                width: base.width * radius.max(0.6),
                tail: base.tail * radius,
                hub_radius: base.hub_radius * radius,
                ..base
            },
        }
    }

    /// The hand's needles: one, or an altimeter's three.
    pub(super) fn pointers(
        &self,
        ring: &HandRing,
        luminous: Option<bool>,
        on_failure: Option<FailureSpec>,
        index: usize,
    ) -> Vec<PointerSource> {
        let lane = self.lane();
        let input_range = match self.kind {
            InstrumentKindSpec::Altimeter { range, .. } => Some(range.map(|v| v as f32)),
            _ => None,
        };
        let failure = |law: &Law| match self.face.and_then(|f| f.on_failure).or(on_failure) {
            Some(FailureSpec::Park { angle_deg }) => FailurePose::Park {
                angle_deg: angle_deg as f32,
            },
            Some(FailureSpec::Hide) => FailurePose::Hide,
            None => park_at_start(law),
        };
        let count = self.template.pointers.len();
        let altimeter = matches!(self.kind, InstrumentKindSpec::Altimeter { .. });
        let bases: Vec<_> = if altimeter {
            self.template.pointers.iter().collect()
        } else {
            self.template.pointers.iter().take(1).collect()
        };
        bases
            .into_iter()
            .enumerate()
            .map(|(i, base)| {
                // The template's altimeter stacks ten-thousands, thousands,
                // hundreds; the pack names hands the other way round.
                let hand = count.saturating_sub(i + 1);
                let law = match ALTIMETER_TURNS.get(hand) {
                    Some(&per_turn) if altimeter => Law {
                        reference_deg: ring.law.reference_deg,
                        kind: LawKind::Turns(per_turn),
                    },
                    _ => ring.law.clone(),
                };
                let style = if altimeter && hand > 0 {
                    base.style
                } else {
                    self.style(base.style, ring.radius)
                };
                PointerSource {
                    lane,
                    on_failure: failure(&law),
                    law,
                    input_range,
                    pivot: ring.center,
                    style: PointerStyle {
                        paint: unlit(style.paint, luminous),
                        ..style
                    },
                    stack_height_mm: base.stack_height_mm + STACK_STEP_MM * index as f32,
                }
            })
            .collect()
    }
}
