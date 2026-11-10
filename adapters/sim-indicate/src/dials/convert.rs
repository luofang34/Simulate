//! A pack instrument as a [`DialSource`]: the pack states the case, units,
//! scales, markings, Kollsman range and, in its optional `dial` blocks, the
//! printed face; Indicate's built-in gauge of the same kind supplies the
//! rest. Each pointer of the case becomes a pointer of the dial, with its
//! own scale, sub-dial and lane.

mod hand;

use indicate_instrument_analog::{
    AnalogDialSpec, DialChannel, DialInput, DialUnit, FailurePose, Paint, WindowSpec,
};
use syspack::{DialFaceSpec, DialUnitSpec, InstrumentKindSpec, InstrumentSpec, KollsmanSpec};

use super::DialError;
use super::house::{self, FACE_SHARE_OF_CASE};
use super::source::{DialSource, LegendSource};
use hand::Hand;

/// Indicate's unit for a pack unit.
pub(super) fn indicate_unit(unit: DialUnitSpec) -> Option<DialUnit> {
    Some(match unit {
        DialUnitSpec::Knots => DialUnit::Knots,
        DialUnitSpec::MilesPerHour => DialUnit::MilesPerHour,
        DialUnitSpec::KilometresPerHour => DialUnit::KilometresPerHour,
        DialUnitSpec::Feet => DialUnit::Feet,
        DialUnitSpec::Metres => DialUnit::Metres,
        DialUnitSpec::FeetPerMinute => DialUnit::FeetPerMinute,
        DialUnitSpec::MetresPerSecond => DialUnit::MetresPerSecond,
        DialUnitSpec::InchesOfMercury => DialUnit::InchesOfMercury,
        DialUnitSpec::Hectopascals => DialUnit::Hectopascals,
        DialUnitSpec::Kilopascals => DialUnit::Kilopascals,
        DialUnitSpec::PoundsPerSquareInch => DialUnit::PoundsPerSquareInch,
        DialUnitSpec::RevolutionsPerMinute => DialUnit::RevolutionsPerMinute,
        DialUnitSpec::DegreesCelsius => DialUnit::DegreesCelsius,
        DialUnitSpec::UsGallons => DialUnit::UsGallons,
    })
}

/// Paint without its glow, for a face the pack says is not luminous.
pub(super) fn unlit(paint: Paint, luminous: Option<bool>) -> Paint {
    if luminous == Some(false) {
        Paint::plain(paint.day)
    } else {
        paint
    }
}

fn legends(
    faces: &[Option<&DialFaceSpec>],
    template: &AnalogDialSpec<'static>,
    print_template: bool,
) -> Vec<LegendSource> {
    let paint = template.legends.first().map_or(template.face, |l| l.paint);
    let printed: Vec<LegendSource> = faces
        .iter()
        .flatten()
        .flat_map(|face| &face.legends)
        .map(|l| LegendSource {
            text: l.text.clone(),
            at: l.at.map(|v| v as f32),
            size: l.size as f32,
            paint,
        })
        .collect();
    if !printed.is_empty() || !print_template {
        return printed;
    }
    template
        .legends
        .iter()
        .map(|l| LegendSource {
            text: l.text.to_owned(),
            at: l.at,
            size: l.size,
            paint: l.paint,
        })
        .collect()
}

/// The Kollsman drum and the lane it reads, from the template's window and
/// the pack's range.
fn window(
    kollsman: &KollsmanSpec,
    template: &AnalogDialSpec<'static>,
) -> Option<(WindowSpec<'static>, DialChannel)> {
    let base = template.windows.first().copied()?;
    let unit = indicate_unit(kollsman.units)?;
    let (value_scale, separator) = match kollsman.units {
        DialUnitSpec::Hectopascals => (1.0, None),
        _ => (100.0, Some((2, "."))),
    };
    let spec = WindowSpec {
        range: [kollsman.min as f32, kollsman.max as f32],
        value_scale,
        separator,
        ..base
    };
    Some((
        spec,
        DialChannel {
            input: DialInput::BaroSetting,
            unit,
        },
    ))
}

impl DialSource {
    /// The dial `spec` describes, checked as Indicate checks it. `tanks`
    /// names the aircraft's fuel tanks in its tank order, so a
    /// `system.fuel.<tank>` source finds its lane.
    pub fn from_instrument(spec: &InstrumentSpec, tanks: &[&str]) -> Result<Self, DialError> {
        let (template, print_template) = house::template(&spec.kind, &spec.source);
        if let InstrumentKindSpec::Direct { .. } = spec.kind {
            return Ok(Self::builtin(spec, template));
        }
        let top = spec.dial.as_ref();
        let luminous = top.and_then(|face| face.luminous);
        let on_failure = top.and_then(|face| face.on_failure);
        let hands: Vec<Hand<'_>> = std::iter::once(Hand::primary(spec, tanks))
            .chain(spec.pointers.iter().map(|p| Hand::further(p, tanks)))
            .collect();
        let mut source = Self {
            case_diameter_mm: (spec.bezel.diameter_m() * 1_000.0) as f32,
            face_radius_mm: 0.0,
            face: template.face,
            bezel: template.bezel,
            rings: Vec::new(),
            legends: legends(
                &hands.iter().map(|h| h.face).collect::<Vec<_>>(),
                template,
                print_template,
            ),
            pointers: Vec::new(),
            window: match &spec.kind {
                InstrumentKindSpec::Altimeter { kollsman, .. } => window(kollsman, template),
                _ => None,
            },
            off_flag: template.off_flag,
            lettering: indicate_instrument_analog::Lettering::DEFAULT,
            builtin: None,
        };
        let case_m = spec.bezel.diameter_m();
        let face_m = top
            .and_then(|face| face.face_diameter_m)
            .unwrap_or(case_m * FACE_SHARE_OF_CASE);
        source.face_radius_mm = (face_m * 500.0) as f32;
        for (index, hand) in hands.iter().enumerate() {
            let markings = if index == 0 {
                spec.markings.as_slice()
            } else {
                &[]
            };
            let ring = hand.ring(markings, luminous, index == 0);
            let pointers = hand.pointers(&ring, luminous, on_failure, index);
            if let Some(ring) = ring.printed {
                source.rings.push(ring);
            }
            source.pointers.extend(pointers);
        }
        source.check().map_err(|source| DialError::Spec {
            instrument: spec.id.clone(),
            source,
        })?;
        Ok(source)
    }
}

impl DialSource {
    /// `template` drawn as it is, in `spec`'s case.
    fn builtin(spec: &InstrumentSpec, template: &'static AnalogDialSpec<'static>) -> Self {
        let case_m = spec.bezel.diameter_m();
        let face_m = spec
            .dial
            .as_ref()
            .and_then(|face| face.face_diameter_m)
            .unwrap_or(case_m * FACE_SHARE_OF_CASE);
        Self {
            case_diameter_mm: (case_m * 1_000.0) as f32,
            face_radius_mm: (face_m * 500.0) as f32,
            face: template.face,
            bezel: template.bezel,
            rings: Vec::new(),
            legends: Vec::new(),
            pointers: Vec::new(),
            window: None,
            off_flag: template.off_flag,
            lettering: template.lettering,
            builtin: Some(template),
        }
    }
}

/// Where a pointer on a scale starting at `law`'s first point rests without
/// input.
pub(super) fn park_at_start(law: &super::source::Law) -> FailurePose {
    let start = match &law.kind {
        super::source::LawKind::Points(points) => points.first().map_or(0.0, |p| p.angle_deg),
        super::source::LawKind::Turns(_) => 0.0,
    };
    FailurePose::Park {
        angle_deg: law.reference_deg + start,
    }
}
