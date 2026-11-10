//! The house style: Indicate's built-in gauge of the same kind, which gives
//! a converted dial whatever its pack does not print.

use indicate_instrument_analog::{
    AIRSPEED_KT, AIRSPEED_MPH, ALTIMETER, AnalogDialSpec, CLOCK, DIRECTIONAL_GYRO, GYRO_HORIZON,
    MAGNETIC_COMPASS, MANIFOLD_PRESSURE, MarkArc, MarkColour, Radial, SYNCHROSCOPE, ScaleLaw,
    ScaleRing, TACHOMETER, TURN_AND_BANK, TickSet, VERTICAL_SPEED,
};
use syspack::{DialUnitSpec, DirectInstrument, InstrumentKindSpec};

/// The share of the case diameter the dial face shows inside its bezel,
/// where the pack does not give the face's diameter. The P-38G's mounting
/// stack bores its bezels to the same share.
pub(super) const FACE_SHARE_OF_CASE: f64 = 0.86;

/// The built-in gauge whose house style `kind` takes, and whether that gauge
/// measures the same thing, so its printed graduations, numerals and
/// legends may stand in for ones the pack leaves out. A gauge Indicate has
/// no built-in for, such as a thermometer, takes the engine gauges' style
/// and prints only what its pack says.
pub(super) fn template(
    kind: &InstrumentKindSpec,
    source: &str,
) -> (&'static AnalogDialSpec<'static>, bool) {
    match kind {
        InstrumentKindSpec::Airspeed {
            units: DialUnitSpec::Knots,
            ..
        } => (&AIRSPEED_KT, true),
        InstrumentKindSpec::Airspeed { .. } => (&AIRSPEED_MPH, true),
        InstrumentKindSpec::Altimeter { .. } => (&ALTIMETER, true),
        InstrumentKindSpec::VerticalSpeed { .. } => (&VERTICAL_SPEED, true),
        InstrumentKindSpec::PressureGauge { .. } => {
            (&MANIFOLD_PRESSURE, source.ends_with(".manifold_pressure"))
        }
        InstrumentKindSpec::Tachometer { .. } => (&TACHOMETER, true),
        InstrumentKindSpec::Thermometer { .. } | InstrumentKindSpec::FuelQuantity { .. } => {
            (&MANIFOLD_PRESSURE, false)
        }
        InstrumentKindSpec::Direct { instrument } => (direct(*instrument), true),
    }
}

/// Indicate's built-in gauge of a directly fed instrument, drawn as it is.
pub(super) const fn direct(instrument: DirectInstrument) -> &'static AnalogDialSpec<'static> {
    match instrument {
        DirectInstrument::GyroHorizon => &GYRO_HORIZON,
        DirectInstrument::DirectionalGyro => &DIRECTIONAL_GYRO,
        DirectInstrument::TurnAndBank => &TURN_AND_BANK,
        DirectInstrument::MagneticCompass => &MAGNETIC_COMPASS,
        DirectInstrument::Clock => &CLOCK,
        DirectInstrument::Synchroscope => &SYNCHROSCOPE,
    }
}

/// The main ring of `template`.
pub(super) fn ring(template: &AnalogDialSpec<'static>) -> Option<&'static ScaleRing<'static>> {
    template.rings.first()
}

/// Where `template`'s scale zero sits, degrees clockwise from twelve.
pub(super) fn reference_deg(template: &AnalogDialSpec<'static>) -> f32 {
    match ring(template).map(|ring| ring.law) {
        Some(ScaleLaw::Piecewise { reference_deg, .. } | ScaleLaw::Turns { reference_deg, .. }) => {
            reference_deg
        }
        None => 0.0,
    }
}

const MINOR: TickSet = TickSet {
    from: 0.0,
    to: 0.0,
    every: 1.0,
    inner: 0.89,
    outer: 0.97,
    width: 0.012,
};
const MAJOR: TickSet = TickSet {
    inner: 0.80,
    width: 0.024,
    ..MINOR
};

/// The minor and major graduation geometry of `template`: its first and
/// last tick runs.
pub(super) fn tick_geometry(template: &AnalogDialSpec<'static>) -> (TickSet, TickSet) {
    let ticks = ring(template).map_or(&[][..], |ring| ring.ticks);
    (
        ticks.first().copied().unwrap_or(MINOR),
        ticks.last().copied().unwrap_or(MAJOR),
    )
}

/// The band an arc occupies, `[inner, outer]` of the ring radius.
pub(super) fn arc_band(template: &AnalogDialSpec<'static>) -> [f32; 2] {
    ring(template)
        .and_then(|ring| ring.arcs.first())
        .map_or([0.86, 0.95], |arc: &MarkArc| [arc.inner, arc.outer])
}

/// A radial line's `[inner, outer, width]`.
pub(super) fn radial_geometry(template: &AnalogDialSpec<'static>) -> [f32; 3] {
    ring(template)
        .and_then(|ring| ring.radials.first())
        .map_or([0.78, 0.98, 0.03], |radial: &Radial| {
            [radial.inner, radial.outer, radial.width]
        })
}

/// Indicate's colour for a pack marking colour.
pub(super) fn mark_colour(colour: syspack::MarkingColour) -> MarkColour {
    match colour {
        syspack::MarkingColour::White => MarkColour::White,
        syspack::MarkingColour::Green => MarkColour::Green,
        syspack::MarkingColour::Yellow => MarkColour::Yellow,
        syspack::MarkingColour::Red => MarkColour::Red,
        syspack::MarkingColour::Blue => MarkColour::Blue,
    }
}
