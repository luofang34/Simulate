//! One round dial: where its scale graduations sit on the face, and the
//! geometry of its case, face, graduations, numerals, markings and needles.
//!
//! Every angle here is clockwise from twelve o'clock as the pilot sees the face,
//! and the face lies in the dial-local `x`/`z` plane looking down `-Y`.

use std::f64::consts::TAU;

use glam::Vec3;
use syspack::{InstrumentKindSpec, InstrumentSpec, MarkingColour, MarkingSpec, ScalePoint};

use super::glyphs::{number, stroke};
use crate::mesh::MeshBuilder;

/// Distances in front of the panel board, along `-Y`, of each layer of a dial.
pub(super) const CASE_FRONT_M: f32 = 0.009;
const FACE_M: f32 = 0.0095;
const PAINT_M: f32 = 0.0102;
/// Plane the needles sweep in, clear of the paint.
pub(super) const NEEDLE_M: f32 = 0.0115;
/// Fraction of the case radius the dial face shows inside the bezel ring.
const FACE_FRACTION: f32 = 0.86;

/// How a dial is graduated: tick spacing, which ticks carry numerals, and what
/// each numeral reads in the dial's units.
#[derive(Clone, Copy, Debug)]
struct Graduation {
    minor: f64,
    major: f64,
    label: f64,
    divisor: f64,
    label_zero: bool,
}

/// Scale and graduation of a needle dial, from its pack entry.
#[derive(Clone, Debug)]
pub(super) struct DialFace {
    scale: Vec<(f64, f64)>,
    zero_offset_deg: f64,
    graduation: Graduation,
    markings: Vec<MarkingSpec>,
    /// Hands the instrument drives: one, or the altimeter's three.
    pub(super) hands: usize,
    pub(super) radius_m: f32,
}

fn points(scale: &[ScalePoint]) -> Vec<(f64, f64)> {
    scale.iter().map(|p| (p.value, p.angle_deg)).collect()
}

impl DialFace {
    /// The face `spec` engraves, or `None` for an instrument without a dial scale.
    pub(super) fn from_spec(spec: &InstrumentSpec) -> Option<Self> {
        let radius_m = (0.5 * spec.bezel.diameter_m()) as f32;
        let g = |minor, major, label, divisor, label_zero| Graduation {
            minor,
            major,
            label,
            divisor,
            label_zero,
        };
        let (scale, graduation, hands) = match &spec.kind {
            InstrumentKindSpec::Airspeed { scale, .. } => {
                (points(scale), g(10.0, 50.0, 100.0, 1.0, false), 1)
            }
            InstrumentKindSpec::VerticalSpeed { scale, .. } => {
                (points(scale), g(500.0, 1000.0, 1000.0, 1000.0, true), 1)
            }
            InstrumentKindSpec::PressureGauge { scale, .. } => {
                (points(scale), g(2.0, 10.0, 10.0, 1.0, true), 1)
            }
            InstrumentKindSpec::Tachometer { scale, .. } => {
                (points(scale), g(100.0, 500.0, 500.0, 100.0, true), 1)
            }
            // One turn of the long hand per 1,000 units, numerals in hundreds.
            InstrumentKindSpec::Altimeter { .. } => (
                vec![(0.0, 0.0), (1000.0, 360.0)],
                g(20.0, 100.0, 100.0, 100.0, true),
                3,
            ),
        };
        let (Some(&(_, first)), Some(&(_, last))) = (scale.first(), scale.last()) else {
            return None;
        };
        Some(Self {
            zero_offset_deg: zero_offset_deg(first, last, hands),
            scale,
            graduation,
            markings: spec.markings.clone(),
            hands,
            radius_m,
        })
    }

    /// The face angle, rad, of the scale's own `0°` mark: the needle rests here
    /// and the mechanism's needle angle is added to it.
    pub(super) fn zero_offset_rad(&self) -> f64 {
        self.zero_offset_deg.to_radians()
    }

    fn face_angle(&self, value: f64) -> f32 {
        ((self.zero_offset_deg + scale_angle_deg(&self.scale, value)).to_radians()) as f32
    }

    /// The case and bezel ring, centred on `centre` (the dial centre on the board).
    pub(super) fn case(&self, mesh: &mut MeshBuilder, centre: Vec3) {
        let back = centre;
        let front = centre - Vec3::Y * CASE_FRONT_M;
        mesh.tube(back, front, self.radius_m, self.radius_m, 36);
    }

    /// The face disc inside the bezel ring.
    pub(super) fn face(&self, mesh: &mut MeshBuilder, centre: Vec3) {
        disc(
            mesh,
            centre - Vec3::Y * FACE_M,
            self.radius_m * FACE_FRACTION,
            36,
        );
    }

    /// Graduations and numerals in white.
    pub(super) fn paint(&self, mesh: &mut MeshBuilder, centre: Vec3) {
        let at = centre - Vec3::Y * PAINT_M;
        let r = self.radius_m * FACE_FRACTION;
        let (lo, hi) = self.value_range();
        let g = self.graduation;
        let steps = ((hi - lo) / g.minor).round().clamp(0.0, 400.0) as u32;
        // A full-turn dial's last graduation is its first.
        let last = if self.hands > 1 {
            steps.saturating_sub(1)
        } else {
            steps
        };
        for i in 0..=last {
            let v = lo + g.minor * f64::from(i);
            let is_major = is_multiple(v, g.major);
            let inner = if is_major { 0.80 } else { 0.88 };
            let width = if is_major { 0.035 } else { 0.018 };
            radial(mesh, at, self.face_angle(v), r * inner, r * 0.98, r * width);
            if is_multiple(v, g.label) && (g.label_zero || v.abs() > 1e-6) {
                let digits = (v.abs() / g.divisor).round() as u32;
                let d = direction(self.face_angle(v));
                number(mesh, digits, at + d * (r * 0.62), r * 0.17);
            }
        }
    }

    /// Coloured radials and arcs, each into the builder of its colour.
    pub(super) fn markings(&self, colours: &mut [MeshBuilder; 5], centre: Vec3) {
        let at = centre - Vec3::Y * (PAINT_M + 0.0003);
        let r = self.radius_m * FACE_FRACTION;
        for marking in &self.markings {
            match *marking {
                MarkingSpec::Radial { at: v, colour } => {
                    if let Some(mesh) = colours.get_mut(colour_index(colour)) {
                        radial(mesh, at, self.face_angle(v), r * 0.70, r, r * 0.05);
                    }
                }
                MarkingSpec::Arc { from, to, colour } => {
                    let Some(mesh) = colours.get_mut(colour_index(colour)) else {
                        continue;
                    };
                    let (a0, a1) = (self.face_angle(from), self.face_angle(to));
                    for k in 0..12 {
                        let t = |k: f32| a0 + (a1 - a0) * k / 12.0;
                        let (p, q) = (direction(t(k as f32)), direction(t(k as f32 + 1.0)));
                        stroke(mesh, at + p * (r * 0.94), at + q * (r * 0.94), r * 0.07);
                    }
                }
            }
        }
    }

    /// The needle of `hand`, pointing up the dial-local `+Z` from the origin.
    pub(super) fn needle(&self, hand: usize) -> MeshBuilder {
        let r = self.radius_m * FACE_FRACTION;
        let (length, width) = match hand {
            0 => (0.90, 0.07),
            1 => (0.58, 0.12),
            _ => (0.95, 0.03),
        };
        let mut mesh = MeshBuilder::new();
        let tail = Vec3::new(0.0, 0.0, -0.18 * r);
        let tip = Vec3::new(0.0, 0.0, length * r);
        let half = 0.5 * width * r;
        mesh.polygon(
            &[
                tail + Vec3::X * half,
                tip + Vec3::new(0.25 * half, 0.0, -2.0 * half),
                tip,
                tip + Vec3::new(-0.25 * half, 0.0, -2.0 * half),
                tail - Vec3::X * half,
            ],
            Vec3::NEG_Y,
        );
        disc(&mut mesh, Vec3::NEG_Y * 0.0003, 0.09 * r, 12);
        mesh
    }

    fn value_range(&self) -> (f64, f64) {
        let lo = self.scale.first().map_or(0.0, |p| p.0);
        let hi = self.scale.last().map_or(0.0, |p| p.0);
        (lo, hi)
    }
}

/// Where a scale's `0°` mark sits on the face: a full-turn dial at twelve, a dial
/// reading both ways (vertical speed) at nine so climb is up, and any other dial
/// with the gap between its end graduations centred at six.
fn zero_offset_deg(first_deg: f64, last_deg: f64, hands: usize) -> f64 {
    if hands > 1 || last_deg - first_deg >= 359.0 {
        0.0
    } else if first_deg < 0.0 {
        -90.0
    } else {
        180.0 + 0.5 * (360.0 - (last_deg - first_deg)) - first_deg
    }
}

/// Piecewise-linear needle angle of `value` on `scale`, held at the end
/// graduations as the mechanism's stops hold it.
fn scale_angle_deg(scale: &[(f64, f64)], value: f64) -> f64 {
    let (Some(&(v0, a0)), Some(&(vn, an))) = (scale.first(), scale.last()) else {
        return 0.0;
    };
    if value <= v0 {
        return a0;
    }
    if value >= vn {
        return an;
    }
    for pair in scale.windows(2) {
        if let [(va, aa), (vb, ab)] = *pair
            && value <= vb
        {
            return aa + (ab - aa) * (value - va) / (vb - va).max(1e-9);
        }
    }
    an
}

fn is_multiple(value: f64, step: f64) -> bool {
    let q = value / step;
    (q - q.round()).abs() < 1e-6
}

/// Unit vector on the face at face angle `angle` (clockwise from twelve).
fn direction(angle: f32) -> Vec3 {
    let (s, c) = angle.sin_cos();
    Vec3::new(s, 0.0, c)
}

fn radial(mesh: &mut MeshBuilder, at: Vec3, angle: f32, r0: f32, r1: f32, width: f32) {
    let d = direction(angle);
    stroke(mesh, at + d * r0, at + d * r1, width);
}

fn disc(mesh: &mut MeshBuilder, centre: Vec3, radius: f32, segments: u32) {
    let rim: Vec<Vec3> = (0..segments)
        .map(|i| {
            let a = (TAU as f32) * i as f32 / segments as f32;
            centre + direction(a) * radius
        })
        .collect();
    mesh.polygon(&rim, Vec3::NEG_Y);
}

/// Index of `colour` in the panel's marking builders.
pub(super) const fn colour_index(colour: MarkingColour) -> usize {
    match colour {
        MarkingColour::White => 0,
        MarkingColour::Green => 1,
        MarkingColour::Yellow => 2,
        MarkingColour::Red => 3,
        MarkingColour::Blue => 4,
    }
}

#[cfg(test)]
pub(super) fn face_angle_for_test(face: &DialFace, value: f64) -> f32 {
    face.face_angle(value)
}
