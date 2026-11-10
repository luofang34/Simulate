//! Numerals painted as strokes of constant width, the plain sans-serif the
//! 339th FS used for its white nose numbers (USAAF 342-FH-3A43614, "129" on
//! 43-2242) and the AAF stencil shops for radio call numbers.
//!
//! Each numeral is a set of centre-line polylines in a cell one unit tall
//! and [`ADVANCE`] wide, `y` up; a painter strokes them at the face's width.

use std::f32::consts::{FRAC_PI_2, PI};

use glam::Vec2;

/// Advance of one numeral, in heights.
pub(crate) const ADVANCE: f32 = 0.62;
/// Width of the painted cell inside the advance.
const W: f32 = 0.48;

/// Points of an elliptical arc about `c` with radii `r`, from `a0` to `a1`.
fn arc(c: Vec2, r: Vec2, a0: f32, a1: f32) -> Vec<Vec2> {
    (0..=12)
        .map(|i| {
            let a = a0 + (a1 - a0) * i as f32 / 12.0;
            c + Vec2::new(a.cos(), a.sin()) * r
        })
        .collect()
}

fn line(points: &[[f32; 2]]) -> Vec<Vec2> {
    points.iter().map(|&p| Vec2::from(p)).collect()
}

/// Upper and lower bowls of the round numerals.
fn bowl(top: bool) -> (Vec2, Vec2) {
    let r = Vec2::new(W * 0.5, 0.25);
    (Vec2::new(W * 0.5, if top { 0.75 } else { 0.25 }), r)
}

/// The strokes of numeral `digit`, or none for another character.
pub(crate) fn strokes(digit: char) -> Vec<Vec<Vec2>> {
    let h = W * 0.5;
    match digit {
        '0' => {
            let r = Vec2::new(h, 0.25);
            let mut ring = arc(Vec2::new(h, 0.75), r, 0.0, PI);
            ring.extend(arc(Vec2::new(h, 0.25), r, PI, 2.0 * PI));
            ring.push(Vec2::new(W, 0.75));
            vec![ring]
        }
        '1' => vec![line(&[[0.10, 0.80], [0.30, 1.0], [0.30, 0.0]])],
        '2' => {
            let (c, r) = bowl(true);
            let mut s = arc(c, r, PI * 0.95, -PI * 0.2);
            s.extend(line(&[[0.0, 0.0], [W, 0.0]]));
            vec![s]
        }
        '3' => {
            let (c, r) = bowl(true);
            let (d, q) = bowl(false);
            vec![
                arc(c, r, PI * 0.9, -FRAC_PI_2),
                arc(d, q, FRAC_PI_2, -PI * 0.9),
            ]
        }
        '4' => vec![line(&[
            [0.36, 0.0],
            [0.36, 1.0],
            [0.0, 0.32],
            [W + 0.04, 0.32],
        ])],
        '5' => {
            let (d, q) = bowl(false);
            let mut s = line(&[[W, 1.0], [0.04, 1.0], [0.0, 0.52]]);
            s.extend(arc(
                d + Vec2::new(0.0, 0.04),
                q * Vec2::new(1.0, 1.15),
                PI * 0.8,
                -PI * 0.85,
            ));
            vec![s]
        }
        '6' => {
            let (d, q) = bowl(false);
            let mut s = arc(Vec2::new(h, 0.55), Vec2::new(h, 0.45), PI * 0.35, PI);
            s.extend(arc(d, q, PI, 3.0 * PI));
            vec![s]
        }
        '7' => vec![line(&[[0.0, 1.0], [W, 1.0], [0.14, 0.0]])],
        '8' => {
            let (c, r) = bowl(true);
            let (d, q) = bowl(false);
            vec![
                arc(c, r * Vec2::new(0.9, 1.0), -FRAC_PI_2, 1.5 * PI),
                arc(d, q, FRAC_PI_2, 2.5 * PI),
            ]
        }
        '9' => {
            let (c, r) = bowl(true);
            let mut s = arc(c, r, 0.0, 2.0 * PI);
            s.extend(arc(Vec2::new(h, 0.45), Vec2::new(h, 0.45), 0.0, -PI * 0.65));
            vec![s]
        }
        _ => Vec::new(),
    }
}

/// Distance from `p` to the nearest stroke of `text` set from the origin,
/// one unit tall, reading along `+x`; infinite for blank text.
pub(crate) fn distance(text: &str, p: Vec2) -> f32 {
    let mut best = f32::INFINITY;
    for (index, ch) in text.chars().enumerate() {
        let origin = Vec2::new(index as f32 * ADVANCE, 0.0);
        let local = p - origin;
        if local.x < -0.3 || local.x > ADVANCE + 0.3 {
            continue;
        }
        for stroke in strokes(ch) {
            for pair in stroke.windows(2) {
                let (Some(&a), Some(&b)) = (pair.first(), pair.get(1)) else {
                    continue;
                };
                let ab = b - a;
                let t = ((local - a).dot(ab) / ab.length_squared().max(1e-9)).clamp(0.0, 1.0);
                best = best.min(local.distance(a + ab * t));
            }
        }
    }
    best
}

/// Width of `text` in heights.
pub(crate) fn width(text: &str) -> f32 {
    let n = text.chars().count() as f32;
    (n - 1.0).max(0.0) * ADVANCE + W
}
