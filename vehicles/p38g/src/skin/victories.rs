//! Victory markings: Petit's two Japanese flags for the two A6M2-N "Rufes"
//! of 27 February 1943 and a ship silhouette for the subchaser attacked off
//! the Shortlands on 29 March 1943 (Pacific Wrecks, 43-2264, citing
//! Lansdale, *13th Fighter Command in WWII*). That they were on 147 is
//! sourced; their form, size and place are not: no photograph of 147 was
//! found. They follow the row of flags above 43-2242's "129" (USAAF
//! 342-FH-3A43614): flags in a line above the nose number, here the ship
//! after them. Every dimension is an assumption.

use glam::Vec2;

use super::weathering::smooth;

/// A flag is 3:2, its red disc three fifths of its height.
const FLAG_WIDTH: f32 = 1.5;
const DISC_RADIUS: f32 = 0.3;
/// Gap between marks, and the silhouette's length, in mark heights.
const GAP: f32 = 0.4;
const SHIP_LENGTH: f32 = 2.8;

/// A small warship in profile, bow to the right, one unit tall at the mast.
const SHIP: &[[f32; 2]] = &[
    [0.0, 0.0],
    [2.55, 0.0],
    [2.8, 0.38],
    [2.2, 0.32],
    [1.75, 0.32],
    [1.75, 0.62],
    [1.35, 0.62],
    [1.35, 1.0],
    [1.25, 1.0],
    [1.25, 0.62],
    [1.0, 0.62],
    [0.95, 0.5],
    [0.6, 0.5],
    [0.6, 0.32],
    [0.05, 0.32],
];

/// What a mark paints at a point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Ink {
    White,
    Red,
}

/// The ink and cover at `at` (inches from the row's centre, reading
/// frame) of a row of `flags` flags then `ships` silhouettes, `height_in`
/// tall, a texel `aa` inches wide.
pub(crate) fn paint(
    flags: u32,
    ships: u32,
    height_in: f32,
    at: Vec2,
    aa: f32,
) -> Option<(Ink, f32)> {
    let h = height_in.max(0.1);
    let marks = flags + ships;
    let width = flags as f32 * FLAG_WIDTH
        + ships as f32 * SHIP_LENGTH
        + marks.saturating_sub(1) as f32 * GAP;
    let mut local = at / h + Vec2::new(width * 0.5, 0.5);
    let edge = |d: f32| 1.0 - smooth(-0.5 * aa, 0.5 * aa, d * h);
    for _ in 0..flags {
        let field = box_distance(
            local - Vec2::new(FLAG_WIDTH * 0.5, 0.5),
            Vec2::new(FLAG_WIDTH * 0.5, 0.5),
        );
        let cover = edge(field);
        if cover > 0.0 {
            let disc = edge((local - Vec2::new(FLAG_WIDTH * 0.5, 0.5)).length() - DISC_RADIUS);
            return Some(if disc > 0.5 {
                (Ink::Red, cover)
            } else {
                (Ink::White, cover)
            });
        }
        local.x -= FLAG_WIDTH + GAP;
    }
    for _ in 0..ships {
        let cover = edge(polygon_distance(local, SHIP));
        if cover > 0.0 {
            return Some((Ink::White, cover));
        }
        local.x -= SHIP_LENGTH + GAP;
    }
    None
}

/// Signed distance to an axis-aligned box of half-size `half`, negative
/// inside.
fn box_distance(p: Vec2, half: Vec2) -> f32 {
    let q = p.abs() - half;
    q.max(Vec2::ZERO).length() + q.x.max(q.y).min(0.0)
}

/// Signed distance from `p` to the closed polygon `corners`, negative
/// inside.
pub(crate) fn polygon_distance(p: Vec2, corners: &[[f32; 2]]) -> f32 {
    let mut distance = f32::INFINITY;
    let mut inside = false;
    for (i, &a) in corners.iter().enumerate() {
        let Some(&b) = corners.get((i + 1) % corners.len()) else {
            continue;
        };
        let (a, b) = (Vec2::from(a), Vec2::from(b));
        let ab = b - a;
        let t = ((p - a).dot(ab) / ab.length_squared().max(1e-12)).clamp(0.0, 1.0);
        distance = distance.min(p.distance(a + ab * t));
        if (a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) / (b.y - a.y) * ab.x {
            inside = !inside;
        }
    }
    if inside { -distance } else { distance }
}
