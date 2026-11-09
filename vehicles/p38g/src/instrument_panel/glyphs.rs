//! Seven-segment numerals painted on dial faces as flat quads.

use glam::Vec3;

use crate::mesh::MeshBuilder;

/// Segments `a..g` (top, upper right, lower right, bottom, lower left, upper
/// left, middle) lit for each digit.
const SEGMENTS: [[bool; 7]; 10] = [
    [true, true, true, true, true, true, false],
    [false, true, true, false, false, false, false],
    [true, true, false, true, true, false, true],
    [true, true, true, true, false, false, true],
    [false, true, true, false, false, true, true],
    [true, false, true, true, false, true, true],
    [true, false, true, true, true, true, true],
    [true, true, true, false, false, false, false],
    [true, true, true, true, true, true, true],
    [true, true, true, true, false, true, true],
];

/// Segment endpoints on a unit cell (`x` right `0..0.6`, `z` up `0..1`).
const STROKES: [([f32; 2], [f32; 2]); 7] = [
    ([0.0, 1.0], [0.6, 1.0]),
    ([0.6, 1.0], [0.6, 0.5]),
    ([0.6, 0.5], [0.6, 0.0]),
    ([0.0, 0.0], [0.6, 0.0]),
    ([0.0, 0.5], [0.0, 0.0]),
    ([0.0, 1.0], [0.0, 0.5]),
    ([0.0, 0.5], [0.6, 0.5]),
];

/// Advance from one digit to the next, in digit heights.
const PITCH: f32 = 0.85;

/// A flat stroke from `a` to `b` of `width` on the dial-local `y = face_y` plane,
/// facing the pilot (`-Y`).
pub(super) fn stroke(mesh: &mut MeshBuilder, a: Vec3, b: Vec3, width: f32) {
    let along = (b - a).normalize_or(Vec3::Z);
    let side = Vec3::new(along.z, 0.0, -along.x) * (0.5 * width);
    let ext = along * (0.5 * width);
    mesh.polygon(
        &[
            a - ext - side,
            b + ext - side,
            b + ext + side,
            a - ext + side,
        ],
        Vec3::NEG_Y,
    );
}

/// Paints `value` centred on `centre` (dial-local, on the face plane) with digits
/// `height` tall.
pub(super) fn number(mesh: &mut MeshBuilder, value: u32, centre: Vec3, height: f32) {
    let text = value.to_string();
    let count = text.len() as f32;
    let width = height * (0.6 + PITCH * (count - 1.0));
    let origin = centre + Vec3::new(-0.5 * width, 0.0, -0.5 * height);
    let stroke_w = height * 0.16;
    for (i, ch) in text.chars().enumerate() {
        let Some(lit) = ch
            .to_digit(10)
            .and_then(|d| SEGMENTS.get(usize::try_from(d).unwrap_or(usize::MAX)))
        else {
            continue;
        };
        let cell = origin + Vec3::new(height * PITCH * i as f32, 0.0, 0.0);
        for (on, ([ax, az], [bx, bz])) in lit.iter().zip(STROKES) {
            if *on {
                let a = cell + Vec3::new(ax, 0.0, az) * height;
                let b = cell + Vec3::new(bx, 0.0, bz) * height;
                stroke(mesh, a, b, stroke_w);
            }
        }
    }
}
