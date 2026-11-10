//! Deterministic value noise for paint variation: the same point of the
//! skin always gets the same grain, on every machine and every bake.

use glam::Vec3;

fn hash(i: [i32; 3]) -> f32 {
    let mut h = (i[0] as u32).wrapping_mul(0x8da6_b343)
        ^ (i[1] as u32).wrapping_mul(0xd816_3841)
        ^ (i[2] as u32).wrapping_mul(0xcb1a_b31f);
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^= h >> 15;
    (h & 0x00ff_ffff) as f32 / 16_777_215.0
}

/// Smooth noise in `0..1` with features about one unit across.
pub(crate) fn value(p: Vec3) -> f32 {
    let i = p.floor();
    let f = p - i;
    let u = f * f * (Vec3::splat(3.0) - 2.0 * f);
    let base = [i.x as i32, i.y as i32, i.z as i32];
    let corner = |dx: i32, dy: i32, dz: i32| {
        hash([
            base[0].wrapping_add(dx),
            base[1].wrapping_add(dy),
            base[2].wrapping_add(dz),
        ])
    };
    let lerp = |a: f32, b: f32, t: f32| a + (b - a) * t;
    let x00 = lerp(corner(0, 0, 0), corner(1, 0, 0), u.x);
    let x10 = lerp(corner(0, 1, 0), corner(1, 1, 0), u.x);
    let x01 = lerp(corner(0, 0, 1), corner(1, 0, 1), u.x);
    let x11 = lerp(corner(0, 1, 1), corner(1, 1, 1), u.x);
    lerp(lerp(x00, x10, u.y), lerp(x01, x11, u.y), u.z)
}

/// Three octaves of [`value`] noise at base scale `scale` (features per
/// metre), in `0..1`.
pub(crate) fn fractal(p: Vec3, scale: f32) -> f32 {
    (value(p * scale) * 0.55
        + value(p * scale * 2.3 + Vec3::splat(17.0)) * 0.3
        + value(p * scale * 5.9 + Vec3::splat(41.0)) * 0.15)
        .clamp(0.0, 1.0)
}
