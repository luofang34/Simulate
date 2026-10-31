#[cfg(test)]
mod tests;

/// A well-mixed 64-bit hash of a lattice point: the same inputs give the same
/// value on every platform, which keeps procedural scenery replayable.
pub(crate) fn lattice_hash(seed: u64, x: i64, y: i64) -> u64 {
    let mut state = seed ^ 0x9E37_79B9_7F4A_7C15;
    for word in [x as u64, y as u64] {
        state = mix(state ^ word);
    }
    state
}

/// SplitMix64's finaliser.
fn mix(value: u64) -> u64 {
    let a = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    let b = (a ^ (a >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    b ^ (b >> 31)
}

/// [`lattice_hash`] as a fraction in `0.0..1.0`.
pub(crate) fn lattice_unit(seed: u64, x: i64, y: i64) -> f64 {
    (lattice_hash(seed, x, y) >> 11) as f64 / (1_u64 << 53) as f64
}

/// Smoothly interpolated lattice noise in `0.0..1.0`, with one lattice cell
/// per unit of `x` and `y`.
pub(crate) fn value_noise(seed: u64, x: f64, y: f64) -> f64 {
    let (cell_x, cell_y) = (x.floor(), y.floor());
    let (ix, iy) = (cell_x as i64, cell_y as i64);
    let fade = |t: f64| t * t * (3.0 - 2.0 * t);
    let (tx, ty) = (fade(x - cell_x), fade(y - cell_y));
    let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;
    let south = lerp(
        lattice_unit(seed, ix, iy),
        lattice_unit(seed, ix + 1, iy),
        tx,
    );
    let north = lerp(
        lattice_unit(seed, ix, iy + 1),
        lattice_unit(seed, ix + 1, iy + 1),
        tx,
    );
    lerp(south, north, ty)
}
