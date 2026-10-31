use std::f64::consts::TAU;

use glam::DVec3;

#[cfg(test)]
mod tests;

/// Scale from a 53-bit integer to the unit interval.
const UNIT_SCALE: f64 = 1.0 / (1u64 << 53) as f64;
/// Bits dropped to fit the 53-bit significand.
const SIGNIFICAND_SHIFT: u32 = 11;

/// Deterministic Gaussian white noise: a SplitMix64 generator whose only state
/// is one `u64`, so a snapshot of a device holds its noise stream exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WhiteNoise {
    state: u64,
}

impl WhiteNoise {
    /// A stream fixed by `seed`.
    pub const fn from_seed(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform on `(0, 1]`, which keeps the logarithm below finite.
    fn next_unit(&mut self) -> f64 {
        ((self.next_u64() >> SIGNIFICAND_SHIFT) + 1) as f64 * UNIT_SCALE
    }

    /// One zero-mean, unit-variance Gaussian draw, by Box-Muller. The pair's
    /// second value is discarded so the state stays a single `u64`.
    pub fn next_gaussian(&mut self) -> f64 {
        let radius = (-2.0 * self.next_unit().ln()).sqrt();
        radius * (TAU * self.next_unit()).cos()
    }

    /// Three independent draws scaled per axis by `sigma`.
    pub fn next_vec3(&mut self, sigma: DVec3) -> DVec3 {
        DVec3::new(
            self.next_gaussian() * sigma.x,
            self.next_gaussian() * sigma.y,
            self.next_gaussian() * sigma.z,
        )
    }
}
