use std::f64::consts::TAU;

#[cfg(test)]
mod tests;

/// Below this mean, Poisson draws are exact (Knuth); at and above it the
/// normal approximation's error in variance and skew is under a few percent of
/// the shot noise, which is itself under the read noise of either sensor.
const EXACT_POISSON_MAX_MEAN: f64 = 30.0;

/// Separates the noise streams of one frame, so adding a stream to a sensor
/// never shifts the draws of another.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum NoiseStreamId {
    /// Per-pixel shot, dark and read noise of a photon-counting detector.
    Detector = 1,
    /// Per-pixel temporal noise of a microbolometer.
    Temporal = 2,
    /// Per-column temporal noise of a microbolometer readout.
    Column = 3,
    /// Fixed per-pixel responsivity of one sensor instance.
    FixedGain = 4,
    /// Fixed per-pixel offset of one sensor instance.
    FixedOffset = 5,
    /// Fixed per-column offset of one sensor instance.
    FixedColumn = 6,
}

/// A deterministic SplitMix64 sequence: the same `(seed, sequence, stream)`
/// gives the same draws on a given platform. The integer state is portable;
/// the normal and Poisson transforms go through the platform's `ln`, `exp`
/// and `sin`, which may differ in the last bit between math libraries.
#[derive(Debug, Clone)]
pub(super) struct NoiseStream {
    state: u64,
    spare_gaussian: Option<f64>,
}

impl NoiseStream {
    /// The stream for one frame `sequence` of the sensor seeded `seed`.
    pub(super) fn new(seed: u64, sequence: u64, stream: NoiseStreamId) -> Self {
        let key = mix(mix(seed) ^ sequence.rotate_left(17) ^ (stream as u64).rotate_left(41));
        Self {
            state: key,
            spare_gaussian: None,
        }
    }

    /// The stream for a quantity fixed for the sensor's life, independent of
    /// any frame.
    pub(super) fn fixed(seed: u64, stream: NoiseStreamId) -> Self {
        Self::new(seed, u64::MAX, stream)
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mix(self.state)
    }

    /// Uniform on the open interval `(0, 1)`.
    fn uniform_open(&mut self) -> f64 {
        const SCALE: f64 = 1.0 / (1_u64 << 53) as f64;
        ((self.next_u64() >> 11) as f64 + 0.5) * SCALE
    }

    /// Standard normal, by Box-Muller.
    pub(super) fn gaussian(&mut self) -> f64 {
        if let Some(spare) = self.spare_gaussian.take() {
            return spare;
        }
        let radius = (-2.0 * self.uniform_open().ln()).sqrt();
        let angle = TAU * self.uniform_open();
        self.spare_gaussian = Some(radius * angle.sin());
        radius * angle.cos()
    }

    /// A Poisson draw of `mean`; zero for a mean that is not positive.
    pub(super) fn poisson(&mut self, mean: f64) -> f64 {
        if mean.is_nan() || mean <= 0.0 {
            return 0.0;
        }
        if mean >= EXACT_POISSON_MAX_MEAN {
            return (mean + mean.sqrt() * self.gaussian()).round().max(0.0);
        }
        let limit = (-mean).exp();
        let mut product = self.uniform_open();
        let mut count = 0.0;
        while product > limit {
            product *= self.uniform_open();
            count += 1.0;
        }
        count
    }
}

/// The SplitMix64 finaliser: a bijection that spreads every input bit.
fn mix(value: u64) -> u64 {
    let mut z = value;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}
