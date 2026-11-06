//! Seeded noise for the onboard sensor bus: a SplitMix64 stream, Box–Muller normals,
//! and exactly discretised first-order Gauss–Markov processes.

use glam::DVec3;

/// A deterministic SplitMix64 stream; its whole state is one `u64`, so it snapshots and
/// restores with the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NoiseStream(u64);

impl NoiseStream {
    /// A stream starting at `seed`.
    pub const fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `(0, 1]`, never zero so its logarithm is finite.
    fn unit(&mut self) -> f64 {
        ((self.next_u64() >> 11).wrapping_add(1)) as f64 / (1_u64 << 53) as f64
    }

    /// One standard normal draw.
    pub fn normal(&mut self) -> f64 {
        let radius = (-2.0 * self.unit().ln()).sqrt();
        radius * (std::f64::consts::TAU * self.unit()).cos()
    }

    /// Three independent standard normal draws.
    pub fn normal3(&mut self) -> DVec3 {
        DVec3::new(self.normal(), self.normal(), self.normal())
    }
}

/// A first-order Gauss–Markov process with correlation time `tau_s` and stationary
/// standard deviation `sigma`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GaussMarkov {
    /// Correlation time, seconds.
    pub tau_s: f64,
    /// Stationary standard deviation, per axis.
    pub sigma: f64,
}

impl GaussMarkov {
    /// A draw from the stationary distribution, for a process already running at start.
    pub fn stationary(self, noise: &mut NoiseStream) -> f64 {
        self.sigma * noise.normal()
    }

    /// Advances `value` by `dt_s` exactly (no Euler error at any step size).
    pub fn step(self, value: f64, dt_s: f64, noise: &mut NoiseStream) -> f64 {
        if self.sigma <= 0.0 || self.tau_s <= 0.0 {
            return 0.0;
        }
        let decay = (-dt_s.max(0.0) / self.tau_s).exp();
        let drive = self.sigma * (1.0 - decay * decay).max(0.0).sqrt();
        value * decay + drive * noise.normal()
    }

    /// [`Self::stationary`] on each axis.
    pub fn stationary3(self, noise: &mut NoiseStream) -> DVec3 {
        self.sigma * noise.normal3()
    }

    /// [`Self::step`] on each axis.
    pub fn step3(self, value: DVec3, dt_s: f64, noise: &mut NoiseStream) -> DVec3 {
        DVec3::new(
            self.step(value.x, dt_s, noise),
            self.step(value.y, dt_s, noise),
            self.step(value.z, dt_s, noise),
        )
    }
}
