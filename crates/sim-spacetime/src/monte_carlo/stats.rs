//! Online Welford summary accumulator and portable empirical quantile estimation.

use serde::{Deserialize, Serialize};

use crate::scalar_math::mul_add_unfused;

/// Online Welford accumulator tracking sample count, mean, population/sample variance,
/// standard deviation, and extrema in $O(1)$ space.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EnsembleSummary {
    count: u64,
    mean: f64,
    m2: f64,
    min: f64,
    max: f64,
}

impl EnsembleSummary {
    /// Creates an empty [`EnsembleSummary`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            count: 0,
            mean: 0.0,
            m2: 0.0,
            min: 0.0,
            max: 0.0,
        }
    }

    /// Updates the running statistics with one scalar observation `value`.
    pub fn push(&mut self, value: f64) {
        if self.count == 0 {
            self.count = 1;
            self.mean = value;
            self.m2 = 0.0;
            self.min = value;
            self.max = value;
            return;
        }
        self.count = self.count.wrapping_add(1);
        let delta = value - self.mean;
        self.mean += delta / (self.count as f64);
        let delta2 = value - self.mean;
        self.m2 += delta * delta2;
        if value < self.min {
            self.min = value;
        }
        if value > self.max {
            self.max = value;
        }
    }

    /// Accumulates all values from `samples` in order.
    #[must_use]
    pub fn from_samples(samples: &[f64]) -> Self {
        let mut summary = Self::new();
        for &value in samples {
            summary.push(value);
        }
        summary
    }

    /// Total number of pushed observations.
    #[must_use]
    pub const fn count(&self) -> u64 {
        self.count
    }

    /// Arithmetic mean of all pushed observations, or `None` when empty.
    #[must_use]
    pub const fn mean(&self) -> Option<f64> {
        if self.count == 0 {
            None
        } else {
            Some(self.mean)
        }
    }

    /// Population variance $\frac{1}{N} \sum_{i=1}^N (x_i - \bar{x})^2$, or `None` when empty.
    #[must_use]
    pub fn variance(&self) -> Option<f64> {
        if self.count == 0 {
            None
        } else {
            Some((self.m2 / (self.count as f64)).max(0.0))
        }
    }

    /// Bessel-corrected sample variance $\frac{1}{N - 1} \sum_{i=1}^N (x_i - \bar{x})^2$,
    /// or `None` when fewer than 2 observations have been pushed.
    #[must_use]
    pub fn sample_variance(&self) -> Option<f64> {
        if self.count < 2 {
            None
        } else {
            Some((self.m2 / ((self.count - 1) as f64)).max(0.0))
        }
    }

    /// Population standard deviation $\sqrt{\operatorname{variance}()}$ via `libm::sqrt`,
    /// or `None` when empty.
    #[must_use]
    pub fn std_dev(&self) -> Option<f64> {
        self.variance().map(libm::sqrt)
    }

    /// Sample standard deviation $\sqrt{\operatorname{sample\_variance}()}$ via `libm::sqrt`,
    /// or `None` when fewer than 2 observations have been pushed.
    #[must_use]
    pub fn sample_std_dev(&self) -> Option<f64> {
        self.sample_variance().map(libm::sqrt)
    }

    /// Minimum pushed observation, or `None` when empty.
    #[must_use]
    pub const fn min(&self) -> Option<f64> {
        if self.count == 0 {
            None
        } else {
            Some(self.min)
        }
    }

    /// Maximum pushed observation, or `None` when empty.
    #[must_use]
    pub const fn max(&self) -> Option<f64> {
        if self.count == 0 {
            None
        } else {
            Some(self.max)
        }
    }

    /// Linearly interpolated empirical quantile at probability `q` in `[0.0, 1.0]`
    /// over a caller-sorted ascending slice `sorted` (Type 7 quantile).
    /// Returns `None` if `sorted` is empty or `q` is non-finite or outside `[0.0, 1.0]`.
    #[must_use]
    pub fn empirical_quantile(sorted: &[f64], q: f64) -> Option<f64> {
        if sorted.is_empty() || !q.is_finite() || !(0.0..=1.0).contains(&q) {
            return None;
        }
        let n = sorted.len();
        if n == 1 {
            return sorted.first().copied();
        }
        let pos = q * ((n - 1) as f64);
        let lo = (libm::floor(pos) as usize).min(n - 1);
        let hi = lo.saturating_add(1).min(n - 1);
        let frac = pos - (lo as f64);
        let v_lo = *sorted.get(lo)?;
        let v_hi = *sorted.get(hi)?;
        if lo == hi || frac == 0.0 || v_lo == v_hi {
            return Some(v_lo);
        }
        Some(mul_add_unfused(v_hi - v_lo, frac, v_lo))
    }
}
