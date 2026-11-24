//! Cryptographic per-trial stream derivation, portable Box-Muller / distribution sampling,
//! and safe row-major Cholesky decomposition.

use rand_chacha::ChaCha20Rng;
use rand_chacha::rand_core::{Rng, SeedableRng};
use serde::{Deserialize, Serialize};

use crate::hash::DomainHasher;
use crate::scalar_math::mul_add_unfused;

use super::MonteCarloError;

const MAX_TRUNCATED_NORMAL_ATTEMPTS: u32 = 10_000;
const UNIT_F64_SCALE: f64 = 1.0 / ((1_u64 << 53) as f64);
const SYMMETRY_REL_TOL: f64 = 1.0e-12;

/// Key isolating an independent Monte Carlo pseudo-random stream by `(run_seed, trial_index, stream_tag)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TrialStreamKey {
    /// Campaign-wide root seed for the Monte Carlo run.
    pub run_seed: u64,
    /// Zero-based Monte Carlo trial index.
    pub trial_index: u64,
    /// Domain or parameter stream identifier isolating concurrent or optional draws.
    pub stream_tag: u64,
}

impl TrialStreamKey {
    /// Constructs a [`TrialStreamKey`].
    #[must_use]
    pub const fn new(run_seed: u64, trial_index: u64, stream_tag: u64) -> Self {
        Self {
            run_seed,
            trial_index,
            stream_tag,
        }
    }

    /// Derives a canonical 256-bit seed via `DomainHasher::new(b"sim-spacetime:mc-trial-seed:v1")`.
    #[must_use]
    pub fn derive_seed(self) -> [u8; 32] {
        let mut h = DomainHasher::new(b"sim-spacetime:mc-trial-seed:v1");
        h.write_u64(self.run_seed);
        h.write_u64(self.trial_index);
        h.write_u64(self.stream_tag);
        h.finish().0
    }
}

/// Deterministic ChaCha20 pseudo-random stream with cached Box-Muller spare normal and pure-Rust
/// `libm` transcendentals for cross-platform bitwise reproducibility.
#[derive(Clone, Debug, PartialEq)]
pub struct TrialRng {
    inner: ChaCha20Rng,
    spare_normal: Option<f64>,
}

impl TrialRng {
    /// Seeds a [`TrialRng`] from a [`TrialStreamKey`].
    #[must_use]
    pub fn from_key(key: TrialStreamKey) -> Self {
        Self::from_seed(key.derive_seed())
    }

    /// Seeds a [`TrialRng`] directly from a 32-byte seed.
    #[must_use]
    pub fn from_seed(seed: [u8; 32]) -> Self {
        Self {
            inner: ChaCha20Rng::from_seed(seed),
            spare_normal: None,
        }
    }

    /// Draws the next uniform 64-bit unsigned integer from the underlying `ChaCha20` stream.
    #[inline]
    #[must_use]
    pub fn next_u64(&mut self) -> u64 {
        self.inner.next_u64()
    }

    /// Draws an IEEE-754 `f64` strictly in `[0.0, 1.0)` using the top 53 bits of [`Self::next_u64`].
    #[inline]
    #[must_use]
    pub fn next_f64_unit(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64) * UNIT_F64_SCALE
    }

    /// Draws a continuous uniform sample in `[min, max)`.
    pub fn uniform(&mut self, min: f64, max: f64) -> Result<f64, MonteCarloError> {
        if !min.is_finite() || !max.is_finite() || min >= max {
            return Err(MonteCarloError::InvalidBounds { min, max });
        }
        let u = self.next_f64_unit();
        let span = max - min;
        let sample = if span.is_finite() {
            mul_add_unfused(span, u, min)
        } else {
            let half_span = 0.5 * max - 0.5 * min;
            mul_add_unfused(half_span, 2.0 * u, min)
        };
        Ok(sample.clamp(min, max))
    }

    /// Draws an unbiased discrete uniform integer in `[min_inclusive, max_inclusive]` via
    /// Lemire/rejection-zone sampling.
    pub fn discrete_uniform(
        &mut self,
        min_inclusive: i64,
        max_inclusive: i64,
    ) -> Result<i64, MonteCarloError> {
        if min_inclusive > max_inclusive {
            return Err(MonteCarloError::InvalidDiscreteBounds {
                min_inclusive,
                max_inclusive,
            });
        }
        let span_minus_one = max_inclusive.abs_diff(min_inclusive);
        if span_minus_one == 0 {
            return Ok(min_inclusive);
        }
        let span_u128 = (span_minus_one as u128) + 1;
        let two_pow_64 = 1_u128 << 64;
        let zone_max = ((two_pow_64 / span_u128) * span_u128 - 1) as u64;
        loop {
            let candidate = self.next_u64();
            if candidate <= zone_max {
                let offset = ((candidate as u128) % span_u128) as u64;
                return Ok(min_inclusive.wrapping_add_unsigned(offset));
            }
        }
    }

    /// Draws one standard normal $\mathcal{N}(0, 1)$ sample using Box-Muller over pure-Rust
    /// `libm::{log, sqrt, sincos}`, caching the paired sine variate for the next call.
    pub fn standard_normal(&mut self) -> f64 {
        if let Some(cached) = self.spare_normal.take() {
            return cached;
        }
        let u1 = self.next_f64_unit().max(f64::MIN_POSITIVE);
        let u2 = self.next_f64_unit();
        let radius = libm::sqrt(-2.0 * libm::log(u1));
        let (sin_theta, cos_theta) = libm::sincos(core::f64::consts::TAU * u2);
        self.spare_normal = Some(radius * sin_theta);
        radius * cos_theta
    }

    /// Draws one univariate Gaussian $\mathcal{N}(\text{mean}, \text{std\_dev}^2)$ sample.
    pub fn normal(&mut self, mean: f64, std_dev: f64) -> Result<f64, MonteCarloError> {
        validate_normal_params(mean, std_dev, "mean", "std_dev")?;
        let z = self.standard_normal();
        Ok(mul_add_unfused(std_dev, z, mean))
    }

    /// Draws one log-normal $\exp(\mathcal{N}(\mu_{\ln}, \sigma_{\ln}^2))$ sample via `libm::exp`.
    pub fn log_normal(&mut self, mu_ln: f64, sigma_ln: f64) -> Result<f64, MonteCarloError> {
        validate_normal_params(mu_ln, sigma_ln, "mu_ln", "sigma_ln")?;
        let z = self.standard_normal();
        Ok(libm::exp(mul_add_unfused(sigma_ln, z, mu_ln)))
    }

    /// Draws one Gaussian sample truncated to `[min, max]` via bounded rejection sampling.
    pub fn truncated_normal(
        &mut self,
        mean: f64,
        std_dev: f64,
        min: f64,
        max: f64,
    ) -> Result<f64, MonteCarloError> {
        validate_normal_params(mean, std_dev, "mean", "std_dev")?;
        if !min.is_finite() || !max.is_finite() || min >= max {
            return Err(MonteCarloError::InvalidBounds { min, max });
        }
        for _ in 0..MAX_TRUNCATED_NORMAL_ATTEMPTS {
            let z = self.standard_normal();
            let candidate = mul_add_unfused(std_dev, z, mean);
            if candidate >= min && candidate <= max {
                return Ok(candidate);
            }
        }
        Err(MonteCarloError::TruncatedNormalRejectionLimitExceeded {
            mean,
            std_dev,
            min,
            max,
            max_attempts: MAX_TRUNCATED_NORMAL_ATTEMPTS,
        })
    }

    /// Computes the lower-triangular Cholesky factor $L$ ($L L^T = \Sigma$) of a row-major
    /// `dim x dim` symmetric positive-definite covariance matrix.
    pub fn cholesky_lower(
        covariance_row_major: &[f64],
        dim: usize,
    ) -> Result<Vec<f64>, MonteCarloError> {
        cholesky_lower(covariance_row_major, dim)
    }

    /// Draws a multivariate correlated Gaussian vector $\mathbf{x} = \boldsymbol{\mu} + L \mathbf{z}$
    /// given `mean` of length `dim` and row-major lower-triangular Cholesky factor `lower_cholesky`
    /// of length `dim * dim`.
    pub fn correlated_normal(
        &mut self,
        mean: &[f64],
        lower_cholesky: &[f64],
    ) -> Result<Vec<f64>, MonteCarloError> {
        let dim = mean.len();
        if dim == 0 {
            return Err(MonteCarloError::InvalidDimension {
                expected: 1,
                actual: 0,
            });
        }
        let expected_len = dim
            .checked_mul(dim)
            .ok_or(MonteCarloError::InvalidDimension {
                expected: usize::MAX,
                actual: dim,
            })?;
        if lower_cholesky.len() != expected_len {
            return Err(MonteCarloError::NonSquareMatrix {
                len: lower_cholesky.len(),
                dim,
            });
        }
        for &m in mean {
            if !m.is_finite() {
                return Err(MonteCarloError::InvalidParameter {
                    parameter: "mean",
                    value: m,
                });
            }
        }
        for &l in lower_cholesky {
            if !l.is_finite() {
                return Err(MonteCarloError::InvalidParameter {
                    parameter: "lower_cholesky",
                    value: l,
                });
            }
        }
        let z: Vec<f64> = (0..dim).map(|_| self.standard_normal()).collect();
        let mut out = Vec::with_capacity(dim);
        for (i, &m_i) in mean.iter().enumerate() {
            let mut acc = m_i;
            for (j, &z_j) in z.iter().take(i.wrapping_add(1)).enumerate() {
                let l_ij = lower_cholesky
                    .get(i.wrapping_mul(dim).wrapping_add(j))
                    .copied()
                    .ok_or(MonteCarloError::NonSquareMatrix {
                        len: lower_cholesky.len(),
                        dim,
                    })?;
                acc = mul_add_unfused(l_ij, z_j, acc);
            }
            out.push(acc);
        }
        Ok(out)
    }
}

/// Computes the lower-triangular Cholesky factor $L$ ($L L^T = \Sigma$) of a row-major `dim x dim`
/// symmetric positive-definite covariance matrix using pure-Rust `libm::sqrt` with zero
/// panic-capable indexing or slicing.
pub fn cholesky_lower(
    covariance_row_major: &[f64],
    dim: usize,
) -> Result<Vec<f64>, MonteCarloError> {
    if dim == 0 {
        return Err(MonteCarloError::InvalidDimension {
            expected: 1,
            actual: 0,
        });
    }
    let expected_len = dim
        .checked_mul(dim)
        .ok_or(MonteCarloError::InvalidDimension {
            expected: usize::MAX,
            actual: dim,
        })?;
    if covariance_row_major.len() != expected_len {
        return Err(MonteCarloError::NonSquareMatrix {
            len: covariance_row_major.len(),
            dim,
        });
    }
    validate_symmetric_covariance(covariance_row_major, dim)?;

    let mut lower = vec![0.0_f64; expected_len];
    for i in 0..dim {
        for j in 0..=i {
            let sum = cholesky_inner_dot(&lower, dim, i, j)?;
            let a_ij = symmetrized_entry(covariance_row_major, dim, i, j)?;
            if i == j {
                let diag = a_ij - sum;
                if !diag.is_finite() || diag <= 0.0 {
                    return Err(MonteCarloError::NotSymmetricPositiveDefinite { row: i, col: i });
                }
                let slot = lower.get_mut(i.wrapping_mul(dim).wrapping_add(i)).ok_or(
                    MonteCarloError::InvalidDimension {
                        expected: expected_len,
                        actual: expected_len,
                    },
                )?;
                *slot = libm::sqrt(diag);
            } else {
                let l_jj = matrix_entry(&lower, dim, j, j)?;
                let slot = lower.get_mut(i.wrapping_mul(dim).wrapping_add(j)).ok_or(
                    MonteCarloError::InvalidDimension {
                        expected: expected_len,
                        actual: expected_len,
                    },
                )?;
                *slot = (a_ij - sum) / l_jj;
            }
        }
    }
    Ok(lower)
}

fn validate_normal_params(
    location: f64,
    scale: f64,
    location_name: &'static str,
    scale_name: &'static str,
) -> Result<(), MonteCarloError> {
    if !location.is_finite() {
        return Err(MonteCarloError::InvalidParameter {
            parameter: location_name,
            value: location,
        });
    }
    if !scale.is_finite() || scale <= 0.0 {
        return Err(MonteCarloError::InvalidParameter {
            parameter: scale_name,
            value: scale,
        });
    }
    Ok(())
}

fn validate_symmetric_covariance(cov: &[f64], dim: usize) -> Result<(), MonteCarloError> {
    for i in 0..dim {
        let a_ii = matrix_entry(cov, dim, i, i)?;
        if !a_ii.is_finite() {
            return Err(MonteCarloError::NotSymmetricPositiveDefinite { row: i, col: i });
        }
        for j in 0..i {
            let a_ij = matrix_entry(cov, dim, i, j)?;
            let a_ji = matrix_entry(cov, dim, j, i)?;
            let a_jj = matrix_entry(cov, dim, j, j)?;
            if !a_ij.is_finite() || !a_ji.is_finite() {
                return Err(MonteCarloError::NotSymmetricPositiveDefinite { row: i, col: j });
            }
            let scale = a_ii.abs().max(a_jj.abs()).max(a_ij.abs()).max(a_ji.abs());
            if (a_ij - a_ji).abs() > SYMMETRY_REL_TOL * scale {
                return Err(MonteCarloError::NotSymmetricPositiveDefinite { row: i, col: j });
            }
        }
    }
    Ok(())
}

fn cholesky_inner_dot(
    lower: &[f64],
    dim: usize,
    row_i: usize,
    row_j: usize,
) -> Result<f64, MonteCarloError> {
    let mut sum = 0.0_f64;
    for k in 0..row_j {
        let l_ik = matrix_entry(lower, dim, row_i, k)?;
        let l_jk = matrix_entry(lower, dim, row_j, k)?;
        sum = mul_add_unfused(l_ik, l_jk, sum);
    }
    Ok(sum)
}

fn symmetrized_entry(
    cov: &[f64],
    dim: usize,
    row: usize,
    col: usize,
) -> Result<f64, MonteCarloError> {
    let a_ij = matrix_entry(cov, dim, row, col)?;
    if row == col {
        return Ok(a_ij);
    }
    let a_ji = matrix_entry(cov, dim, col, row)?;
    if a_ij == a_ji {
        Ok(a_ij)
    } else {
        Ok(0.5 * a_ij + 0.5 * a_ji)
    }
}

fn matrix_entry(mat: &[f64], dim: usize, row: usize, col: usize) -> Result<f64, MonteCarloError> {
    mat.get(row.wrapping_mul(dim).wrapping_add(col))
        .copied()
        .ok_or(MonteCarloError::NonSquareMatrix {
            len: mat.len(),
            dim,
        })
}
