//! Reproducible Monte Carlo stream management, parametric distributions, run manifests,
//! Cholesky-correlated Gaussian sampling, and online Welford ensemble statistics.
//!
//! All elementary transcendentals inside this module (`sqrt`, `log`, `sincos`, `exp`, `floor`)
//! execute unconditionally through pure-Rust `libm` and unfused IEEE-754 operations so native
//! (`x86_64`, `aarch64`) and `wasm32` builds produce bit-identical draws and moments.

mod manifest;
mod rng;
mod stats;
#[cfg(test)]
mod tests;

use crate::hash::ContentHash;

pub use manifest::{
    DistributionSpec, MONTE_CARLO_MANIFEST_SCHEMA_VERSION, MonteCarloConfig,
    MonteCarloManifestDigest, MonteCarloRunManifest, ParameterDispersionSpec,
    sample_trial_parameters,
};
pub use rng::{TrialRng, TrialStreamKey, cholesky_lower};
pub use stats::EnsembleSummary;

/// Errors returned by Monte Carlo RNG sampling, Cholesky factorization, and manifest validation.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum MonteCarloError {
    /// Continuous distribution bounds were non-finite or inverted (`min >= max`).
    #[error("invalid distribution bounds: min={min}, max={max}")]
    InvalidBounds {
        /// Lower bound.
        min: f64,
        /// Upper bound.
        max: f64,
    },
    /// Discrete uniform bounds were inverted (`min_inclusive > max_inclusive`).
    #[error("invalid discrete uniform bounds: min={min_inclusive}, max={max_inclusive}")]
    InvalidDiscreteBounds {
        /// Inclusive lower bound.
        min_inclusive: i64,
        /// Inclusive upper bound.
        max_inclusive: i64,
    },
    /// Distribution or vector parameter was non-finite or non-positive where positivity is required.
    #[error("invalid distribution parameter `{parameter}`: {value}")]
    InvalidParameter {
        /// Parameter name.
        parameter: &'static str,
        /// Invalid scalar value.
        value: f64,
    },
    /// Dispersion parameter name was empty or whitespace-only.
    #[error("dispersion parameter name cannot be empty")]
    EmptyParameterName,
    /// Two dispersion specifications in a [`MonteCarloConfig`] shared the same parameter name.
    #[error("duplicate dispersion parameter name `{name}`")]
    DuplicateParameterName {
        /// Duplicated parameter name.
        name: String,
    },
    /// Matrix or mean-vector dimension was zero or exceeded addressable bounds.
    #[error("invalid dimension: expected {expected}, got {actual}")]
    InvalidDimension {
        /// Expected dimension or minimum bound.
        expected: usize,
        /// Actual dimension.
        actual: usize,
    },
    /// Row-major matrix length did not equal `dim * dim`.
    #[error("matrix of length {len} is not square for dimension {dim}")]
    NonSquareMatrix {
        /// Actual flat slice length.
        len: usize,
        /// Expected square matrix dimension.
        dim: usize,
    },
    /// Covariance matrix was non-finite, asymmetric, or not strictly positive-definite.
    #[error("covariance matrix is not symmetric positive-definite at ({row}, {col})")]
    NotSymmetricPositiveDefinite {
        /// Matrix row index where validation or factorization failed.
        row: usize,
        /// Matrix column index where validation or factorization failed.
        col: usize,
    },
    /// Bounded rejection sampling for a truncated normal failed to accept within the attempt cap.
    #[error(
        "truncated normal rejection sampling exceeded {max_attempts} attempts for mean={mean}, std_dev={std_dev}, bounds=[{min}, {max}]"
    )]
    TruncatedNormalRejectionLimitExceeded {
        /// Underlying Gaussian mean.
        mean: f64,
        /// Underlying Gaussian standard deviation.
        std_dev: f64,
        /// Inclusive lower truncation bound.
        min: f64,
        /// Inclusive upper truncation bound.
        max: f64,
        /// Rejection attempt cap exhausted.
        max_attempts: u32,
    },
    /// Manifest `config_hash` did not match the canonical hash recomputed from `config`.
    #[error("manifest integrity mismatch: expected config hash {expected}, got {actual}")]
    ManifestIntegrityMismatch {
        /// Recomputed canonical hash of `config`.
        expected: ContentHash,
        /// Stored `config_hash` field on the manifest.
        actual: ContentHash,
    },
    /// Manifest `schema_version` is unsupported by this version of `sim-spacetime`.
    #[error("unsupported Monte Carlo manifest schema version {version}")]
    UnsupportedSchemaVersion {
        /// Manifest schema version encountered.
        version: u32,
    },
}
