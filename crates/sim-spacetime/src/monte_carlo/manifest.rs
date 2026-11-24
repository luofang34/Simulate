//! Serializable Monte Carlo dispersion specifications, campaign configuration,
//! independent per-parameter sampling, and tamper-evident run manifests.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::hash::{ContentHash, DomainHasher};

use super::MonteCarloError;
use super::rng::{TrialRng, TrialStreamKey};

/// Current schema version for [`MonteCarloRunManifest`].
pub const MONTE_CARLO_MANIFEST_SCHEMA_VERSION: u32 = 1;

/// Parametric probability distribution for a scalar Monte Carlo dispersion parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DistributionSpec {
    /// Deterministic constant scalar value.
    Constant {
        /// Fixed parameter value.
        value: f64,
    },
    /// Continuous uniform distribution over `[min, max)`.
    Uniform {
        /// Inclusive lower bound.
        min: f64,
        /// Exclusive upper bound (`max > min`).
        max: f64,
    },
    /// Univariate Gaussian distribution $\mathcal{N}(\text{mean}, \text{std\_dev}^2)$.
    Normal {
        /// Distribution mean.
        mean: f64,
        /// Strictly positive standard deviation.
        std_dev: f64,
    },
    /// Log-normal distribution $\exp(\mathcal{N}(\mu_{\ln}, \sigma_{\ln}^2))$.
    LogNormal {
        /// Mean of the natural logarithm.
        mu_ln: f64,
        /// Strictly positive standard deviation of the natural logarithm.
        sigma_ln: f64,
    },
    /// Gaussian distribution truncated to `[min, max]`.
    TruncatedNormal {
        /// Underlying Gaussian mean.
        mean: f64,
        /// Underlying strictly positive Gaussian standard deviation.
        std_dev: f64,
        /// Inclusive lower truncation bound.
        min: f64,
        /// Inclusive upper truncation bound (`max > min`).
        max: f64,
    },
}

impl DistributionSpec {
    /// Validates distribution parameters (finiteness, strict positivity, and bound ordering).
    pub fn validate(&self) -> Result<(), MonteCarloError> {
        match *self {
            Self::Constant { value } => {
                if !value.is_finite() {
                    return Err(MonteCarloError::InvalidParameter {
                        parameter: "value",
                        value,
                    });
                }
            }
            Self::Uniform { min, max } => {
                if !min.is_finite() || !max.is_finite() || min >= max {
                    return Err(MonteCarloError::InvalidBounds { min, max });
                }
            }
            Self::Normal { mean, std_dev } => {
                validate_location_scale(mean, std_dev, "mean", "std_dev")?;
            }
            Self::LogNormal { mu_ln, sigma_ln } => {
                validate_location_scale(mu_ln, sigma_ln, "mu_ln", "sigma_ln")?;
            }
            Self::TruncatedNormal {
                mean,
                std_dev,
                min,
                max,
            } => {
                validate_location_scale(mean, std_dev, "mean", "std_dev")?;
                if !min.is_finite() || !max.is_finite() || min >= max {
                    return Err(MonteCarloError::InvalidBounds { min, max });
                }
            }
        }
        Ok(())
    }

    /// Draws one scalar realization from this distribution using `rng`.
    pub fn sample(&self, rng: &mut TrialRng) -> Result<f64, MonteCarloError> {
        self.validate()?;
        match *self {
            Self::Constant { value } => Ok(value),
            Self::Uniform { min, max } => rng.uniform(min, max),
            Self::Normal { mean, std_dev } => rng.normal(mean, std_dev),
            Self::LogNormal { mu_ln, sigma_ln } => rng.log_normal(mu_ln, sigma_ln),
            Self::TruncatedNormal {
                mean,
                std_dev,
                min,
                max,
            } => rng.truncated_normal(mean, std_dev, min, max),
        }
    }

    /// Writes this distribution canonically into `h` using IEEE-754 canonical float encoding.
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::Constant { value } => {
                h.write_u8(0);
                h.write_f64_canonical(value);
            }
            Self::Uniform { min, max } => {
                h.write_u8(1);
                h.write_f64_canonical(min);
                h.write_f64_canonical(max);
            }
            Self::Normal { mean, std_dev } => {
                h.write_u8(2);
                h.write_f64_canonical(mean);
                h.write_f64_canonical(std_dev);
            }
            Self::LogNormal { mu_ln, sigma_ln } => {
                h.write_u8(3);
                h.write_f64_canonical(mu_ln);
                h.write_f64_canonical(sigma_ln);
            }
            Self::TruncatedNormal {
                mean,
                std_dev,
                min,
                max,
            } => {
                h.write_u8(4);
                h.write_f64_canonical(mean);
                h.write_f64_canonical(std_dev);
                h.write_f64_canonical(min);
                h.write_f64_canonical(max);
            }
        }
    }
}

/// Named dispersion parameter and its independent RNG stream tag within a Monte Carlo campaign.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParameterDispersionSpec {
    /// Unique parameter identifier within the campaign.
    pub name: String,
    /// Caller-chosen stream tag mixed with `name` to isolate this parameter's RNG stream.
    pub stream_tag: u64,
    /// Parametric probability distribution for this parameter.
    pub distribution: DistributionSpec,
}

impl ParameterDispersionSpec {
    /// Validates that `name` is non-empty (not whitespace-only) and `distribution` is valid.
    pub fn validate(&self) -> Result<(), MonteCarloError> {
        if self.name.trim().is_empty() {
            return Err(MonteCarloError::EmptyParameterName);
        }
        self.distribution.validate()
    }

    /// Writes this parameter specification canonically into `h`.
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_bytes(self.name.as_bytes());
        h.write_u64(self.stream_tag);
        self.distribution.write_to_hasher(h);
    }

    /// Derives the isolated [`TrialStreamKey`] for this parameter at `(run_seed, trial_index)`.
    #[must_use]
    pub fn stream_key(&self, run_seed: u64, trial_index: u64) -> TrialStreamKey {
        // Mix `stream_tag` and `name` so distinct parameters never collide even if they share a tag.
        let mut h = DomainHasher::new(b"sim-spacetime:mc-param-stream:v1");
        h.write_u64(self.stream_tag);
        h.write_bytes(self.name.as_bytes());
        let digest = h.finish();
        let [b0, b1, b2, b3, b4, b5, b6, b7, ..] = *digest.as_bytes();
        TrialStreamKey::new(
            run_seed,
            trial_index,
            u64::from_le_bytes([b0, b1, b2, b3, b4, b5, b6, b7]),
        )
    }
}

/// Complete configuration of a reproducible Monte Carlo campaign.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MonteCarloConfig {
    /// Campaign label.
    pub campaign_name: String,
    /// Root campaign seed shared across all trials and parameter streams.
    pub run_seed: u64,
    /// Planned number of trials in the campaign.
    pub trial_count: u64,
    /// Independent per-parameter dispersion specifications.
    pub dispersions: Vec<ParameterDispersionSpec>,
}

impl MonteCarloConfig {
    /// Validates every parameter dispersion and verifies that all parameter names are unique.
    pub fn validate(&self) -> Result<(), MonteCarloError> {
        let mut seen = BTreeSet::new();
        for param in &self.dispersions {
            param.validate()?;
            if !seen.insert(param.name.as_str()) {
                return Err(MonteCarloError::DuplicateParameterName {
                    name: param.name.clone(),
                });
            }
        }
        Ok(())
    }

    /// Computes the canonical SHA-256 content hash of this configuration.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-spacetime:mc-config:v1");
        h.write_bytes(self.campaign_name.as_bytes());
        h.write_u64(self.run_seed);
        h.write_u64(self.trial_count);
        h.write_u64(self.dispersions.len() as u64);
        for param in &self.dispersions {
            param.write_to_hasher(&mut h);
        }
        h.finish()
    }
}

/// Eq/Hash-safe digest header of a [`MonteCarloRunManifest`], containing zero floating-point fields.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct MonteCarloManifestDigest {
    /// Root SHA-256 digest over `(schema_version, config_hash, model_digest)`.
    pub root_hash: ContentHash,
    /// Manifest schema version.
    pub schema_version: u32,
    /// Canonical SHA-256 digest of the [`MonteCarloConfig`].
    pub config_hash: ContentHash,
    /// Optional SHA-256 digest of the physical/simulation model or binary evaluated by the run.
    pub model_digest: Option<ContentHash>,
}

/// Versioned, integrity-sealed manifest for a Monte Carlo campaign.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MonteCarloRunManifest {
    /// Schema version ([`MONTE_CARLO_MANIFEST_SCHEMA_VERSION`]).
    pub schema_version: u32,
    /// Campaign dispersion and seeding configuration.
    pub config: MonteCarloConfig,
    /// Canonical SHA-256 digest of `config`.
    pub config_hash: ContentHash,
    /// Optional SHA-256 digest of the physical model, scenario, or executable evaluated by the run.
    pub model_digest: Option<ContentHash>,
}

impl MonteCarloRunManifest {
    /// Current schema version of [`MonteCarloRunManifest`].
    pub const SCHEMA_VERSION: u32 = MONTE_CARLO_MANIFEST_SCHEMA_VERSION;

    /// Validates `config`, seals its canonical `config_hash`, and builds a [`MonteCarloRunManifest`].
    pub fn build(
        config: MonteCarloConfig,
        model_digest: Option<ContentHash>,
    ) -> Result<Self, MonteCarloError> {
        config.validate()?;
        let config_hash = config.content_hash();
        Ok(Self {
            schema_version: Self::SCHEMA_VERSION,
            config,
            config_hash,
            model_digest,
        })
    }

    /// Verifies manifest schema version, parameter validation, and `config_hash` integrity.
    pub fn verify_integrity(&self) -> Result<(), MonteCarloError> {
        if self.schema_version != Self::SCHEMA_VERSION {
            return Err(MonteCarloError::UnsupportedSchemaVersion {
                version: self.schema_version,
            });
        }
        self.config.validate()?;
        let expected = self.config.content_hash();
        if expected != self.config_hash {
            return Err(MonteCarloError::ManifestIntegrityMismatch {
                expected,
                actual: self.config_hash,
            });
        }
        Ok(())
    }

    /// Computes the canonical manifest digest over `(schema_version, config.content_hash(), model_digest)`
    /// using `DomainHasher::new(b"sim-spacetime:mc-run-manifest:v1")`.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-spacetime:mc-run-manifest:v1");
        h.write_u32(self.schema_version);
        h.write_hash(self.config.content_hash());
        match self.model_digest {
            None => h.write_u8(0),
            Some(digest) => {
                h.write_u8(1);
                h.write_hash(digest);
            }
        }
        h.finish()
    }

    /// Extracts an `Eq + Ord + Hash`-safe [`MonteCarloManifestDigest`] for comparison and indexing.
    #[must_use]
    pub fn digest(&self) -> MonteCarloManifestDigest {
        let config_hash = self.config.content_hash();
        MonteCarloManifestDigest {
            root_hash: self.content_hash(),
            schema_version: self.schema_version,
            config_hash,
            model_digest: self.model_digest,
        }
    }

    /// Compares two manifests by their canonical `Eq`-safe digests.
    #[must_use]
    pub fn digest_eq(&self, other: &Self) -> bool {
        self.digest() == other.digest()
    }
}

/// Samples all dispersed parameters for `trial_index`, using an independent stream per named
/// parameter so adding, removing, or reordering parameters never perturbs existing parameters.
pub fn sample_trial_parameters(
    config: &MonteCarloConfig,
    trial_index: u64,
) -> Result<BTreeMap<String, f64>, MonteCarloError> {
    config.validate()?;
    let mut values = BTreeMap::new();
    for param in &config.dispersions {
        let mut rng = TrialRng::from_key(param.stream_key(config.run_seed, trial_index));
        let sample = param.distribution.sample(&mut rng)?;
        values.insert(param.name.clone(), sample);
    }
    Ok(values)
}

fn validate_location_scale(
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
