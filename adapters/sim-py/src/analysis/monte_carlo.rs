//! Reproducible Monte Carlo dispersion streams and SHA-256 run manifests (`sim_spacetime::monte_carlo`).

use pyo3::{prelude::*, types::PyDict};
use sim_spacetime::{
    DistributionSpec, EnsembleSummary, MonteCarloConfig, MonteCarloRunManifest,
    ParameterDispersionSpec, TrialRng, cholesky_lower, sample_trial_parameters,
};

use crate::error::{SimPyError, require_finite};

/// Deterministic ChaCha20 Monte Carlo stream bound to a sealed [`MonteCarloRunManifest`].
#[pyclass]
pub struct MonteCarloStream {
    rng: TrialRng,
    manifest: MonteCarloRunManifest,
}

#[pymethods]
impl MonteCarloStream {
    /// Creates a deterministic stream keyed by `(seed, label, stream_index, trial_index)`.
    #[new]
    #[pyo3(signature = (seed, label, stream_index = 0, trial_index = 0))]
    fn new(seed: u64, label: &str, stream_index: u64, trial_index: u64) -> PyResult<Self> {
        let param = ParameterDispersionSpec {
            name: label.to_owned(),
            stream_tag: stream_index,
            distribution: DistributionSpec::Normal {
                mean: 0.0,
                std_dev: 1.0,
            },
        };
        let key = param.stream_key(seed, trial_index);
        let config = MonteCarloConfig {
            campaign_name: label.to_owned(),
            run_seed: seed,
            trial_count: 1,
            dispersions: vec![param],
        };
        let manifest = MonteCarloRunManifest::build(config, None).map_err(SimPyError::from)?;
        manifest.verify_integrity().map_err(SimPyError::from)?;
        Ok(Self {
            rng: TrialRng::from_key(key),
            manifest,
        })
    }

    /// Draws the next raw 64-bit unsigned integer from the `ChaCha20` stream.
    fn next_u64(&mut self) -> u64 {
        self.rng.next_u64()
    }

    /// Draws a unit `f64` in `[0.0, 1.0)`.
    fn next_unit(&mut self) -> f64 {
        self.rng.next_f64_unit()
    }

    /// Draws a continuous uniform sample in `[min, max)`.
    fn uniform(&mut self, min: f64, max: f64) -> PyResult<f64> {
        Ok(self.rng.uniform(min, max).map_err(SimPyError::from)?)
    }

    /// Draws a univariate Gaussian sample `N(mean, std_dev^2)` via portable Box-Muller.
    #[pyo3(signature = (mean = 0.0, std_dev = 1.0))]
    fn normal(&mut self, mean: f64, std_dev: f64) -> PyResult<f64> {
        Ok(self.rng.normal(mean, std_dev).map_err(SimPyError::from)?)
    }

    /// Draws a Gaussian sample truncated to `[min, max]`.
    fn truncated_normal(&mut self, mean: f64, std_dev: f64, min: f64, max: f64) -> PyResult<f64> {
        Ok(self
            .rng
            .truncated_normal(mean, std_dev, min, max)
            .map_err(SimPyError::from)?)
    }

    /// Draws a correlated multivariate Gaussian vector given `mean` and row-major `covariance_row_major`.
    fn correlated_normal(
        &mut self,
        mean: Vec<f64>,
        covariance_row_major: Vec<f64>,
    ) -> PyResult<Vec<f64>> {
        let lower = cholesky_lower(&covariance_row_major, mean.len()).map_err(SimPyError::from)?;
        Ok(self
            .rng
            .correlated_normal(&mean, &lower)
            .map_err(SimPyError::from)?)
    }

    /// Canonical SHA-256 hex digest of the sealed [`MonteCarloRunManifest`].
    #[getter]
    fn manifest_sha256(&self) -> String {
        self.manifest.content_hash().to_hex()
    }

    /// Canonical SHA-256 hex digest of the [`MonteCarloConfig`].
    #[getter]
    fn config_sha256(&self) -> String {
        self.manifest.config_hash.to_hex()
    }

    /// Serializes the sealed [`MonteCarloRunManifest`] as canonical JSON.
    fn manifest_json(&self) -> PyResult<String> {
        Ok(serde_json::to_string(&self.manifest).map_err(SimPyError::from)?)
    }
}

/// Generates `draw_count` deterministic Monte Carlo samples and returns them alongside the SHA-256 manifest digest.
#[pyfunction]
#[pyo3(signature = (seed, label, stream_index = 0, draw_count = 16, distribution = None, param_a = None, param_b = None))]
#[allow(clippy::too_many_arguments)]
pub fn monte_carlo_sample(
    py: Python<'_>,
    seed: u64,
    label: &str,
    stream_index: u64,
    draw_count: usize,
    distribution: Option<&str>,
    param_a: Option<f64>,
    param_b: Option<f64>,
) -> PyResult<Py<PyDict>> {
    let dist = parse_distribution(distribution, param_a, param_b)?;
    let config = MonteCarloConfig {
        campaign_name: label.to_owned(),
        run_seed: seed,
        trial_count: draw_count as u64,
        dispersions: vec![ParameterDispersionSpec {
            name: label.to_owned(),
            stream_tag: stream_index,
            distribution: dist,
        }],
    };
    let manifest = MonteCarloRunManifest::build(config, None).map_err(SimPyError::from)?;
    manifest.verify_integrity().map_err(SimPyError::from)?;
    let mut samples = Vec::with_capacity(draw_count);
    for trial in 0..draw_count {
        let drawn =
            sample_trial_parameters(&manifest.config, trial as u64).map_err(SimPyError::from)?;
        let val = drawn.get(label).copied().ok_or_else(|| {
            SimPyError::InvalidInput(format!("missing sampled parameter `{label}`"))
        })?;
        samples.push(val);
    }
    let summary = EnsembleSummary::from_samples(&samples);
    let manifest_json = serde_json::to_string(&manifest).map_err(SimPyError::from)?;
    let dict = PyDict::new(py);
    dict.set_item("samples", samples)?;
    dict.set_item("manifest_sha256", manifest.content_hash().to_hex())?;
    dict.set_item("config_sha256", manifest.config_hash.to_hex())?;
    dict.set_item("schema_version", manifest.schema_version)?;
    dict.set_item("manifest_json", manifest_json)?;
    dict.set_item("count", summary.count())?;
    dict.set_item("mean", summary.mean())?;
    dict.set_item(
        "std_dev",
        summary.sample_std_dev().or_else(|| summary.std_dev()),
    )?;
    dict.set_item("min", summary.min())?;
    dict.set_item("max", summary.max())?;
    Ok(dict.unbind())
}

fn parse_distribution(
    distribution: Option<&str>,
    param_a: Option<f64>,
    param_b: Option<f64>,
) -> Result<DistributionSpec, SimPyError> {
    let kind = distribution.map_or("normal", str::trim);
    match kind.to_ascii_lowercase().as_str() {
        "normal" | "standard_normal" => Ok(DistributionSpec::Normal {
            mean: require_finite(param_a.unwrap_or(0.0), "param_a")?,
            std_dev: require_finite(param_b.unwrap_or(1.0), "param_b")?,
        }),
        "uniform" => Ok(DistributionSpec::Uniform {
            min: require_finite(param_a.unwrap_or(0.0), "param_a")?,
            max: require_finite(param_b.unwrap_or(1.0), "param_b")?,
        }),
        "log_normal" | "lognormal" => Ok(DistributionSpec::LogNormal {
            mu_ln: require_finite(param_a.unwrap_or(0.0), "param_a")?,
            sigma_ln: require_finite(param_b.unwrap_or(1.0), "param_b")?,
        }),
        "constant" => Ok(DistributionSpec::Constant {
            value: require_finite(param_a.unwrap_or(0.0), "param_a")?,
        }),
        _ => Err(SimPyError::InvalidInput(format!(
            "unsupported distribution `{kind}` (expected normal, uniform, log_normal, or constant)"
        ))),
    }
}

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<MonteCarloStream>()?;
    m.add_function(wrap_pyfunction!(monte_carlo_sample, m)?)?;
    Ok(())
}
