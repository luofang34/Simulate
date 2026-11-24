//! Analysis bindings: geodesic H3 planetary coverage grid evaluation (`sim-analysis`)
//! and reproducible Monte Carlo dispersion streams and manifests (`sim-spacetime`).

mod coverage;
mod monte_carlo;
#[cfg(test)]
mod tests;

use pyo3::prelude::*;

/// Registers coverage and Monte Carlo bindings on the Python module.
pub(crate) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    coverage::register(m)?;
    monte_carlo::register(m)?;
    Ok(())
}
