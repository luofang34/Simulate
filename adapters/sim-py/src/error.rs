//! Typed error mapping from `Simulate` domain errors into Python exceptions.

use glam::DVec3;
use pyo3::{
    PyErr,
    exceptions::{PyRuntimeError, PyValueError},
};
use thiserror::Error;

/// Unified error type preserving `Simulate` typed error sources before converting to `PyErr`.
#[derive(Debug, Error)]
pub(crate) enum SimPyError {
    /// Time-scale catalog or conversion failure.
    #[error(transparent)]
    Time(#[from] sim_spacetime::TimeError),
    /// SGP4/SDP4 catalog parsing or propagation failure.
    #[error(transparent)]
    Sgp4(#[from] sim_orbital::Sgp4Error),
    /// Variational / B-plane encounter computation failure.
    #[error(transparent)]
    Variational(#[from] sim_orbital::VariationalError),
    /// CCSDS NDM parsing or serialization error.
    #[error(transparent)]
    CcsdsNdm(#[from] ccsds_ndm::Error),
    /// CCSDS ASCII epoch parsing error.
    #[error(transparent)]
    CcsdsEpoch(#[from] ccsds_ndm::EpochError),
    /// `sim-orbital` CCSDS adapter conversion error.
    #[error(transparent)]
    CcsdsAdapter(#[from] sim_orbital::CcsdsError),
    /// NAIF SPICE DAF/SPK/CK parsing, evaluation, or encoding error.
    #[error(transparent)]
    Spice(#[from] spice_kernels::Error),
    /// Geodesic H3 coverage grid validation or evaluation error.
    #[error(transparent)]
    Coverage(#[from] sim_analysis::CoverageGridError),
    /// Monte Carlo RNG, Cholesky, or manifest error.
    #[error(transparent)]
    MonteCarlo(#[from] sim_spacetime::MonteCarloError),
    /// Trajectory query error.
    #[error(transparent)]
    Trajectory(#[from] sim_spacetime::TrajectoryQueryError),
    /// Embedded solar-system catalog verification error.
    #[error(transparent)]
    BuiltinSolar(#[from] sim_orbital::BuiltinSolarError),
    /// JSON serialization or deserialization error.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    /// Caller supplied malformed or non-finite scalar/vector inputs.
    #[error("{0}")]
    InvalidInput(String),
}

impl From<SimPyError> for PyErr {
    fn from(err: SimPyError) -> Self {
        match &err {
            SimPyError::BuiltinSolar(_) => PyRuntimeError::new_err(err.to_string()),
            _ => PyValueError::new_err(err.to_string()),
        }
    }
}

/// Validates that `value` is finite (`!NaN && !Inf`).
pub(crate) fn require_finite(value: f64, name: &str) -> Result<f64, SimPyError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(SimPyError::InvalidInput(format!(
            "`{name}` must be finite, got {value}"
        )))
    }
}

/// Validates that all three Cartesian components are finite and converts to [`DVec3`].
pub(crate) fn require_finite_vec3(v: [f64; 3], name: &str) -> Result<DVec3, SimPyError> {
    let [x, y, z] = v;
    if x.is_finite() && y.is_finite() && z.is_finite() {
        Ok(DVec3::new(x, y, z))
    } else {
        Err(SimPyError::InvalidInput(format!(
            "`{name}` components must all be finite, got [{x}, {y}, {z}]"
        )))
    }
}
