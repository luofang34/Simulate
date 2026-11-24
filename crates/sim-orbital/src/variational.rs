//! Variational equations, state transition matrix (`STM`), covariance propagation,
//! local `RIC` frame transforms, and short-term B-plane encounter collision probability.

mod bplane;
mod covariance;
mod gradient;
mod stm;
#[cfg(test)]
mod tests;

use crate::{dop853::Dop853Error, gravity::GravityError, time::TdbEpoch};

pub use bplane::{BPlaneEncounterResult, compute_bplane_collision_probability};
pub use covariance::{ProcessNoiseQ, StateCovariance6, icrf_to_ric_rotation};
pub use gradient::gravity_gradient_icrf;
pub use stm::{StmPropagationResult, propagate_state_and_stm};

/// Errors produced during variational propagation, covariance transforms, or B-plane analysis.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum VariationalError {
    /// Gravitational acceleration or gradient tensor evaluation failed.
    #[error("gravity error during variational evaluation: {0}")]
    Gravity(#[from] GravityError),
    /// Adaptive `DOP853` integration of the 42-dimensional variational ODE failed.
    #[error("DOP853 variational propagation failed: {0}")]
    Propagation(#[from] Dop853Error<GravityError>),
    /// Requested propagation interval has `final_epoch < initial_epoch`.
    #[error(
        "invalid variational epoch interval: final_epoch {final_epoch:?} precedes initial_epoch {initial_epoch:?}"
    )]
    InvalidEpochInterval {
        /// Propagation start epoch.
        initial_epoch: TdbEpoch,
        /// Propagation end epoch.
        final_epoch: TdbEpoch,
    },
    /// Integrator tolerances or step bounds are non-finite or non-positive.
    #[error("invalid DOP853 variational tolerances or step bounds")]
    InvalidTolerances,
    /// Input position, velocity, or matrix contained non-finite floating-point values.
    #[error("non-finite state or covariance input")]
    NonFiniteState,
    /// Radial vector or specific angular momentum $\mathbf{r} \times \mathbf{v}$ is degenerate.
    #[error("degenerate ICRF state for RIC orbital frame construction")]
    DegenerateRicFrame,
    /// Combined hard-body collision radius $R_c$ is non-finite or non-positive.
    #[error("invalid combined hard-body radius {combined_radius_m} m (must be finite and > 0)")]
    InvalidCombinedRadius {
        /// Invalid hard-body radius supplied by the caller (`m`).
        combined_radius_m: f64,
    },
    /// Relative encounter velocity is zero or non-finite, leaving the B-plane undefined.
    #[error("degenerate encounter with zero or non-finite relative velocity")]
    DegenerateEncounter,
    /// Input covariance matrix failed symmetry or positive-semidefiniteness checks.
    #[error("covariance matrix is asymmetric or not positive semidefinite")]
    NonPositiveSemidefiniteCovariance,
    /// Combined $2 \times 2$ B-plane position covariance is singular or non-positive-definite.
    #[error("projected 2x2 B-plane position covariance is singular")]
    SingularBPlaneCovariance,
}
