//! Radio signal layer over the geometric link analysis: antenna patterns, path loss and
//! Doppler, pluggable propagation losses, and the per-epoch link budget.
//!
//! Geometry comes from `evaluate_endpoint_link_access`; this layer only adds the
//! electromagnetic terms for epochs the geometry already accepts.

mod antenna;
mod budget;
mod evaluator;
mod path;
mod units;
mod world_prop;

#[cfg(test)]
mod tests;

pub use antenna::AntennaPattern;
pub use budget::{LinkBudgetInputs, compute_link_budget};
pub(crate) use evaluator::budget_sample;
pub use evaluator::{LinkBudgetModels, evaluate_endpoint_link_budget};
pub use path::{
    ExceedancePercentages, MediumQueryLoss, PathGeometry, PropagationLoss, PropagationLossError,
    doppler_shift_hz, free_space_path_loss_db,
};
pub use units::{db_from_ratio, ratio_from_db};
pub use world_prop::{
    DeltaBullingtonPropagationLoss, F1245Pattern, ItmPropagationLoss, KnifeEdgePropagationLoss,
    KnifeEdgeSource, MainBeamPolicy, S465Pattern, S580Pattern, SmoothEarthPropagationLoss,
};
