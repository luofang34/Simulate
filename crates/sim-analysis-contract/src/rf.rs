//! Serializable radio-link description and result types: antennas, pointing, receiver noise,
//! propagation-loss models, and the per-epoch link budget evaluated by `sim-analysis`.
//!
//! Only data lives here; the physics that turns these into numbers is in `sim-analysis`.

mod antenna;
mod budget;
mod link;
mod noise;
mod propagation;

pub use antenna::{AntennaSpec, CosinePowerPatchSpec, ParabolicDishSpec};
pub use budget::{
    BudgetUnavailableReason, LinkBudget, LinkBudgetReport, LinkBudgetSample, LinkEnd,
    PropagationLossTerm,
};
pub use link::{AntennaPointing, LinkDirection, RfLinkSpec, RfReceiverSpec, RfTransmitterSpec};
pub use noise::{BOLTZMANN_CONSTANT_J_K, ReceiverNoise};
pub use propagation::PropagationModelSpec;
