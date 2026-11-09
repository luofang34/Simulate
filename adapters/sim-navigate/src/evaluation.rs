//! Scoring navigation outputs against simulation truth, means by means.
//!
//! Three record tiers stay apart by type: [`SimTruth`] is what the simulator
//! knows about the vehicle, [`SensorTruth`] is the ground truth of one
//! sensor channel's observation, and [`NavigationOutput`] is what a
//! navigation means computed. Truth travels as [`Truth`] and outputs as
//! [`Estimate`], so a function that needs one cannot be handed the other.
//! Scores are pure functions of a truth and an output; wall time and memory
//! from [`ComputeMeter`] sit beside them and never enter the simulation.

mod ablation;
mod compute;
mod metrics;
mod records;

pub use ablation::{AblationEvaluation, AblationSet, AblationSummary, NavMeans, NavigationOrigin};
pub use compute::{ComputeMeter, ComputeSample, ComputeStats};
pub use metrics::{EvaluationSummary, MeansEvaluation, Percentiles, PositionScore, score_position};
pub use records::{
    CameraTruth, Estimate, ImageTruth, NavigationOutput, SensorTruth, SensorTruthRecord, SimTruth,
    StarFieldTruth, StarTruth, ThermalTruth, Truth,
};
