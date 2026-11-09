//! Spaceflight mission planning (`MissionPlan`), independent runtime execution state
//! (`MissionExecutionState`), lifecycle control (`start_mission_execution`,
//! `resume_mission_execution`, `cancel_mission_execution`), event-driven segment propagation,
//! solver request identity (`SolverRequestIdentity`), and finite-burn targeting (`solve_apoapsis_target`).

mod error;
mod event_root;
mod execution;
mod plan;
mod propagation;
mod solver;
mod targeter;

pub use error::{MissionDesignError, MissionLifecycleStatus, SolverFailureReason};
pub use execution::{
    ExecutionLimits, MissionExecutionReport, MissionExecutionState, ResumptionGuarantee,
    SegmentExecutionRecord, SegmentTerminationReason, cancel_mission_execution,
    execute_mission_plan, execute_mission_plan_with_limits, resume_mission_execution,
    start_mission_execution,
};
pub use plan::{
    BurnSteeringFrame, CrossingDirection, MissionPlan, MissionSegmentNode, OrbitalScalarEventKind,
    PlanBudgetScope, SegmentBurnLaw, SegmentStoppingCondition,
};
pub use solver::{
    SolverConvergenceStatus, SolverRequestIdentity, TargetGoalSpec, TargetVariableSpec,
    VersionedSolverResult,
};
pub use targeter::{ApoapsisTargetProblem, solve_apoapsis_target};
