//! Structured error and failure-reason types for `sim-mission` execution,
//! checkpoint resumption, and differential-correction solver commits.

use serde::{Deserialize, Serialize};
use sim_orbital::{ContentHash, Dop853Error, EphemerisError, GravityError, NaifId, VesselId};
use sim_spacetime::{SimInstant, SpacetimeTransformError, TdbEpoch};
use thiserror::Error;

use crate::plan::OrbitalScalarEventKind;

/// Lifecycle state of an active or completed mission execution (`MissionExecutionState`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MissionLifecycleStatus {
    /// Initialized but not yet started.
    Idle,
    /// Actively executing segments.
    Running,
    /// Paused cleanly at a segment boundary or mid-segment execution limit (`pause_after_s`).
    Paused,
    /// All segments in the [`MissionPlan`](crate::MissionPlan) completed.
    Completed,
    /// Execution terminated due to an error (`EventNotTriggered`, `FuelBudgetExceeded`, etc.).
    Failed,
    /// Explicitly cancelled by the caller or operator.
    Cancelled,
}

/// Reason a differential-correction solver (`solve_apoapsis_target`) terminated without converging.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SolverFailureReason {
    /// Finite-difference Jacobian $\partial r_a / \partial \Delta t_{\text{burn}}$ was nearly singular.
    SingularSensitivity,
    /// Trial evaluation exhausted propellant or exceeded `MissionPlan::max_fuel_budget_kg`.
    FuelExhausted,
    /// Trial evaluation failed during segment execution or event root isolation.
    TrialExecutionFailed,
    /// Exceeded `max_iterations` without reaching `radius_tolerance_m`.
    MaxIterationsExceeded,
    /// Solved burn duration fell outside `[min_burn_duration_s, max_burn_duration_s]` and remained unconverged.
    ClampedAtBoundsWithoutConvergence,
}

/// Errors returned during mission plan execution, checkpoint resumption, or solver result commit.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum MissionDesignError {
    /// Coordinate frame or state transformation error.
    #[error("spacetime frame transform error: {0}")]
    Transform(#[from] SpacetimeTransformError),
    /// Ephemeris lookup error.
    #[error("ephemeris error in mission execution: {0}")]
    Ephemeris(#[from] EphemerisError),
    /// Gravity evaluation error.
    #[error("gravity error in mission execution: {0}")]
    Gravity(#[from] GravityError),
    /// Continuous `DOP853` integration error.
    #[error("DOP853 propagation error in mission segment: {0}")]
    Propagation(#[from] Dop853Error<GravityError>),
    /// Plan had zero segments or an invalid segment duration/tolerance parameter.
    #[error("invalid mission plan ({plan_id}): {reason}")]
    InvalidPlan {
        /// Plan ID.
        plan_id: u64,
        /// Explanation of the invalid field.
        reason: String,
    },
    /// Checkpoint `VesselId` did not match `MissionPlan::vessel_id`.
    #[error("vessel ID mismatch: plan expects {expected:?}, checkpoint has {actual:?}")]
    VesselIdMismatch {
        /// Plan vessel ID.
        expected: VesselId,
        /// Checkpoint vessel ID.
        actual: VesselId,
    },
    /// Plan dynamical model identity did not match the execution force model or unsupported CR3BP model was passed.
    #[error("dynamical model mismatch on plan {plan_id}: {reason}")]
    DynamicalModelMismatch {
        /// Plan ID.
        plan_id: u64,
        /// Description of the mismatch.
        reason: String,
    },
    /// Attempted to resume or commit a solver result against a different `(plan_id, version)`.
    #[error(
        "plan version mismatch: expected (plan_id={expected_plan_id}, version={expected_version}), got (plan_id={actual_plan_id}, version={actual_version})"
    )]
    PlanVersionMismatch {
        /// Expected plan ID.
        expected_plan_id: u64,
        /// Expected plan version.
        expected_version: u64,
        /// Actual plan ID.
        actual_plan_id: u64,
        /// Actual plan version.
        actual_version: u64,
    },
    /// Plan had the same `(plan_id, version)` but its segment contents, order, burn laws, stopping conditions,
    /// or budget limits were mutated (`plan.content_hash() != expected_hash`).
    #[error(
        "plan content mutated without bumping version on (plan_id={plan_id}, version={version}): expected hash {expected_hash}, got {actual_hash}"
    )]
    PlanContentMutatedWithoutVersionBump {
        /// Plan ID.
        plan_id: u64,
        /// Unbumped plan version.
        version: u64,
        /// Hash recorded when execution started or solver request was issued.
        expected_hash: ContentHash,
        /// Hash of the mutated plan passed at resume or commit.
        actual_hash: ContentHash,
    },
    /// Time anchor (`SimClockAdapter::anchor_tdb()`) changed between mission start / solver request and resume / commit.
    #[error("clock TDB anchor mutated: expected {expected_anchor:?}, got {actual_anchor:?}")]
    TimeAnchorMismatch {
        /// Expected TDB anchor epoch.
        expected_anchor: TdbEpoch,
        /// Actual TDB anchor epoch on the supplied `SimClockAdapter`.
        actual_anchor: TdbEpoch,
    },
    /// Initial checkpoint content hash changed between solver request and result commit.
    #[error("initial checkpoint hash mismatch: expected {expected_hash}, got {actual_hash}")]
    InitialCheckpointMismatch {
        /// Expected initial checkpoint snapshot hash.
        expected_hash: ContentHash,
        /// Actual checkpoint snapshot hash.
        actual_hash: ContentHash,
    },
    /// Initial `SimInstant` changed between solver request and result commit.
    #[error("initial SimInstant mismatch: expected {expected_instant:?}, got {actual_instant:?}")]
    InitialInstantMismatch {
        /// Expected initial master-clock instant.
        expected_instant: SimInstant,
        /// Actual master-clock instant.
        actual_instant: SimInstant,
    },
    /// Environment or solver request identity field (`input_fingerprint`, `vary_spec_hash`, `goal_spec_hash`, `request_generation`) did not match.
    #[error("solver or execution identity mismatch on field `{field}`")]
    IdentityFieldMismatch {
        /// Name of the mismatched identity field.
        field: &'static str,
    },
    /// Solver result artifact (`updated_plan`, `final_execution`, or solved residual/duration/status)
    /// lost binding with the verified solver conclusion.
    #[error("solver artifact binding mismatch on `{field}`: {reason}")]
    SolverArtifactBindingMismatch {
        /// Name of the unbound or tampered artifact field.
        field: &'static str,
        /// Explanation of the binding violation.
        reason: String,
    },
    /// Attempted an invalid lifecycle operation (e.g. resuming a `Completed`, `Failed`, or `Cancelled` mission).
    #[error("invalid lifecycle transition from {status:?} during `{operation}`")]
    InvalidLifecycleTransition {
        /// Current lifecycle status.
        status: MissionLifecycleStatus,
        /// Attempted operation name.
        operation: &'static str,
    },
    /// Scalar stopping event did not trigger within `max_duration_s`.
    #[error(
        "segment {segment_id} (`{label}`) failed to trigger event {event:?} within max_duration_s = {max_duration_s} s"
    )]
    EventNotTriggered {
        /// Segment ID.
        segment_id: u32,
        /// Segment label.
        label: String,
        /// Event kind that failed to cross zero.
        event: OrbitalScalarEventKind,
        /// Maximum search horizon (`s`).
        max_duration_s: f64,
    },
    /// Spacecraft propellant mass reached zero during a finite-burn segment.
    #[error("propellant exhausted (0 kg remaining) during segment {segment_id} (`{label}`)")]
    PropellantExhausted {
        /// Segment ID.
        segment_id: u32,
        /// Segment label.
        label: String,
    },
    /// Cumulative propellant consumption exceeded `MissionPlan::max_fuel_budget_kg`.
    #[error(
        "plan fuel budget exceeded during segment {segment_id}: consumed {consumed_kg:.3} kg > budget {budget_kg:.3} kg"
    )]
    FuelBudgetExceeded {
        /// Segment ID.
        segment_id: u32,
        /// Consumed propellant (`kg`).
        consumed_kg: f64,
        /// Configured budget (`kg`).
        budget_kg: f64,
    },
    /// Cumulative ideal $\Delta v$ exceeded `MissionPlan::delta_v_budget_m_s`.
    #[error(
        "plan delta-v budget exceeded during segment {segment_id}: used {used_m_s:.3} m/s > budget {budget_m_s:.3} m/s"
    )]
    DeltaVBudgetExceeded {
        /// Segment ID.
        segment_id: u32,
        /// Accumulated $\Delta v$ (`m/s`).
        used_m_s: f64,
        /// Configured $\Delta v$ budget (`m/s`).
        budget_m_s: f64,
    },
    /// Cumulative accepted `DOP853` steps across the plan exceeded `ExecutionLimits::max_accepted_steps`.
    #[error(
        "mission execution exceeded max_accepted_steps ({max_steps}) during segment {segment_id}"
    )]
    ExecutionStepCapExceeded {
        /// Segment ID.
        segment_id: u32,
        /// Step cap.
        max_steps: u32,
    },
    /// Differential-correction targeter failed to converge.
    #[error(
        "apoapsis targeter failed after {iterations} iterations (residual_m = {final_residual_m:.3}): {reason:?}"
    )]
    TargeterDidNotConverge {
        /// Specific failure reason.
        reason: SolverFailureReason,
        /// Iterations performed.
        iterations: u32,
        /// Final apoapsis radius residual (`m`).
        final_residual_m: f64,
    },
    /// Missing gravitational parameter $\mu$ for a central body.
    #[error("missing gravitational parameter mu for body {0}")]
    MissingBodyMu(NaifId),
    /// Finite-burn targeter requires `PropellantMode::FiniteVariableMass`.
    #[error("finite-burn targeter requires PropellantMode::FiniteVariableMass")]
    RequiresFiniteVariableMass,
}
