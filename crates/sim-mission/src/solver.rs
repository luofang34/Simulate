//! Solver request identity (`SolverRequestIdentity`), decision variables (`TargetVariableSpec`),
//! goals (`TargetGoalSpec`), and verified result commit (`VersionedSolverResult`).

mod verification;

use serde::{Deserialize, Serialize};
use sim_orbital::{
    ContentHash, DomainHasher, NaifId, PlanningInputFingerprint, SimClockAdapter, TdbEpoch,
    VesselCheckpoint,
};
use sim_spacetime::SimInstant;

use crate::{
    error::{MissionDesignError, SolverFailureReason},
    execution::MissionExecutionReport,
    plan::MissionPlan,
};

use verification::{
    compute_conclusion_binding_hash, verify_conclusion_and_plan_binding,
    verify_execution_artifact_binding, verify_request_and_context,
};

/// Decision variable adjusted by a mission differential-correction solver.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum TargetVariableSpec {
    /// Vary the `FixedDuration` of a finite-burn segment (`segment_id`) within `[min_burn_duration_s, max_burn_duration_s]`.
    VarySegmentBurnDuration {
        /// Segment ID of the burn segment in the [`MissionPlan`].
        segment_id: u32,
        /// Minimum allowed burn duration (`s`, `>= 0`).
        min_burn_duration_s: f64,
        /// Maximum allowed burn duration (`s`).
        max_burn_duration_s: f64,
        /// Finite-difference step $\delta t$ used to approximate the Jacobian (`s`, `> 0`).
        fd_perturbation_s: f64,
    },
}

impl TargetVariableSpec {
    /// Canonical SHA-256 hash of this decision variable specification.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-mission:target-variable-spec:v1");
        match *self {
            Self::VarySegmentBurnDuration {
                segment_id,
                min_burn_duration_s,
                max_burn_duration_s,
                fd_perturbation_s,
            } => {
                h.write_u8(1);
                h.write_u32(segment_id);
                h.write_f64_canonical(min_burn_duration_s);
                h.write_f64_canonical(max_burn_duration_s);
                h.write_f64_canonical(fd_perturbation_s);
            }
        }
        h.finish()
    }
}

/// Terminal equality goal targeted by a differential-correction solver.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum TargetGoalSpec {
    /// Achieve terminal radius $\|\mathbf{r}_{\text{rel}}(t_f)\| = R_{\text{target}}$ relative to `body` within `tolerance_m`.
    TerminalRadius {
        /// Central body for $\|\mathbf{r}_{\text{rel}}\|$.
        body: NaifId,
        /// Target radius (`m`).
        target_radius_m: f64,
        /// Convergence tolerance (`m`, `> 0`).
        tolerance_m: f64,
    },
}

impl TargetGoalSpec {
    /// Canonical SHA-256 hash of this goal specification.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-mission:target-goal-spec:v1");
        match *self {
            Self::TerminalRadius {
                body,
                target_radius_m,
                tolerance_m,
            } => {
                h.write_u8(1);
                h.write_i32(body.0);
                h.write_f64_canonical(target_radius_m);
                h.write_f64_canonical(tolerance_m);
            }
        }
        h.finish()
    }
}

/// Complete cryptographic identity of a differential-correction solver request.
///
/// Binds the base plan identity (`target_plan_id`, `base_plan_version`, `base_plan_content_hash`),
/// initial state (`initial_checkpoint_hash`), time anchor (`anchor_tdb`, `initial_instant`),
/// variable and goal specifications (`vary_spec_hash`, `goal_spec_hash`), environment and
/// numerical strategy (`input_fingerprint`), and `request_generation`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolverRequestIdentity {
    /// Target `MissionPlan::plan_id`.
    pub target_plan_id: u64,
    /// Base `MissionPlan::version` on which the solve was requested.
    pub base_plan_version: u64,
    /// Full content hash of the base `MissionPlan` (`MissionPlan::content_hash()`).
    pub base_plan_content_hash: ContentHash,
    /// Full snapshot content hash of the initial `VesselCheckpoint` (`snapshot_content_hash()`).
    pub initial_checkpoint_hash: ContentHash,
    /// Master-clock TDB anchor (`SimClockAdapter::anchor_tdb()`).
    pub anchor_tdb: TdbEpoch,
    /// Initial master-clock instant (`VesselCheckpoint::instant`).
    #[serde(with = "sim_spacetime::sim_instant_serde")]
    pub initial_instant: SimInstant,
    /// Content hash of the [`TargetVariableSpec`].
    pub vary_spec_hash: ContentHash,
    /// Content hash of the [`TargetGoalSpec`].
    pub goal_spec_hash: ContentHash,
    /// Environment, force-model, ephemeris, orientation, and tolerance fingerprint.
    pub input_fingerprint: PlanningInputFingerprint,
    /// Monotonic solver request generation number preventing stale async result commits.
    pub request_generation: u64,
}

impl SolverRequestIdentity {
    /// Constructs the canonical [`SolverRequestIdentity`] for a targeting request.
    #[must_use]
    pub fn build(
        base_plan: &MissionPlan,
        initial_checkpoint: &VesselCheckpoint,
        clock: &SimClockAdapter,
        vary_spec: &TargetVariableSpec,
        goal_spec: &TargetGoalSpec,
        input_fingerprint: PlanningInputFingerprint,
        request_generation: u64,
    ) -> Self {
        Self {
            target_plan_id: base_plan.plan_id,
            base_plan_version: base_plan.version,
            base_plan_content_hash: base_plan.content_hash(),
            initial_checkpoint_hash: initial_checkpoint.checkpoint_hash(),
            anchor_tdb: clock.anchor_tdb(),
            initial_instant: initial_checkpoint.instant,
            vary_spec_hash: vary_spec.content_hash(),
            goal_spec_hash: goal_spec.content_hash(),
            input_fingerprint,
            request_generation,
        }
    }
}

/// Convergence status of a differential-correction solver run.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SolverConvergenceStatus {
    /// Converged within the goal tolerance.
    Converged,
    /// Failed to converge for the specified reason.
    Failed(SolverFailureReason),
}

/// Versioned solver output bound to a complete [`SolverRequestIdentity`] and sealed
/// against post-solve mutation of `updated_plan`, `final_execution`, or convergence metrics.
#[derive(Clone, Debug, PartialEq)]
pub struct VersionedSolverResult {
    /// Complete identity of the request that produced this result.
    pub request_identity: SolverRequestIdentity,
    /// Decision variable specification used in the solve.
    pub vary_spec: TargetVariableSpec,
    /// Terminal goal specification verified by the solve.
    pub goal_spec: TargetGoalSpec,
    /// Convergence status.
    pub status: SolverConvergenceStatus,
    /// Number of Newton/secant iterations performed.
    pub iterations: u32,
    /// Solved burn duration (`s`).
    pub solved_burn_duration_s: f64,
    /// Achieved terminal radius (`m`).
    pub achieved_radius_m: f64,
    /// Final signed residual $r_{\text{achieved}} - r_{\text{target}}$ (`m`).
    pub residual_m: f64,
    /// Updated [`MissionPlan`] with `version = base_plan_version + 1` and solved burn duration.
    pub updated_plan: MissionPlan,
    /// Full [`MissionExecutionReport`] of `updated_plan`.
    pub final_execution: MissionExecutionReport,
    pub(super) verified_updated_plan_hash: ContentHash,
    pub(super) verified_final_checkpoint_hash: ContentHash,
    pub(super) verified_trajectory_hash: ContentHash,
    pub(super) conclusion_binding_hash: ContentHash,
}

impl VersionedSolverResult {
    /// Constructs a sealed [`VersionedSolverResult`] binding the solver conclusion to
    /// `updated_plan` and `final_execution`.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new_verified(
        request_identity: SolverRequestIdentity,
        vary_spec: TargetVariableSpec,
        goal_spec: TargetGoalSpec,
        status: SolverConvergenceStatus,
        iterations: u32,
        solved_burn_duration_s: f64,
        achieved_radius_m: f64,
        residual_m: f64,
        updated_plan: MissionPlan,
        final_execution: MissionExecutionReport,
    ) -> Self {
        let verified_updated_plan_hash = updated_plan.content_hash();
        let verified_final_checkpoint_hash = final_execution.final_checkpoint.checkpoint_hash();
        let verified_trajectory_hash = final_execution.trajectory_product.provenance.content_hash;
        let conclusion_binding_hash = compute_conclusion_binding_hash(
            &request_identity,
            status,
            iterations,
            solved_burn_duration_s,
            achieved_radius_m,
            residual_m,
            verified_updated_plan_hash,
            verified_final_checkpoint_hash,
            verified_trajectory_hash,
        );
        Self {
            request_identity,
            vary_spec,
            goal_spec,
            status,
            iterations,
            solved_burn_duration_s,
            achieved_radius_m,
            residual_m,
            updated_plan,
            final_execution,
            verified_updated_plan_hash,
            verified_final_checkpoint_hash,
            verified_trajectory_hash,
            conclusion_binding_hash,
        }
    }

    /// Verifies that this solver result is applicable to `(expected_request, current_plan, current_checkpoint, current_clock)`
    /// and that `updated_plan`, `final_execution`, and all convergence metrics remain bound to the solver verification conclusion.
    pub fn verify_applicable(
        &self,
        expected_request: &SolverRequestIdentity,
        current_plan: &MissionPlan,
        current_checkpoint: &VesselCheckpoint,
        current_clock: &SimClockAdapter,
    ) -> Result<(), MissionDesignError> {
        verify_request_and_context(
            self,
            expected_request,
            current_plan,
            current_checkpoint,
            current_clock,
        )?;
        verify_conclusion_and_plan_binding(self, current_plan)?;
        verify_execution_artifact_binding(self)
    }

    /// Verifies [`Self::verify_applicable`] and atomically commits `self.updated_plan` into `current_plan`.
    pub fn commit_to_plan(
        &self,
        expected_request: &SolverRequestIdentity,
        current_plan: &mut MissionPlan,
        current_checkpoint: &VesselCheckpoint,
        current_clock: &SimClockAdapter,
    ) -> Result<(), MissionDesignError> {
        self.verify_applicable(
            expected_request,
            current_plan,
            current_checkpoint,
            current_clock,
        )?;
        *current_plan = self.updated_plan.clone();
        Ok(())
    }
}
