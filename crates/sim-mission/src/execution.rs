//! Independent mission execution state (`MissionExecutionState`), lifecycle transitions
//! (`Idle`, `Running`, `Paused`, `Completed`, `Failed`, `Cancelled`), explicit resumption
//! guarantees (`ResumptionGuarantee`), and checkpointed execution/resumption entry points.

mod budget_accounting;
mod segment_runner;

use serde::{Deserialize, Serialize};
use sim_orbital::{
    BodyOrientationCatalog, ContentHash, ContinuousPlantState, EphemerisCatalog, ForceModel,
    OrbitalTrajectoryProduct, PlanningInputFingerprint, PlantTrajectorySegment, SimClockAdapter,
    TdbEpoch, VesselCheckpoint, VesselConfig,
};
use sim_spacetime::{DynamicalModelIdentity, SimInstant, TICKS_PER_SECOND};

use crate::{
    error::{MissionDesignError, MissionLifecycleStatus},
    plan::{MissionPlan, OrbitalScalarEventKind},
};

use segment_runner::{SegmentRunContext, run_mission_segment};

/// Explicit numerical consistency guarantee of a paused-and-resumed mission execution
/// relative to a single uninterrupted execution of the same [`MissionPlan`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ResumptionGuarantee {
    /// Execution either ran uninterrupted or paused strictly at a segment boundary
    /// (`segment_elapsed_ticks == 0`). `DOP853` restarts at the segment boundary with
    /// identical initial step selection, guaranteeing `0 ulp` bitwise identity with
    /// an uninterrupted run.
    BitwiseIdenticalAtSegmentBoundary,
    /// Execution was paused inside a segment (`segment_elapsed_ticks > 0`) by `pause_after_s`,
    /// introducing an intermediate `DOP853` knot. The resumed trajectory is guaranteed to
    /// agree with an uninterrupted run within the configured [`sim_orbital::Dop853Tolerances`].
    IntegratorToleranceConsistentMidSegment,
}

/// Step-count and wall/sim-duration limits enforced across both `FixedDuration` and `ScalarEvent` segments.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExecutionLimits {
    /// Maximum cumulative accepted `DOP853` steps allowed before failing with `ExecutionStepCapExceeded`.
    pub max_accepted_steps: u32,
    /// Optional simulation-time limit (`s`) after which execution pauses cleanly (`MissionLifecycleStatus::Paused`).
    pub pause_after_s: Option<f64>,
}

impl Default for ExecutionLimits {
    fn default() -> Self {
        Self {
            max_accepted_steps: 200_000,
            pause_after_s: None,
        }
    }
}

/// Why an individual [`MissionSegmentNode`](crate::MissionSegmentNode) completed.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SegmentTerminationReason {
    /// Completed its full `FixedDuration`.
    FixedDurationCompleted,
    /// Triggered its `ScalarEvent` stopping condition.
    ScalarEventTriggered(OrbitalScalarEventKind),
}

/// Execution telemetry record for one completed segment in a [`MissionPlan`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SegmentExecutionRecord {
    /// Segment identifier (`MissionSegmentNode::segment_id`).
    pub segment_id: u32,
    /// Segment label.
    pub label: String,
    /// Segment start epoch (`TDB`).
    pub start_epoch: TdbEpoch,
    /// Segment completion epoch (`TDB`).
    pub end_epoch: TdbEpoch,
    /// Total elapsed duration of the segment (`s`).
    pub duration_s: f64,
    /// Propellant consumed across the segment (`kg`).
    pub propellant_consumed_kg: f64,
    /// Ideal rocket $\Delta v$ expended across the segment (`m/s`).
    pub delta_v_m_s: f64,
    /// Accepted `DOP853` steps taken in the segment.
    pub accepted_steps: u32,
    /// Reason the segment terminated.
    pub termination: SegmentTerminationReason,
}

/// Independent runtime execution state of a [`MissionPlan`], separated from `sim-orbital::DiscreteGncState`.
///
/// Binds the full cryptographic plan identity (`plan_content_hash`), the master-clock TDB anchor
/// (`anchor_tdb`), the initial checkpoint identity (`initial_checkpoint_hash`), and the environment
/// [`PlanningInputFingerprint`] so any silent mutation of segments, time anchors, or force models
/// is rejected on resumption.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MissionExecutionState {
    /// Bound `MissionPlan::plan_id`.
    pub plan_id: u64,
    /// Bound `MissionPlan::version`.
    pub plan_version: u64,
    /// Full SHA-256 content hash of the `MissionPlan` (`MissionPlan::content_hash()`).
    pub plan_content_hash: ContentHash,
    /// Bound `SimClockAdapter::anchor_tdb()` epoch.
    pub anchor_tdb: TdbEpoch,
    /// Master-clock instant when mission execution started.
    #[serde(with = "sim_spacetime::sim_instant_serde")]
    pub initial_instant: SimInstant,
    /// Snapshot hash of the initial `VesselCheckpoint`.
    pub initial_checkpoint_hash: ContentHash,
    /// Cryptographic fingerprint of the force model, ephemeris, orientation, and vessel config.
    pub input_fingerprint: PlanningInputFingerprint,
    /// Current lifecycle status (`Idle`, `Running`, `Paused`, `Completed`, `Failed`, `Cancelled`).
    pub status: MissionLifecycleStatus,
    /// Index of the currently active (or next-to-execute) segment in `plan.segments`.
    pub active_segment_index: usize,
    /// Elapsed master-clock ticks already integrated within `plan.segments[active_segment_index]`.
    pub segment_elapsed_ticks: u128,
    /// Cumulative propellant consumed across all segments and partial chunks (`kg`).
    pub plan_consumed_propellant_kg: f64,
    /// Cumulative ideal $\Delta v$ expended across all segments and partial chunks (`m/s`).
    pub accumulated_delta_v_m_s: f64,
    /// Consistency guarantee of the trajectory across pause/resume boundaries.
    pub resumption_guarantee: ResumptionGuarantee,
    /// Completed segment records in execution order.
    pub completed_segments: Vec<SegmentExecutionRecord>,
    /// Accumulated dense trajectory segments from all completed and partial chunks.
    pub accumulated_segments: Vec<PlantTrajectorySegment>,
}

/// Result returned by [`start_mission_execution`], [`resume_mission_execution`], and [`execute_mission_plan`].
#[derive(Clone, Debug, PartialEq)]
pub struct MissionExecutionReport {
    /// Plan ID.
    pub plan_id: u64,
    /// Plan version.
    pub plan_version: u64,
    /// Full cryptographic hash of the executed `MissionPlan`.
    pub plan_content_hash: ContentHash,
    /// Resulting lifecycle status (`Completed` or `Paused`).
    pub status: MissionLifecycleStatus,
    /// Consistency guarantee (`BitwiseIdenticalAtSegmentBoundary` vs `IntegratorToleranceConsistentMidSegment`).
    pub resumption_guarantee: ResumptionGuarantee,
    /// Updated spacecraft checkpoint at the end (or pause point) of execution.
    pub final_checkpoint: VesselCheckpoint,
    /// Updated independent [`MissionExecutionState`] for inspection or subsequent [`resume_mission_execution`].
    pub execution_state: MissionExecutionState,
    /// Completed segment records up to this point.
    pub segment_records: Vec<SegmentExecutionRecord>,
    /// Cumulative propellant mass consumed (`kg`).
    pub total_propellant_consumed_kg: f64,
    /// Cumulative ideal $\Delta v$ expended (`m/s`).
    pub total_delta_v_m_s: f64,
    /// Propagator-agnostic orbital trajectory product covering the executed span.
    pub trajectory_product: OrbitalTrajectoryProduct,
}

/// Starts a fresh [`MissionPlan`] execution from `initial_checkpoint` with default [`ExecutionLimits`].
#[allow(clippy::too_many_arguments)]
pub fn execute_mission_plan(
    plan: &MissionPlan,
    initial_checkpoint: &VesselCheckpoint,
    vessel_cfg: &VesselConfig,
    clock: &SimClockAdapter,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<MissionExecutionReport, MissionDesignError> {
    start_mission_execution(
        plan,
        initial_checkpoint,
        vessel_cfg,
        clock,
        force_model,
        ephemeris,
        orientation,
        ExecutionLimits::default(),
    )
}

/// Alias of [`start_mission_execution`] executing a fresh [`MissionPlan`] under explicit [`ExecutionLimits`].
pub use start_mission_execution as execute_mission_plan_with_limits;

/// Initializes a fresh [`MissionExecutionState`] and executes `plan` from `initial_checkpoint` under `limits`.
#[allow(clippy::too_many_arguments)]
pub fn start_mission_execution(
    plan: &MissionPlan,
    initial_checkpoint: &VesselCheckpoint,
    vessel_cfg: &VesselConfig,
    clock: &SimClockAdapter,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    limits: ExecutionLimits,
) -> Result<MissionExecutionReport, MissionDesignError> {
    validate_plan_and_model(plan, initial_checkpoint, force_model)?;
    let fp = PlanningInputFingerprint::compute(
        plan.model_identity,
        force_model,
        ephemeris,
        orientation,
        vessel_cfg,
    );
    let exec_state = MissionExecutionState {
        plan_id: plan.plan_id,
        plan_version: plan.version,
        plan_content_hash: plan.content_hash(),
        anchor_tdb: clock.anchor_tdb(),
        initial_instant: initial_checkpoint.instant,
        initial_checkpoint_hash: initial_checkpoint.checkpoint_hash(),
        input_fingerprint: fp,
        status: MissionLifecycleStatus::Running,
        active_segment_index: 0,
        segment_elapsed_ticks: 0,
        plan_consumed_propellant_kg: 0.0,
        accumulated_delta_v_m_s: 0.0,
        resumption_guarantee: ResumptionGuarantee::BitwiseIdenticalAtSegmentBoundary,
        completed_segments: Vec::new(),
        accumulated_segments: Vec::new(),
    };
    drive_execution_loop(
        plan,
        initial_checkpoint.clone(),
        exec_state,
        vessel_cfg,
        clock,
        force_model,
        ephemeris,
        orientation,
        limits,
    )
}

/// Resumes a `Paused` [`MissionExecutionState`] from `paused_checkpoint`, verifying full
/// `MissionPlan::content_hash()`, `SimClockAdapter::anchor_tdb()`, and `PlanningInputFingerprint`.
#[allow(clippy::too_many_arguments)]
pub fn resume_mission_execution(
    plan: &MissionPlan,
    paused_checkpoint: &VesselCheckpoint,
    mut exec_state: MissionExecutionState,
    vessel_cfg: &VesselConfig,
    clock: &SimClockAdapter,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    limits: ExecutionLimits,
) -> Result<MissionExecutionReport, MissionDesignError> {
    if exec_state.status != MissionLifecycleStatus::Paused {
        return Err(MissionDesignError::InvalidLifecycleTransition {
            status: exec_state.status,
            operation: "resume_mission_execution",
        });
    }
    if plan.plan_id != exec_state.plan_id || plan.version != exec_state.plan_version {
        return Err(MissionDesignError::PlanVersionMismatch {
            expected_plan_id: exec_state.plan_id,
            expected_version: exec_state.plan_version,
            actual_plan_id: plan.plan_id,
            actual_version: plan.version,
        });
    }
    let actual_plan_hash = plan.content_hash();
    if actual_plan_hash != exec_state.plan_content_hash {
        return Err(MissionDesignError::PlanContentMutatedWithoutVersionBump {
            plan_id: plan.plan_id,
            version: plan.version,
            expected_hash: exec_state.plan_content_hash,
            actual_hash: actual_plan_hash,
        });
    }
    if clock.anchor_tdb() != exec_state.anchor_tdb {
        return Err(MissionDesignError::TimeAnchorMismatch {
            expected_anchor: exec_state.anchor_tdb,
            actual_anchor: clock.anchor_tdb(),
        });
    }
    validate_plan_and_model(plan, paused_checkpoint, force_model)?;
    let fp = PlanningInputFingerprint::compute(
        plan.model_identity,
        force_model,
        ephemeris,
        orientation,
        vessel_cfg,
    );
    if fp != exec_state.input_fingerprint {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "input_fingerprint",
        });
    }
    exec_state.status = MissionLifecycleStatus::Running;
    drive_execution_loop(
        plan,
        paused_checkpoint.clone(),
        exec_state,
        vessel_cfg,
        clock,
        force_model,
        ephemeris,
        orientation,
        limits,
    )
}

/// Explicitly cancels an `Idle`, `Running`, or `Paused` [`MissionExecutionState`].
pub fn cancel_mission_execution(
    exec_state: &mut MissionExecutionState,
) -> Result<(), MissionDesignError> {
    match exec_state.status {
        MissionLifecycleStatus::Idle
        | MissionLifecycleStatus::Running
        | MissionLifecycleStatus::Paused => {
            exec_state.status = MissionLifecycleStatus::Cancelled;
            Ok(())
        }
        other => Err(MissionDesignError::InvalidLifecycleTransition {
            status: other,
            operation: "cancel_mission_execution",
        }),
    }
}

fn validate_plan_and_model(
    plan: &MissionPlan,
    checkpoint: &VesselCheckpoint,
    force_model: &ForceModel,
) -> Result<(), MissionDesignError> {
    if plan.segments.is_empty() {
        return Err(MissionDesignError::InvalidPlan {
            plan_id: plan.plan_id,
            reason: "mission plan must contain at least one segment".to_owned(),
        });
    }
    if checkpoint.vessel_id != plan.vessel_id {
        return Err(MissionDesignError::VesselIdMismatch {
            expected: plan.vessel_id,
            actual: checkpoint.vessel_id,
        });
    }
    match plan.model_identity {
        DynamicalModelIdentity::SolarSystemEphemerisNBody { force_model_hash } => {
            if force_model_hash != force_model.content_hash() {
                return Err(MissionDesignError::DynamicalModelMismatch {
                    plan_id: plan.plan_id,
                    reason: "plan force_model_hash does not match execution ForceModel".to_owned(),
                });
            }
            Ok(())
        }
        other => Err(MissionDesignError::DynamicalModelMismatch {
            plan_id: plan.plan_id,
            reason: format!(
                "mission executor requires SolarSystemEphemerisNBody force model, got {other:?}"
            ),
        }),
    }
}

#[allow(clippy::too_many_arguments)]
fn drive_execution_loop(
    plan: &MissionPlan,
    mut ckpt: VesselCheckpoint,
    mut exec_state: MissionExecutionState,
    vessel_cfg: &VesselConfig,
    clock: &SimClockAdapter,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    limits: ExecutionLimits,
) -> Result<MissionExecutionReport, MissionDesignError> {
    let initial_state_for_product = ckpt.plant;
    let initial_epoch_for_product = clock.instant_to_tdb(ckpt.instant);
    let mut run_steps_used = 0_u32;
    let mut run_elapsed_s = 0.0_f64;

    while let Some(seg_node) = plan.segments.get(exec_state.active_segment_index) {
        if let Some(pause_s) = limits.pause_after_s
            && run_elapsed_s + 1.0e-12 >= pause_s
        {
            exec_state.status = MissionLifecycleStatus::Paused;
            if exec_state.segment_elapsed_ticks > 0 {
                exec_state.resumption_guarantee =
                    ResumptionGuarantee::IntegratorToleranceConsistentMidSegment;
            }
            break;
        }
        let ctx = SegmentRunContext {
            plan,
            seg_node,
            vessel_cfg,
            force_model,
            ephemeris,
            orientation,
            start_epoch: clock.instant_to_tdb(ckpt.instant),
            start_state: ckpt.plant,
            already_elapsed_ticks: exec_state.segment_elapsed_ticks,
            prior_consumed_fuel_kg: exec_state.plan_consumed_propellant_kg,
            prior_delta_v_m_s: exec_state.accumulated_delta_v_m_s,
            remaining_step_budget: limits.max_accepted_steps.saturating_sub(run_steps_used),
            remaining_pause_s: limits.pause_after_s.map(|p| (p - run_elapsed_s).max(0.0)),
        };
        let outcome = run_mission_segment(&ctx)?;
        run_steps_used = run_steps_used.saturating_add(outcome.accepted_steps);
        run_elapsed_s += (outcome.advance_elapsed_ticks as f64) / (TICKS_PER_SECOND as f64);
        if apply_segment_outcome(&mut ckpt, &mut exec_state, clock, outcome) {
            break;
        }
    }

    Ok(build_execution_report(
        plan,
        vessel_cfg,
        initial_state_for_product,
        initial_epoch_for_product,
        ckpt,
        exec_state,
    ))
}

fn apply_segment_outcome(
    ckpt: &mut VesselCheckpoint,
    exec_state: &mut MissionExecutionState,
    clock: &SimClockAdapter,
    outcome: segment_runner::SegmentRunOutcome,
) -> bool {
    ckpt.plant = outcome.end_state;
    ckpt.instant = clock
        .tdb_to_instant(outcome.end_epoch)
        .unwrap_or(ckpt.instant);
    exec_state.plan_consumed_propellant_kg += outcome.propellant_consumed_kg;
    exec_state.accumulated_delta_v_m_s += outcome.delta_v_m_s;
    exec_state
        .accumulated_segments
        .extend(outcome.dense_segments);
    if outcome.paused_mid_segment {
        exec_state.segment_elapsed_ticks = exec_state
            .segment_elapsed_ticks
            .saturating_add(outcome.advance_elapsed_ticks);
        exec_state.status = MissionLifecycleStatus::Paused;
        exec_state.resumption_guarantee =
            ResumptionGuarantee::IntegratorToleranceConsistentMidSegment;
        return true;
    }
    if let Some(rec) = outcome.completed_record {
        exec_state.completed_segments.push(rec);
    }
    exec_state.active_segment_index = exec_state.active_segment_index.wrapping_add(1);
    exec_state.segment_elapsed_ticks = 0;
    ckpt.gnc.maneuver_cursor = ckpt.gnc.maneuver_cursor.wrapping_add(1);
    false
}

fn build_execution_report(
    plan: &MissionPlan,
    vessel_cfg: &VesselConfig,
    initial_state_for_product: ContinuousPlantState,
    initial_epoch_for_product: TdbEpoch,
    ckpt: VesselCheckpoint,
    mut exec_state: MissionExecutionState,
) -> MissionExecutionReport {
    if exec_state.active_segment_index >= plan.segments.len() {
        exec_state.status = MissionLifecycleStatus::Completed;
    }
    let prod_start_ep = exec_state
        .accumulated_segments
        .first()
        .map_or(initial_epoch_for_product, |s| s.start_epoch);
    let prod_start_state =
        exec_state
            .accumulated_segments
            .first()
            .map_or(initial_state_for_product, |s| {
                let y0 = s.as_dense().evaluate_tau(0.0);
                ContinuousPlantState::from_ode_array(s.primary_body, &y0, true)
            });
    let trajectory_product = OrbitalTrajectoryProduct::from_segments(
        plan.plan_id,
        exec_state.input_fingerprint,
        vessel_cfg.mass_model.dry_mass_kg,
        prod_start_state,
        ckpt.plant,
        prod_start_ep,
        exec_state.accumulated_segments.clone(),
    );
    MissionExecutionReport {
        plan_id: plan.plan_id,
        plan_version: plan.version,
        plan_content_hash: exec_state.plan_content_hash,
        status: exec_state.status,
        resumption_guarantee: exec_state.resumption_guarantee,
        final_checkpoint: ckpt,
        segment_records: exec_state.completed_segments.clone(),
        total_propellant_consumed_kg: exec_state.plan_consumed_propellant_kg,
        total_delta_v_m_s: exec_state.accumulated_delta_v_m_s,
        execution_state: exec_state,
        trajectory_product,
    }
}
