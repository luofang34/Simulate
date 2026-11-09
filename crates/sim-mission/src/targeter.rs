//! Finite-burn apoapsis differential-correction targeter (`ApoapsisTargetProblem`,
//! `solve_apoapsis_target`) producing a verified [`VersionedSolverResult`].

use sim_orbital::{
    BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, ForceModel, NaifId,
    PlanningInputFingerprint, PropellantMode, SimClockAdapter, TdbEpoch, VesselCheckpoint,
    VesselConfig,
};
use sim_spacetime::sqrt;

use crate::{
    error::{MissionDesignError, SolverFailureReason},
    execution::{MissionExecutionReport, execute_mission_plan},
    plan::{MissionPlan, SegmentStoppingCondition},
    solver::{
        SolverConvergenceStatus, SolverRequestIdentity, TargetGoalSpec, TargetVariableSpec,
        VersionedSolverResult,
    },
};

/// Problem specification for finite-burn apoapsis targeting (`VarySegmentBurnDuration` $\to$ `TerminalRadius`).
#[derive(Clone, Debug, PartialEq)]
pub struct ApoapsisTargetProblem {
    /// Base mission plan containing the burn segment and subsequent coast-to-apoapsis segment.
    pub base_plan: MissionPlan,
    /// Decision variable specification (`VarySegmentBurnDuration`).
    pub vary: TargetVariableSpec,
    /// Terminal radius goal specification (`TerminalRadius`).
    pub goal: TargetGoalSpec,
    /// Monotonic solver request generation number.
    pub request_generation: u64,
    /// Maximum damped Newton/secant iterations (`>= 1`).
    pub max_iterations: u32,
}

struct TrialEvalContext<'a> {
    problem: &'a ApoapsisTargetProblem,
    burn_idx: usize,
    goal_body: NaifId,
    initial_checkpoint: &'a VesselCheckpoint,
    vessel_cfg: &'a VesselConfig,
    clock: &'a SimClockAdapter,
    force_model: &'a ForceModel,
    ephemeris: &'a dyn EphemerisCatalog,
    orientation: &'a dyn BodyOrientationCatalog,
}

/// Solves a finite-burn apoapsis targeting problem via damped finite-difference Newton/secant
/// differential correction over full-ephemeris `DOP853` execution, returning a [`VersionedSolverResult`]
/// bound to the complete [`SolverRequestIdentity`].
pub fn solve_apoapsis_target(
    problem: &ApoapsisTargetProblem,
    initial_checkpoint: &VesselCheckpoint,
    vessel_cfg: &VesselConfig,
    clock: &SimClockAdapter,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<VersionedSolverResult, MissionDesignError> {
    let (burn_idx, cur_burn_s) = validate_target_problem(problem, vessel_cfg)?;
    let TargetGoalSpec::TerminalRadius {
        body: goal_body, ..
    } = problem.goal;
    let fp = PlanningInputFingerprint::compute(
        problem.base_plan.model_identity,
        force_model,
        ephemeris,
        orientation,
        vessel_cfg,
    );
    let request_identity = SolverRequestIdentity::build(
        &problem.base_plan,
        initial_checkpoint,
        clock,
        &problem.vary,
        &problem.goal,
        fp,
        problem.request_generation,
    );
    let ctx = TrialEvalContext {
        problem,
        burn_idx,
        goal_body,
        initial_checkpoint,
        vessel_cfg,
        clock,
        force_model,
        ephemeris,
        orientation,
    };
    run_damped_newton_iterations(&ctx, request_identity, cur_burn_s)
}

fn run_damped_newton_iterations(
    ctx: &TrialEvalContext<'_>,
    request_identity: SolverRequestIdentity,
    mut cur_burn_s: f64,
) -> Result<VersionedSolverResult, MissionDesignError> {
    let TargetVariableSpec::VarySegmentBurnDuration {
        min_burn_duration_s,
        max_burn_duration_s,
        fd_perturbation_s,
        ..
    } = ctx.problem.vary;
    let TargetGoalSpec::TerminalRadius {
        target_radius_m,
        tolerance_m,
        ..
    } = ctx.problem.goal;

    let mut iter = 0_u32;
    let mut last_residual_m = f64::INFINITY;
    let mut clamped_streak = 0_u32;

    while iter < ctx.problem.max_iterations {
        iter = iter.wrapping_add(1);
        let (r_cur, plan_cur, rep_cur) = eval_trial_burn(ctx, cur_burn_s, iter, last_residual_m)?;
        let res = r_cur - target_radius_m;
        last_residual_m = res;
        if res.abs() <= tolerance_m {
            return Ok(VersionedSolverResult::new_verified(
                request_identity,
                ctx.problem.vary,
                ctx.problem.goal,
                SolverConvergenceStatus::Converged,
                iter,
                cur_burn_s,
                r_cur,
                res,
                plan_cur,
                rep_cur,
            ));
        }
        cur_burn_s = compute_next_damped_burn_s(
            ctx,
            cur_burn_s,
            r_cur,
            res,
            min_burn_duration_s,
            max_burn_duration_s,
            fd_perturbation_s,
            iter,
            &mut clamped_streak,
        )?;
    }

    Err(MissionDesignError::TargeterDidNotConverge {
        reason: SolverFailureReason::MaxIterationsExceeded,
        iterations: iter,
        final_residual_m: last_residual_m,
    })
}

fn validate_target_problem(
    problem: &ApoapsisTargetProblem,
    vessel_cfg: &VesselConfig,
) -> Result<(usize, f64), MissionDesignError> {
    let TargetVariableSpec::VarySegmentBurnDuration {
        segment_id,
        min_burn_duration_s,
        max_burn_duration_s,
        fd_perturbation_s,
    } = problem.vary;
    let TargetGoalSpec::TerminalRadius {
        target_radius_m,
        tolerance_m,
        ..
    } = problem.goal;

    if vessel_cfg.propellant_mode != PropellantMode::FiniteVariableMass {
        return Err(MissionDesignError::RequiresFiniteVariableMass);
    }
    if !min_burn_duration_s.is_finite()
        || !max_burn_duration_s.is_finite()
        || min_burn_duration_s < 0.0
        || max_burn_duration_s <= min_burn_duration_s
        || !fd_perturbation_s.is_finite()
        || fd_perturbation_s <= 0.0
        || !target_radius_m.is_finite()
        || target_radius_m <= 0.0
        || !tolerance_m.is_finite()
        || tolerance_m <= 0.0
        || problem.max_iterations == 0
    {
        return Err(MissionDesignError::InvalidPlan {
            plan_id: problem.base_plan.plan_id,
            reason: "invalid ApoapsisTargetProblem bounds or tolerances".to_owned(),
        });
    }

    let burn_idx = problem
        .base_plan
        .segments
        .iter()
        .position(|s| s.segment_id == segment_id)
        .ok_or_else(|| MissionDesignError::InvalidPlan {
            plan_id: problem.base_plan.plan_id,
            reason: format!("burn segment_id {segment_id} not found in base_plan"),
        })?;

    match problem.base_plan.segments.get(burn_idx).map(|s| s.stopping) {
        Some(SegmentStoppingCondition::FixedDuration { duration_s }) => Ok((
            burn_idx,
            duration_s.clamp(min_burn_duration_s, max_burn_duration_s),
        )),
        _ => Err(MissionDesignError::InvalidPlan {
            plan_id: problem.base_plan.plan_id,
            reason: format!("segment {segment_id} must have FixedDuration stopping condition"),
        }),
    }
}

fn eval_trial_burn(
    ctx: &TrialEvalContext<'_>,
    burn_s: f64,
    iter: u32,
    last_residual_m: f64,
) -> Result<(f64, MissionPlan, MissionExecutionReport), MissionDesignError> {
    let mut trial_plan = ctx.problem.base_plan.clone();
    trial_plan.version = ctx.problem.base_plan.version.wrapping_add(1);
    if let Some(seg) = trial_plan.segments.get_mut(ctx.burn_idx) {
        seg.stopping = SegmentStoppingCondition::FixedDuration { duration_s: burn_s };
    }
    let report = execute_mission_plan(
        &trial_plan,
        ctx.initial_checkpoint,
        ctx.vessel_cfg,
        ctx.clock,
        ctx.force_model,
        ctx.ephemeris,
        ctx.orientation,
    )
    .map_err(|err| {
        let reason = match err {
            MissionDesignError::PropellantExhausted { .. }
            | MissionDesignError::FuelBudgetExceeded { .. } => SolverFailureReason::FuelExhausted,
            _ => SolverFailureReason::TrialExecutionFailed,
        };
        MissionDesignError::TargeterDidNotConverge {
            reason,
            iterations: iter,
            final_residual_m: last_residual_m,
        }
    })?;
    let end_ep = ctx.clock.instant_to_tdb(report.final_checkpoint.instant);
    let r_m = terminal_radius_m(
        &report.final_checkpoint,
        ctx.goal_body,
        end_ep,
        ctx.ephemeris,
    )?;
    Ok((r_m, trial_plan, report))
}

#[allow(clippy::too_many_arguments)]
fn compute_next_damped_burn_s(
    ctx: &TrialEvalContext<'_>,
    cur_burn_s: f64,
    r_cur: f64,
    res: f64,
    min_burn_duration_s: f64,
    max_burn_duration_s: f64,
    fd_perturbation_s: f64,
    iter: u32,
    clamped_streak: &mut u32,
) -> Result<f64, MissionDesignError> {
    let step_fd = if cur_burn_s + fd_perturbation_s <= max_burn_duration_s {
        fd_perturbation_s
    } else {
        -fd_perturbation_s
    };
    let pert_burn_s = (cur_burn_s + step_fd).clamp(min_burn_duration_s, max_burn_duration_s);
    let (r_pert, _, _) = eval_trial_burn(ctx, pert_burn_s, iter, res)?;
    let dr_dt = (r_pert - r_cur) / step_fd;
    if !dr_dt.is_finite() || dr_dt.abs() < 1.0e-3 {
        return Err(MissionDesignError::TargeterDidNotConverge {
            reason: SolverFailureReason::SingularSensitivity,
            iterations: iter,
            final_residual_m: res,
        });
    }

    let raw_step = -(res / dr_dt);
    let max_step = 0.5 * (max_burn_duration_s - min_burn_duration_s);
    let damped_step = raw_step.clamp(-max_step, max_step);
    let unclamped_next = cur_burn_s + damped_step;
    let next_burn_s = unclamped_next.clamp(min_burn_duration_s, max_burn_duration_s);
    if (next_burn_s - unclamped_next).abs() > 1.0e-9 && (next_burn_s - cur_burn_s).abs() <= 1.0e-9 {
        *clamped_streak = clamped_streak.wrapping_add(1);
        if *clamped_streak >= 2 {
            return Err(MissionDesignError::TargeterDidNotConverge {
                reason: SolverFailureReason::ClampedAtBoundsWithoutConvergence,
                iterations: iter,
                final_residual_m: res,
            });
        }
    } else {
        *clamped_streak = 0;
    }
    Ok(next_burn_s)
}

fn terminal_radius_m(
    ckpt: &VesselCheckpoint,
    goal_body: NaifId,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
) -> Result<f64, MissionDesignError> {
    let pos_rel = if ckpt.plant.primary_body == goal_body {
        ckpt.plant.pos_rel_icrf_m
    } else {
        let shift = ephemeris.geometric_state_icrf(
            ckpt.plant.primary_body,
            goal_body,
            epoch,
            DerivativeOrder::Position,
        )?;
        ckpt.plant.pos_rel_icrf_m + shift.pos_m
    };
    Ok(sqrt(pos_rel.length_squared()))
}
