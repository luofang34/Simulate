//! Step-by-step segment execution engine enforcing `max_accepted_steps`, propellant/$\Delta v$
//! budgets, immediate dense-output event root isolation, and `pause_after_s` limits on both
//! `FixedDuration` and `ScalarEvent` segments.

use core::cell::RefCell;

use sim_orbital::{
    BodyOrientationCatalog, ContinuousPlantState, EphemerisCatalog, ForceModel,
    PlantTrajectorySegment, TdbEpoch, VesselConfig, propagate_plant_until_step,
};
use sim_spacetime::{TICKS_PER_SECOND, TickSpan};

use crate::{
    error::MissionDesignError,
    event_root::{find_earliest_event_root_in_segment, truncate_segment_at_tau},
    execution::{
        SegmentExecutionRecord, SegmentTerminationReason,
        budget_accounting::{
            build_fixed_duration_outcome, check_step_propellant_and_budgets,
            compute_propellant_and_dv, enforce_cumulative_budgets, sub_span_tdb,
        },
    },
    plan::{
        CrossingDirection, MissionPlan, MissionSegmentNode, OrbitalScalarEventKind,
        SegmentStoppingCondition,
    },
    propagation::StageEvaluatedBurnLaw,
};

pub(crate) struct SegmentRunOutcome {
    pub(crate) end_epoch: TdbEpoch,
    pub(crate) end_state: ContinuousPlantState,
    pub(crate) dense_segments: Vec<PlantTrajectorySegment>,
    pub(crate) accepted_steps: u32,
    pub(crate) propellant_consumed_kg: f64,
    pub(crate) delta_v_m_s: f64,
    pub(crate) paused_mid_segment: bool,
    pub(crate) advance_elapsed_ticks: u128,
    pub(crate) completed_record: Option<SegmentExecutionRecord>,
}

pub(crate) struct SegmentRunContext<'a> {
    pub(crate) plan: &'a MissionPlan,
    pub(crate) seg_node: &'a MissionSegmentNode,
    pub(crate) vessel_cfg: &'a VesselConfig,
    pub(crate) force_model: &'a ForceModel,
    pub(crate) ephemeris: &'a dyn EphemerisCatalog,
    pub(crate) orientation: &'a dyn BodyOrientationCatalog,
    pub(crate) start_epoch: TdbEpoch,
    pub(crate) start_state: ContinuousPlantState,
    pub(crate) already_elapsed_ticks: u128,
    pub(crate) prior_consumed_fuel_kg: f64,
    pub(crate) prior_delta_v_m_s: f64,
    pub(crate) remaining_step_budget: u32,
    pub(crate) remaining_pause_s: Option<f64>,
}

pub(crate) fn run_mission_segment(
    ctx: &SegmentRunContext<'_>,
) -> Result<SegmentRunOutcome, MissionDesignError> {
    let already_elapsed_s = (ctx.already_elapsed_ticks as f64) / (TICKS_PER_SECOND as f64);
    match ctx.seg_node.stopping {
        SegmentStoppingCondition::FixedDuration { duration_s } => {
            if !duration_s.is_finite() || duration_s < 0.0 {
                return Err(MissionDesignError::InvalidPlan {
                    plan_id: ctx.plan.plan_id,
                    reason: format!(
                        "segment {} has negative duration {duration_s}",
                        ctx.seg_node.segment_id
                    ),
                });
            }
            run_fixed_duration_segment(ctx, duration_s, already_elapsed_s)
        }
        SegmentStoppingCondition::ScalarEvent {
            event,
            direction,
            max_duration_s,
            min_search_offset_s,
            time_tolerance_s,
        } => {
            if !max_duration_s.is_finite()
                || max_duration_s <= 0.0
                || !min_search_offset_s.is_finite()
                || min_search_offset_s < 0.0
                || !time_tolerance_s.is_finite()
                || time_tolerance_s <= 0.0
            {
                return Err(MissionDesignError::InvalidPlan {
                    plan_id: ctx.plan.plan_id,
                    reason: format!(
                        "segment {} has invalid scalar event bounds",
                        ctx.seg_node.segment_id
                    ),
                });
            }
            run_scalar_event_segment(
                ctx,
                event,
                direction,
                max_duration_s,
                min_search_offset_s,
                time_tolerance_s,
                already_elapsed_s,
            )
        }
    }
}

fn compute_segment_horizon(
    ctx: &SegmentRunContext<'_>,
    total_duration_s: f64,
    already_elapsed_s: f64,
) -> (f64, u128, TdbEpoch, bool) {
    let rem_s = (total_duration_s - already_elapsed_s).max(0.0);
    let (chunk_s, paused_mid_segment) = match ctx.remaining_pause_s {
        Some(pause_s) if pause_s + 1.0e-12 < rem_s => (pause_s.max(0.0), true),
        _ => (rem_s, false),
    };
    let chunk_ticks = (chunk_s * (TICKS_PER_SECOND as f64)).round() as u128;
    let target_end_epoch = ctx
        .start_epoch
        .add_span(TickSpan::from_ticks_u128(chunk_ticks));
    (chunk_s, chunk_ticks, target_end_epoch, paused_mid_segment)
}

fn run_fixed_duration_segment(
    ctx: &SegmentRunContext<'_>,
    total_duration_s: f64,
    already_elapsed_s: f64,
) -> Result<SegmentRunOutcome, MissionDesignError> {
    let (chunk_s, chunk_ticks, target_end_epoch, paused_mid_segment) =
        compute_segment_horizon(ctx, total_duration_s, already_elapsed_s);
    let control = StageEvaluatedBurnLaw::new(
        ctx.plan.plan_id,
        ctx.seg_node.segment_id,
        ctx.seg_node.burn_law,
        ctx.ephemeris,
    );
    let step_err: RefCell<Option<MissionDesignError>> = RefCell::new(None);
    let mut steps_seen = 0_u32;

    let prop_res = propagate_plant_until_step(
        &ctx.start_state,
        ctx.start_epoch,
        target_end_epoch,
        None,
        &ctx.vessel_cfg.tolerances,
        &ctx.vessel_cfg.mass_model,
        ctx.vessel_cfg.propellant_mode,
        ctx.vessel_cfg.rw_mode,
        ctx.force_model,
        ctx.ephemeris,
        ctx.orientation,
        &control,
        false,
        &mut |seg| {
            steps_seen = steps_seen.wrapping_add(1);
            if steps_seen > ctx.remaining_step_budget {
                *step_err.borrow_mut() = Some(MissionDesignError::ExecutionStepCapExceeded {
                    segment_id: ctx.seg_node.segment_id,
                    max_steps: ctx.remaining_step_budget,
                });
                return true;
            }
            if let Err(e) = check_step_propellant_and_budgets(ctx, seg) {
                *step_err.borrow_mut() = Some(e);
                return true;
            }
            false
        },
    );
    if let Some(err) = control.take_error() {
        return Err(err);
    }
    let prop_out = prop_res?;
    if let Some(err) = step_err.into_inner() {
        return Err(err);
    }

    build_fixed_duration_outcome(
        ctx,
        total_duration_s,
        chunk_s,
        chunk_ticks,
        target_end_epoch,
        paused_mid_segment,
        prop_out.final_state,
        prop_out.segments,
        prop_out.stats.accepted_steps,
    )
}

#[allow(clippy::too_many_arguments)]
fn run_scalar_event_segment(
    ctx: &SegmentRunContext<'_>,
    event: OrbitalScalarEventKind,
    direction: CrossingDirection,
    max_duration_s: f64,
    min_search_offset_s: f64,
    time_tolerance_s: f64,
    already_elapsed_s: f64,
) -> Result<SegmentRunOutcome, MissionDesignError> {
    let (horizon_s, horizon_ticks, horizon_end_epoch, paused_by_limit) =
        compute_segment_horizon(ctx, max_duration_s, already_elapsed_s);
    let control = StageEvaluatedBurnLaw::new(
        ctx.plan.plan_id,
        ctx.seg_node.segment_id,
        ctx.seg_node.burn_law,
        ctx.ephemeris,
    );
    let step_err: RefCell<Option<MissionDesignError>> = RefCell::new(None);
    let found_root: RefCell<Option<(usize, f64)>> = RefCell::new(None);
    let mut steps_seen = 0_u32;

    let prop_res = propagate_plant_until_step(
        &ctx.start_state,
        ctx.start_epoch,
        horizon_end_epoch,
        None,
        &ctx.vessel_cfg.tolerances,
        &ctx.vessel_cfg.mass_model,
        ctx.vessel_cfg.propellant_mode,
        ctx.vessel_cfg.rw_mode,
        ctx.force_model,
        ctx.ephemeris,
        ctx.orientation,
        &control,
        false,
        &mut |seg| {
            let seg_idx = steps_seen as usize;
            steps_seen = steps_seen.wrapping_add(1);
            inspect_scalar_event_step(
                ctx,
                seg,
                seg_idx,
                steps_seen,
                event,
                direction,
                already_elapsed_s,
                min_search_offset_s,
                time_tolerance_s,
                &step_err,
                &found_root,
            )
        },
    );
    if let Some(err) = control.take_error() {
        return Err(err);
    }
    let prop_out = prop_res?;
    if let Some(err) = step_err.into_inner() {
        return Err(err);
    }
    resolve_scalar_event_outcome(
        ctx,
        event,
        max_duration_s,
        horizon_s,
        horizon_ticks,
        horizon_end_epoch,
        paused_by_limit,
        found_root.into_inner(),
        prop_out.final_state,
        prop_out.segments,
        prop_out.stats.accepted_steps,
    )
}

#[allow(clippy::too_many_arguments)]
fn resolve_scalar_event_outcome(
    ctx: &SegmentRunContext<'_>,
    event: OrbitalScalarEventKind,
    max_duration_s: f64,
    horizon_s: f64,
    horizon_ticks: u128,
    horizon_end_epoch: TdbEpoch,
    paused_by_limit: bool,
    found_root: Option<(usize, f64)>,
    final_state: ContinuousPlantState,
    segments: Vec<PlantTrajectorySegment>,
    accepted_steps: u32,
) -> Result<SegmentRunOutcome, MissionDesignError> {
    if let Some((seg_idx, root_tau_s)) = found_root {
        return finalize_triggered_event_segment(
            ctx,
            event,
            segments,
            accepted_steps,
            seg_idx,
            root_tau_s,
        );
    }
    if paused_by_limit {
        return build_fixed_duration_outcome(
            ctx,
            max_duration_s,
            horizon_s,
            horizon_ticks,
            horizon_end_epoch,
            true,
            final_state,
            segments,
            accepted_steps,
        );
    }
    Err(MissionDesignError::EventNotTriggered {
        segment_id: ctx.seg_node.segment_id,
        label: ctx.seg_node.label.clone(),
        event,
        max_duration_s,
    })
}

#[allow(clippy::too_many_arguments)]
fn inspect_scalar_event_step(
    ctx: &SegmentRunContext<'_>,
    seg: &PlantTrajectorySegment,
    seg_idx: usize,
    steps_seen: u32,
    event: OrbitalScalarEventKind,
    direction: CrossingDirection,
    already_elapsed_s: f64,
    min_search_offset_s: f64,
    time_tolerance_s: f64,
    step_err: &RefCell<Option<MissionDesignError>>,
    found_root: &RefCell<Option<(usize, f64)>>,
) -> bool {
    if steps_seen > ctx.remaining_step_budget {
        *step_err.borrow_mut() = Some(MissionDesignError::ExecutionStepCapExceeded {
            segment_id: ctx.seg_node.segment_id,
            max_steps: ctx.remaining_step_budget,
        });
        return true;
    }
    let seg_offset_s =
        already_elapsed_s + seg.start_epoch.signed_offset_seconds_from(ctx.start_epoch);
    let tau_min_s = (min_search_offset_s - seg_offset_s).max(0.0);

    match find_earliest_event_root_in_segment(
        seg,
        event,
        direction,
        tau_min_s,
        time_tolerance_s,
        ctx.ephemeris,
    ) {
        Ok(Some(root_tau_s)) => {
            let (trunc_seg, _) = truncate_segment_at_tau(seg, root_tau_s);
            if let Err(e) = check_step_propellant_and_budgets(ctx, &trunc_seg) {
                *step_err.borrow_mut() = Some(e);
            } else {
                *found_root.borrow_mut() = Some((seg_idx, root_tau_s));
            }
            return true;
        }
        Ok(None) => {}
        Err(e) => {
            *step_err.borrow_mut() = Some(e);
            return true;
        }
    }
    if let Err(e) = check_step_propellant_and_budgets(ctx, seg) {
        *step_err.borrow_mut() = Some(e);
        return true;
    }
    false
}

fn finalize_triggered_event_segment(
    ctx: &SegmentRunContext<'_>,
    event: OrbitalScalarEventKind,
    mut segments: Vec<PlantTrajectorySegment>,
    accepted_steps: u32,
    seg_idx: usize,
    root_tau_s: f64,
) -> Result<SegmentRunOutcome, MissionDesignError> {
    segments.truncate(seg_idx.wrapping_add(1));
    let Some(last_seg) = segments.last().cloned() else {
        return Err(MissionDesignError::EventNotTriggered {
            segment_id: ctx.seg_node.segment_id,
            label: ctx.seg_node.label.clone(),
            event,
            max_duration_s: 0.0,
        });
    };
    let (trunc_seg, end_state) = truncate_segment_at_tau(&last_seg, root_tau_s);
    let root_ticks = (trunc_seg.duration_s * (TICKS_PER_SECOND as f64)).round() as u128;
    let end_epoch = trunc_seg
        .start_epoch
        .add_span(TickSpan::from_ticks_u128(root_ticks));
    if let Some(slot) = segments.last_mut() {
        *slot = trunc_seg;
    }

    let chunk_dur_s = end_epoch
        .signed_offset_seconds_from(ctx.start_epoch)
        .max(0.0);
    let chunk_ticks = (chunk_dur_s * (TICKS_PER_SECOND as f64)).round() as u128;
    let (fuel_used, dv) = compute_propellant_and_dv(
        ctx.seg_node.burn_law,
        &ctx.vessel_cfg.mass_model,
        ctx.vessel_cfg.propellant_mode,
        &ctx.start_state,
        &end_state,
        chunk_dur_s,
    );
    enforce_cumulative_budgets(ctx, fuel_used, dv)?;

    let seg_start_ep = sub_span_tdb(
        ctx.start_epoch,
        TickSpan::from_ticks_u128(ctx.already_elapsed_ticks),
    );
    let total_seg_dur_s = end_epoch.signed_offset_seconds_from(seg_start_ep).max(0.0);
    let rec = SegmentExecutionRecord {
        segment_id: ctx.seg_node.segment_id,
        label: ctx.seg_node.label.clone(),
        start_epoch: seg_start_ep,
        end_epoch,
        duration_s: total_seg_dur_s,
        propellant_consumed_kg: fuel_used,
        delta_v_m_s: dv,
        accepted_steps,
        termination: SegmentTerminationReason::ScalarEventTriggered(event),
    };

    Ok(SegmentRunOutcome {
        end_epoch,
        end_state,
        dense_segments: segments,
        accepted_steps,
        propellant_consumed_kg: fuel_used,
        delta_v_m_s: dv,
        paused_mid_segment: false,
        advance_elapsed_ticks: chunk_ticks,
        completed_record: Some(rec),
    })
}
