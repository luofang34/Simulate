//! Step-level propellant exhaustion, ideal rocket $\Delta v$, and cumulative fuel/$\Delta v$
//! budget enforcement for [`segment_runner`](super::segment_runner).

use sim_orbital::{
    ContinuousPlantState, MassPropertiesModel, PlantTrajectorySegment, PropellantMode, TdbEpoch,
};
use sim_spacetime::{TICKS_PER_SECOND, TickSpan, ln};

use crate::{
    error::MissionDesignError,
    execution::{
        SegmentExecutionRecord, SegmentTerminationReason,
        segment_runner::{SegmentRunContext, SegmentRunOutcome},
    },
    plan::SegmentBurnLaw,
};

pub(super) fn check_step_propellant_and_budgets(
    ctx: &SegmentRunContext<'_>,
    seg: &PlantTrajectorySegment,
) -> Result<(), MissionDesignError> {
    let y_end = seg.as_dense().evaluate_tau(seg.duration_s);
    let st_end = ContinuousPlantState::from_ode_array(seg.primary_body, &y_end, true);
    if matches!(ctx.seg_node.burn_law, SegmentBurnLaw::ConstantThrust { thrust_n, .. } if thrust_n > 0.0)
        && ctx.vessel_cfg.propellant_mode == PropellantMode::FiniteVariableMass
        && st_end.propellant_mass_kg <= 1.0e-9
    {
        return Err(MissionDesignError::PropellantExhausted {
            segment_id: ctx.seg_node.segment_id,
            label: ctx.seg_node.label.clone(),
        });
    }
    let elapsed_s = seg.start_epoch.signed_offset_seconds_from(ctx.start_epoch) + seg.duration_s;
    let (fuel_used, dv) = compute_propellant_and_dv(
        ctx.seg_node.burn_law,
        &ctx.vessel_cfg.mass_model,
        ctx.vessel_cfg.propellant_mode,
        &ctx.start_state,
        &st_end,
        elapsed_s,
    );
    enforce_cumulative_budgets(ctx, fuel_used, dv)
}

pub(super) fn compute_propellant_and_dv(
    burn_law: SegmentBurnLaw,
    mass_model: &MassPropertiesModel,
    prop_mode: PropellantMode,
    start_state: &ContinuousPlantState,
    end_state: &ContinuousPlantState,
    duration_s: f64,
) -> (f64, f64) {
    let fuel_used = (start_state.propellant_mass_kg - end_state.propellant_mass_kg).max(0.0);
    let dv = match burn_law {
        SegmentBurnLaw::Coast => 0.0,
        SegmentBurnLaw::ConstantThrust {
            thrust_n,
            exhaust_vel_m_s,
            ..
        } => {
            if thrust_n <= 0.0 {
                0.0
            } else if prop_mode == PropellantMode::FiniteVariableMass && fuel_used > 1.0e-12 {
                let m0 = mass_model.total_mass_kg(start_state.propellant_mass_kg);
                let m1 = mass_model
                    .total_mass_kg(end_state.propellant_mass_kg)
                    .max(1.0);
                exhaust_vel_m_s * ln((m0 / m1).max(1.0))
            } else {
                let m0 = mass_model
                    .total_mass_kg(start_state.propellant_mass_kg)
                    .max(1.0);
                (thrust_n / m0) * duration_s.max(0.0)
            }
        }
    };
    (fuel_used, dv)
}

pub(super) fn enforce_cumulative_budgets(
    ctx: &SegmentRunContext<'_>,
    chunk_fuel_kg: f64,
    chunk_dv_m_s: f64,
) -> Result<(), MissionDesignError> {
    let tot_fuel = ctx.prior_consumed_fuel_kg + chunk_fuel_kg;
    if let Some(max_fuel) = ctx.plan.max_fuel_budget_kg
        && tot_fuel > max_fuel + 1.0e-9
    {
        return Err(MissionDesignError::FuelBudgetExceeded {
            segment_id: ctx.seg_node.segment_id,
            consumed_kg: tot_fuel,
            budget_kg: max_fuel,
        });
    }
    let tot_dv = ctx.prior_delta_v_m_s + chunk_dv_m_s;
    if let Some(max_dv) = ctx.plan.delta_v_budget_m_s
        && tot_dv > max_dv + 1.0e-9
    {
        return Err(MissionDesignError::DeltaVBudgetExceeded {
            segment_id: ctx.seg_node.segment_id,
            used_m_s: tot_dv,
            budget_m_s: max_dv,
        });
    }
    Ok(())
}

pub(super) fn sub_span_tdb(epoch: TdbEpoch, span: TickSpan) -> TdbEpoch {
    let (ds, dt) = span.seconds_and_subsec_ticks();
    if epoch.subsec_ticks() >= dt {
        TdbEpoch::new(epoch.secs_j2000() - (ds as i64), epoch.subsec_ticks() - dt)
    } else {
        TdbEpoch::new(
            epoch.secs_j2000() - (ds as i64) - 1,
            epoch.subsec_ticks() + TICKS_PER_SECOND - dt,
        )
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn build_fixed_duration_outcome(
    ctx: &SegmentRunContext<'_>,
    total_duration_s: f64,
    chunk_s: f64,
    chunk_ticks: u128,
    target_end_epoch: TdbEpoch,
    paused_mid_segment: bool,
    final_state: ContinuousPlantState,
    segments: Vec<PlantTrajectorySegment>,
    accepted_steps: u32,
) -> Result<SegmentRunOutcome, MissionDesignError> {
    let (fuel_used, dv) = compute_propellant_and_dv(
        ctx.seg_node.burn_law,
        &ctx.vessel_cfg.mass_model,
        ctx.vessel_cfg.propellant_mode,
        &ctx.start_state,
        &final_state,
        chunk_s,
    );
    enforce_cumulative_budgets(ctx, fuel_used, dv)?;

    let completed_record = if paused_mid_segment {
        None
    } else {
        let seg_start_ep = sub_span_tdb(
            ctx.start_epoch,
            TickSpan::from_ticks_u128(ctx.already_elapsed_ticks),
        );
        Some(SegmentExecutionRecord {
            segment_id: ctx.seg_node.segment_id,
            label: ctx.seg_node.label.clone(),
            start_epoch: seg_start_ep,
            end_epoch: target_end_epoch,
            duration_s: total_duration_s,
            propellant_consumed_kg: fuel_used,
            delta_v_m_s: dv,
            accepted_steps,
            termination: SegmentTerminationReason::FixedDurationCompleted,
        })
    };

    Ok(SegmentRunOutcome {
        end_epoch: target_end_epoch,
        end_state: final_state,
        dense_segments: segments,
        accepted_steps,
        propellant_consumed_kg: fuel_used,
        delta_v_m_s: dv,
        paused_mid_segment,
        advance_elapsed_ticks: chunk_ticks,
        completed_record,
    })
}
