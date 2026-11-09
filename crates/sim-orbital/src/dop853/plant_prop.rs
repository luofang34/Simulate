//! 6-DoF spacecraft plant propagation wrapper over `DOP853` with quaternion
//! geodesic error scaling and SPK granule-knot alignment.

use core::cell::Cell;

use glam::DQuat;
use sim_spacetime::{TICKS_PER_SECOND, TickSpan};

use crate::{
    dense_segment::{DenseSegment, PlantTrajectorySegment},
    dop853::{
        Dop853Error, Dop853Stats, Dop853Tolerances, build_dense_segment,
        coeffs::{E3, E5, N_STAGES_EXTENDED},
        combine_hairer_err, eval_main_stages, next_step_factor,
    },
    ephemeris::EphemerisCatalog,
    gravity::{ForceModel, GravityError},
    orientation::BodyOrientationCatalog,
    plant::{
        ContinuousControlLaw, ContinuousPlantState, MassPropertiesModel, PLANT_ODE_DIM,
        PropagationAudit, PropellantMode, ReactionWheelMode, evaluate_plant_ode_rhs,
        quat_geodesic_angle_rad, quat_normalize_portable,
    },
    time::TdbEpoch,
};

/// Output of propagating a 6-DoF spacecraft plant across `[start_epoch, end_epoch]`.
#[derive(Clone, Debug, PartialEq)]
pub struct PlantPropagationOutput {
    /// Final normalized continuous plant state at `end_epoch`.
    pub final_state: ContinuousPlantState,
    /// Contiguous degree-7 dense-output segments covering `[start_epoch, end_epoch]`.
    pub segments: Vec<PlantTrajectorySegment>,
    /// Cumulative idealization/exceedance audit flags from accepted steps only.
    pub audit: PropagationAudit,
    /// Step counts, RHS evaluations, and continuation step hint.
    pub stats: Dop853Stats,
}

/// Propagates a 6-DoF [`ContinuousPlantState`] across `[start_epoch, end_epoch]` with
/// quaternion geodesic error control, Chebyshev knot hints, and pure stage evaluation.
#[allow(clippy::too_many_arguments)]
pub fn propagate_plant_interval(
    initial: &ContinuousPlantState,
    start_epoch: TdbEpoch,
    end_epoch: TdbEpoch,
    h_trial_hint_s: Option<f64>,
    tol: &Dop853Tolerances,
    mass_model: &MassPropertiesModel,
    propellant_mode: PropellantMode,
    rw_mode: ReactionWheelMode,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    control: &dyn ContinuousControlLaw,
) -> Result<PlantPropagationOutput, Dop853Error<GravityError>> {
    propagate_plant_until_step(
        initial,
        start_epoch,
        end_epoch,
        h_trial_hint_s,
        tol,
        mass_model,
        propellant_mode,
        rw_mode,
        force_model,
        ephemeris,
        orientation,
        control,
        false,
        &mut |_| false,
    )
}

/// Propagates a 6-DoF [`ContinuousPlantState`] step-by-step across `[start_epoch, end_epoch]`,
/// invoking `stop_after_step` immediately as each [`PlantTrajectorySegment`] is accepted and
/// terminating early when `stop_after_step` returns `true`.
#[allow(clippy::too_many_arguments)]
pub fn propagate_plant_until_step<P>(
    initial: &ContinuousPlantState,
    start_epoch: TdbEpoch,
    end_epoch: TdbEpoch,
    h_trial_hint_s: Option<f64>,
    tol: &Dop853Tolerances,
    mass_model: &MassPropertiesModel,
    propellant_mode: PropellantMode,
    rw_mode: ReactionWheelMode,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    control: &dyn ContinuousControlLaw,
    backtrack_on_rhs_err: bool,
    stop_after_step: &mut P,
) -> Result<PlantPropagationOutput, Dop853Error<GravityError>>
where
    P: FnMut(&PlantTrajectorySegment) -> bool,
{
    let total_ticks = end_epoch.ticks_since(start_epoch);
    if total_ticks == 0 {
        return Ok(PlantPropagationOutput {
            final_state: *initial,
            segments: Vec::new(),
            audit: PropagationAudit::default(),
            stats: Dop853Stats::default(),
        });
    }

    let knots =
        force_model.interior_knots_in(initial.primary_body, start_epoch, end_epoch, ephemeris);
    let step_audit = Cell::new(PropagationAudit::default());
    let mut eval_rhs = |step_base_epoch: TdbEpoch,
                        tau_stage: f64,
                        y_stage: &[f64; PLANT_ODE_DIM]|
     -> Result<[f64; PLANT_ODE_DIM], GravityError> {
        let ticks = (tau_stage.max(0.0) * (TICKS_PER_SECOND as f64)).round() as u128;
        let ep = step_base_epoch.add_span(TickSpan::from_ticks_u128(ticks));
        let tau_interval = step_base_epoch.signed_offset_seconds_from(start_epoch) + tau_stage;
        let st = ContinuousPlantState::from_ode_array(initial.primary_body, y_stage, false);
        let (dy, aud) = evaluate_plant_ode_rhs(
            tau_interval,
            ep,
            &st,
            mass_model,
            propellant_mode,
            rw_mode,
            force_model,
            ephemeris,
            orientation,
            control,
        )?;
        step_audit.set(step_audit.get().merged(aud));
        Ok(dy)
    };

    run_plant_adaptive_loop(
        initial,
        (start_epoch, total_ticks, h_trial_hint_s),
        tol,
        (&knots, &step_audit, backtrack_on_rhs_err),
        &mut eval_rhs,
        stop_after_step,
    )
}

fn run_plant_adaptive_loop<F, P>(
    initial: &ContinuousPlantState,
    (start_epoch, mut rem_ticks, h_trial_hint_s): (TdbEpoch, u128, Option<f64>),
    tol: &Dop853Tolerances,
    (knots, step_audit, backtrack_on_rhs_err): (&[TdbEpoch], &Cell<PropagationAudit>, bool),
    eval_rhs: &mut F,
    stop_after_step: &mut P,
) -> Result<PlantPropagationOutput, Dop853Error<GravityError>>
where
    F: FnMut(TdbEpoch, f64, &[f64; PLANT_ODE_DIM]) -> Result<[f64; PLANT_ODE_DIM], GravityError>,
    P: FnMut(&PlantTrajectorySegment) -> bool,
{
    let (mut cur_epoch, mut y, mut h_nom, mut k_table, mut stats) =
        init_plant_loop(initial, start_epoch, h_trial_hint_s, tol, eval_rhs)?;
    let (mut total_audit, mut segments) = (PropagationAudit::default(), Vec::new());
    let (mut iters, mut prev_rejected) = (0_u32, false);

    while rem_ticks > 0 {
        if iters >= tol.max_steps {
            return Err(Dop853Error::MaxStepsExceeded {
                max_steps: tol.max_steps,
                tau_s: cur_epoch.signed_offset_seconds_from(start_epoch),
            });
        }
        iters = iters.wrapping_add(1);
        step_audit.set(PropagationAudit::default());
        let (step_ticks, h, clipped) = select_step_ticks(cur_epoch, rem_ticks, h_nom, knots);
        let stage_res = eval_main_stages(
            0.0,
            h,
            &y,
            &mut k_table,
            &mut |ts, ys| eval_rhs(cur_epoch, ts, ys),
            &mut stats.rhs_evals,
        );
        let Some(y_new) = unpack_or_backtrack_stages(
            stage_res,
            (backtrack_on_rhs_err, h, tol.h_min_s),
            (&mut prev_rejected, &mut h_nom, &mut stats),
        )?
        else {
            continue;
        };
        let err = plant_error_norm(h, &y, &y_new, &k_table, tol);
        if err > 1.0 {
            prev_rejected = true;
            let tau_s = cur_epoch.signed_offset_seconds_from(start_epoch);
            h_nom = reject_plant_step(tau_s, h, next_step_factor(err), tol, &mut stats)?;
            continue;
        }
        let next_h_nom = accepted_next_h(h, h_nom, err, prev_rejected, clipped, tol);
        prev_rejected = false;
        let seg = accept_plant_step(
            initial.primary_body,
            &mut cur_epoch,
            (step_ticks, h, next_h_nom),
            (&mut y, y_new, &mut k_table),
            (step_audit.get(), &mut total_audit),
            eval_rhs,
            &mut stats,
        )?;
        let should_stop = stop_after_step(&seg);
        segments.push(seg);
        rem_ticks -= step_ticks;
        h_nom = next_h_nom;
        if should_stop {
            break;
        }
    }
    stats.next_h_hint_s = h_nom;
    let final_state = ContinuousPlantState::from_ode_array(initial.primary_body, &y, false);
    Ok(PlantPropagationOutput {
        final_state,
        segments,
        audit: total_audit,
        stats,
    })
}

type StagePair = ([f64; PLANT_ODE_DIM], [f64; PLANT_ODE_DIM]);

fn unpack_or_backtrack_stages(
    stage_res: Result<StagePair, Dop853Error<GravityError>>,
    (backtrack_on_rhs_err, h, h_min_s): (bool, f64, f64),
    (prev_rejected, h_nom, stats): (&mut bool, &mut f64, &mut Dop853Stats),
) -> Result<Option<[f64; PLANT_ODE_DIM]>, Dop853Error<GravityError>> {
    match stage_res {
        Ok((y_new, _)) => Ok(Some(y_new)),
        Err(Dop853Error::Rhs(_)) if backtrack_on_rhs_err && h > h_min_s * 1.000_001 => {
            *prev_rejected = true;
            stats.rejected_steps = stats.rejected_steps.wrapping_add(1);
            *h_nom = (h * 0.5).max(h_min_s);
            Ok(None)
        }
        Err(err) => Err(err),
    }
}

type LoopInitState = (
    TdbEpoch,
    [f64; PLANT_ODE_DIM],
    f64,
    [[f64; PLANT_ODE_DIM]; N_STAGES_EXTENDED],
    Dop853Stats,
);

fn init_plant_loop<F>(
    initial: &ContinuousPlantState,
    start_epoch: TdbEpoch,
    h_trial_hint_s: Option<f64>,
    tol: &Dop853Tolerances,
    eval_rhs: &mut F,
) -> Result<LoopInitState, Dop853Error<GravityError>>
where
    F: FnMut(TdbEpoch, f64, &[f64; PLANT_ODE_DIM]) -> Result<[f64; PLANT_ODE_DIM], GravityError>,
{
    let y = initial.to_ode_array();
    let h_nom = h_trial_hint_s
        .unwrap_or(tol.h_init_s)
        .clamp(tol.h_min_s, tol.h_max_s);
    let mut k_table = [[0.0_f64; PLANT_ODE_DIM]; N_STAGES_EXTENDED];
    if let Some(slot) = k_table.first_mut() {
        *slot = eval_rhs(start_epoch, 0.0, &y).map_err(Dop853Error::Rhs)?;
    }
    let stats = Dop853Stats {
        rhs_evals: 1,
        next_h_hint_s: h_nom,
        ..Dop853Stats::default()
    };
    Ok((start_epoch, y, h_nom, k_table, stats))
}

fn accepted_next_h(
    h: f64,
    h_nom: f64,
    err: f64,
    prev_rejected: bool,
    clipped: bool,
    tol: &Dop853Tolerances,
) -> f64 {
    let fac = if prev_rejected {
        next_step_factor(err).min(1.0)
    } else {
        next_step_factor(err)
    };
    if !clipped || fac < 1.0 {
        (h * fac).clamp(tol.h_min_s, tol.h_max_s)
    } else {
        h_nom
    }
}

fn select_step_ticks(
    cur_epoch: TdbEpoch,
    rem_ticks: u128,
    h_nom: f64,
    knots: &[TdbEpoch],
) -> (u128, f64, bool) {
    let tps = TICKS_PER_SECOND as f64;
    let nom_ticks = ((h_nom * tps).round() as u128).max(1);
    let mut step_ticks = nom_ticks.min(rem_ticks);
    let mut clipped = step_ticks < nom_ticks;
    for &k in knots {
        let kt = k.ticks_since(cur_epoch);
        if kt > 0 && kt < step_ticks {
            step_ticks = kt;
            clipped = true;
        }
    }
    (step_ticks, (step_ticks as f64) / tps, clipped)
}

fn reject_plant_step(
    tau_s: f64,
    h: f64,
    fac: f64,
    tol: &Dop853Tolerances,
    stats: &mut Dop853Stats,
) -> Result<f64, Dop853Error<GravityError>> {
    stats.rejected_steps = stats.rejected_steps.wrapping_add(1);
    let h_next = h * fac.min(0.8);
    if h_next < tol.h_min_s {
        return Err(Dop853Error::StepSizeUnderflow {
            tau_s,
            h_s: h_next,
            h_min_s: tol.h_min_s,
        });
    }
    Ok(h_next.min(tol.h_max_s))
}

#[allow(clippy::too_many_arguments)]
fn accept_plant_step<F>(
    primary_body: crate::ids::NaifId,
    cur_epoch: &mut TdbEpoch,
    (step_ticks, h, next_h_nom): (u128, f64, f64),
    (y, y_new, k_table): (
        &mut [f64; PLANT_ODE_DIM],
        [f64; PLANT_ODE_DIM],
        &mut [[f64; PLANT_ODE_DIM]; N_STAGES_EXTENDED],
    ),
    (step_aud, total_audit): (PropagationAudit, &mut PropagationAudit),
    eval_rhs: &mut F,
    stats: &mut Dop853Stats,
) -> Result<PlantTrajectorySegment, Dop853Error<GravityError>>
where
    F: FnMut(TdbEpoch, f64, &[f64; PLANT_ODE_DIM]) -> Result<[f64; PLANT_ODE_DIM], GravityError>,
{
    let seg_epoch = *cur_epoch;
    let dense = build_dense_segment(
        0.0,
        h,
        y,
        &y_new,
        k_table,
        &mut |ts, ys| eval_rhs(seg_epoch, ts, ys),
        &mut stats.rhs_evals,
    )?;
    let local_dense = DenseSegment {
        start_tau_s: 0.0,
        h_s: h,
        y_old: dense.y_old,
        f_coeffs: dense.f_coeffs,
    };
    *total_audit = total_audit.merged(step_aud);
    *cur_epoch = cur_epoch.add_span(TickSpan::from_ticks_u128(step_ticks));
    *y = normalize_ode_quat_and_clamp_mass(y_new);
    let end_state = ContinuousPlantState::from_ode_array(primary_body, y, false);
    let k0_next = eval_rhs(*cur_epoch, 0.0, y).map_err(Dop853Error::Rhs)?;
    stats.rhs_evals = stats.rhs_evals.wrapping_add(1);
    if let Some(first) = k_table.first_mut() {
        *first = k0_next;
    }
    stats.accepted_steps = stats.accepted_steps.wrapping_add(1);
    Ok(PlantTrajectorySegment::from_dense(
        seg_epoch,
        step_ticks,
        primary_body,
        &local_dense,
        end_state,
        next_h_nom,
        step_aud,
    ))
}

fn plant_error_norm(
    h: f64,
    y_0: &[f64; PLANT_ODE_DIM],
    y_new: &[f64; PLANT_ODE_DIM],
    k_table: &[[f64; PLANT_ODE_DIM]; N_STAGES_EXTENDED],
    tol: &Dop853Tolerances,
) -> f64 {
    let mut raw_e5 = [0.0_f64; PLANT_ODE_DIM];
    let mut raw_e3 = [0.0_f64; PLANT_ODE_DIM];
    for (j, (&w5, &w3)) in E5.iter().zip(E3.iter()).enumerate() {
        if let Some(kj) = k_table.get(j) {
            for ((e5, e3), &kv) in raw_e5.iter_mut().zip(raw_e3.iter_mut()).zip(kj.iter()) {
                *e5 += h * w5 * kv;
                *e3 += h * w3 * kv;
            }
        }
    }

    let atols = [
        tol.atol_pos_m,
        tol.atol_pos_m,
        tol.atol_pos_m,
        tol.atol_vel_m_s,
        tol.atol_vel_m_s,
        tol.atol_vel_m_s,
        0.0,
        0.0,
        0.0,
        0.0,
        tol.atol_omega_rad_s,
        tol.atol_omega_rad_s,
        tol.atol_omega_rad_s,
        tol.atol_mass_kg,
        tol.atol_h_rw_nms,
        tol.atol_h_rw_nms,
        tol.atol_h_rw_nms,
    ];

    let mut err5_sq = 0.0_f64;
    let mut err3_sq = 0.0_f64;
    for idx in [0usize, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15, 16] {
        let y0 = y_0.get(idx).copied().unwrap_or(0.0).abs();
        let y1 = y_new.get(idx).copied().unwrap_or(0.0).abs();
        let sc = atols.get(idx).copied().unwrap_or(1e-9) + tol.rtol * y0.max(y1);
        let r5 = raw_e5.get(idx).copied().unwrap_or(0.0) / sc;
        let r3 = raw_e3.get(idx).copied().unwrap_or(0.0) / sc;
        err5_sq += r5 * r5;
        err3_sq += r3 * r3;
    }

    let [_, _, _, _, _, _, qx, qy, qz, qw, ..] = *y_new;
    let [_, _, _, _, _, _, dq5x, dq5y, dq5z, dq5w, ..] = raw_e5;
    let [_, _, _, _, _, _, dq3x, dq3y, dq3z, dq3w, ..] = raw_e3;
    let q8 = DQuat::from_xyzw(qx, qy, qz, qw);
    let q5 = DQuat::from_xyzw(qx - dq5x, qy - dq5y, qz - dq5z, qw - dq5w);
    let q3 = DQuat::from_xyzw(qx - dq3x, qy - dq3y, qz - dq3z, qw - dq3w);
    let sc_q = tol.atol_att_rad + tol.rtol;
    let rq5 = quat_geodesic_angle_rad(q8, q5) / sc_q;
    let rq3 = quat_geodesic_angle_rad(q8, q3) / sc_q;
    err5_sq += rq5 * rq5;
    err3_sq += rq3 * rq3;

    combine_hairer_err(1.0, err5_sq, err3_sq, 14.0)
}

fn normalize_ode_quat_and_clamp_mass(mut y: [f64; PLANT_ODE_DIM]) -> [f64; PLANT_ODE_DIM] {
    let [_, _, _, _, _, _, qx, qy, qz, qw, _, _, _, mp, ..] = y;
    let qn = quat_normalize_portable(DQuat::from_xyzw(qx, qy, qz, qw));
    if let Some(q_slice) = y.get_mut(6..10) {
        q_slice.copy_from_slice(&[qn.x, qn.y, qn.z, qn.w]);
    }
    if let Some(slot) = y.get_mut(13) {
        *slot = mp.max(0.0);
    }
    y
}
