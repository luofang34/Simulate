//! Internal dense-output scalar event evaluation and bracketed root isolation.

use sim_orbital::{
    ContinuousPlantState, DenseSegment, DerivativeOrder, EphemerisCatalog, PLANT_ODE_DIM,
    PlantTrajectorySegment, SynodicFrame, TdbEpoch,
};
use sim_spacetime::{TICKS_PER_SECOND, TickSpan, sqrt};

use crate::{
    error::MissionDesignError,
    plan::{CrossingDirection, OrbitalScalarEventKind},
};

/// Evaluates the scalar event function $g(t, \mathbf{y})$ at `(epoch, state)`.
pub(crate) fn evaluate_scalar_event(
    event: OrbitalScalarEventKind,
    epoch: TdbEpoch,
    state: &ContinuousPlantState,
    ephemeris: &dyn EphemerisCatalog,
) -> Result<f64, MissionDesignError> {
    match event {
        OrbitalScalarEventKind::PeriapsisPassage { body }
        | OrbitalScalarEventKind::ApoapsisPassage { body } => {
            let (pos_rel, vel_rel) = relative_pv_icrf(state, body, epoch, ephemeris)?;
            Ok(pos_rel.dot(vel_rel))
        }
        OrbitalScalarEventKind::RadiusCrossing {
            body,
            target_radius_m,
        } => {
            let (pos_rel, _) = relative_pv_icrf(state, body, epoch, ephemeris)?;
            Ok(sqrt(pos_rel.length_squared()) - target_radius_m)
        }
        OrbitalScalarEventKind::RemainingPropellantCrossing {
            target_propellant_kg,
        } => Ok(state.propellant_mass_kg - target_propellant_kg),
        OrbitalScalarEventKind::SynodicXCrossing {
            primary,
            secondary,
            target_x_rel_secondary_m,
        } => {
            let (pos_rel_sec, vel_rel_sec) = relative_pv_icrf(state, secondary, epoch, ephemeris)?;
            let syn = SynodicFrame::from_ephemeris(primary, secondary, epoch, ephemeris)
                .map_err(sim_spacetime::SpacetimeTransformError::from)?;
            let (pos_syn, _) = syn.icrf_rel_secondary_to_synodic(pos_rel_sec, vel_rel_sec);
            Ok(pos_syn.x - target_x_rel_secondary_m)
        }
    }
}

fn relative_pv_icrf(
    state: &ContinuousPlantState,
    target_body: sim_orbital::NaifId,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
) -> Result<(glam::DVec3, glam::DVec3), MissionDesignError> {
    if state.primary_body == target_body {
        return Ok((state.pos_rel_icrf_m, state.vel_rel_icrf_m_s));
    }
    let shift = ephemeris.geometric_state_icrf(
        state.primary_body,
        target_body,
        epoch,
        DerivativeOrder::Velocity,
    )?;
    Ok((
        state.pos_rel_icrf_m + shift.pos_m,
        state.vel_rel_icrf_m_s + shift.vel_m_s,
    ))
}

/// Scans a single accepted [`PlantTrajectorySegment`] over `[tau_min_s, seg.duration_s]`
/// for the earliest root of `event` matching `direction`.
pub(crate) fn find_earliest_event_root_in_segment(
    seg: &PlantTrajectorySegment,
    event: OrbitalScalarEventKind,
    direction: CrossingDirection,
    tau_min_s: f64,
    time_tolerance_s: f64,
    ephemeris: &dyn EphemerisCatalog,
) -> Result<Option<f64>, MissionDesignError> {
    let dur = seg.duration_s;
    let t_lo = tau_min_s.clamp(0.0, dur);
    if dur <= 0.0 || t_lo >= dur {
        return Ok(None);
    }

    let effective_dir = match event {
        OrbitalScalarEventKind::PeriapsisPassage { .. } => CrossingDirection::Rising,
        OrbitalScalarEventKind::ApoapsisPassage { .. } => CrossingDirection::Falling,
        _ => direction,
    };

    let eval_at = |tau: f64| -> Result<f64, MissionDesignError> {
        let tau_c = tau.clamp(0.0, dur);
        let y = seg.as_dense().evaluate_tau(tau_c);
        let st = ContinuousPlantState::from_ode_array(seg.primary_body, &y, true);
        let ticks = (tau_c * (TICKS_PER_SECOND as f64)).round() as u128;
        let ep = seg.start_epoch.add_span(TickSpan::from_ticks_u128(ticks));
        evaluate_scalar_event(event, ep, &st, ephemeris)
    };

    let n_sub = 16_usize;
    let dt = (dur - t_lo) / (n_sub as f64);
    let mut prev_tau = t_lo;
    let mut prev_g = eval_at(prev_tau)?;

    let mut k = 1_usize;
    while k <= n_sub {
        let cur_tau = if k == n_sub {
            dur
        } else {
            t_lo + (k as f64) * dt
        };
        let cur_g = eval_at(cur_tau)?;
        if is_valid_sign_change(prev_g, cur_g, effective_dir) {
            let root_tau = bisect_secant_segment_root(
                prev_tau,
                cur_tau,
                prev_g,
                cur_g,
                time_tolerance_s.max(1.0e-9),
                &eval_at,
            )?;
            return Ok(Some(root_tau));
        }
        prev_tau = cur_tau;
        prev_g = cur_g;
        k = k.wrapping_add(1);
    }
    Ok(None)
}

fn is_valid_sign_change(g0: f64, g1: f64, dir: CrossingDirection) -> bool {
    let rising = g0 < 0.0 && g1 >= 0.0;
    let falling = g0 > 0.0 && g1 <= 0.0;
    match dir {
        CrossingDirection::Rising => rising,
        CrossingDirection::Falling => falling,
        CrossingDirection::Any => rising || falling,
    }
}

fn bisect_secant_segment_root<F>(
    mut a: f64,
    mut b: f64,
    mut fa: f64,
    mut fb: f64,
    tol_s: f64,
    eval_at: &F,
) -> Result<f64, MissionDesignError>
where
    F: Fn(f64) -> Result<f64, MissionDesignError>,
{
    let mut iter = 0_u32;
    while iter < 60 && (b - a).abs() > tol_s {
        let sec = if (fb - fa).abs() > 1.0e-18 {
            a - fa * (b - a) / (fb - fa)
        } else {
            0.5 * (a + b)
        };
        let margin = 0.1 * (b - a);
        let mid = if sec > a + margin && sec < b - margin {
            sec
        } else {
            0.5 * (a + b)
        };
        let fm = eval_at(mid)?;
        if fm == 0.0 {
            return Ok(mid);
        }
        if (fa < 0.0 && fm < 0.0) || (fa > 0.0 && fm > 0.0) {
            a = mid;
            fa = fm;
        } else {
            b = mid;
            fb = fm;
        }
        iter = iter.wrapping_add(1);
    }
    Ok(0.5 * (a + b))
}

/// Truncates `seg` to `[0, root_tau_s]` by re-fitting an exact degree-7 dense segment
/// over the sub-interval `[0, root_tau_s]`, returning `(truncated_segment, state_at_root)`.
pub(crate) fn truncate_segment_at_tau(
    seg: &PlantTrajectorySegment,
    root_tau_s: f64,
) -> (PlantTrajectorySegment, ContinuousPlantState) {
    let tau_end = root_tau_s.clamp(0.0, seg.duration_s);
    let dense = seg.as_dense();
    let y_end = dense.evaluate_tau(tau_end);
    let state_end = ContinuousPlantState::from_ode_array(seg.primary_body, &y_end, true);
    if tau_end <= 1.0e-12 || (tau_end - seg.duration_s).abs() <= 1.0e-12 {
        return (seg.clone(), state_end);
    }

    // Re-expand p(s * tau_end) for s in [0, 1] by scaling the monomial/CONTD8 basis or
    // evaluating the degree-7 polynomial at Chebyshev nodes — wait: for a polynomial of
    // degree <= 7 in s = tau / h, we can re-evaluate p(tau) on [0, tau_end] via its
    // exact Taylor/finite-difference or keep linear/cubic/degree-7 interpolation!
    // Even simpler: in CONTD8 basis:
    // y(s) = r0 + s*(r1 + (1-s)*(r2 + s*(r3 + (1-s)*(r4 + s*(r5 + (1-s)*(r6 + s*r7))))))
    // Every polynomial of degree <= 7 on [0, tau_end] is uniquely determined by its values at 8 nodes,
    // or we can solve the lower-triangular/Vandermonde system for r0..r7 on [0, tau_end]!
    let (y_old, new_coeffs) = reexpand_contd8_subinterval(&dense, tau_end);
    let duration_ticks = ((tau_end * (sim_spacetime::TICKS_PER_SECOND as f64))
        .round()
        .max(1.0)) as u128;
    let truncated = PlantTrajectorySegment {
        start_epoch: seg.start_epoch,
        duration_ticks,
        duration_s: tau_end,
        primary_body: seg.primary_body,
        y_old,
        f_coeffs: new_coeffs,
        end_state: state_end,
        next_h_hint_s: seg.next_h_hint_s,
        audit: seg.audit,
    };
    (truncated, state_end)
}

fn reexpand_contd8_subinterval(
    dense: &DenseSegment<PLANT_ODE_DIM>,
    tau_end: f64,
) -> ([f64; PLANT_ODE_DIM], [[f64; PLANT_ODE_DIM]; 7]) {
    // Sample at 8 Chebyshev/uniform nodes u_k in [0, 1] and fit the exact degree-7 polynomial
    // in the CONTD8 nested basis:
    // phi_0(u) = 1
    // phi_1(u) = u
    // phi_2(u) = u(1-u)
    // phi_3(u) = u^2(1-u)
    // phi_4(u) = u^2(1-u)^2
    // phi_5(u) = u^3(1-u)^2
    // phi_6(u) = u^3(1-u)^3
    // phi_7(u) = u^4(1-u)^3
    let y0 = dense.evaluate_tau(0.0);
    let y1 = dense.evaluate_tau(tau_end);
    let mut f_coeffs = [[0.0_f64; PLANT_ODE_DIM]; 7];
    let mut r1_arr = [0.0_f64; PLANT_ODE_DIM];
    for ((slot, &a), &b) in r1_arr.iter_mut().zip(y0.iter()).zip(y1.iter()) {
        *slot = b - a;
    }
    if let Some(slot0) = f_coeffs.get_mut(0) {
        *slot0 = r1_arr;
    }

    let nodes = [
        1.0 / 7.0,
        2.0 / 7.0,
        3.0 / 7.0,
        4.0 / 7.0,
        5.0 / 7.0,
        6.0 / 7.0,
    ];
    let mut rhs = [[0.0_f64; PLANT_ODE_DIM]; 6];
    let mut mat = [[0.0_f64; 6]; 6];

    for (row_idx, &u) in nodes.iter().enumerate() {
        let v = 1.0 - u;
        let uv = u * v;
        let basis = [
            uv,
            uv * u,
            uv * uv,
            uv * uv * u,
            uv * uv * uv,
            uv * uv * uv * u,
        ];
        if let Some(m_row) = mat.get_mut(row_idx) {
            *m_row = basis;
        }
        let yu = dense.evaluate_tau(u * tau_end);
        if let Some(rhs_row) = rhs.get_mut(row_idx) {
            for ((slot, &y_val), (&r0_val, &r1_val)) in rhs_row
                .iter_mut()
                .zip(yu.iter())
                .zip(y0.iter().zip(r1_arr.iter()))
            {
                *slot = y_val - (r0_val + u * r1_val);
            }
        }
    }

    solve_6x6_in_place(&mut mat, &mut rhs);
    for (k, row) in rhs.iter().enumerate() {
        if let Some(target) = f_coeffs.get_mut(k.wrapping_add(1)) {
            *target = *row;
        }
    }
    (y0, f_coeffs)
}

fn solve_6x6_in_place(a: &mut [[f64; 6]; 6], b: &mut [[f64; PLANT_ODE_DIM]; 6]) {
    let mut col = 0_usize;
    while col < 6 {
        let mut pivot = col;
        let mut max_abs = a.get(col).and_then(|r| r.get(col)).map_or(0.0, |v| v.abs());
        let mut row = col.wrapping_add(1);
        while row < 6 {
            let val = a.get(row).and_then(|r| r.get(col)).map_or(0.0, |v| v.abs());
            if val > max_abs {
                max_abs = val;
                pivot = row;
            }
            row = row.wrapping_add(1);
        }
        if pivot != col {
            a.swap(col, pivot);
            b.swap(col, pivot);
        }
        let diag = a.get(col).and_then(|r| r.get(col)).copied().unwrap_or(1.0);
        if diag.abs() > 1.0e-18 {
            let mut r = col.wrapping_add(1);
            while r < 6 {
                let factor = a
                    .get(r)
                    .and_then(|row| row.get(col))
                    .copied()
                    .unwrap_or(0.0)
                    / diag;
                let pivot_a = a.get(col).copied().unwrap_or([0.0; 6]);
                let pivot_b = b.get(col).copied().unwrap_or([0.0; PLANT_ODE_DIM]);
                if let Some(r_a) = a.get_mut(r) {
                    for (slot, &pv) in r_a.iter_mut().zip(pivot_a.iter()).skip(col) {
                        *slot -= factor * pv;
                    }
                }
                if let Some(r_b) = b.get_mut(r) {
                    for (slot, &pv) in r_b.iter_mut().zip(pivot_b.iter()) {
                        *slot -= factor * pv;
                    }
                }
                r = r.wrapping_add(1);
            }
        }
        col = col.wrapping_add(1);
    }

    let mut row_rev = 6_usize;
    while row_rev > 0 {
        row_rev -= 1;
        let diag = a
            .get(row_rev)
            .and_then(|r| r.get(row_rev))
            .copied()
            .unwrap_or(1.0);
        if diag.abs() <= 1.0e-18 {
            continue;
        }
        let mut sum = b.get(row_rev).copied().unwrap_or([0.0; PLANT_ODE_DIM]);
        let mut c = row_rev.wrapping_add(1);
        while c < 6 {
            let a_rc = a
                .get(row_rev)
                .and_then(|r| r.get(c))
                .copied()
                .unwrap_or(0.0);
            let b_c = b.get(c).copied().unwrap_or([0.0; PLANT_ODE_DIM]);
            for (s, &bc) in sum.iter_mut().zip(b_c.iter()) {
                *s -= a_rc * bc;
            }
            c = c.wrapping_add(1);
        }
        if let Some(b_row) = b.get_mut(row_rev) {
            for (slot, &s) in b_row.iter_mut().zip(sum.iter()) {
                *slot = s / diag;
            }
        }
    }
}
