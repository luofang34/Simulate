//! Generic fixed-step and adaptive `DOP853` integrators over `[f64; N]`.

use crate::{
    dense_segment::DenseSegment,
    dop853::{
        Dop853Error, Dop853GenericConfig, Dop853Stats, build_dense_segment, clamp_step_to_knot,
        coeffs::{E3, E5, N_STAGES, N_STAGES_EXTENDED},
        combine_hairer_err, compensated_add, eval_main_stages, next_step_factor,
    },
};

fn generic_error_norm<const N: usize>(
    h: f64,
    y_0: &[f64; N],
    y_new: &[f64; N],
    k_table: &[[f64; N]; N_STAGES_EXTENDED],
    rtol: f64,
    atol: f64,
) -> f64 {
    let mut err5_sq = 0.0_f64;
    let mut err3_sq = 0.0_f64;
    let mut idx: usize = 0;
    while idx < N {
        let mut e5 = 0.0_f64;
        let mut e3 = 0.0_f64;
        let mut j: usize = 0;
        while j <= N_STAGES {
            let kv = k_table
                .get(j)
                .and_then(|r| r.get(idx))
                .copied()
                .unwrap_or(0.0);
            e5 += E5.get(j).copied().unwrap_or(0.0) * kv;
            e3 += E3.get(j).copied().unwrap_or(0.0) * kv;
            j = j.wrapping_add(1);
        }
        let y0 = y_0.get(idx).copied().unwrap_or(0.0).abs();
        let y1 = y_new.get(idx).copied().unwrap_or(0.0).abs();
        let sc = atol + rtol * y0.max(y1);
        let r5 = e5 / sc;
        let r3 = e3 / sc;
        err5_sq += r5 * r5;
        err3_sq += r3 * r3;
        idx = idx.wrapping_add(1);
    }
    combine_hairer_err(h, err5_sq, err3_sq, N as f64)
}

/// Executes `n_steps` uniform fixed-size `DOP853` steps over `[0, total_tau_s]` for exact
/// step-halving convergence order verification.
pub fn dop853_fixed_steps<const N: usize, E, F>(
    y_init: [f64; N],
    total_tau_s: f64,
    n_steps: u32,
    mut rhs: F,
) -> Result<([f64; N], Vec<DenseSegment<N>>), Dop853Error<E>>
where
    F: FnMut(f64, &[f64; N]) -> Result<[f64; N], E>,
{
    let count = n_steps.max(1);
    let h = total_tau_s / f64::from(count);
    let mut y = y_init;
    let mut k_table = [[0.0_f64; N]; N_STAGES_EXTENDED];
    let mut evals: u64 = 1;
    let k0 = rhs(0.0, &y).map_err(Dop853Error::Rhs)?;
    if let Some(first) = k_table.first_mut() {
        *first = k0;
    }
    let mut segments = Vec::with_capacity(count as usize);

    let mut step: u32 = 0;
    while step < count {
        let tau = f64::from(step) * h;
        let (y_new, _) = eval_main_stages(tau, h, &y, &mut k_table, &mut rhs, &mut evals)?;
        let seg = build_dense_segment(tau, h, &y, &y_new, &mut k_table, &mut rhs, &mut evals)?;
        segments.push(seg);
        y = y_new;
        if let Some(fsal) = k_table.get(N_STAGES).copied()
            && let Some(first) = k_table.first_mut()
        {
            *first = fsal;
        }
        step = step.wrapping_add(1);
    }
    Ok((y, segments))
}

fn apply_compensated_step<const N: usize>(
    y: &[f64; N],
    y_comp: &mut [f64; N],
    dy_step: &[f64; N],
    h: f64,
) -> [f64; N] {
    let mut y_next = *y;
    for ((yn, yc), &dn) in y_next.iter_mut().zip(y_comp.iter_mut()).zip(dy_step.iter()) {
        *yn = compensated_add(*yn, yc, h * dn);
    }
    y_next
}

/// Integrates a generic `N`-dimensional ODE across $\tau \in [0, \text{duration\_s}]$ with
/// adaptive step control, `CONTD8` dense output, and optional interior knot alignment.
pub fn dop853_integrate_generic<const N: usize, E, F>(
    y_init: [f64; N],
    duration_s: f64,
    cfg: &Dop853GenericConfig,
    knot_taus: &[f64],
    mut rhs: F,
) -> Result<([f64; N], Vec<DenseSegment<N>>, Dop853Stats), Dop853Error<E>>
where
    F: FnMut(f64, &[f64; N]) -> Result<[f64; N], E>,
{
    if duration_s <= 0.0 {
        return Ok((y_init, Vec::new(), Dop853Stats::default()));
    }
    let (mut tau, mut y, mut y_comp) = (0.0_f64, y_init, [0.0_f64; N]);
    let mut h_nom = cfg.h_init_s.clamp(cfg.h_min_s, cfg.h_max_s);
    let mut k_table = [[0.0_f64; N]; N_STAGES_EXTENDED];
    let mut stats = Dop853Stats {
        rhs_evals: 1,
        next_h_hint_s: h_nom,
        ..Dop853Stats::default()
    };
    if let Some(slot) = k_table.first_mut() {
        *slot = rhs(0.0, &y).map_err(Dop853Error::Rhs)?;
    }
    let (mut segments, mut iters, mut prev_rejected) = (Vec::new(), 0_u32, false);
    let mut tau_comp = 0.0_f64;

    while tau < duration_s - 1.0e-14 * duration_s.max(1.0) {
        if iters >= cfg.max_steps {
            return Err(Dop853Error::MaxStepsExceeded {
                max_steps: cfg.max_steps,
                tau_s: tau,
            });
        }
        iters = iters.wrapping_add(1);
        let (h, clipped) = clamp_step_to_knot(tau, h_nom, duration_s, knot_taus);
        let (y_trial, dy_step) =
            eval_main_stages(tau, h, &y, &mut k_table, &mut rhs, &mut stats.rhs_evals)?;
        let err = generic_error_norm(h, &y, &y_trial, &k_table, cfg.rtol, cfg.atol);
        let fac = next_step_factor(err);
        if err <= 1.0 {
            let eff_fac = if prev_rejected { fac.min(1.0) } else { fac };
            prev_rejected = false;
            let y_next = apply_compensated_step(&y, &mut y_comp, &dy_step, h);
            segments.push(build_dense_segment(
                tau,
                h,
                &y,
                &y_next,
                &mut k_table,
                &mut rhs,
                &mut stats.rhs_evals,
            )?);
            tau = compensated_add(tau, &mut tau_comp, h);
            y = y_next;
            if let Some(fsal) = k_table.get(N_STAGES).copied()
                && let Some(first) = k_table.first_mut()
            {
                *first = fsal;
            }
            stats.accepted_steps = stats.accepted_steps.wrapping_add(1);
            if !clipped || eff_fac < 1.0 {
                h_nom = (h * eff_fac).clamp(cfg.h_min_s, cfg.h_max_s);
            }
        } else {
            prev_rejected = true;
            h_nom = reject_generic_step(tau, h, fac, cfg, &mut stats)?;
        }
    }
    stats.next_h_hint_s = h_nom;
    Ok((y, segments, stats))
}

fn reject_generic_step<E>(
    tau_s: f64,
    h: f64,
    fac: f64,
    cfg: &Dop853GenericConfig,
    stats: &mut Dop853Stats,
) -> Result<f64, Dop853Error<E>> {
    stats.rejected_steps = stats.rejected_steps.wrapping_add(1);
    let h_next = h * fac.min(0.8);
    if h_next < cfg.h_min_s {
        return Err(Dop853Error::StepSizeUnderflow {
            tau_s,
            h_s: h_next,
            h_min_s: cfg.h_min_s,
        });
    }
    Ok(h_next.min(cfg.h_max_s))
}
