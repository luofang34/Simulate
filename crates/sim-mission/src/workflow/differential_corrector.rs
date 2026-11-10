//! Multi-variable, multi-goal scaled differential corrector with variable bounds,
//! central-difference sensitivity Jacobian, damped Gauss-Newton / Newton steps,
//! and per-iteration convergence diagnostics.

use serde::{Deserialize, Serialize};
use sim_spacetime::sqrt;

use crate::{MissionDesignError, SolverConvergenceStatus, SolverFailureReason};

/// Scaled control variable specification for multi-variable differential correction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScaledControlVariable {
    /// Human-readable variable identifier (e.g. `"burn_1_duration_s"`, `"burn_1_yaw_rad"`).
    pub name: String,
    /// Initial unscaled guess $u_0$.
    pub initial_value: f64,
    /// Characteristic scale $s_u > 0$ used to normalize the control variable ($\tilde{u} = u / s_u$).
    pub scale: f64,
    /// Unscaled central-difference perturbation step $\delta u > 0$.
    pub perturbation_step: f64,
    /// Inclusive lower bound $u_{\min}$.
    pub min_value: f64,
    /// Inclusive upper bound $u_{\max}$.
    pub max_value: f64,
    /// Maximum unscaled step size $|\Delta u|$ allowed in a single iteration.
    pub max_step: f64,
}

/// Scaled equality goal specification for multi-goal differential correction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScaledEqualityGoal {
    /// Human-readable goal identifier (e.g. `"apoapsis_radius_m"`, `"inclination_rad"`).
    pub name: String,
    /// Desired target value $g_{\text{target}}$.
    pub target_value: f64,
    /// Characteristic scale $s_g > 0$ used to normalize the residual ($\tilde{r} = (g - g_{\text{target}}) / s_g$).
    pub scale: f64,
    /// Unscaled absolute convergence tolerance $\epsilon_g > 0$ ($|g - g_{\text{target}}| \le \epsilon_g$).
    pub tolerance: f64,
}

/// Per-iteration diagnostic record of the multi-variable differential corrector.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DifferentialCorrectorIterationRecord {
    /// 0-based iteration index (`0` for the initial seed evaluation).
    pub iteration: u32,
    /// Unscaled control variable vector $\mathbf{u}_k$.
    pub variables: Vec<f64>,
    /// Evaluated unscaled goal values $\mathbf{g}(\mathbf{u}_k)$.
    pub achieved_goals: Vec<f64>,
    /// Unscaled residual vector $\mathbf{r}_k = \mathbf{g}(\mathbf{u}_k) - \mathbf{g}_{\text{target}}$.
    pub unscaled_residuals: Vec<f64>,
    /// Normalized RMS residual $\|\tilde{\mathbf{r}}_k\|_2 / \sqrt{M}$.
    pub scaled_rms_residual: f64,
    /// Row-major unscaled sensitivity Jacobian $\partial g_i / \partial u_j$ (`m_goals * n_vars`).
    pub unscaled_jacobian_row_major: Vec<f64>,
    /// Whether any variable was clamped against `[min_value, max_value]` at the end of this step.
    pub hit_bounds: bool,
}

/// Complete report from [`solve_multivariable_differential_correction`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DifferentialCorrectorReport {
    /// Final convergence status (`Converged` or `Failed`).
    pub status: SolverConvergenceStatus,
    /// Total iterations performed.
    pub iterations: u32,
    /// Final unscaled control variable vector $\mathbf{u}_\star$.
    pub final_variables: Vec<f64>,
    /// Final achieved goal values $\mathbf{g}(\mathbf{u}_\star)$.
    pub final_achieved_goals: Vec<f64>,
    /// Final unscaled residuals $\mathbf{r}_\star$.
    pub final_unscaled_residuals: Vec<f64>,
    /// Final normalized RMS residual.
    pub final_scaled_rms_residual: f64,
    /// Final row-major unscaled sensitivity Jacobian $\partial g_i / \partial u_j$.
    pub final_jacobian_row_major: Vec<f64>,
    /// Chronological iteration trace including initial seed (`iteration == 0`).
    pub history: Vec<DifferentialCorrectorIterationRecord>,
}

/// Solves an $M$-goal, $N$-variable targeting problem via scaled damped Gauss-Newton / Newton
/// differential correction with box bounds and central-difference Jacobian sensitivities.
pub fn solve_multivariable_differential_correction<F>(
    variables: &[ScaledControlVariable],
    goals: &[ScaledEqualityGoal],
    max_iterations: u32,
    damping_lambda: f64,
    mut evaluate_goals: F,
) -> Result<DifferentialCorrectorReport, MissionDesignError>
where
    F: FnMut(&[f64]) -> Result<Vec<f64>, MissionDesignError>,
{
    validate_corrector_inputs(variables, goals, max_iterations, damping_lambda)?;
    let n_vars = variables.len();
    let m_goals = goals.len();
    let mut u: Vec<f64> = variables
        .iter()
        .map(|v| v.initial_value.clamp(v.min_value, v.max_value))
        .collect();
    let mut history = Vec::new();
    let mut last_jac = vec![0.0; m_goals.saturating_mul(n_vars)];

    let mut iter = 0_u32;
    loop {
        let achieved = evaluate_goals(&u)?;
        if achieved.len() != m_goals || achieved.iter().any(|x| !x.is_finite()) {
            let r = SolverFailureReason::TrialExecutionFailed;
            return Ok(build_failed_report(
                r, iter, u, achieved, goals, last_jac, history,
            ));
        }
        let (residuals, scaled_rms, all_converged) = compute_residuals(&achieved, goals);
        if all_converged {
            return Ok(build_converged_report(
                iter, u, achieved, residuals, scaled_rms, last_jac, history,
            ));
        }
        if iter >= max_iterations {
            let r = SolverFailureReason::MaxIterationsExceeded;
            return Ok(build_failed_report(
                r, iter, u, achieved, goals, last_jac, history,
            ));
        }
        let (jac_unscaled, jac_scaled) =
            compute_central_jacobian(&u, variables, goals, &mut evaluate_goals)?;
        last_jac.clone_from(&jac_unscaled);
        let scaled_res: Vec<f64> = residuals
            .iter()
            .zip(goals.iter())
            .map(|(&r, g)| r / g.scale)
            .collect();
        let Some(delta_tilde) = solve_damped_normal_equations(
            &jac_scaled,
            &scaled_res,
            m_goals,
            n_vars,
            damping_lambda,
        ) else {
            let r = SolverFailureReason::SingularSensitivity;
            return Ok(build_failed_report(
                r,
                iter.wrapping_add(1),
                u,
                achieved,
                goals,
                last_jac,
                history,
            ));
        };
        let hit_bounds = apply_bounded_step(&mut u, &delta_tilde, variables);
        history.push(DifferentialCorrectorIterationRecord {
            iteration: iter,
            variables: u.clone(),
            achieved_goals: achieved,
            unscaled_residuals: residuals,
            scaled_rms_residual: scaled_rms,
            unscaled_jacobian_row_major: jac_unscaled,
            hit_bounds,
        });
        iter = iter.wrapping_add(1);
    }
}

fn build_converged_report(
    iter: u32,
    u: Vec<f64>,
    achieved: Vec<f64>,
    residuals: Vec<f64>,
    scaled_rms: f64,
    last_jac: Vec<f64>,
    mut history: Vec<DifferentialCorrectorIterationRecord>,
) -> DifferentialCorrectorReport {
    history.push(DifferentialCorrectorIterationRecord {
        iteration: iter,
        variables: u.clone(),
        achieved_goals: achieved.clone(),
        unscaled_residuals: residuals.clone(),
        scaled_rms_residual: scaled_rms,
        unscaled_jacobian_row_major: last_jac.clone(),
        hit_bounds: false,
    });
    DifferentialCorrectorReport {
        status: SolverConvergenceStatus::Converged,
        iterations: iter,
        final_variables: u,
        final_achieved_goals: achieved,
        final_unscaled_residuals: residuals,
        final_scaled_rms_residual: scaled_rms,
        final_jacobian_row_major: last_jac,
        history,
    }
}

fn validate_corrector_inputs(
    variables: &[ScaledControlVariable],
    goals: &[ScaledEqualityGoal],
    max_iterations: u32,
    damping_lambda: f64,
) -> Result<(), MissionDesignError> {
    if variables.is_empty()
        || goals.is_empty()
        || max_iterations == 0
        || !damping_lambda.is_finite()
        || damping_lambda < 0.0
    {
        return Err(MissionDesignError::InvalidPlan {
            plan_id: 0,
            reason: "invalid differential corrector dimensions or damping".to_owned(),
        });
    }
    if variables.iter().any(|v| {
        !v.initial_value.is_finite()
            || !v.scale.is_finite()
            || v.scale <= 0.0
            || !v.perturbation_step.is_finite()
            || v.perturbation_step <= 0.0
            || !v.min_value.is_finite()
            || !v.max_value.is_finite()
            || v.min_value > v.max_value
            || !v.max_step.is_finite()
            || v.max_step <= 0.0
    }) || goals.iter().any(|g| {
        !g.target_value.is_finite()
            || !g.scale.is_finite()
            || g.scale <= 0.0
            || !g.tolerance.is_finite()
            || g.tolerance <= 0.0
    }) {
        return Err(MissionDesignError::InvalidPlan {
            plan_id: 0,
            reason: "invalid control variable or equality goal specification".to_owned(),
        });
    }
    Ok(())
}

fn compute_residuals(achieved: &[f64], goals: &[ScaledEqualityGoal]) -> (Vec<f64>, f64, bool) {
    let mut residuals = Vec::with_capacity(goals.len());
    let mut sum_sq = 0.0;
    let mut all_converged = true;
    for (&val, g) in achieved.iter().zip(goals.iter()) {
        let r = val - g.target_value;
        if r.abs() > g.tolerance {
            all_converged = false;
        }
        let r_tilde = r / g.scale;
        sum_sq += r_tilde * r_tilde;
        residuals.push(r);
    }
    let rms = sqrt(sum_sq / (goals.len().max(1) as f64));
    (residuals, rms, all_converged)
}

fn compute_central_jacobian<F>(
    u: &[f64],
    variables: &[ScaledControlVariable],
    goals: &[ScaledEqualityGoal],
    evaluate_goals: &mut F,
) -> Result<(Vec<f64>, Vec<f64>), MissionDesignError>
where
    F: FnMut(&[f64]) -> Result<Vec<f64>, MissionDesignError>,
{
    let n_vars = variables.len();
    let m_goals = goals.len();
    let mut jac_unscaled = vec![0.0; m_goals.saturating_mul(n_vars)];
    let mut jac_scaled = vec![0.0; m_goals.saturating_mul(n_vars)];

    for (j, var) in variables.iter().enumerate() {
        let base_u = u.get(j).copied().unwrap_or(var.initial_value);
        let u_plus = (base_u + var.perturbation_step).min(var.max_value);
        let u_minus = (base_u - var.perturbation_step).max(var.min_value);
        let span = u_plus - u_minus;
        if span.abs() <= 1.0e-15 {
            continue;
        }
        let mut p_plus = u.to_vec();
        if let Some(slot) = p_plus.get_mut(j) {
            *slot = u_plus;
        }
        let mut p_minus = u.to_vec();
        if let Some(slot) = p_minus.get_mut(j) {
            *slot = u_minus;
        }
        let g_plus = evaluate_goals(&p_plus)?;
        let g_minus = evaluate_goals(&p_minus)?;
        for (i, goal) in goals.iter().enumerate() {
            let gp = g_plus.get(i).copied().unwrap_or(0.0);
            let gm = g_minus.get(i).copied().unwrap_or(0.0);
            let dg_du = (gp - gm) / span;
            let idx = i.saturating_mul(n_vars).wrapping_add(j);
            if let Some(slot) = jac_unscaled.get_mut(idx) {
                *slot = dg_du;
            }
            if let Some(slot) = jac_scaled.get_mut(idx) {
                *slot = dg_du * (var.scale / goal.scale);
            }
        }
    }
    Ok((jac_unscaled, jac_scaled))
}

fn solve_damped_normal_equations(
    jac_scaled: &[f64],
    scaled_res: &[f64],
    m_goals: usize,
    n_vars: usize,
    lambda: f64,
) -> Option<Vec<f64>> {
    let mut a = vec![0.0; n_vars.saturating_mul(n_vars)];
    let mut b = vec![0.0; n_vars];
    for i in 0..n_vars {
        let mut rhs = 0.0;
        for k in 0..m_goals {
            let j_ki = jac_scaled
                .get(k.saturating_mul(n_vars).wrapping_add(i))
                .copied()
                .unwrap_or(0.0);
            let r_k = scaled_res.get(k).copied().unwrap_or(0.0);
            rhs -= j_ki * r_k;
        }
        if let Some(slot) = b.get_mut(i) {
            *slot = rhs;
        }
        for j in 0..n_vars {
            let mut sum = if i == j { lambda } else { 0.0 };
            for k in 0..m_goals {
                let j_ki = jac_scaled
                    .get(k.saturating_mul(n_vars).wrapping_add(i))
                    .copied()
                    .unwrap_or(0.0);
                let j_kj = jac_scaled
                    .get(k.saturating_mul(n_vars).wrapping_add(j))
                    .copied()
                    .unwrap_or(0.0);
                sum += j_ki * j_kj;
            }
            if let Some(slot) = a.get_mut(i.saturating_mul(n_vars).wrapping_add(j)) {
                *slot = sum;
            }
        }
    }
    gaussian_solve(&mut a, &mut b, n_vars)
}

fn gaussian_solve(a: &mut [f64], b: &mut [f64], n: usize) -> Option<Vec<f64>> {
    for col in 0..n {
        let mut pivot_row = col;
        let mut pivot_abs = a
            .get(col.saturating_mul(n).wrapping_add(col))
            .copied()
            .unwrap_or(0.0)
            .abs();
        for row in col.wrapping_add(1)..n {
            let candidate = a
                .get(row.saturating_mul(n).wrapping_add(col))
                .copied()
                .unwrap_or(0.0)
                .abs();
            if candidate > pivot_abs {
                pivot_abs = candidate;
                pivot_row = row;
            }
        }
        if pivot_abs <= 1.0e-12 || !pivot_abs.is_finite() {
            return None;
        }
        if pivot_row != col {
            swap_rows(a, b, n, col, pivot_row);
        }
        let diag = a
            .get(col.saturating_mul(n).wrapping_add(col))
            .copied()
            .unwrap_or(1.0);
        for row in col.wrapping_add(1)..n {
            let factor = a
                .get(row.saturating_mul(n).wrapping_add(col))
                .copied()
                .unwrap_or(0.0)
                / diag;
            for k in col..n {
                let upper = a
                    .get(col.saturating_mul(n).wrapping_add(k))
                    .copied()
                    .unwrap_or(0.0);
                if let Some(slot) = a.get_mut(row.saturating_mul(n).wrapping_add(k)) {
                    *slot -= factor * upper;
                }
            }
            let b_col = b.get(col).copied().unwrap_or(0.0);
            if let Some(slot) = b.get_mut(row) {
                *slot -= factor * b_col;
            }
        }
    }
    Some(back_substitute(a, b, n))
}

fn swap_rows(a: &mut [f64], b: &mut [f64], n: usize, r1: usize, r2: usize) {
    for k in r1..n {
        let idx1 = r1.saturating_mul(n).wrapping_add(k);
        let idx2 = r2.saturating_mul(n).wrapping_add(k);
        let v1 = a.get(idx1).copied().unwrap_or(0.0);
        let v2 = a.get(idx2).copied().unwrap_or(0.0);
        if let Some(s1) = a.get_mut(idx1) {
            *s1 = v2;
        }
        if let Some(s2) = a.get_mut(idx2) {
            *s2 = v1;
        }
    }
    let b1 = b.get(r1).copied().unwrap_or(0.0);
    let b2 = b.get(r2).copied().unwrap_or(0.0);
    if let Some(s1) = b.get_mut(r1) {
        *s1 = b2;
    }
    if let Some(s2) = b.get_mut(r2) {
        *s2 = b1;
    }
}

fn back_substitute(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut x = vec![0.0; n];
    let mut row_rev = n;
    while row_rev > 0 {
        let i = row_rev - 1;
        let mut rhs = b.get(i).copied().unwrap_or(0.0);
        for j in i.wrapping_add(1)..n {
            let a_ij = a
                .get(i.saturating_mul(n).wrapping_add(j))
                .copied()
                .unwrap_or(0.0);
            let x_j = x.get(j).copied().unwrap_or(0.0);
            rhs -= a_ij * x_j;
        }
        let diag = a
            .get(i.saturating_mul(n).wrapping_add(i))
            .copied()
            .unwrap_or(1.0);
        if let Some(slot) = x.get_mut(i) {
            *slot = rhs / diag;
        }
        row_rev -= 1;
    }
    x
}

fn apply_bounded_step(
    u: &mut [f64],
    delta_tilde: &[f64],
    variables: &[ScaledControlVariable],
) -> bool {
    let mut hit_bounds = false;
    for ((u_val, &dt), var) in u.iter_mut().zip(delta_tilde.iter()).zip(variables.iter()) {
        let raw_step = (dt * var.scale).clamp(-var.max_step, var.max_step);
        let candidate = *u_val + raw_step;
        let clamped = candidate.clamp(var.min_value, var.max_value);
        if (clamped - candidate).abs() > 1.0e-12 {
            hit_bounds = true;
        }
        *u_val = clamped;
    }
    hit_bounds
}

fn build_failed_report(
    reason: SolverFailureReason,
    iterations: u32,
    u: Vec<f64>,
    achieved: Vec<f64>,
    goals: &[ScaledEqualityGoal],
    last_jac: Vec<f64>,
    history: Vec<DifferentialCorrectorIterationRecord>,
) -> DifferentialCorrectorReport {
    let (residuals, scaled_rms, _) = compute_residuals(&achieved, goals);
    DifferentialCorrectorReport {
        status: SolverConvergenceStatus::Failed(reason),
        iterations,
        final_variables: u,
        final_achieved_goals: achieved,
        final_unscaled_residuals: residuals,
        final_scaled_rms_residual: scaled_rms,
        final_jacobian_row_major: last_jac,
        history,
    }
}
