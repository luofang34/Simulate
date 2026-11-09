//! Finite-difference Jacobian sensitivity evaluation and damped Gauss-Newton
//! parameter-identification (`(J^T W J + λ I) Δθ = -J^T W r`) solver.

use crate::calibration::results::{
    ParameterAdjustment, RawSuiteEvaluation, SensitivityReport, build_report,
};
use crate::calibration::schema::CalibrationSuite;
use crate::calibration::solver::{ParameterOverrides, run_suite_with_overrides};
use crate::{LoadedSyspack, PackError};

fn perturbation_specs() -> [(&'static str, f64, ParameterOverrides); 4] {
    [
        (
            "volumetric_efficiency_scale",
            0.02,
            ParameterOverrides {
                volumetric_efficiency_scale: Some(1.02),
                ..ParameterOverrides::default()
            },
        ),
        (
            "bsfc_scale",
            0.02,
            ParameterOverrides {
                bsfc_scale: Some(1.02),
                ..ParameterOverrides::default()
            },
        ),
        (
            "friction_hp_delta",
            2.0,
            ParameterOverrides {
                friction_hp_delta: Some(2.0),
                ..ParameterOverrides::default()
            },
        ),
        (
            "equiv_mass_lbs",
            25.0,
            ParameterOverrides {
                extra_mass_lbs: 25.0,
                ..ParameterOverrides::default()
            },
        ),
    ]
}

pub(super) fn compute_sensitivities(
    package: &LoadedSyspack,
    suite: &CalibrationSuite,
) -> Result<SensitivityReport, PackError> {
    let base_raw = run_suite_with_overrides(package, suite, ParameterOverrides::default())?;
    let base_vec = normalized_residuals(suite, &base_raw);
    let baseline_cost: f64 = base_vec.iter().map(|r| r * r).sum();
    let baseline = build_report(suite, base_raw);

    let specs = perturbation_specs();
    let mut cols: Vec<Vec<f64>> = Vec::with_capacity(specs.len());
    let mut adjustments = Vec::with_capacity(specs.len());
    for (name, step, ovr) in specs {
        let pert = run_suite_with_overrides(package, suite, ovr)?;
        let pert_vec = normalized_residuals(suite, &pert);
        let col: Vec<f64> = pert_vec
            .iter()
            .zip(base_vec.iter())
            .map(|(p, b)| (p - b) / step)
            .collect();
        let d_climb = mean_diff(
            pert.climbs.iter().map(|c| c.sim_fpm),
            baseline.climbs.iter().map(|c| c.sim_fpm),
            step,
        );
        let d_ktas = mean_diff(
            pert.cruises.iter().map(|c| c.sim_ktas),
            baseline.cruises.iter().map(|c| c.sim_ktas),
            step,
        );
        let d_gph = mean_diff(
            pert.cruises.iter().filter_map(|c| c.sim_gph),
            baseline.cruises.iter().filter_map(|c| c.sim_gph),
            step,
        );
        cols.push(col);
        adjustments.push(ParameterAdjustment {
            parameter: name.to_owned(),
            perturbation_step: step,
            mean_d_climb_fpm: d_climb,
            mean_d_cruise_ktas: d_ktas,
            mean_d_cruise_gph: d_gph,
            recommended_delta: 0.0,
        });
    }

    let deltas = damped_gauss_newton(&cols, &base_vec, 0.25);
    for (adj, d) in adjustments.iter_mut().zip(deltas.iter().copied()) {
        adj.recommended_delta = d;
    }
    let linearized_predicted_cost = predicted_cost(&base_vec, &cols, &deltas);
    Ok(SensitivityReport {
        baseline,
        baseline_cost,
        linearized_predicted_cost,
        adjustments,
    })
}

fn normalized_residuals(suite: &CalibrationSuite, raw: &RawSuiteEvaluation) -> Vec<f64> {
    let tol = &suite.tolerances;
    let mut r = Vec::new();
    for s in &raw.stalls {
        r.push(s.residual_kcas / tol.stall_kt);
    }
    for c in &raw.climbs {
        r.push(c.residual_fpm / tol.climb_fpm);
    }
    for cr in &raw.cruises {
        r.push(cr.residual_ktas / tol.cruise_ktas);
        if let Some(rg) = cr.residual_gph {
            r.push(rg / tol.cruise_gph);
        }
    }
    for (e, spec) in raw.empirical.iter().zip(suite.empirical_segments.iter()) {
        r.push(e.residual_gs_kt / spec.gs_tolerance_kt.max(1.0));
    }
    r
}

fn damped_gauss_newton(cols: &[Vec<f64>], r: &[f64], lambda: f64) -> Vec<f64> {
    let p = cols.len();
    let mut a = vec![0.0_f64; p * p];
    let mut b = vec![0.0_f64; p];
    for (i, ci) in cols.iter().enumerate() {
        let mut grad = 0.0;
        for (v, rv) in ci.iter().zip(r.iter()) {
            grad += v * rv;
        }
        if let Some(slot) = b.get_mut(i) {
            *slot = -grad;
        }
        for (j, cj) in cols.iter().enumerate() {
            let mut dot = if i == j { lambda } else { 0.0 };
            for (vi, vj) in ci.iter().zip(cj.iter()) {
                dot += vi * vj;
            }
            if let Some(slot) = a.get_mut(i * p + j) {
                *slot = dot;
            }
        }
    }
    solve_linear_system(a, b, p)
}

fn solve_linear_system(mut a: Vec<f64>, mut b: Vec<f64>, n: usize) -> Vec<f64> {
    for k in 0..n {
        let diag = a.get(k * n + k).copied().unwrap_or(0.0);
        if diag.abs() < 1e-12 {
            continue;
        }
        for i in k.wrapping_add(1)..n {
            let factor = a.get(i * n + k).copied().unwrap_or(0.0) / diag;
            for j in k..n {
                let akj = a.get(k * n + j).copied().unwrap_or(0.0);
                if let Some(aij) = a.get_mut(i * n + j) {
                    *aij -= factor * akj;
                }
            }
            let bk = b.get(k).copied().unwrap_or(0.0);
            if let Some(bi) = b.get_mut(i) {
                *bi -= factor * bk;
            }
        }
    }
    let mut x = vec![0.0_f64; n];
    for k in (0..n).rev() {
        let diag = a.get(k * n + k).copied().unwrap_or(0.0);
        if diag.abs() < 1e-12 {
            continue;
        }
        let mut sum = b.get(k).copied().unwrap_or(0.0);
        for j in k.wrapping_add(1)..n {
            let akj = a.get(k * n + j).copied().unwrap_or(0.0);
            let xj = x.get(j).copied().unwrap_or(0.0);
            sum -= akj * xj;
        }
        if let Some(xk) = x.get_mut(k) {
            *xk = sum / diag;
        }
    }
    x
}

fn predicted_cost(base: &[f64], cols: &[Vec<f64>], deltas: &[f64]) -> f64 {
    let mut cost = 0.0;
    for (idx, r0) in base.iter().copied().enumerate() {
        let mut lin = r0;
        for (col, d) in cols.iter().zip(deltas.iter().copied()) {
            lin += col.get(idx).copied().unwrap_or(0.0) * d;
        }
        cost += lin * lin;
    }
    cost
}

fn mean_diff(pert: impl Iterator<Item = f64>, base: impl Iterator<Item = f64>, step: f64) -> f64 {
    let mut sum = 0.0;
    let mut count = 0_u32;
    for (p, b) in pert.zip(base) {
        sum += (p - b) / step;
        count = count.wrapping_add(1);
    }
    if count == 0 {
        0.0
    } else {
        sum / f64::from(count)
    }
}
