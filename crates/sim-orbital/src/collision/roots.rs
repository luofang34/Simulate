//! Recursive de Casteljau scalar Bernstein root, initial-overlap, and multiplicity-2
//! grazing-tangency isolation.

use crate::collision::bernstein::{
    eval_bernstein_scalar, eval_bernstein_scalar_with_deriv, split_bernstein_mid,
};

/// Classification of an isolated root or contact transition on $g(\tau) = 0$.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ContactRootKind {
    /// Clearance already non-positive at segment start ($g(\tau_0) \le 0$).
    InitialOverlap,
    /// Transversal entry into contact/hazard zone ($g(\tau^*) = 0,\; \dot{g}(\tau^*) < 0$).
    Entry,
    /// Transversal exit from contact/hazard zone ($g(\tau^*) = 0,\; \dot{g}(\tau^*) > 0$).
    Exit,
    /// Tangential grazing contact ($g(\tau^*) \approx 0,\; \dot{g}(\tau^*) = 0$) or
    /// conservative remainder-enclosed micro-bracket.
    Grazing,
}

/// One certified isolated contact root on a segment $[\tau_a, \tau_b]$.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IsolatedContactRoot {
    /// Segment-local offset $\tau^* \in [\tau_a, \tau_b]$ ($\text{s}$).
    pub tau_s: f64,
    /// Evaluated polynomial clearance $g(\tau^*)$.
    pub value: f64,
    /// Classification of the crossing or tangency.
    pub kind: ContactRootKind,
}

/// Isolates all roots ($g(\tau) = 0$), initial overlaps ($g(\tau_a) \le 0$), and grazing
/// tangencies ($g(\tau^*) = \dot{g}(\tau^*) = 0$) of a degree-$(N-1)$ Bernstein polynomial
/// on $[\tau_a, \tau_b]$ in strict chronological order, accounting for trajectory remainder
/// `remainder_bound_fn(sub_span_s) >= 0`.
pub fn isolate_scalar_bernstein_roots<const N: usize, R>(
    ctrl: &[f64; N],
    tau_a: f64,
    tau_b: f64,
    tol_tau_s: f64,
    graze_value_tol: f64,
    remainder_bound_fn: &R,
) -> Vec<IsolatedContactRoot>
where
    R: Fn(f64) -> f64,
{
    let mut raw = Vec::new();
    let g0 = ctrl.first().copied().unwrap_or(0.0);
    if g0 <= 0.0 {
        raw.push(IsolatedContactRoot {
            tau_s: tau_a,
            value: g0,
            kind: ContactRootKind::InitialOverlap,
        });
    }
    let ctx = RootSearchCtx {
        root_start_tau: tau_a,
        tol_tau_s: tol_tau_s.max(1.0e-12),
        graze_value_tol: graze_value_tol.max(1.0e-14),
        remainder_bound_fn,
    };
    subdivide_scalar_roots(ctrl, tau_a, tau_b, &ctx, &mut raw, 0);
    coalesce_roots(raw, ctx.tol_tau_s * 4.0)
}

struct RootSearchCtx<'a, R> {
    root_start_tau: f64,
    tol_tau_s: f64,
    graze_value_tol: f64,
    remainder_bound_fn: &'a R,
}

fn subdivide_scalar_roots<const N: usize, R>(
    ctrl: &[f64; N],
    a: f64,
    b: f64,
    ctx: &RootSearchCtx<'_, R>,
    out: &mut Vec<IsolatedContactRoot>,
    depth: u32,
) where
    R: Fn(f64) -> f64,
{
    let span = b - a;
    if span <= 0.0 {
        return;
    }
    let rem = (ctx.remainder_bound_fn)(span).max(0.0);
    let (mut c_min, mut c_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for &c in ctrl {
        c_min = c_min.min(c);
        c_max = c_max.max(c);
    }
    if c_min > rem || c_max < -rem {
        return;
    }

    if rem <= ctx.graze_value_tol {
        if let Some(root) = try_monotone_or_convex_graze(ctrl, a, b, ctx) {
            out.push(root);
            return;
        }
        if is_strictly_monotone_same_sign(ctrl) {
            return;
        }
    }

    if span <= ctx.tol_tau_s || depth >= 54 {
        let mid_val = eval_bernstein_scalar(ctrl, 0.5);
        if mid_val.abs() <= rem + ctx.graze_value_tol {
            out.push(IsolatedContactRoot {
                tau_s: 0.5 * (a + b),
                value: mid_val,
                kind: ContactRootKind::Grazing,
            });
        }
        return;
    }

    let (left, right) = split_bernstein_mid(ctrl);
    let mid = 0.5 * (a + b);
    subdivide_scalar_roots(&left, a, mid, ctx, out, depth.wrapping_add(1));
    subdivide_scalar_roots(&right, mid, b, ctx, out, depth.wrapping_add(1));
}

fn is_strictly_monotone_same_sign<const N: usize>(ctrl: &[f64; N]) -> bool {
    if N <= 1 {
        return true;
    }
    let (d_min, d_max) = deriv_bounds(ctrl);
    let mono = d_min > 0.0 || d_max < 0.0;
    let ga = ctrl.first().copied().unwrap_or(0.0);
    let gb = ctrl.last().copied().unwrap_or(0.0);
    mono && ((ga > 0.0 && gb > 0.0) || (ga < 0.0 && gb < 0.0))
}

fn try_monotone_or_convex_graze<const N: usize, R>(
    ctrl: &[f64; N],
    a: f64,
    b: f64,
    ctx: &RootSearchCtx<'_, R>,
) -> Option<IsolatedContactRoot> {
    if N <= 1 {
        return None;
    }
    let (d_min, d_max) = deriv_bounds(ctrl);
    let ga = ctrl.first().copied().unwrap_or(0.0);
    let gb = ctrl.last().copied().unwrap_or(0.0);

    if (d_min > 0.0 || d_max < 0.0) && ga * gb <= 0.0 {
        if a == ctx.root_start_tau && ga == 0.0 && gb > 0.0 {
            return None;
        }
        let s = refine_monotone_root_unit(ctrl);
        let (val, dg) = eval_bernstein_scalar_with_deriv(ctrl, s);
        let kind = if dg < 0.0 {
            ContactRootKind::Entry
        } else {
            ContactRootKind::Exit
        };
        return Some(IsolatedContactRoot {
            tau_s: a + s * (b - a),
            value: val,
            kind,
        });
    }

    if N >= 3 && ga > 0.0 && gb > 0.0 {
        let (dd_min, _) = second_deriv_bounds(ctrl);
        let d0 = first_diff(ctrl, 0);
        let d_last = first_diff(ctrl, N - 2);
        if dd_min > 0.0 && d0 <= 0.0 && d_last >= 0.0 {
            let s_min = refine_extremum_unit(ctrl);
            let val = eval_bernstein_scalar(ctrl, s_min);
            if val.abs() <= ctx.graze_value_tol {
                return Some(IsolatedContactRoot {
                    tau_s: a + s_min * (b - a),
                    value: val,
                    kind: ContactRootKind::Grazing,
                });
            }
        }
    }
    None
}

fn first_diff<const N: usize>(ctrl: &[f64; N], idx: usize) -> f64 {
    let l = ctrl.get(idx).copied().unwrap_or(0.0);
    let r = ctrl.get(idx.wrapping_add(1)).copied().unwrap_or(0.0);
    ((N - 1) as f64) * (r - l)
}

fn deriv_bounds<const N: usize>(ctrl: &[f64; N]) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut i: usize = 0;
    while i + 1 < N {
        let d = first_diff(ctrl, i);
        lo = lo.min(d);
        hi = hi.max(d);
        i = i.wrapping_add(1);
    }
    (lo, hi)
}

fn second_deriv_bounds<const N: usize>(ctrl: &[f64; N]) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let scale = ((N - 1) * (N - 2)) as f64;
    let mut i: usize = 0;
    while i + 2 < N {
        let c0 = ctrl.get(i).copied().unwrap_or(0.0);
        let c1 = ctrl.get(i.wrapping_add(1)).copied().unwrap_or(0.0);
        let c2 = ctrl.get(i.wrapping_add(2)).copied().unwrap_or(0.0);
        let dd = scale * (c2 - 2.0 * c1 + c0);
        lo = lo.min(dd);
        hi = hi.max(dd);
        i = i.wrapping_add(1);
    }
    (lo, hi)
}

fn refine_monotone_root_unit<const N: usize>(ctrl: &[f64; N]) -> f64 {
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    let mut flo = ctrl.first().copied().unwrap_or(0.0);
    let fhi = ctrl.last().copied().unwrap_or(0.0);
    if flo == 0.0 {
        return 0.0;
    }
    if fhi == 0.0 {
        return 1.0;
    }
    let mut x = (flo.abs() / (flo.abs() + fhi.abs())).clamp(0.0, 1.0);
    for _ in 0..32_u32 {
        let (fx, dfx) = eval_bernstein_scalar_with_deriv(ctrl, x);
        if fx.abs() <= 1.0e-15 || (hi - lo) <= 1.0e-15 {
            break;
        }
        if (flo < 0.0 && fx < 0.0) || (flo > 0.0 && fx > 0.0) {
            lo = x;
            flo = fx;
        } else {
            hi = x;
        }
        if dfx.abs() > 1.0e-20 {
            let xn = x - fx / dfx;
            x = if xn > lo && xn < hi {
                xn
            } else {
                0.5 * (lo + hi)
            };
        } else {
            x = 0.5 * (lo + hi);
        }
    }
    x
}

fn refine_extremum_unit<const N: usize>(ctrl: &[f64; N]) -> f64 {
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    for _ in 0..52_u32 {
        let mid = 0.5 * (lo + hi);
        let (_, dm) = eval_bernstein_scalar_with_deriv(ctrl, mid);
        if dm.abs() <= 1.0e-15 || (hi - lo) <= 1.0e-15 {
            return mid;
        }
        if dm < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

fn coalesce_roots(roots: Vec<IsolatedContactRoot>, min_sep_s: f64) -> Vec<IsolatedContactRoot> {
    let mut out: Vec<IsolatedContactRoot> = Vec::with_capacity(roots.len());
    for r in roots {
        if let Some(prev) = out.last_mut()
            && (r.tau_s - prev.tau_s).abs() <= min_sep_s
            && (prev.kind == ContactRootKind::Grazing || r.kind == prev.kind)
        {
            if r.value.abs() < prev.value.abs() {
                prev.tau_s = r.tau_s;
                prev.value = r.value;
            }
            continue;
        }
        out.push(r);
    }
    out
}
