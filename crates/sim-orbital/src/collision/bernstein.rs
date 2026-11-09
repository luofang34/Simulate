//! Certified Bernstein convex-hull basis conversion, de Casteljau subdivision,
//! and dual-certified 3D hull distance.

use glam::DVec3;

use crate::scalar_math::sqrt;

#[inline]
const fn binom(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    if k == 0 || k == n {
        return 1;
    }
    let mut num: u64 = 1;
    let mut den: u64 = 1;
    let mut i: usize = 0;
    while i < k {
        num *= (n - i) as u64;
        den *= (i + 1) as u64;
        i = i.wrapping_add(1);
    }
    num / den
}

/// Converts monomial coefficients $p(x) = \sum_{k=0}^{N-1} a_k x^k$ on $x \in [0, 1]$
/// into exact degree-$(N-1)$ Bernstein control points $[b_0, \dots, b_{N-1}]$.
#[must_use]
pub fn monomial_to_bernstein<const N: usize>(mono: &[f64; N]) -> [f64; N] {
    let deg = N.saturating_sub(1);
    let mut out = [0.0_f64; N];
    let mut j: usize = 0;
    while j < N {
        let mut sum = 0.0_f64;
        let mut k: usize = 0;
        while k <= j {
            let ak = mono.get(k).copied().unwrap_or(0.0);
            let num = binom(j, k) as f64;
            let den = binom(deg, k) as f64;
            sum += ak * (num / den);
            k = k.wrapping_add(1);
        }
        if let Some(slot) = out.get_mut(j) {
            *slot = sum;
        }
        j = j.wrapping_add(1);
    }
    out
}

/// Evaluates a degree-$(N-1)$ scalar Bernstein polynomial at $x \in [0, 1]$ via de Casteljau.
#[must_use]
pub fn eval_bernstein_scalar<const N: usize>(ctrl: &[f64; N], x: f64) -> f64 {
    if N == 0 {
        return 0.0;
    }
    let one_minus_x = 1.0 - x;
    let mut work = *ctrl;
    let mut len = N.saturating_sub(1);
    while len >= 1 {
        let mut i: usize = 0;
        while i < len {
            let l = work.get(i).copied().unwrap_or(0.0);
            let r = work.get(i.wrapping_add(1)).copied().unwrap_or(0.0);
            if let Some(slot) = work.get_mut(i) {
                *slot = one_minus_x * l + x * r;
            }
            i = i.wrapping_add(1);
        }
        len -= 1;
    }
    work.first().copied().unwrap_or(0.0)
}

/// Evaluates $(p(x), p'(x))$ wrt normalized $x \in [0, 1]$ for a degree-$(N-1)$ Bernstein polynomial.
#[must_use]
pub fn eval_bernstein_scalar_with_deriv<const N: usize>(ctrl: &[f64; N], x: f64) -> (f64, f64) {
    if N <= 1 {
        return (ctrl.first().copied().unwrap_or(0.0), 0.0);
    }
    let one_minus_x = 1.0 - x;
    let mut work = *ctrl;
    let mut len = N.saturating_sub(1);
    while len >= 2 {
        let mut i: usize = 0;
        while i < len {
            let l = work.get(i).copied().unwrap_or(0.0);
            let r = work.get(i.wrapping_add(1)).copied().unwrap_or(0.0);
            if let Some(slot) = work.get_mut(i) {
                *slot = one_minus_x * l + x * r;
            }
            i = i.wrapping_add(1);
        }
        len -= 1;
    }
    let b0 = work.first().copied().unwrap_or(0.0);
    let b1 = work.get(1).copied().unwrap_or(0.0);
    let val = one_minus_x * b0 + x * b1;
    let deriv = ((N - 1) as f64) * (b1 - b0);
    (val, deriv)
}

/// Splits a degree-$(N-1)$ scalar Bernstein polynomial at $s = 0.5$ into `(left, right)`.
#[must_use]
pub fn split_bernstein_mid<const N: usize>(ctrl: &[f64; N]) -> ([f64; N], [f64; N]) {
    let mut left = [0.0_f64; N];
    let mut right = [0.0_f64; N];
    let mut work = *ctrl;
    if let Some(slot) = left.first_mut() {
        *slot = work.first().copied().unwrap_or(0.0);
    }
    if let Some(slot) = right.last_mut() {
        *slot = work.last().copied().unwrap_or(0.0);
    }
    let mut step: usize = 1;
    while step < N {
        let len = N.saturating_sub(step);
        let mut i: usize = 0;
        while i < len {
            let l = work.get(i).copied().unwrap_or(0.0);
            let r = work.get(i.wrapping_add(1)).copied().unwrap_or(0.0);
            if let Some(slot) = work.get_mut(i) {
                *slot = 0.5 * (l + r);
            }
            i = i.wrapping_add(1);
        }
        if let Some(slot) = left.get_mut(step) {
            *slot = work.first().copied().unwrap_or(0.0);
        }
        if let Some(slot) = right.get_mut(N - 1 - step) {
            *slot = work.get(len.saturating_sub(1)).copied().unwrap_or(0.0);
        }
        step = step.wrapping_add(1);
    }
    (left, right)
}

/// Splits a degree-$(N-1)$ 3D Bernstein curve at $s = 0.5$ into `(left, right)`.
#[must_use]
pub fn split_bernstein_vec3_mid<const N: usize>(ctrl: &[DVec3; N]) -> ([DVec3; N], [DVec3; N]) {
    let mut left = [DVec3::ZERO; N];
    let mut right = [DVec3::ZERO; N];
    let mut work = *ctrl;
    if let Some(slot) = left.first_mut() {
        *slot = work.first().copied().unwrap_or(DVec3::ZERO);
    }
    if let Some(slot) = right.last_mut() {
        *slot = work.last().copied().unwrap_or(DVec3::ZERO);
    }
    let mut step: usize = 1;
    while step < N {
        let len = N.saturating_sub(step);
        let mut i: usize = 0;
        while i < len {
            let l = work.get(i).copied().unwrap_or(DVec3::ZERO);
            let r = work.get(i.wrapping_add(1)).copied().unwrap_or(DVec3::ZERO);
            if let Some(slot) = work.get_mut(i) {
                *slot = 0.5 * (l + r);
            }
            i = i.wrapping_add(1);
        }
        if let Some(slot) = left.get_mut(step) {
            *slot = work.first().copied().unwrap_or(DVec3::ZERO);
        }
        if let Some(slot) = right.get_mut(N - 1 - step) {
            *slot = work
                .get(len.saturating_sub(1))
                .copied()
                .unwrap_or(DVec3::ZERO);
        }
        step = step.wrapping_add(1);
    }
    (left, right)
}

/// Evaluates a degree-$(N-1)$ 3D Bernstein curve at $x \in [0, 1]$ via de Casteljau.
#[must_use]
pub fn eval_bernstein_vec3<const N: usize>(ctrl: &[DVec3; N], x: f64) -> DVec3 {
    if N == 0 {
        return DVec3::ZERO;
    }
    let one_minus_x = 1.0 - x;
    let mut work = *ctrl;
    let mut len = N.saturating_sub(1);
    while len >= 1 {
        let mut i: usize = 0;
        while i < len {
            let l = work.get(i).copied().unwrap_or(DVec3::ZERO);
            let r = work.get(i.wrapping_add(1)).copied().unwrap_or(DVec3::ZERO);
            if let Some(slot) = work.get_mut(i) {
                *slot = one_minus_x * l + x * r;
            }
            i = i.wrapping_add(1);
        }
        len -= 1;
    }
    work.first().copied().unwrap_or(DVec3::ZERO)
}

/// Computes a **dual-certified conservative lower bound** on
/// $\operatorname{dist}(\mathbf{0}, \operatorname{Conv}(\mathbf{b}_0, \dots, \mathbf{b}_{N-1}))$.
#[must_use]
pub fn certified_hull_min_norm_3d<const N: usize>(ctrl: &[DVec3; N]) -> f64 {
    if N == 0 {
        return 0.0;
    }
    let mut max_norm = 0.0_f64;
    let mut centroid = DVec3::ZERO;
    for &p in ctrl {
        max_norm = max_norm.max(sqrt(p.length_squared()));
        centroid += p;
    }
    let mut v = centroid / (N as f64);
    for _ in 0..16_u32 {
        if v.length_squared() <= 1.0e-30 {
            return 0.0;
        }
        let mut best_p = *ctrl.first().unwrap_or(&DVec3::ZERO);
        let mut best_dot = best_p.dot(v);
        for &p in ctrl.iter().skip(1) {
            let d = p.dot(v);
            if d < best_dot {
                best_dot = d;
                best_p = p;
            }
        }
        let diff = v - best_p;
        let denom = diff.length_squared();
        if denom <= 1.0e-30 {
            break;
        }
        let gamma = (v.dot(diff) / denom).clamp(0.0, 1.0);
        if gamma <= 1.0e-14 {
            break;
        }
        v -= gamma * diff;
    }

    let mut best_cert = 0.0_f64;
    let candidates = [
        v,
        centroid,
        *ctrl.first().unwrap_or(&DVec3::ZERO),
        *ctrl.last().unwrap_or(&DVec3::ZERO),
    ];
    let roundoff = 16.0 * f64::EPSILON * max_norm;
    for cand in candidates {
        let n_sq = cand.length_squared();
        if n_sq > 1.0e-30 {
            let u = cand / sqrt(n_sq);
            let mut min_proj = f64::INFINITY;
            for &p in ctrl {
                min_proj = min_proj.min(p.dot(u));
            }
            best_cert = best_cert.max((min_proj - roundoff).max(0.0));
        }
    }
    best_cert
}
