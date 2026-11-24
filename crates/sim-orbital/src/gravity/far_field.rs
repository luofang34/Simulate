//! Where a body's spherical-harmonic field stops mattering.
//!
//! Far from an extended body its non-central field falls off as $(R/r)^n \mu / r^2$
//! while every other force keeps its size, so beyond some radius the field is below
//! anything the integrators resolve and evaluating it (a body rotation, the harmonic
//! recurrence, and for a body other than the origin an ephemeris lookup) is wasted.
//! [`harmonic_reach_m`] is that radius for one body: beyond it the field is skipped
//! outright, inside it the field is evaluated exactly as without a reach, so nothing
//! flown near a body changes.

use sim_bodies::HarmonicGravity;

use crate::scalar_math::sqrt;

/// Largest acceleration (`m/s²`) a skipped harmonic field may have.
///
/// The default [`crate::Dop853Tolerances`] admit `1e-9 m/s` of velocity error per step
/// of up to `3,600 s`, which is `2.8e-13 m/s²` held over a whole step, and the
/// $L_2$ holding prediction admits `1e-8 m/s` per step of up to `43,200 s`
/// (`2.3e-13 m/s²`); the relative tolerance admits far more at interplanetary speeds.
/// A skipped field is below even the smaller of these, so dropping it moves each step's
/// result by less than the step's own accepted error. The skip is systematic rather
/// than random: held for 30 days it displaces a trajectory by at most
/// $\tfrac12 a t^2 \approx 0.3$ m, against sunlight-pressure, ephemeris and
/// gravitational-parameter uncertainties that are orders of magnitude larger.
pub const HARMONIC_FAR_FIELD_ACCEL_M_S2: f64 = 1.0e-13;

/// Doublings of the reference radius tried before the reach is taken as unbounded.
const MAX_DOUBLINGS: u32 = 256;
/// Bisection steps narrowing the reach once it is bracketed.
const BISECTION_STEPS: u32 = 64;

/// Distance from the centre of `field`'s body (`m`) beyond which the terms `field` sums
/// provably accelerate less than `threshold_m_s2`; `f64::INFINITY` (never skipped) for a
/// non-positive or non-finite threshold, a field with no non-central term, or one that
/// never falls below the threshold.
///
/// The bound per term follows the solid-harmonic recurrence: an acceleration of degree
/// $n$ is built from normalized $\bar V_{n+1,k}, \bar W_{n+1,k}$, each at most
/// $(R/r)^{n+2}\sqrt{2n+3}$ in magnitude because a fully normalized associated Legendre
/// function of degree $n+1$ is at most $\sqrt{2n+3}$.
#[must_use]
pub fn harmonic_reach_m(field: &HarmonicGravity, threshold_m_s2: f64) -> f64 {
    let r_eq = field.radius_m();
    let usable_threshold = threshold_m_s2.is_finite() && threshold_m_s2 > 0.0;
    if !has_terms(field) || !usable_threshold {
        return f64::INFINITY;
    }
    let bound = |r: f64| field_bound_m_s2(field, r);
    let mut outside = r_eq;
    let mut doublings = 0_u32;
    while bound(outside) > threshold_m_s2 {
        if doublings >= MAX_DOUBLINGS {
            return f64::INFINITY;
        }
        outside *= 2.0;
        doublings = doublings.wrapping_add(1);
    }
    if doublings == 0 {
        return r_eq;
    }
    let mut inside = 0.5 * outside;
    for _ in 0..BISECTION_STEPS {
        let mid = 0.5 * (inside + outside);
        if bound(mid) > threshold_m_s2 {
            inside = mid;
        } else {
            outside = mid;
        }
    }
    outside
}

/// The non-central terms `field` sums that are not zero, as `(n, m, C̄nm, S̄nm)`.
fn summed_terms(field: &HarmonicGravity) -> impl Iterator<Item = (u32, u32, f64, f64)> + '_ {
    let truncation = field.truncation();
    (truncation.min_degree.max(1)..=truncation.degree).flat_map(move |n| {
        (0..=n.min(truncation.order)).filter_map(move |m| {
            let (c_bar, s_bar) = field.coefficient(n, m);
            let degree = u32::try_from(n).ok()?;
            let order = u32::try_from(m).ok()?;
            (c_bar != 0.0 || s_bar != 0.0).then_some((degree, order, c_bar, s_bar))
        })
    })
}

fn has_terms(field: &HarmonicGravity) -> bool {
    summed_terms(field).next().is_some()
}

/// Upper bound on the magnitude of the non-central acceleration `field` sums anywhere at
/// distance `r_m` from its body's centre (`m/s²`).
fn field_bound_m_s2(field: &HarmonicGravity, r_m: f64) -> f64 {
    let ratio = field.radius_m() / r_m;
    let central = field.gm_m3_s2() / (r_m * r_m);
    let mut total = 0.0_f64;
    for (degree, order, c_bar, s_bar) in summed_terms(field) {
        let mut falloff = 1.0_f64;
        for _ in 0..degree {
            falloff *= ratio;
        }
        total += central * falloff * term_factor(degree, order, c_bar, s_bar);
    }
    total
}

/// $|\mathbf a_{nm}| \le \frac{\mu}{r^2}(R/r)^n$ times this, from the recurrence's
/// coefficients and the bound on normalized $\bar V, \bar W$ of degree $n + 1$.
fn term_factor(degree: u32, order: u32, c_bar: f64, s_bar: f64) -> f64 {
    let n = f64::from(degree);
    let m = f64::from(order);
    let vw_max = sqrt(2.0 * n + 3.0);
    if order == 0 {
        let f_xy = sqrt(((2.0 * n + 1.0) * (n + 1.0) * (n + 2.0)) / (2.0 * (2.0 * n + 3.0)));
        let f_z = (n + 1.0) * sqrt((2.0 * n + 1.0) / (2.0 * n + 3.0));
        return c_bar.abs() * vw_max * sqrt(2.0 * f_xy * f_xy + f_z * f_z);
    }
    let f_plus = 0.5 * sqrt(((2.0 * n + 1.0) * (n + m + 1.0) * (n + m + 2.0)) / (2.0 * n + 3.0));
    let f_minus = if order == 1 {
        sqrt(((2.0 * n + 1.0) * n * (n + 1.0)) / (2.0 * (2.0 * n + 3.0)))
    } else {
        0.5 * sqrt(((2.0 * n + 1.0) * (n - m + 1.0) * (n - m + 2.0)) / (2.0 * n + 3.0))
    };
    let f_z = sqrt(((2.0 * n + 1.0) * (n + m + 1.0) * (n - m + 1.0)) / (2.0 * n + 3.0));
    let lateral = f_plus + f_minus;
    (c_bar.abs() + s_bar.abs()) * vw_max * sqrt(2.0 * lateral * lateral + f_z * f_z)
}
