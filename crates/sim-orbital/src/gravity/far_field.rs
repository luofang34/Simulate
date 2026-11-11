//! Where a body's spherical-harmonic field stops mattering.
//!
//! Far from an extended body its non-central field falls off as $(R/r)^n \mu / r^2$
//! while every other force keeps its size, so beyond some radius the field is below
//! anything the integrators resolve and evaluating it (a body rotation, the harmonic
//! recurrence, and for a body other than the origin an ephemeris lookup) is wasted.
//! [`harmonic_reach_m`] is that radius for one body: beyond it the field is skipped
//! outright, inside it the field is evaluated exactly as without a reach, so nothing
//! flown near a body changes.

use crate::{constants::BodyGravitySpec, scalar_math::sqrt};

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

/// Distance from `spec`'s centre (`m`) beyond which its harmonic terms of degree at
/// most `max_degree` provably accelerate less than `threshold_m_s2`; `f64::INFINITY`
/// (never skipped) for a non-positive or non-finite threshold, a body without such
/// terms, or a field that never falls below the threshold.
///
/// The bound per term follows the recurrence [`super::evaluate_spherical_harmonics_fixed`]
/// evaluates: an acceleration of degree $n$ is built from normalized
/// $\bar V_{n+1,k}, \bar W_{n+1,k}$, each at most $(R/r)^{n+2}\sqrt{2n+3}$ in
/// magnitude because a fully normalized associated Legendre function of degree $n+1$
/// is at most $\sqrt{2n+3}$.
#[must_use]
pub fn harmonic_reach_m(spec: &BodyGravitySpec, max_degree: u32, threshold_m_s2: f64) -> f64 {
    let r_eq = spec.equatorial_radius_m;
    let has_terms = spec
        .harmonics
        .iter()
        .any(|t| t.degree >= 1 && t.order <= t.degree && t.degree <= max_degree);
    let usable_threshold = threshold_m_s2.is_finite() && threshold_m_s2 > 0.0;
    if !has_terms || r_eq <= 0.0 || spec.mu_m3_s2 <= 0.0 || !usable_threshold {
        return f64::INFINITY;
    }
    let bound = |r: f64| field_bound_m_s2(spec, max_degree, r);
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

/// Upper bound on the magnitude of `spec`'s harmonic acceleration of degree at most
/// `max_degree` anywhere at distance `r_m` from its centre (`m/s²`).
fn field_bound_m_s2(spec: &BodyGravitySpec, max_degree: u32, r_m: f64) -> f64 {
    let ratio = spec.equatorial_radius_m / r_m;
    let central = spec.mu_m3_s2 / (r_m * r_m);
    let mut total = 0.0_f64;
    for term in &spec.harmonics {
        let n = term.degree;
        if n == 0 || term.order > n || n > max_degree {
            continue;
        }
        let mut falloff = 1.0_f64;
        for _ in 0..n {
            falloff *= ratio;
        }
        total += central * falloff * term_factor(n, term.order, term.c_bar, term.s_bar);
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
