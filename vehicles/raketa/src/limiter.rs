//! Exact quadratic feasible-interval `4.0G` proper-acceleration limiter for `Raketa`.
//!
//! Solves for main-engine specific thrust $a \in [0, a_{\text{cmd}}]$ along unit thrust axis
//! $\hat{\mathbf{e}}_T$ in the presence of non-main-engine specific force
//! $\mathbf{a}_{\text{ext}} = (\mathbf{F}_{\text{rcs}} + \mathbf{F}_{\text{aero}}) / m$:
//! - **Never adds uncommanded thrust** ($a_{\text{cmd}} = 0 \implies a_{\text{eff}} = 0$).
//! - **Never clamps passive aerodynamic drag or ground contact**.
//! - When the feasible interval $[0, a_{\text{cmd}}] \cap [-b - \sqrt{\Delta}, -b + \sqrt{\Delta}]$
//!   is non-empty, selects $a_{\text{eff}} = \min(a_{\text{cmd}}, -b + \sqrt{\Delta})$.
//! - When infeasible (e.g. $\mathbf{a}_{\text{ext}} = -5g_0\hat{\mathbf{e}}_T$ with $a_{\text{cmd}} = 2g_0$,
//!   or $\|\mathbf{a}_{\text{ext},\perp}\| > 4g_0$), selects the unique minimizer of
//!   $\|a\hat{\mathbf{e}}_T + \mathbf{a}_{\text{ext}}\|^2$ on $[0, a_{\text{cmd}}]$:
//!   $$a_{\text{eff}} = \operatorname{clamp}(-b, 0, a_{\text{cmd}})$$
//!   and asserts [`ALERT_PROPER_ACCEL_EXCEEDANCE`].

use glam::DVec3;
use indicate_alerts::AlertId;
use sim_orbital::{FOUR_G_PROPER_ACCEL_M_S2, STANDARD_GRAVITY_G0_M_S2};

/// Avionics alert identifier asserted whenever passive aerodynamic/external load makes the
/// `4.0G` proper-acceleration sphere unreachable on $a \in [0, a_{\text{cmd}}]$.
pub const ALERT_PROPER_ACCEL_EXCEEDANCE: AlertId = AlertId(0x4734);

/// Input to the exact quadratic `4.0G` proper-acceleration limiter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProperAccelLimiterInput {
    /// Pilot/GNC commanded main-engine specific thrust (`m/s^2`, clamped to `[0, max_avail_accel_m_s2]`).
    pub commanded_thrust_accel_m_s2: f64,
    /// Unit main-engine thrust direction $\hat{\mathbf{e}}_T$ in body coordinates (default `+Y`).
    pub unit_thrust_axis_body: DVec3,
    /// Non-main-engine specific force $\mathbf{a}_{\text{ext}} = (\mathbf{F}_{\text{rcs}} + \mathbf{F}_{\text{aero}})/m$ in body coordinates (`m/s^2`).
    pub ext_specific_force_body_m_s2: DVec3,
    /// Maximum allowed total proper acceleration $a_{\max}$ (`m/s^2`, default `4.0G = 39.2266`).
    pub max_proper_accel_m_s2: f64,
}

impl ProperAccelLimiterInput {
    /// Constructs a limiter input for `Raketa`'s `+Y` main engine under the `4.0G` limit.
    #[must_use]
    pub const fn raketa_plus_y(
        commanded_thrust_accel_m_s2: f64,
        ext_specific_force_body_m_s2: DVec3,
    ) -> Self {
        Self {
            commanded_thrust_accel_m_s2,
            unit_thrust_axis_body: DVec3::Y,
            ext_specific_force_body_m_s2,
            max_proper_accel_m_s2: FOUR_G_PROPER_ACCEL_M_S2,
        }
    }
}

/// Output of [`limit_main_thrust_proper_accel`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProperAccelLimiterOutput {
    /// Effective main-engine specific thrust $a_{\text{eff}} \in [0, a_{\text{cmd}}]$ (`m/s^2`).
    pub effective_thrust_accel_m_s2: f64,
    /// Resulting total proper acceleration $\|a_{\text{eff}}\hat{\mathbf{e}}_T + \mathbf{a}_{\text{ext}}\|$ (`m/s^2`).
    pub resulting_proper_accel_m_s2: f64,
    /// Resulting total proper acceleration in standard gravities (`G`).
    pub resulting_proper_accel_g: f64,
    /// Projection $b = \hat{\mathbf{e}}_T \cdot \mathbf{a}_{\text{ext}}$ (`m/s^2`).
    pub axial_projection_b_m_s2: f64,
    /// Quadratic discriminant $\Delta = a_{\max}^2 - \|\mathbf{a}_{\text{ext},\perp}\|^2$ (`m^2/s^4`).
    pub discriminant_m2_s4: f64,
    /// `true` iff no $a \in [0, a_{\text{cmd}}]$ can satisfy $\|a\hat{\mathbf{e}}_T + \mathbf{a}_{\text{ext}}\| \le a_{\max}$.
    pub exceedance_infeasible: bool,
    /// Avionics alert asserted when `exceedance_infeasible == true`.
    pub active_alert: Option<AlertId>,
}

/// Evaluates the exact quadratic feasible-interval `4.0G` proper-acceleration limiter.
#[must_use]
pub fn limit_main_thrust_proper_accel(input: &ProperAccelLimiterInput) -> ProperAccelLimiterOutput {
    let a_max = input.max_proper_accel_m_s2.max(0.0);
    let a_cmd = input.commanded_thrust_accel_m_s2.clamp(0.0, a_max);
    let e_norm = libm::sqrt(input.unit_thrust_axis_body.length_squared());
    let e_t = if e_norm > 1.0e-12 {
        input.unit_thrust_axis_body / e_norm
    } else {
        DVec3::Y
    };
    let a_ext = input.ext_specific_force_body_m_s2;
    let b = e_t.dot(a_ext);
    let ext_sq = a_ext.length_squared();
    let perp_sq = (ext_sq - b * b).max(0.0);
    let disc = a_max * a_max - perp_sq;

    let (a_eff, infeasible) = if disc >= 0.0 {
        let root_span = libm::sqrt(disc);
        let lo = (0.0_f64).max(-b - root_span);
        let hi = a_cmd.min(-b + root_span);
        if lo <= hi + 1.0e-12 {
            (hi.max(0.0), false)
        } else {
            ((-b).clamp(0.0, a_cmd), true)
        }
    } else {
        ((-b).clamp(0.0, a_cmd), true)
    };

    let total_vec = e_t * a_eff + a_ext;
    let total_mag = libm::sqrt(total_vec.length_squared());
    ProperAccelLimiterOutput {
        effective_thrust_accel_m_s2: a_eff,
        resulting_proper_accel_m_s2: total_mag,
        resulting_proper_accel_g: total_mag / STANDARD_GRAVITY_G0_M_S2,
        axial_projection_b_m_s2: b,
        discriminant_m2_s4: disc,
        exceedance_infeasible: infeasible,
        active_alert: if infeasible {
            Some(ALERT_PROPER_ACCEL_EXCEEDANCE)
        } else {
            None
        },
    }
}
