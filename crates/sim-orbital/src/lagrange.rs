//! Three-body rotating synodic frames, collinear Lagrange point ($L_1, L_2$) locators,
//! approximate linear/empirical $L_2$ libration seeds, and single-shooting finite-horizon
//! terminal-$x$ differential correction over full-ephemeris `DOP853`.

mod holding;
pub mod targeting;

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_spacetime::SynodicError;

use crate::{
    constants::ConstantsError,
    dop853::Dop853Error,
    ephemeris::EphemerisError,
    gravity::GravityError,
    ids::NaifId,
    scalar_math::{powf, sqrt},
    time::TdbEpoch,
};

pub use holding::{HoldingBurnSearch, L2HoldingDynamics, L2HoldingTarget, l2_holding_tolerances};
pub use sim_spacetime::{CollinearLagrangePoint, SynodicFrame, solve_collinear_lagrange_gamma};
pub use targeting::{
    L2BoundedArcTargetReport, L2BoundedArcTargetSpec, L2SingleShootingFailureReason,
    L2SingleShootingStatus, target_l2_terminal_x_arc_dop853,
};

/// Errors produced by synodic frame construction or $L_2$ single-shooting targeting.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum LagrangeError {
    /// Ephemeris lookup failed while constructing the rotating frame.
    #[error("ephemeris error in synodic frame: {0}")]
    Ephemeris(#[from] EphemerisError),
    /// Gravitational constant lookup failed.
    #[error("constants error in Lagrange calculation: {0}")]
    Constants(#[from] ConstantsError),
    /// Gravity evaluation failed.
    #[error("gravity error in Lagrange calculation: {0}")]
    Gravity(#[from] GravityError),
    /// Continuous `DOP853` plant propagation failed during shooting or arc evaluation.
    #[error("DOP853 propagation error during L2 targeting: {0}")]
    Propagation(#[from] Dop853Error<GravityError>),
    /// Primary and secondary bodies are identical, coincident, or have collinear position and velocity.
    #[error(
        "degenerate synodic frame for primary {primary} and secondary {secondary}: separation_m = {separation_m}, |h| = {specific_angular_momentum_m2_s} m^2/s"
    )]
    DegenerateSynodicFrame {
        /// Primary body ID.
        primary: NaifId,
        /// Secondary body ID.
        secondary: NaifId,
        /// Instantaneous primary–secondary distance (`m`).
        separation_m: f64,
        /// Specific relative angular momentum magnitude $\|\mathbf{r}_{12} \times \mathbf{v}_{12}\|$ (`m^2/s`).
        specific_angular_momentum_m2_s: f64,
    },
    /// Invalid horizon, amplitude, or tolerance parameters passed to the single-shooting targeter.
    #[error(
        "invalid L2 targeting parameters: horizon_s = {horizon_s}, az_amplitude_m = {az_amplitude_m}, tolerance_x_m = {tolerance_x_m}"
    )]
    InvalidTargetParameters {
        /// Requested targeting horizon (`s`).
        horizon_s: f64,
        /// Requested $Z$ amplitude (`m`).
        az_amplitude_m: f64,
        /// Terminal synodic $x$ residual tolerance (`m`).
        tolerance_x_m: f64,
    },
    /// Single-shooting secant corrector did not converge to the requested terminal $x$ residual tolerance.
    #[error(
        "L2 single-shooting corrector failed after {iterations} iterations (residual_x_m = {residual_x_m}): {reason:?}"
    )]
    SingleShootingFailed {
        /// Reason the secant corrector terminated without converging.
        reason: L2SingleShootingFailureReason,
        /// Number of shooting evaluations performed.
        iterations: u32,
        /// Final terminal synodic $x(t_f) - x_{L_2}$ residual (`m`).
        residual_x_m: f64,
    },
    /// No burn within the limit turns a spacecraft's drift away from $L_2$ around.
    #[error(
        "no burn within {max_burn_m_s} m/s holds the spacecraft at {epoch:?}: it is already too far from the libration point"
    )]
    NoHoldingBurn {
        /// The epoch the burn was solved at.
        epoch: TdbEpoch,
        /// The limit searched to (`m/s`).
        max_burn_m_s: f64,
    },
    /// A holding target's span, bound, burn limit or crossing count is not positive
    /// and finite, or its deadband is negative.
    #[error(
        "holding-burn lookahead, bound, burn limits and crossing count must be positive and finite"
    )]
    InvalidHoldingTarget,
    /// A state handed to the synodic conversion is not about the secondary body.
    #[error("state is about body {found}, but the libration point is measured from {expected}")]
    WrongCentralBody {
        /// The secondary body the state must be about.
        expected: NaifId,
        /// The body it is about.
        found: NaifId,
    },
}

impl From<SynodicError> for LagrangeError {
    fn from(err: SynodicError) -> Self {
        match err {
            SynodicError::Ephemeris(e) => Self::Ephemeris(e),
            SynodicError::DegenerateSynodicFrame {
                primary,
                secondary,
                separation_m,
                specific_angular_momentum_m2_s,
            } => Self::DegenerateSynodicFrame {
                primary,
                secondary,
                separation_m,
                specific_angular_momentum_m2_s,
            },
        }
    }
}

/// Dynamical classification of a libration-point trajectory or mission segment.
///
/// Explicitly distinguishes a finite-horizon targeted arc from a true periodic CR3BP Halo orbit,
/// a multi-frequency quasi-periodic Lissajous orbit, and a long-term stationkept ephemeris mission.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum L2OrbitRegimeKind {
    /// Finite-duration trajectory arc whose initial $\dot{y}_0$ was adjusted so the terminal
    /// synodic $x(t_f)$ matches $x_{L_2}$ at a single fixed epoch $t_f = t_0 + T$.
    /// Does not imply periodicity, continuous-time invariance, or multi-revolution stability.
    FiniteHorizonTargetedArc,
    /// Closed periodic Halo orbit in the autonomous Circular Restricted Three-Body Problem (`CR3BP`).
    PeriodicHaloOrbitCr3bp,
    /// Multi-frequency quasi-periodic Lissajous libration trajectory around $L_2$.
    QuasiPeriodicLissajousArc,
    /// Long-duration full-ephemeris mission maintained via discrete stationkeeping maneuvers.
    EphemerisStationkeptMission,
}

/// Approximate linear/empirical initial libration seed state around $L_2$.
///
/// Uses the linearized collinear CR3BP in-plane frequency $\omega_p$, mode-coupling factor $k$,
/// and an empirical amplitude relation $A_x = \max(0.62 \, A_z, 2.1\times 10^8\text{ m})$ (not a full
/// 3rd-order Richardson series expansion) as an initial guess for differential correction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApproximateL2LibrationSeed {
    /// Equilibrium $L_2$ coordinate $x_{L_2} > 0$ relative to `secondary` (`m`).
    pub l2_x_rel_secondary_m: f64,
    /// Initial synodic position $(x_0, 0, z_0)$ relative to `secondary` (`m`).
    pub pos_syn_rel_secondary_m: DVec3,
    /// Initial synodic velocity $(0, \dot{y}_0, 0)$ relative to `secondary` (`m/s`).
    pub vel_syn_rel_secondary_m_s: DVec3,
    /// Approximate in-plane $X$ amplitude $A_x$ (`m`).
    pub ax_m: f64,
    /// Approximate in-plane $Y$ amplitude $A_y$ (`m`).
    pub ay_m: f64,
    /// Out-of-plane $Z$ amplitude $A_z$ (`m`).
    pub az_m: f64,
    /// Linearized libration period estimate $T_{\text{lin}} = 2\pi / (\omega_p \Omega)$ (`s`).
    pub period_s: f64,
}

/// Computes an approximate linear/empirical $L_2$ libration seed state for a specified $A_z$ amplitude (`m`).
#[must_use]
pub fn approximate_l2_libration_seed(
    frame: &SynodicFrame,
    mu_primary: f64,
    mu_secondary: f64,
    az_amplitude_m: f64,
    southern: bool,
) -> ApproximateL2LibrationSeed {
    let mu_star = mu_secondary / (mu_primary + mu_secondary).max(1.0);
    let gamma = solve_collinear_lagrange_gamma(mu_star, CollinearLagrangePoint::L2);
    let d_l2 = gamma * frame.separation_m;
    let c2 = (1.0 - mu_star) / powf(1.0 + gamma, 3.0) + mu_star / (gamma * gamma * gamma);
    let wp_sq = 0.5 * (2.0 - c2 + sqrt((9.0 * c2 - 8.0) * c2));
    let wp = sqrt(wp_sq.max(1.0));
    let k_ratio = (wp_sq + 1.0 + 2.0 * c2) / (2.0 * wp);
    let az = az_amplitude_m.abs();
    let ax = (az * 0.62).max(2.1e8);
    let ay = k_ratio * ax;
    let z_sign = if southern { -1.0 } else { 1.0 };
    let omega_n = frame.omega_icrf_rad_s.length();
    let pos_syn = DVec3::new(d_l2 - ax, 0.0, z_sign * az);
    let vel_syn = DVec3::new(0.0, wp * omega_n * ay, 0.0);
    let period_s = (std::f64::consts::TAU / (wp * omega_n)).max(1.0);
    ApproximateL2LibrationSeed {
        l2_x_rel_secondary_m: d_l2,
        pos_syn_rel_secondary_m: pos_syn,
        vel_syn_rel_secondary_m_s: vel_syn,
        ax_m: ax,
        ay_m: ay,
        az_m: az,
        period_s,
    }
}
