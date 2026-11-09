//! Collinear Lagrange equilibrium solver and instantaneous two-body rotating synodic frame (`SynodicFrame`).

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    catalogs::{DerivativeOrder, EphemerisCatalog, EphemerisError},
    ids::NaifId,
    scalar_math::{powf, sqrt},
    time::TdbEpoch,
};

/// Collinear Lagrange equilibrium points along the primary–secondary line.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CollinearLagrangePoint {
    /// Interior point $L_1$ between primary $P_1$ and secondary $P_2$.
    L1,
    /// Exterior point $L_2$ beyond secondary $P_2$ on the anti-primary axis (e.g. JWST Sun–Earth $L_2$).
    L2,
}

/// Solves Euler's exact quintic equation for the dimensionless distance ratio
/// $\gamma = r_{L_i} / d_{12} > 0$ from secondary $P_2$ given $\mu^* = \mu_2 / (\mu_1 + \mu_2)$.
#[must_use]
pub fn solve_collinear_lagrange_gamma(mu_star: f64, point: CollinearLagrangePoint) -> f64 {
    let mu = mu_star.clamp(1.0e-15, 0.5);
    let mut g = powf(mu / 3.0, 1.0 / 3.0);
    let mut iter = 0_u32;
    while iter < 30 {
        let g2 = g * g;
        let g3 = g2 * g;
        let g4 = g3 * g;
        let g5 = g4 * g;
        let (f, df) = match point {
            CollinearLagrangePoint::L1 => (
                g5 - (3.0 - mu) * g4 + (3.0 - 2.0 * mu) * g3 - mu * g2 + 2.0 * mu * g - mu,
                5.0 * g4 - 4.0 * (3.0 - mu) * g3 + 3.0 * (3.0 - 2.0 * mu) * g2 - 2.0 * mu * g
                    + 2.0 * mu,
            ),
            CollinearLagrangePoint::L2 => (
                g5 + (3.0 - mu) * g4 + (3.0 - 2.0 * mu) * g3 - mu * g2 - 2.0 * mu * g - mu,
                5.0 * g4 + 4.0 * (3.0 - mu) * g3 + 3.0 * (3.0 - 2.0 * mu) * g2
                    - 2.0 * mu * g
                    - 2.0 * mu,
            ),
        };
        if df.abs() < 1.0e-20 {
            break;
        }
        let step = f / df;
        g -= step;
        if step.abs() <= 1.0e-15 * g.abs() {
            break;
        }
        iter = iter.wrapping_add(1);
    }
    g
}

/// Errors produced when constructing a [`SynodicFrame`].
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SynodicError {
    /// Ephemeris lookup failed while constructing the rotating frame.
    #[error("ephemeris error in synodic frame: {0}")]
    Ephemeris(#[from] EphemerisError),
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
}

/// Instantaneous two-body rotating synodic coordinate frame centered at `secondary` ($P_2$).
///
/// Axes convention:
/// - $+\hat{\mathbf{x}}_{\text{syn}}$: unit vector from `primary` ($P_1$) to `secondary` ($P_2$),
/// - $+\hat{\mathbf{z}}_{\text{syn}}$: instantaneous orbital angular momentum direction $\mathbf{h} / \|\mathbf{h}\|$ where $\mathbf{h} = \mathbf{r}_{12} \times \mathbf{v}_{12}$,
/// - $+\hat{\mathbf{y}}_{\text{syn}} = \hat{\mathbf{z}}_{\text{syn}} \times \hat{\mathbf{x}}_{\text{syn}}$.
///
/// The instantaneous angular velocity of the rotating basis in `ICRF` is the full Darboux vector:
/// $$\boldsymbol{\omega}_{\text{syn}} = \frac{\mathbf{h}}{r_{12}^2} + \frac{r_{12} (\mathbf{a}_{12} \cdot \hat{\mathbf{z}}_{\text{syn}})}{\|\mathbf{h}\|} \hat{\mathbf{x}}_{\text{syn}}$$
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SynodicFrame {
    /// Primary body (`NaifId::SUN` for Sun–Earth $L_2$).
    pub primary: NaifId,
    /// Secondary body (`NaifId::EARTH` or `NaifId::EARTH_MOON_BARYCENTER`).
    pub secondary: NaifId,
    /// Epoch of the instantaneous frame.
    pub epoch: TdbEpoch,
    /// Distance $d_{12} = \|\mathbf{x}_2 - \mathbf{x}_1\|$ (`m`).
    pub separation_m: f64,
    /// Radial rate $\dot{d}_{12} = \mathbf{r}_{12} \cdot \mathbf{v}_{12} / d_{12}$ (`m/s`).
    pub separation_rate_m_s: f64,
    /// Rotation matrix mapping `ICRF` vectors into rotating synodic axes.
    pub r_icrf_to_synodic: DMat3,
    /// Full instantaneous frame angular velocity $\boldsymbol{\omega}_{\text{syn}}$ in `ICRF` (`rad/s`).
    pub omega_icrf_rad_s: DVec3,
}

impl SynodicFrame {
    /// Constructs the instantaneous rotating synodic frame at `epoch` from `ephemeris`.
    pub fn from_ephemeris(
        primary: NaifId,
        secondary: NaifId,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<Self, SynodicError> {
        if primary == secondary {
            return Err(SynodicError::DegenerateSynodicFrame {
                primary,
                secondary,
                separation_m: 0.0,
                specific_angular_momentum_m2_s: 0.0,
            });
        }
        let rel = ephemeris.geometric_state_icrf(
            secondary,
            primary,
            epoch,
            DerivativeOrder::Acceleration,
        )?;
        let r_mag = sqrt(rel.pos_m.length_squared());
        let h_vec = rel.pos_m.cross(rel.vel_m_s);
        let h_mag = sqrt(h_vec.length_squared());
        if !r_mag.is_finite() || !h_mag.is_finite() || r_mag <= 1.0 || h_mag <= 1.0 {
            return Err(SynodicError::DegenerateSynodicFrame {
                primary,
                secondary,
                separation_m: r_mag,
                specific_angular_momentum_m2_s: h_mag,
            });
        }
        let x_hat = rel.pos_m / r_mag;
        let z_hat = h_vec / h_mag;
        let y_hat = z_hat.cross(x_hat);
        let r_syn_to_icrf = DMat3::from_cols(x_hat, y_hat, z_hat);
        let accel_icrf = rel.acc_m_s2;
        let omega_x = r_mag * accel_icrf.dot(z_hat) / h_mag;
        let omega_icrf = h_vec / (r_mag * r_mag) + omega_x * x_hat;
        Ok(Self {
            primary,
            secondary,
            epoch,
            separation_m: r_mag,
            separation_rate_m_s: rel.pos_m.dot(rel.vel_m_s) / r_mag,
            r_icrf_to_synodic: r_syn_to_icrf.transpose(),
            omega_icrf_rad_s: omega_icrf,
        })
    }

    /// Rotates a pure displacement/position vector from `secondary`-centered `ICRF` to synodic axes.
    #[must_use]
    pub fn transform_position_icrf_to_synodic(&self, pos_rel_icrf_m: DVec3) -> DVec3 {
        self.r_icrf_to_synodic * pos_rel_icrf_m
    }

    /// Rotates a pure displacement/position vector from synodic axes to `secondary`-centered `ICRF`.
    #[must_use]
    pub fn transform_position_synodic_to_icrf(&self, pos_syn_m: DVec3) -> DVec3 {
        self.r_icrf_to_synodic.transpose() * pos_syn_m
    }

    /// Converts a state $(\mathbf{r}_{\text{rel},2}, \mathbf{v}_{\text{rel},2})$ in `secondary`-centered
    /// `ICRF` into dimensional rotating synodic coordinates $(\mathbf{r}_{\text{syn}}, \mathbf{v}_{\text{syn}})$.
    #[must_use]
    pub fn icrf_rel_secondary_to_synodic(
        &self,
        pos_rel_icrf_m: DVec3,
        vel_rel_icrf_m_s: DVec3,
    ) -> (DVec3, DVec3) {
        let pos_syn = self.r_icrf_to_synodic * pos_rel_icrf_m;
        let vel_rot_icrf = vel_rel_icrf_m_s - self.omega_icrf_rad_s.cross(pos_rel_icrf_m);
        let vel_syn = self.r_icrf_to_synodic * vel_rot_icrf;
        (pos_syn, vel_syn)
    }

    /// Converts dimensional rotating synodic coordinates $(\mathbf{r}_{\text{syn}}, \mathbf{v}_{\text{syn}})$
    /// relative to `secondary` into `secondary`-centered `ICRF` $(\mathbf{r}_{\text{rel},2}, \mathbf{v}_{\text{rel},2})$.
    #[must_use]
    pub fn synodic_to_icrf_rel_secondary(
        &self,
        pos_syn_m: DVec3,
        vel_syn_m_s: DVec3,
    ) -> (DVec3, DVec3) {
        let r_syn_to_icrf = self.r_icrf_to_synodic.transpose();
        let pos_icrf = r_syn_to_icrf * pos_syn_m;
        let vel_icrf = r_syn_to_icrf * vel_syn_m_s + self.omega_icrf_rad_s.cross(pos_icrf);
        (pos_icrf, vel_icrf)
    }

    /// Returns the position of $L_1$ or $L_2$ in `secondary`-centered synodic coordinates (`m`).
    #[must_use]
    pub fn lagrange_point_synodic_m(
        &self,
        mu_primary: f64,
        mu_secondary: f64,
        point: CollinearLagrangePoint,
    ) -> DVec3 {
        let mu_star = mu_secondary / (mu_primary + mu_secondary).max(1.0);
        let gamma = solve_collinear_lagrange_gamma(mu_star, point);
        let sign = match point {
            CollinearLagrangePoint::L1 => -1.0,
            CollinearLagrangePoint::L2 => 1.0,
        };
        DVec3::new(sign * gamma * self.separation_m, 0.0, 0.0)
    }

    /// Evaluates the instantaneous CR3BP Jacobi quantity $C_J = 2\Omega_{\text{eff}} - v_{\text{syn}}^2$ (`m^2/s^2`).
    #[must_use]
    pub fn cr3bp_jacobi_diagnostic(
        &self,
        mu_primary: f64,
        mu_secondary: f64,
        pos_syn_m: DVec3,
        vel_syn_m_s: DVec3,
    ) -> f64 {
        let r1_vec = pos_syn_m + DVec3::new(self.separation_m, 0.0, 0.0);
        let r1 = r1_vec.length().max(1.0);
        let r2 = pos_syn_m.length().max(1.0);
        let mu_tot = (mu_primary + mu_secondary).max(1.0);
        let x_bary = pos_syn_m.x + (mu_primary / mu_tot) * self.separation_m;
        let y_bary = pos_syn_m.y;
        let omega_z = (self.r_icrf_to_synodic * self.omega_icrf_rad_s).z;
        let two_u = (omega_z * omega_z) * (x_bary * x_bary + y_bary * y_bary)
            + 2.0 * (mu_primary / r1 + mu_secondary / r2);
        two_u - vel_syn_m_s.length_squared()
    }

    /// Alias for [`Self::cr3bp_jacobi_diagnostic`].
    #[must_use]
    pub fn jacobi_constant(
        &self,
        mu_primary: f64,
        mu_secondary: f64,
        pos_syn_m: DVec3,
        vel_syn_m_s: DVec3,
    ) -> f64 {
        self.cr3bp_jacobi_diagnostic(mu_primary, mu_secondary, pos_syn_m, vel_syn_m_s)
    }
}
