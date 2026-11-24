//! Analytical multi-body point-mass and spherical-harmonic gravity gradient tensor
//! $G(\mathbf{r}, t) = \partial\mathbf{a}_{\text{grav}} / \partial\mathbf{r}$ in `ICRF`.

use glam::{DMat3, DVec3};

use crate::{
    ephemeris::{DerivativeOrder, EphemerisCatalog},
    gravity::{DynamicalMode, ForceModel, GravityError, third_body_tidal_accel_icrf},
    ids::NaifId,
    orientation::BodyOrientationCatalog,
    time::TdbEpoch,
};

/// Evaluates the analytical gravity-gradient tensor
/// $G(\mathbf{r}, t) = \partial\mathbf{a}_{\text{grav}}/\partial\mathbf{r}$ (`1/s²`)
/// in `primary_body`-centered `ICRF` axes at `epoch`.
pub fn gravity_gradient_icrf(
    force_model: &ForceModel,
    primary_body: NaifId,
    pos_rel_icrf_m: DVec3,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<DMat3, GravityError> {
    force_model.relative_gravitational_gradient_icrf(
        primary_body,
        pos_rel_icrf_m,
        epoch,
        ephemeris,
        orientation,
    )
}

impl ForceModel {
    /// Evaluates the analytical gravity-gradient tensor
    /// $G(\mathbf{r}, t) = \partial\mathbf{a}_{\text{grav}}/\partial\mathbf{r}$ (`1/s²`)
    /// in `primary_body`-centered `ICRF` axes at `epoch`.
    pub fn relative_gravitational_gradient_icrf(
        &self,
        primary_body: NaifId,
        pos_rel_icrf_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<DMat3, GravityError> {
        let mut grad = if primary_body == NaifId::SSB {
            self.barycentric_point_mass_gradient(pos_rel_icrf_m, epoch, ephemeris)?
        } else {
            self.planet_centered_point_mass_gradient(
                primary_body,
                pos_rel_icrf_m,
                epoch,
                ephemeris,
            )?
        };
        let (_, harm_grad) = self.harmonics_accel_and_gradient(
            primary_body,
            pos_rel_icrf_m,
            epoch,
            ephemeris,
            orientation,
            false,
        )?;
        grad += harm_grad;
        Ok(symmetrize_dmat3(grad))
    }

    /// Evaluates both relative gravitational acceleration (`m/s²`) and its exact analytical
    /// gravity-gradient tensor (`1/s²`) in `primary_body`-centered `ICRF` axes at `epoch`.
    pub fn relative_gravitational_accel_and_gradient_icrf(
        &self,
        primary_body: NaifId,
        pos_rel_icrf_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<(DVec3, DMat3), GravityError> {
        let (mut accel, mut grad) = if primary_body == NaifId::SSB {
            self.barycentric_point_mass_accel_and_gradient(pos_rel_icrf_m, epoch, ephemeris)?
        } else {
            self.planet_centered_point_mass_accel_and_gradient(
                primary_body,
                pos_rel_icrf_m,
                epoch,
                ephemeris,
            )?
        };
        let (harm_acc, harm_grad) = self.harmonics_accel_and_gradient(
            primary_body,
            pos_rel_icrf_m,
            epoch,
            ephemeris,
            orientation,
            true,
        )?;
        accel += harm_acc;
        grad += harm_grad;
        Ok((accel, symmetrize_dmat3(grad)))
    }

    fn barycentric_point_mass_gradient(
        &self,
        pos_bary_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<DMat3, GravityError> {
        let (_, grad) =
            self.barycentric_point_mass_accel_and_gradient(pos_bary_m, epoch, ephemeris)?;
        Ok(grad)
    }

    fn barycentric_point_mass_accel_and_gradient(
        &self,
        pos_bary_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<(DVec3, DMat3), GravityError> {
        let mut acc = DVec3::ZERO;
        let mut grad = DMat3::ZERO;
        for &body in self.attracting_bodies() {
            let mu = self.constants().require_mu(body)?;
            let x_body = ephemeris
                .geometric_state_icrf(body, NaifId::SSB, epoch, DerivativeOrder::Position)?
                .pos_m;
            let d = pos_bary_m - x_body;
            let d_sq = d.length_squared();
            if !d_sq.is_finite() || d_sq <= 0.0 {
                return Err(GravityError::SingularRadius(body));
            }
            let d_norm = libm::sqrt(d_sq);
            acc -= mu * d / (d_sq * d_norm);
            grad += point_mass_gradient_icrf(d, mu, body)?;
        }
        Ok((acc, grad))
    }

    fn planet_centered_point_mass_gradient(
        &self,
        primary_body: NaifId,
        pos_rel_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<DMat3, GravityError> {
        if self
            .attracting_bodies()
            .binary_search(&primary_body)
            .is_err()
        {
            return Err(GravityError::UnsupportedPrimaryOrigin(primary_body));
        }
        let mu_p = self.constants().require_mu(primary_body)?;
        let mut grad = point_mass_gradient_icrf(pos_rel_m, mu_p, primary_body)?;
        for &body in self.attracting_bodies() {
            if body == primary_body {
                continue;
            }
            let mu_k = self.constants().require_mu(body)?;
            let s_k = ephemeris
                .geometric_state_icrf(body, primary_body, epoch, DerivativeOrder::Position)?
                .pos_m;
            grad += point_mass_gradient_icrf(pos_rel_m - s_k, mu_k, body)?;
        }
        Ok(grad)
    }

    fn planet_centered_point_mass_accel_and_gradient(
        &self,
        primary_body: NaifId,
        pos_rel_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<(DVec3, DMat3), GravityError> {
        if self
            .attracting_bodies()
            .binary_search(&primary_body)
            .is_err()
        {
            return Err(GravityError::UnsupportedPrimaryOrigin(primary_body));
        }
        let mu_p = self.constants().require_mu(primary_body)?;
        let r_sq = pos_rel_m.length_squared();
        if !r_sq.is_finite() || r_sq <= 0.0 {
            return Err(GravityError::SingularRadius(primary_body));
        }
        let mut acc = -mu_p * pos_rel_m / (r_sq * libm::sqrt(r_sq));
        let mut grad = point_mass_gradient_icrf(pos_rel_m, mu_p, primary_body)?;
        let mut sum_origin_model = DVec3::ZERO;

        for &body in self.attracting_bodies() {
            if body == primary_body {
                continue;
            }
            let mu_k = self.constants().require_mu(body)?;
            let s_k = ephemeris
                .geometric_state_icrf(body, primary_body, epoch, DerivativeOrder::Position)?
                .pos_m;
            acc += third_body_tidal_accel_icrf(pos_rel_m, s_k, mu_k, body)?;
            grad += point_mass_gradient_icrf(pos_rel_m - s_k, mu_k, body)?;
            if self.dynamical_mode() == DynamicalMode::PrescribedEphemeris {
                let s_sq = s_k.length_squared();
                if !s_sq.is_finite() || s_sq <= 0.0 {
                    return Err(GravityError::SingularRadius(body));
                }
                sum_origin_model += mu_k * s_k / (s_sq * libm::sqrt(s_sq));
            }
        }

        if self.dynamical_mode() == DynamicalMode::PrescribedEphemeris {
            let x_p = ephemeris.geometric_state_icrf(
                primary_body,
                NaifId::SSB,
                epoch,
                DerivativeOrder::Acceleration,
            )?;
            acc += sum_origin_model - x_p.acc_m_s2;
        }
        Ok((acc, grad))
    }

    fn harmonics_accel_and_gradient(
        &self,
        primary_body: NaifId,
        pos_rel_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
        need_accel: bool,
    ) -> Result<(DVec3, DMat3), GravityError> {
        let mut acc = DVec3::ZERO;
        let mut grad = DMat3::ZERO;
        for (index, &hb) in self.harmonic_bodies().iter().enumerate() {
            let rel_to_hb_icrf = if hb == primary_body {
                pos_rel_m
            } else {
                let s_hb = ephemeris
                    .geometric_state_icrf(hb, primary_body, epoch, DerivativeOrder::Position)?
                    .pos_m;
                pos_rel_m - s_hb
            };
            if !self.harmonics_reach(index, rel_to_hb_icrf) {
                continue;
            }
            let rot = orientation.body_rotation(hb, epoch)?;
            if let Some(sample) =
                self.harmonic_sample_with_gradient(index, rel_to_hb_icrf, rot.r_icrf_to_fixed, true)
            {
                let r_t = rot.r_icrf_to_fixed.transpose();
                if need_accel {
                    acc += r_t * sample.acceleration_m_s2;
                }
                if let Some(g_fixed) = sample.gradient_per_s2 {
                    grad += r_t * g_fixed * rot.r_icrf_to_fixed;
                }
            }
        }
        Ok((acc, grad))
    }
}

fn point_mass_gradient_icrf(
    d_icrf_m: DVec3,
    mu_m3_s2: f64,
    body_id: NaifId,
) -> Result<DMat3, GravityError> {
    let d_sq = d_icrf_m.length_squared();
    if !d_sq.is_finite() || d_sq <= 0.0 {
        return Err(GravityError::SingularRadius(body_id));
    }
    let d_norm = libm::sqrt(d_sq);
    let u = d_icrf_m / d_norm;
    let k = mu_m3_s2 / (d_sq * d_norm);
    let xx = k * (3.0 * u.x * u.x - 1.0);
    let yy = k * (3.0 * u.y * u.y - 1.0);
    let zz = k * (3.0 * u.z * u.z - 1.0);
    let xy = k * (3.0 * u.x * u.y);
    let xz = k * (3.0 * u.x * u.z);
    let yz = k * (3.0 * u.y * u.z);
    Ok(DMat3::from_cols(
        DVec3::new(xx, xy, xz),
        DVec3::new(xy, yy, yz),
        DVec3::new(xz, yz, zz),
    ))
}

#[inline]
fn symmetrize_dmat3(m: DMat3) -> DMat3 {
    0.5 * (m + m.transpose())
}
