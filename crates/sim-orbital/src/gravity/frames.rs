//! Barycentric potential/power evaluation and LCA coordinate-origin switching
//! for [`ForceModel`].

use glam::DVec3;

use crate::{
    ephemeris::{DerivativeOrder, EphemerisCatalog, KinematicStateIcrf},
    gravity::{
        DynamicalMode, ForceModel, GravityError, InertialPotentialSample,
        evaluate_spherical_harmonics_fixed,
    },
    ids::NaifId,
    orientation::BodyOrientationCatalog,
    scalar_math::sqrt,
    time::TdbEpoch,
};

impl ForceModel {
    /// Evaluates inertial gravitational potential $U(\mathbf{x}, t)$, time derivative
    /// $\frac{\partial U}{\partial t}(\mathbf{x}, t)$, and acceleration $-\nabla U(\mathbf{x}, t)$
    /// at barycentric (`SSB`) position `pos_bary_icrf_m`.
    pub fn barycentric_potential_and_power(
        &self,
        pos_bary_icrf_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<InertialPotentialSample, GravityError> {
        let (mut u_total, mut du_dt, mut acc_total) = (0.0_f64, 0.0_f64, DVec3::ZERO);

        for &body in self.attracting_bodies() {
            let mu = self.constants().require_mu(body)?;
            let st = ephemeris.geometric_state_icrf(
                body,
                NaifId::SSB,
                epoch,
                DerivativeOrder::Velocity,
            )?;
            let d = pos_bary_icrf_m - st.pos_m;
            let d_sq = d.length_squared();
            if d_sq <= 0.0 {
                return Err(GravityError::SingularRadius(body));
            }
            let d_norm = sqrt(d_sq);
            let a_pm = -mu * d / (d_sq * d_norm);
            u_total -= mu / d_norm;
            du_dt += a_pm.dot(st.vel_m_s);
            acc_total += a_pm;
        }

        for &hb in self.harmonic_bodies() {
            let Some(spec) = self.constants().get(hb) else {
                continue;
            };
            if spec.harmonics.is_empty() || spec.equatorial_radius_m <= 0.0 {
                continue;
            }
            let st = ephemeris.geometric_state_icrf(
                hb,
                NaifId::SSB,
                epoch,
                DerivativeOrder::Velocity,
            )?;
            let rot = orientation.body_rotation(hb, epoch)?;
            let pos_fixed = rot.r_icrf_to_fixed * (pos_bary_icrf_m - st.pos_m);
            let filtered: Vec<_> = spec
                .harmonics
                .iter()
                .copied()
                .filter(|t| t.degree <= self.max_harmonic_degree())
                .collect();
            let eval = evaluate_spherical_harmonics_fixed(
                pos_fixed,
                spec.mu_m3_s2,
                spec.equatorial_radius_m,
                &filtered,
            );
            let dr_fixed_dt_neg =
                rot.omega_body_rad_s.cross(pos_fixed) + rot.r_icrf_to_fixed * st.vel_m_s;
            u_total += eval.potential_j_kg;
            du_dt += eval.accel_fixed_m_s2.dot(dr_fixed_dt_neg);
            acc_total += rot.r_icrf_to_fixed.transpose() * eval.accel_fixed_m_s2;
        }

        Ok(InertialPotentialSample {
            potential_j_kg: u_total,
            partial_u_dt_w_kg: du_dt,
            accel_icrf_m_s2: acc_total,
        })
    }

    /// Reconstructs full barycentric (`SSB / ICRF`) position, velocity, and acceleration
    /// from a `primary_body`-relative state and relative acceleration $\ddot{\mathbf{r}}$.
    pub fn reconstruct_barycentric_state(
        &self,
        primary_body: NaifId,
        pos_rel_icrf_m: DVec3,
        vel_rel_icrf_m_s: DVec3,
        acc_rel_icrf_m_s2: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<KinematicStateIcrf, GravityError> {
        if primary_body == NaifId::SSB {
            return Ok(KinematicStateIcrf {
                pos_m: pos_rel_icrf_m,
                vel_m_s: vel_rel_icrf_m_s,
                acc_m_s2: acc_rel_icrf_m_s2,
            });
        }
        let origin = ephemeris.geometric_state_icrf(
            primary_body,
            NaifId::SSB,
            epoch,
            DerivativeOrder::Acceleration,
        )?;
        Ok(KinematicStateIcrf {
            pos_m: origin.pos_m + pos_rel_icrf_m,
            vel_m_s: origin.vel_m_s + vel_rel_icrf_m_s,
            acc_m_s2: origin.acc_m_s2 + acc_rel_icrf_m_s2,
        })
    }

    /// Converts relative `(pos_m, vel_m_s)` from `old_primary` to `new_primary` at `epoch`
    /// using LCA segment cancellation so local Earth-Moon switches never pass through `1 AU` `SSB`.
    pub fn switch_primary_frame(
        &self,
        old_primary: NaifId,
        new_primary: NaifId,
        pos_rel_icrf_m: DVec3,
        vel_rel_icrf_m_s: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<(DVec3, DVec3), GravityError> {
        if old_primary == new_primary {
            return Ok((pos_rel_icrf_m, vel_rel_icrf_m_s));
        }
        let delta = ephemeris.geometric_state_icrf(
            old_primary,
            new_primary,
            epoch,
            DerivativeOrder::Velocity,
        )?;
        Ok((
            pos_rel_icrf_m + delta.pos_m,
            vel_rel_icrf_m_s + delta.vel_m_s,
        ))
    }

    /// Collects sorted interior polynomial granule knot timestamps across `[start, end]`
    /// for all ephemeris chains queried while propagating in `primary_body`.
    pub fn interior_knots_in(
        &self,
        primary_body: NaifId,
        start: TdbEpoch,
        end: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Vec<TdbEpoch> {
        let mut knots = Vec::new();
        if primary_body != NaifId::SSB
            && self.dynamical_mode() == DynamicalMode::PrescribedEphemeris
        {
            knots.extend(ephemeris.segment_knots_in(primary_body, NaifId::SSB, start, end));
        }
        for &body in self.attracting_bodies() {
            if body != primary_body {
                knots.extend(ephemeris.segment_knots_in(body, primary_body, start, end));
            }
        }
        knots.sort();
        knots.dedup();
        knots
    }
}
