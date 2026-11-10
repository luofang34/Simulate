//! Multi-body Newtonian + spherical-harmonic gravity and accelerating-frame mechanics.
//!
//! Implements Battin's cancellation-free $f(q)$ third-body formulation paired with
//! the exact non-rotating accelerating-frame origin defect $\delta\mathbf{a}_{\text{origin},P}(t)$
//! so reconstructed barycentric trajectories remain invariant to coordinate origin $P$.

mod frames;
pub mod harmonics;

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    constants::{ConstantsError, GravityConstantsCatalog},
    ephemeris::{DerivativeOrder, EphemerisCatalog, EphemerisError},
    hash::{ContentHash, DomainHasher},
    ids::NaifId,
    orientation::{BodyOrientationCatalog, OrientationError},
    scalar_math::sqrt,
    time::TdbEpoch,
};

pub use harmonics::{HarmonicFieldEval, evaluate_spherical_harmonics_fixed};

/// How primary-frame origin acceleration $\ddot{\mathbf{x}}_P(t)$ relates to the force model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DynamicalMode {
    /// Body trajectories follow an external ephemeris (such as `DE440s + mar099s`) whose
    /// dynamics differ from the reduced active set $\mathcal{B}$. Includes the exact time-only
    /// origin-acceleration defect $\delta\mathbf{a}_{\text{origin},P}(t)$ so reconstructed
    /// inertial acceleration $\ddot{\mathbf{x}}_P^{\,\text{eph}}(t) + \ddot{\mathbf{r}}$ is
    /// strictly independent of primary body $P$.
    PrescribedEphemeris,
    /// Body trajectories satisfy the exact same reduced point-mass force model, so
    /// $\delta\mathbf{a}_{\text{origin},P}(t) \equiv \mathbf{0}$ identically.
    SelfConsistentNBody,
}

/// Errors returned while constructing or evaluating a [`ForceModel`].
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum GravityError {
    /// Invalid or overlapping gravitational constants configuration.
    #[error("gravity constants error: {0}")]
    Constants(#[from] ConstantsError),
    /// Ephemeris lookup failed while resolving body positions/accelerations.
    #[error("ephemeris error: {0}")]
    Ephemeris(#[from] EphemerisError),
    /// Orientation lookup failed while rotating an extended harmonic body.
    #[error("orientation error: {0}")]
    Orientation(#[from] OrientationError),
    /// Primary coordinate origin is neither `SSB (0)` nor a member of the active attracting set.
    #[error("primary body {0} is neither SSB (0) nor in the active attracting set")]
    UnsupportedPrimaryOrigin(NaifId),
    /// Spacecraft collided with a point-mass singularity ($\|\mathbf{r} - \mathbf{s}_k\| = 0$).
    #[error("singular zero distance to attracting body {0}")]
    SingularRadius(NaifId),
}

/// Evaluated barycentric potential $U(\mathbf{x}, t)$, partial time derivative $\partial U / \partial t$,
/// and inertial gravitational acceleration $\mathbf{a}_{\text{grav,ICRF}}(\mathbf{x}, t)$.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InertialPotentialSample {
    /// Total specific gravitational potential energy $U(\mathbf{x}, t) < 0$ in $\text{J}/\text{kg}$.
    pub potential_j_kg: f64,
    /// Explicit time derivative $\frac{\partial U}{\partial t}(\mathbf{x}, t)$ at fixed inertial $\mathbf{x}$ ($\text{W}/\text{kg}$).
    pub partial_u_dt_w_kg: f64,
    /// Total inertial gravitational acceleration $-\nabla_{\mathbf{x}} U(\mathbf{x}, t)$ in `ICRF` ($\text{m}/\text{s}^2$).
    pub accel_icrf_m_s2: DVec3,
}

/// Evaluates Battin's rational third-body function
/// $f(q) = q \frac{3 + 3q + q^2}{1 + (1 + q)^{3/2}}$ for $1 + q > 0$.
#[inline]
#[must_use]
pub fn battin_f(q: f64) -> f64 {
    let one_plus_q = (1.0 + q).max(0.0);
    let denom = 1.0 + one_plus_q * sqrt(one_plus_q);
    q * (3.0 + q * (3.0 + q)) / denom
}

/// Evaluates the cancellation-free third-body differential point-mass acceleration
/// $\Delta\mathbf{a}_k(\mathbf{r}, \mathbf{s}) = -\mu_k\frac{\mathbf{r} - \mathbf{s}}{\|\mathbf{r} - \mathbf{s}\|^3} - \mu_k\frac{\mathbf{s}}{\|\mathbf{s}\|^3}$.
///
/// Uses Battin's $f(q)$ identity when $|q| \le 0.25$ (`< 5e-15` relative error in LEO solar/lunar
/// regimes) and switches to direct subtraction when the spacecraft approaches body $k$ ($q \to -1$),
/// where $\|\mathbf{d}\| \ll \|\mathbf{s}\|$ has zero cancellation.
pub fn third_body_tidal_accel_icrf(
    r_m: DVec3,
    s_body_rel_primary_m: DVec3,
    mu_m3_s2: f64,
    body_id: NaifId,
) -> Result<DVec3, GravityError> {
    let d = r_m - s_body_rel_primary_m;
    let d_sq = d.length_squared();
    let s_sq = s_body_rel_primary_m.length_squared();
    if d_sq <= 0.0 || s_sq <= 0.0 {
        return Err(GravityError::SingularRadius(body_id));
    }
    let d_norm = sqrt(d_sq);
    let inv_d3 = 1.0 / (d_sq * d_norm);
    let q = r_m.dot(r_m - 2.0 * s_body_rel_primary_m) / s_sq;
    if q.abs() <= 0.25 {
        let fq = battin_f(q);
        Ok(-mu_m3_s2 * inv_d3 * (r_m + fq * s_body_rel_primary_m))
    } else {
        let s_norm = sqrt(s_sq);
        let inv_s3 = 1.0 / (s_sq * s_norm);
        Ok(-mu_m3_s2 * (d * inv_d3 + s_body_rel_primary_m * inv_s3))
    }
}

/// Configured multi-body point-mass + extended spherical-harmonic + optional cannonball SRP force model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ForceModel {
    attracting_bodies: Vec<NaifId>,
    harmonic_bodies: Vec<NaifId>,
    dynamical_mode: DynamicalMode,
    max_harmonic_degree: u32,
    #[serde(default)]
    cannonball_srp_cr_area_over_mass_m2_kg: f64,
    constants: GravityConstantsCatalog,
    hash: ContentHash,
}

impl ForceModel {
    /// Validates disjoint hierarchy and constructs a hashed [`ForceModel`].
    pub fn new(
        constants: GravityConstantsCatalog,
        attracting_bodies: Vec<NaifId>,
        harmonic_bodies: Vec<NaifId>,
        dynamical_mode: DynamicalMode,
        max_harmonic_degree: u32,
    ) -> Result<Self, GravityError> {
        constants.validate_attracting_hierarchy(&attracting_bodies)?;
        let mut sorted_attr = attracting_bodies;
        sorted_attr.sort();
        let mut sorted_harm = harmonic_bodies;
        sorted_harm.sort();
        sorted_harm.dedup();
        for &hb in &sorted_harm {
            let _ = constants.require_mu(hb)?;
        }

        let hash = compute_force_hash(
            &constants,
            &sorted_attr,
            &sorted_harm,
            dynamical_mode,
            max_harmonic_degree,
            0.0,
        );
        Ok(Self {
            attracting_bodies: sorted_attr,
            harmonic_bodies: sorted_harm,
            dynamical_mode,
            max_harmonic_degree,
            cannonball_srp_cr_area_over_mass_m2_kg: 0.0,
            constants,
            hash,
        })
    }

    /// Returns a copy of this [`ForceModel`] with isotropic cannonball solar radiation pressure
    /// parameterized by effective reflectivity-weighted area-to-mass ratio $C_R A / m$ (`m²/kg`).
    #[must_use]
    pub fn with_cannonball_srp(mut self, cr_area_over_mass_m2_kg: f64) -> Self {
        let clamped = cr_area_over_mass_m2_kg.max(0.0);
        self.cannonball_srp_cr_area_over_mass_m2_kg = clamped;
        self.hash = compute_force_hash(
            &self.constants,
            &self.attracting_bodies,
            &self.harmonic_bodies,
            self.dynamical_mode,
            self.max_harmonic_degree,
            clamped,
        );
        self
    }

    /// Effective reflectivity-weighted area-to-mass ratio $C_R A / m$ (`m²/kg`) for cannonball SRP.
    #[must_use]
    pub const fn cannonball_srp_cr_area_over_mass_m2_kg(&self) -> f64 {
        self.cannonball_srp_cr_area_over_mass_m2_kg
    }

    /// Standard solar-system force model (`Sun`, `Mercury..Saturn` barycenters, `Earth`, `Moon`,
    /// `Mars`, with Earth/Moon/Mars spherical harmonics under `PrescribedEphemeris`).
    pub fn standard_solar_system(constants: GravityConstantsCatalog) -> Result<Self, GravityError> {
        Self::new(
            constants,
            vec![
                NaifId::SUN,
                NaifId::MERCURY_BARYCENTER,
                NaifId::VENUS_BARYCENTER,
                NaifId::EARTH,
                NaifId::MOON,
                NaifId::MARS,
                NaifId::JUPITER_BARYCENTER,
                NaifId::SATURN_BARYCENTER,
            ],
            vec![NaifId::EARTH, NaifId::MOON, NaifId::MARS],
            DynamicalMode::PrescribedEphemeris,
            8,
        )
    }

    /// Canonical SHA-256 content hash of this force model configuration and its constants.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.hash
    }

    /// Active point-mass attracting bodies $\mathcal{B}$ in canonical `NaifId` order.
    #[must_use]
    pub fn attracting_bodies(&self) -> &[NaifId] {
        &self.attracting_bodies
    }

    /// Extended harmonic bodies $\mathcal{H}$ in canonical `NaifId` order.
    #[must_use]
    pub fn harmonic_bodies(&self) -> &[NaifId] {
        &self.harmonic_bodies
    }

    /// Active origin-acceleration dynamical mode.
    #[must_use]
    pub const fn dynamical_mode(&self) -> DynamicalMode {
        self.dynamical_mode
    }

    /// Underlying gravitational constants catalog.
    #[must_use]
    pub const fn constants(&self) -> &GravityConstantsCatalog {
        &self.constants
    }

    /// Maximum spherical-harmonic degree evaluated for extended bodies.
    #[must_use]
    pub const fn max_harmonic_degree(&self) -> u32 {
        self.max_harmonic_degree
    }

    /// Evaluates relative gravitational acceleration $\ddot{\mathbf{r}}_{\text{grav}}$ in
    /// `primary_body`-centered ICRF-aligned coordinates at `epoch`.
    ///
    /// Supports both planet-centered origins ($P \in \mathcal{B}$) and `NaifId::SSB` ($P = 0$).
    pub fn relative_gravitational_accel_icrf(
        &self,
        primary_body: NaifId,
        pos_rel_icrf_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<DVec3, GravityError> {
        let mut acc = if primary_body == NaifId::SSB {
            self.barycentric_point_mass_accel(pos_rel_icrf_m, epoch, ephemeris)?
        } else {
            self.planet_centered_point_mass_accel(primary_body, pos_rel_icrf_m, epoch, ephemeris)?
        };

        acc += self.origin_independent_harmonics_accel(
            primary_body,
            pos_rel_icrf_m,
            epoch,
            ephemeris,
            orientation,
        )?;
        if self.cannonball_srp_cr_area_over_mass_m2_kg > 0.0 {
            acc += self.cannonball_srp_accel(primary_body, pos_rel_icrf_m, epoch, ephemeris)?;
        }
        Ok(acc)
    }

    fn cannonball_srp_accel(
        &self,
        primary_body: NaifId,
        pos_rel_icrf_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<DVec3, GravityError> {
        const SOLAR_PRESSURE_1AU_PA: f64 = 1_361.0 / 299_792_458.0;
        const AU_M: f64 = 149_597_870_700.0;
        let sun_rel_primary = if primary_body == NaifId::SUN {
            DVec3::ZERO
        } else {
            ephemeris
                .geometric_state_icrf(NaifId::SUN, primary_body, epoch, DerivativeOrder::Position)?
                .pos_m
        };
        let sun_to_sc = pos_rel_icrf_m - sun_rel_primary;
        let r_sq = sun_to_sc.length_squared();
        if r_sq <= 0.0 {
            return Err(GravityError::SingularRadius(NaifId::SUN));
        }
        let r_norm = sqrt(r_sq);
        let flux_pa = SOLAR_PRESSURE_1AU_PA * (AU_M * AU_M / r_sq);
        Ok((flux_pa * self.cannonball_srp_cr_area_over_mass_m2_kg / r_norm) * sun_to_sc)
    }

    fn barycentric_point_mass_accel(
        &self,
        pos_bary_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<DVec3, GravityError> {
        let mut acc = DVec3::ZERO;
        for &body in &self.attracting_bodies {
            let mu = self.constants.require_mu(body)?;
            let x_body = ephemeris
                .geometric_state_icrf(body, NaifId::SSB, epoch, DerivativeOrder::Position)?
                .pos_m;
            let d = pos_bary_m - x_body;
            let d_sq = d.length_squared();
            if d_sq <= 0.0 {
                return Err(GravityError::SingularRadius(body));
            }
            acc -= mu * d / (d_sq * sqrt(d_sq));
        }
        Ok(acc)
    }

    fn planet_centered_point_mass_accel(
        &self,
        primary_body: NaifId,
        pos_rel_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<DVec3, GravityError> {
        if self.attracting_bodies.binary_search(&primary_body).is_err() {
            return Err(GravityError::UnsupportedPrimaryOrigin(primary_body));
        }
        let mu_p = self.constants.require_mu(primary_body)?;
        let r_sq = pos_rel_m.length_squared();
        if r_sq <= 0.0 {
            return Err(GravityError::SingularRadius(primary_body));
        }
        let mut acc = -mu_p * pos_rel_m / (r_sq * sqrt(r_sq));
        let mut sum_origin_model = DVec3::ZERO;

        for &body in &self.attracting_bodies {
            if body == primary_body {
                continue;
            }
            let mu_k = self.constants.require_mu(body)?;
            let s_k = ephemeris
                .geometric_state_icrf(body, primary_body, epoch, DerivativeOrder::Position)?
                .pos_m;
            acc += third_body_tidal_accel_icrf(pos_rel_m, s_k, mu_k, body)?;
            if self.dynamical_mode == DynamicalMode::PrescribedEphemeris {
                let s_sq = s_k.length_squared();
                if s_sq <= 0.0 {
                    return Err(GravityError::SingularRadius(body));
                }
                sum_origin_model += mu_k * s_k / (s_sq * sqrt(s_sq));
            }
        }

        if self.dynamical_mode == DynamicalMode::PrescribedEphemeris {
            let x_p = ephemeris.geometric_state_icrf(
                primary_body,
                NaifId::SSB,
                epoch,
                DerivativeOrder::Acceleration,
            )?;
            let origin_defect = sum_origin_model - x_p.acc_m_s2;
            acc += origin_defect;
        }
        Ok(acc)
    }

    fn origin_independent_harmonics_accel(
        &self,
        primary_body: NaifId,
        pos_rel_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<DVec3, GravityError> {
        let mut acc = DVec3::ZERO;
        for &hb in &self.harmonic_bodies {
            let Some(spec) = self.constants.get(hb) else {
                continue;
            };
            if spec.harmonics.is_empty() || spec.equatorial_radius_m <= 0.0 {
                continue;
            }
            let rel_to_hb_icrf = if hb == primary_body {
                pos_rel_m
            } else {
                let s_hb = ephemeris
                    .geometric_state_icrf(hb, primary_body, epoch, DerivativeOrder::Position)?
                    .pos_m;
                pos_rel_m - s_hb
            };
            let rot = orientation.body_rotation(hb, epoch)?;
            let pos_fixed = rot.r_icrf_to_fixed * rel_to_hb_icrf;
            let filtered: Vec<_> = spec
                .harmonics
                .iter()
                .copied()
                .filter(|t| t.degree <= self.max_harmonic_degree)
                .collect();
            let eval = evaluate_spherical_harmonics_fixed(
                pos_fixed,
                spec.mu_m3_s2,
                spec.equatorial_radius_m,
                &filtered,
            );
            acc += rot.r_icrf_to_fixed.transpose() * eval.accel_fixed_m_s2;
        }
        Ok(acc)
    }
}

fn compute_force_hash(
    constants: &GravityConstantsCatalog,
    attracting: &[NaifId],
    harmonics: &[NaifId],
    mode: DynamicalMode,
    max_deg: u32,
    srp_cr_area_over_mass_m2_kg: f64,
) -> ContentHash {
    let mut h = DomainHasher::new(b"sim-orbital:force-model:v1");
    h.write_bytes(&constants.content_hash().0);
    h.write_u8(match mode {
        DynamicalMode::PrescribedEphemeris => 1,
        DynamicalMode::SelfConsistentNBody => 2,
    });
    h.write_u64(u64::from(max_deg));
    h.write_u64(attracting.len() as u64);
    for id in attracting {
        h.write_i32(id.0);
    }
    h.write_u64(harmonics.len() as u64);
    for id in harmonics {
        h.write_i32(id.0);
    }
    if srp_cr_area_over_mass_m2_kg > 0.0 {
        h.write_u64(srp_cr_area_over_mass_m2_kg.to_bits());
    }
    h.finish()
}
