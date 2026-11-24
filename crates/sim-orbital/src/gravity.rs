//! Multi-body Newtonian + spherical-harmonic gravity and accelerating-frame mechanics.
//!
//! Implements Battin's cancellation-free $f(q)$ third-body formulation paired with
//! the exact non-rotating accelerating-frame origin defect $\delta\mathbf{a}_{\text{origin},P}(t)$
//! so reconstructed barycentric trajectories remain invariant to coordinate origin $P$.

mod far_field;
mod frames;
#[cfg(test)]
mod tests;

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_bodies::{GravitySample, HarmonicGravity};

use crate::{
    constants::{ConstantsError, GravityConstantsCatalog},
    ephemeris::{DerivativeOrder, EphemerisCatalog, EphemerisError},
    hash::{ContentHash, DomainHasher},
    ids::NaifId,
    orientation::{BodyOrientationCatalog, OrientationError},
    scalar_math::sqrt,
    time::TdbEpoch,
};

pub use far_field::{HARMONIC_FAR_FIELD_ACCEL_M_S2, harmonic_reach_m};

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
    /// A body's magnetic field has no value where and when an environment load read it.
    #[error("magnetic field of {body} has no value: {source}")]
    MagneticField {
        /// The body whose field was read.
        body: NaifId,
        /// The field's reason, with the place and date.
        #[source]
        source: sim_geo::MagneticFieldError,
    },
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

/// Configured multi-body point-mass + extended spherical-harmonic force model.
#[derive(Clone, Debug, PartialEq)]
pub struct ForceModel {
    attracting_bodies: Vec<NaifId>,
    harmonic_bodies: Vec<NaifId>,
    dynamical_mode: DynamicalMode,
    max_harmonic_degree: u32,
    /// Harmonic fields weaker than this (`m/s²`) are skipped; see [`harmonic_reach_m`].
    harmonic_far_field_accel_m_s2: f64,
    /// The non-central terms of each of `harmonic_bodies`' fields up to
    /// `max_harmonic_degree`, in the same order; `None` for a body with none.
    harmonic_fields: Vec<Option<HarmonicGravity>>,
    /// Reach of each of `harmonic_bodies`' fields (`m`), in the same order.
    harmonic_reach_m: Vec<f64>,
    constants: GravityConstantsCatalog,
    hash: ContentHash,
}

impl ForceModel {
    /// Validates disjoint hierarchy and constructs a hashed [`ForceModel`] that skips
    /// harmonic fields weaker than [`HARMONIC_FAR_FIELD_ACCEL_M_S2`].
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
        let mut harmonic_fields = Vec::with_capacity(sorted_harm.len());
        for &hb in &sorted_harm {
            constants.require_mu(hb)?;
            harmonic_fields.push(non_central_field(&constants, hb, max_harmonic_degree)?);
        }

        let model = Self {
            attracting_bodies: sorted_attr,
            harmonic_bodies: sorted_harm,
            dynamical_mode,
            max_harmonic_degree,
            harmonic_far_field_accel_m_s2: 0.0,
            harmonic_fields,
            harmonic_reach_m: Vec::new(),
            constants,
            hash: ContentHash([0; 32]),
        };
        Ok(model.with_harmonic_far_field_accel_m_s2(HARMONIC_FAR_FIELD_ACCEL_M_S2))
    }

    /// This model skipping harmonic fields weaker than `accel_m_s2` (`m/s²`) beyond
    /// each body's [`harmonic_reach_m`]; zero evaluates every field everywhere.
    #[must_use]
    pub fn with_harmonic_far_field_accel_m_s2(mut self, accel_m_s2: f64) -> Self {
        self.harmonic_far_field_accel_m_s2 = accel_m_s2;
        self.harmonic_reach_m = self
            .harmonic_fields
            .iter()
            .map(|field| {
                field
                    .as_ref()
                    .map_or(f64::INFINITY, |field| harmonic_reach_m(field, accel_m_s2))
            })
            .collect();
        self.hash = compute_force_hash(&self);
        self
    }

    /// Acceleration (`m/s²`) below which a harmonic field is skipped.
    #[must_use]
    pub const fn harmonic_far_field_accel_m_s2(&self) -> f64 {
        self.harmonic_far_field_accel_m_s2
    }

    /// Distance from `body`'s centre (`m`) beyond which its harmonic field is skipped;
    /// `None` when `body` is not a harmonic body of this model.
    #[must_use]
    pub fn harmonic_reach_of_m(&self, body: NaifId) -> Option<f64> {
        let index = self.harmonic_bodies.binary_search(&body).ok()?;
        self.harmonic_reach_m.get(index).copied()
    }

    /// The non-central gravitation of the harmonic body at `index`, at `offset_icrf_m`
    /// from its centre, in its own fixed axes; `None` for a body with no such terms.
    ///
    /// A position the series has no value at (a non-finite one, or the centre) adds
    /// nothing here: the point-mass term reports that singularity.
    pub(crate) fn harmonic_sample_with_gradient(
        &self,
        index: usize,
        offset_icrf_m: DVec3,
        r_icrf_to_fixed: glam::DMat3,
        with_gradient: bool,
    ) -> Option<GravitySample> {
        let field = self.harmonic_fields.get(index)?.as_ref()?;
        field
            .sample(r_icrf_to_fixed * offset_icrf_m, with_gradient)
            .ok()
    }

    /// Whether the harmonic body at `index` has a field that reaches `offset_m`.
    pub(crate) fn harmonics_reach(&self, index: usize, offset_m: DVec3) -> bool {
        let has_field = matches!(self.harmonic_fields.get(index), Some(Some(_)));
        let reach = self.harmonic_reach_m.get(index).copied();
        has_field && reach.is_none_or(|reach| offset_m.length_squared() <= reach * reach)
    }

    /// The solar system as `constants`' body catalog describes it: the Sun and the
    /// planetary systems out to Saturn as point masses under `PrescribedEphemeris`, with
    /// each described body attracting in its own right in place of its system's
    /// barycentre. Every described body's field is summed as far as its constants
    /// state it, so the degree a descriptor gives its gravity is the degree both
    /// tiers fly.
    pub fn standard_solar_system(constants: GravityConstantsCatalog) -> Result<Self, GravityError> {
        let described: Vec<NaifId> = constants
            .body_catalog()
            .iter()
            .map(|body| body.naif)
            .collect();
        let replaced: Vec<NaifId> = described
            .iter()
            .filter_map(|body| body.parent_planetary_barycenter())
            .collect();
        let systems = [
            NaifId::SUN,
            NaifId::MERCURY_BARYCENTER,
            NaifId::VENUS_BARYCENTER,
            NaifId::EARTH_MOON_BARYCENTER,
            NaifId::MARS_BARYCENTER,
            NaifId::JUPITER_BARYCENTER,
            NaifId::SATURN_BARYCENTER,
        ];
        let attracting = systems
            .into_iter()
            .filter(|system| !replaced.contains(system))
            .chain(described.iter().copied())
            .collect();
        let stated_degree = described
            .iter()
            .filter_map(|&body| constants.get(body)?.harmonics.as_ref())
            .map(|field| field.truncation().degree)
            .max()
            .unwrap_or(0);
        Self::new(
            constants,
            attracting,
            described,
            DynamicalMode::PrescribedEphemeris,
            u32::try_from(stated_degree).unwrap_or(u32::MAX),
        )
    }

    /// The bodies this model's constants describe.
    #[must_use]
    pub fn body_catalog(&self) -> &std::sync::Arc<sim_bodies::BodyCatalog> {
        self.constants.body_catalog()
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
        Ok(acc)
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
        for (index, &hb) in self.harmonic_bodies.iter().enumerate() {
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
            if let Some(sample) = self.harmonic_sample_with_gradient(
                index,
                rel_to_hb_icrf,
                rot.r_icrf_to_fixed,
                false,
            ) {
                acc += rot.r_icrf_to_fixed.transpose() * sample.acceleration_m_s2;
            }
        }
        Ok(acc)
    }
}

/// `body`'s field without its central term and cut to `max_degree`, or `None` when
/// nothing is left of it.
fn non_central_field(
    constants: &GravityConstantsCatalog,
    body: NaifId,
    max_degree: u32,
) -> Result<Option<HarmonicGravity>, GravityError> {
    let Some(field) = constants.get(body).and_then(|spec| spec.harmonics.as_ref()) else {
        return Ok(None);
    };
    let cap = max_degree as usize;
    if cap == 0 || field.truncation().degree == 0 {
        return Ok(None);
    }
    let cut = field
        .clone()
        .capped(1, cap, cap)
        .map_err(|source| ConstantsError::InvalidHarmonics { body, source })?;
    Ok(Some(cut))
}

/// Hashes everything that shapes the dynamics, the far-field threshold included: it
/// decides which fields a trajectory far from a body is flown with, so two models
/// that differ only in it fly different trajectories and must not share a hash.
fn compute_force_hash(model: &ForceModel) -> ContentHash {
    let mut h = DomainHasher::new(b"sim-orbital:force-model:v2");
    h.write_bytes(&model.constants.content_hash().0);
    h.write_u8(match model.dynamical_mode {
        DynamicalMode::PrescribedEphemeris => 1,
        DynamicalMode::SelfConsistentNBody => 2,
    });
    h.write_u64(u64::from(model.max_harmonic_degree));
    h.write_u64(model.attracting_bodies.len() as u64);
    for id in &model.attracting_bodies {
        h.write_i32(id.0);
    }
    h.write_u64(model.harmonic_bodies.len() as u64);
    for id in &model.harmonic_bodies {
        h.write_i32(id.0);
    }
    h.write_f64_canonical(model.harmonic_far_field_accel_m_s2);
    h.finish()
}
