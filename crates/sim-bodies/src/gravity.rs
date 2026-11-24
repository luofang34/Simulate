//! A body's gravitation from a spherical-harmonic model.

use std::sync::Arc;

use glam::{DMat3, DVec3};
use sim_geo::{BodyEllipsoid, GravityField};
use sim_spacetime::DomainHasher;
use world_potential::{
    Derivatives, FieldError, GravityModel, HarmonicCoefficients, SymmetricTensor, TideSystem,
    Truncation, Workspace, egm96_degree_36, read_gfc, read_shadr, triangular_index, triangular_len,
};

use crate::FieldBuildError;
use crate::scratch::with_workspace;

#[cfg(test)]
mod tests;

/// One fully normalised Stokes coefficient pair `(C̄nm, S̄nm)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HarmonicTerm {
    /// Degree `n`.
    pub degree: u32,
    /// Order `0 <= m <= n`.
    pub order: u32,
    /// Fully normalised cosine coefficient `C̄nm`.
    pub c_bar: f64,
    /// Fully normalised sine coefficient `S̄nm`; unused when `m = 0`.
    pub s_bar: f64,
}

impl HarmonicTerm {
    /// The unnormalised second zonal harmonic `J2 = -C20` as the normalised
    /// `C̄20 = -J2 / √5`.
    #[must_use]
    pub fn from_unnormalized_j2(j2: f64) -> Self {
        Self {
            degree: 2,
            order: 0,
            c_bar: -j2 / 5.0_f64.sqrt(),
            s_bar: 0.0,
        }
    }
}

/// Gravitation at one body-fixed point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravitySample {
    /// The summed potential `U`, m²/s², positive (the geodesy sign), so the
    /// attraction is `+∇U`.
    pub potential_m2_s2: f64,
    /// The attraction, body-fixed m/s².
    pub acceleration_m_s2: DVec3,
    /// The gravity-gradient tensor `∂g_i/∂x_j`, 1/s², when asked for.
    pub gradient_per_s2: Option<DMat3>,
}

/// A body's gravitation: `GM`, a reference radius and fully normalised
/// coefficients, summed over a chosen range of degrees and orders.
///
/// The coefficients are shared, so cloning a model or cutting it to a lower
/// degree copies nothing; evaluation owns its scratch memory and is the same
/// bits on every thread.
#[derive(Clone, PartialEq)]
pub struct HarmonicGravity {
    model: GravityModel<Arc<[f64]>>,
}

impl core::fmt::Debug for HarmonicGravity {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("HarmonicGravity")
            .field("gm_m3_s2", &self.model.gm_m3ps2())
            .field("radius_m", &self.model.radius_m())
            .field("stored_degree", &self.model.coefficients().degree())
            .field("truncation", &self.model.truncation())
            .finish()
    }
}

pub(crate) fn tensor_to_mat3(tensor: SymmetricTensor) -> DMat3 {
    let [x, y, z] = tensor.to_matrix();
    // Symmetric, so rows and columns are the same thing.
    DMat3::from_cols(
        DVec3::from_array(x),
        DVec3::from_array(y),
        DVec3::from_array(z),
    )
}

impl HarmonicGravity {
    const WHAT: &'static str = "gravity";

    /// The field of `gm_m3_s2` and `radius_m` with the dense coefficient
    /// triangles `cosine` and `sine` up to `degree`, summed in full.
    ///
    /// # Errors
    ///
    /// [`FieldBuildError::Model`] for non-positive constants or triangles
    /// of the wrong length.
    pub fn from_triangles(
        gm_m3_s2: f64,
        radius_m: f64,
        degree: usize,
        cosine: Vec<f64>,
        sine: Vec<f64>,
        tide_system: TideSystem,
    ) -> Result<Self, FieldBuildError> {
        let bad = FieldBuildError::model(Self::WHAT);
        let coefficients =
            HarmonicCoefficients::new(degree, Arc::from(cosine), Arc::from(sine)).map_err(&bad)?;
        let model =
            GravityModel::new(gm_m3_s2, radius_m, coefficients, tide_system).map_err(bad)?;
        Ok(Self { model })
    }

    /// The central term plus `terms`, which need not be sorted or complete.
    ///
    /// # Errors
    ///
    /// As [`Self::from_triangles`], and for a term whose order exceeds its
    /// degree.
    pub fn from_terms(
        gm_m3_s2: f64,
        radius_m: f64,
        terms: &[HarmonicTerm],
        tide_system: TideSystem,
    ) -> Result<Self, FieldBuildError> {
        let degree = terms.iter().map(|t| t.degree as usize).max().unwrap_or(0);
        let mut cosine = vec![0.0; triangular_len(degree)];
        let mut sine = vec![0.0; triangular_len(degree)];
        if let Some(central) = cosine.first_mut() {
            *central = 1.0;
        }
        for term in terms {
            let (n, m) = (term.degree as usize, term.order as usize);
            let index = triangular_index(n, m);
            let (Some(c), Some(s), true) = (cosine.get_mut(index), sine.get_mut(index), m <= n)
            else {
                return Err(FieldBuildError::model(Self::WHAT)(
                    FieldError::InvalidTruncation,
                ));
            };
            *c = term.c_bar;
            *s = term.s_bar;
        }
        Self::from_triangles(gm_m3_s2, radius_m, degree, cosine, sine, tide_system)
    }

    /// A point mass, or any spherically symmetric body.
    ///
    /// # Errors
    ///
    /// As [`Self::from_triangles`].
    pub fn point_mass(gm_m3_s2: f64, radius_m: f64) -> Result<Self, FieldBuildError> {
        Self::from_terms(gm_m3_s2, radius_m, &[], TideSystem::Unstated)
    }

    /// The central term and the oblateness `j2` (unnormalised).
    ///
    /// # Errors
    ///
    /// As [`Self::from_triangles`].
    pub fn oblate(gm_m3_s2: f64, radius_m: f64, j2: f64) -> Result<Self, FieldBuildError> {
        let terms = [HarmonicTerm::from_unnormalized_j2(j2)];
        Self::from_terms(gm_m3_s2, radius_m, &terms, TideSystem::Unstated)
    }

    /// The reference field of `body`: the point mass and `J2` of
    /// [`BodyEllipsoid::gravity_fixed`], which the navigator also models.
    /// It adds the gradient tensor the closed form does not state.
    ///
    /// # Errors
    ///
    /// As [`Self::from_triangles`].
    pub fn reference(body: &BodyEllipsoid) -> Result<Self, FieldBuildError> {
        Self::oblate(body.mu_m3_s2, body.semi_major_m, body.j2)
    }

    /// A model `world-potential` read or built, with its truncation.
    ///
    /// # Errors
    ///
    /// As [`Self::from_triangles`].
    pub fn from_model(model: &GravityModel<Vec<f64>>) -> Result<Self, FieldBuildError> {
        let coefficients = model.coefficients();
        let full = Self::from_triangles(
            model.gm_m3ps2(),
            model.radius_m(),
            coefficients.degree(),
            coefficients.cosine().to_vec(),
            coefficients.sine().to_vec(),
            model.tide_system(),
        )?;
        let mut cut = full.truncated(model.truncation())?;
        if let Some(provenance) = model.provenance() {
            cut.model = cut.model.with_provenance(provenance);
        }
        Ok(cut)
    }

    /// The Earth's EGM96 to degree and order 36, tide-free, with the
    /// model's own `GM` and radius.
    ///
    /// # Errors
    ///
    /// None for the embedded file; the `Result` is the reader's.
    pub fn egm96_degree_36() -> Result<Self, FieldBuildError> {
        let model = egm96_degree_36().map_err(FieldBuildError::parse("ICGEM gfc"))?;
        Self::from_model(&model)
    }

    /// An ICGEM `.gfc` text, keeping degrees up to `max_degree` when given.
    ///
    /// # Errors
    ///
    /// [`FieldBuildError::Parse`] naming the line.
    pub fn from_gfc(text: &str, max_degree: Option<usize>) -> Result<Self, FieldBuildError> {
        let file = read_gfc(text, max_degree).map_err(FieldBuildError::parse("ICGEM gfc"))?;
        Self::from_model(&file.model)
    }

    /// A PDS SHADR table (lunar and planetary fields), keeping degrees up
    /// to `max_degree` when given.
    ///
    /// # Errors
    ///
    /// [`FieldBuildError::Parse`] naming the line.
    pub fn from_shadr(text: &str, max_degree: Option<usize>) -> Result<Self, FieldBuildError> {
        let file = read_shadr(text, max_degree).map_err(FieldBuildError::parse("PDS SHADR"))?;
        Self::from_model(&file.model)
    }

    /// This field summed over `truncation` instead.
    ///
    /// # Errors
    ///
    /// [`FieldBuildError::Model`] for a truncation above the stored degree
    /// or with its order or minimum degree above its degree.
    pub fn truncated(self, truncation: Truncation) -> Result<Self, FieldBuildError> {
        let model = self
            .model
            .with_truncation(truncation)
            .map_err(FieldBuildError::model(Self::WHAT))?;
        Ok(Self { model })
    }

    /// This field summed to at most `degree` and `order`, from `min_degree`
    /// up: `min_degree = 1` leaves out the central term for a propagator
    /// that carries its own.
    ///
    /// # Errors
    ///
    /// As [`Self::truncated`], when `min_degree` exceeds the capped degree.
    pub fn capped(
        self,
        min_degree: usize,
        degree: usize,
        order: usize,
    ) -> Result<Self, FieldBuildError> {
        let now = self.model.truncation();
        let degree = degree.min(now.degree);
        let order = order.min(now.order).min(degree);
        self.truncated(Truncation {
            min_degree,
            degree,
            order,
        })
    }

    /// `GM` the coefficients are stated for, m³/s².
    #[must_use]
    pub fn gm_m3_s2(&self) -> f64 {
        self.model.gm_m3ps2()
    }

    /// Reference radius the coefficients are stated for, m.
    #[must_use]
    pub fn radius_m(&self) -> f64 {
        self.model.radius_m()
    }

    /// The terms summed.
    #[must_use]
    pub fn truncation(&self) -> Truncation {
        self.model.truncation()
    }

    /// The permanent-tide convention of `C̄20`.
    #[must_use]
    pub fn tide_system(&self) -> TideSystem {
        self.model.tide_system()
    }

    /// `(C̄nm, S̄nm)` as stored; zero outside the stored triangle.
    #[must_use]
    pub fn coefficient(&self, degree: usize, order: usize) -> (f64, f64) {
        use world_potential::CoefficientSource;
        self.model.coefficients().coefficient(degree, order)
    }

    /// Writes everything that shapes an acceleration: the constants, the
    /// range of terms summed, and every coefficient in that range.
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        let truncation = self.truncation();
        h.write_f64_canonical(self.gm_m3_s2());
        h.write_f64_canonical(self.radius_m());
        h.write_u64(truncation.min_degree as u64);
        h.write_u64(truncation.degree as u64);
        h.write_u64(truncation.order as u64);
        for degree in 0..=truncation.degree {
            for order in 0..=degree.min(truncation.order) {
                let (c_bar, s_bar) = self.coefficient(degree, order);
                h.write_f64_canonical(c_bar);
                h.write_f64_canonical(s_bar);
            }
        }
    }

    /// Gravitation at `fixed`, with the gradient tensor when
    /// `with_gradient`. The scratch memory is this call's own.
    ///
    /// # Errors
    ///
    /// [`FieldError::InvalidPosition`] for a non-finite position, the
    /// centre, or a point so deep inside the reference sphere that the
    /// series overflows.
    pub fn sample(&self, fixed: DVec3, with_gradient: bool) -> Result<GravitySample, FieldError> {
        with_workspace(self.model.truncation().degree, |workspace| {
            self.sample_in(workspace, fixed, with_gradient)
        })
    }

    /// As [`Self::sample`], in a workspace the caller keeps across
    /// evaluations.
    ///
    /// # Errors
    ///
    /// As [`Self::sample`], and for a workspace below this field's degree.
    pub fn sample_in(
        &self,
        workspace: &mut Workspace<'_>,
        fixed: DVec3,
        with_gradient: bool,
    ) -> Result<GravitySample, FieldError> {
        let derivatives = match with_gradient {
            true => Derivatives::Hessian,
            false => Derivatives::Gradient,
        };
        let g = self
            .model
            .gravitation(workspace, fixed.to_array(), derivatives)?;
        Ok(GravitySample {
            potential_m2_s2: g.potential_m2ps2,
            acceleration_m_s2: DVec3::from_array(g.acceleration_mps2),
            gradient_per_s2: g.gradient_per_s2.map(tensor_to_mat3),
        })
    }
}

impl GravityField for HarmonicGravity {
    /// Zero where the series has no value (the body's centre), as
    /// [`BodyEllipsoid::gravity_fixed`] is there.
    fn attraction_fixed(&self, fixed: DVec3) -> DVec3 {
        self.sample(fixed, false)
            .map_or(DVec3::ZERO, |sample| sample.acceleration_m_s2)
    }

    fn attraction_gradient_fixed(&self, fixed: DVec3) -> Option<DMat3> {
        self.sample(fixed, true).ok()?.gradient_per_s2
    }
}
