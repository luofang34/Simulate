//! A body's internal magnetic field from Gauss coefficients.

use glam::{DMat3, DVec3};
use sim_geo::{MagneticField, MagneticFieldError};
use world_core::{DecimalYear, ValidityWindow};
use world_potential::{
    Derivatives, FieldError, FieldSample, MagneticModel, MagneticSeries, VectorField, igrf14,
    read_cof,
};

use crate::FieldBuildError;
use crate::gravity::tensor_to_mat3;
use crate::scratch::with_workspace;

#[cfg(test)]
mod tests;

/// A body with no global magnetic field, as the Moon and Mars are: zero
/// everywhere, on every date.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NoMagneticField;

impl MagneticField for NoMagneticField {
    fn field_fixed_nt(
        &self,
        _fixed: DVec3,
        _decimal_year: f64,
    ) -> Result<DVec3, MagneticFieldError> {
        Ok(DVec3::ZERO)
    }

    fn field_gradient_fixed_nt_per_m(
        &self,
        _fixed: DVec3,
        _decimal_year: f64,
    ) -> Result<Option<DMat3>, MagneticFieldError> {
        Ok(Some(DMat3::ZERO))
    }
}

#[derive(Clone, PartialEq)]
enum Source {
    /// One model with linear secular variation over its validity.
    Model(MagneticModel<Vec<f64>>),
    /// Consecutive models, as IGRF's five-year epochs.
    Series(MagneticSeries),
}

/// A body's main magnetic field from Schmidt Gauss coefficients and their
/// secular variation, never extrapolated past the dates its model covers.
#[derive(Clone, PartialEq)]
pub struct HarmonicMagnetism {
    source: Source,
}

impl core::fmt::Debug for HarmonicMagnetism {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("HarmonicMagnetism")
            .field("degree", &self.degree())
            .field("validity_years", &self.validity_years())
            .finish()
    }
}

/// `error` as the seam's error for a sample at `fixed`.
fn seam_error(error: FieldError, fixed: DVec3) -> MagneticFieldError {
    match error {
        FieldError::OutsideValidity { at, validity } => MagneticFieldError::OutsideValidity {
            decimal_year: at.value(),
            from: validity.from.value(),
            until: validity.until.value(),
        },
        _ => MagneticFieldError::OutsideDomain { fixed_m: fixed },
    }
}

impl HarmonicMagnetism {
    const WHAT: &'static str = "magnetic";

    /// IGRF-14, the Earth's main field from 1900 to 2030.
    ///
    /// # Errors
    ///
    /// None for the embedded file; the `Result` is the reader's.
    pub fn igrf14() -> Result<Self, FieldBuildError> {
        let series = igrf14().map_err(FieldBuildError::parse("IGRF table"))?;
        Ok(Self {
            source: Source::Series(series),
        })
    }

    /// A WMM `.COF` text for a model of reference radius `radius_m`.
    ///
    /// # Errors
    ///
    /// [`FieldBuildError::Parse`] naming the line.
    pub fn from_cof(text: &str, radius_m: f64) -> Result<Self, FieldBuildError> {
        let file = read_cof(text, radius_m).map_err(FieldBuildError::parse("WMM COF"))?;
        Ok(Self {
            source: Source::Model(file.model),
        })
    }

    /// A centred dipole of reference radius `radius_m` from its Schmidt
    /// Gauss coefficients `[g10, g11, h11]`, nT, constant between the
    /// decimal years `from` and `until`.
    ///
    /// # Errors
    ///
    /// [`FieldBuildError::Model`] for a non-positive radius or a non-finite
    /// coefficient.
    pub fn dipole(
        radius_m: f64,
        gauss_nt: [f64; 3],
        from: f64,
        until: f64,
    ) -> Result<Self, FieldBuildError> {
        let bad = FieldBuildError::model(Self::WHAT);
        let validity = ValidityWindow {
            from: DecimalYear(from),
            until: DecimalYear(until),
        };
        let fixed =
            MagneticModel::dipole(radius_m, gauss_nt, DecimalYear(from), validity).map_err(&bad)?;
        let owned = |c: &world_potential::HarmonicCoefficients<[f64; 3]>| {
            world_potential::HarmonicCoefficients::new(1, c.cosine().to_vec(), c.sine().to_vec())
        };
        let main = owned(fixed.main()).map_err(&bad)?;
        let secular = owned(fixed.secular()).map_err(&bad)?;
        let model = MagneticModel::new(radius_m, DecimalYear(from), validity, main, secular)
            .map_err(bad)?;
        Ok(Self {
            source: Source::Model(model),
        })
    }

    /// The first and last decimal year the field has a value for, or `None`
    /// for an empty series.
    #[must_use]
    pub fn validity_years(&self) -> Option<(f64, f64)> {
        let window = match &self.source {
            Source::Model(model) => Some(model.validity()),
            Source::Series(series) => series.validity(),
        }?;
        Some((window.from.value(), window.until.value()))
    }

    /// The highest degree summed.
    #[must_use]
    pub fn degree(&self) -> usize {
        match &self.source {
            Source::Model(model) => model.workspace_degree(),
            Source::Series(series) => series.workspace_degree(),
        }
    }

    fn sample(
        &self,
        fixed: DVec3,
        decimal_year: f64,
        derivatives: Derivatives,
    ) -> Result<FieldSample, MagneticFieldError> {
        let at = DecimalYear(decimal_year);
        with_workspace(self.degree(), |workspace| match &self.source {
            Source::Model(model) => model.sample(workspace, fixed.to_array(), at, derivatives),
            Source::Series(series) => series.sample(workspace, fixed.to_array(), at, derivatives),
        })
        .map_err(|error| seam_error(error, fixed))
    }
}

impl MagneticField for HarmonicMagnetism {
    fn field_fixed_nt(&self, fixed: DVec3, decimal_year: f64) -> Result<DVec3, MagneticFieldError> {
        let sample = self.sample(fixed, decimal_year, Derivatives::Gradient)?;
        Ok(DVec3::from_array(sample.vector))
    }

    fn field_gradient_fixed_nt_per_m(
        &self,
        fixed: DVec3,
        decimal_year: f64,
    ) -> Result<Option<DMat3>, MagneticFieldError> {
        let sample = self.sample(fixed, decimal_year, Derivatives::Hessian)?;
        Ok(sample.gradient.map(tensor_to_mat3))
    }
}
