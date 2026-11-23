//! The truth magnetic field of a simulated world.

use glam::{DMat3, DVec3};

/// Why a magnetic field has no value at a place and date.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MagneticFieldError {
    /// The date is outside the dates the field's model covers; a model is
    /// never extrapolated past them.
    OutsideValidity {
        /// The date asked for, decimal years.
        decimal_year: f64,
        /// First date the model covers, decimal years.
        from: f64,
        /// Last date the model covers, decimal years.
        until: f64,
    },
    /// The position is non-finite, at the body's centre, or outside a
    /// survey grid that refuses to extrapolate.
    OutsideDomain {
        /// The body-fixed position asked for, metres.
        fixed_m: DVec3,
    },
}

impl core::fmt::Display for MagneticFieldError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::OutsideValidity {
                decimal_year,
                from,
                until,
            } => write!(
                f,
                "decimal year {decimal_year} is outside the magnetic model's {from}..={until}"
            ),
            Self::OutsideDomain { fixed_m } => {
                write!(f, "no magnetic field is defined at body-fixed {fixed_m} m")
            }
        }
    }
}

impl core::error::Error for MagneticFieldError {}

/// The magnetic flux density of a central body, nanotesla, at a body-fixed
/// position in metres on a date in decimal years, in body-fixed axes.
///
/// One field is the truth of one world, as [`crate::GravityField`] is for
/// gravity: every magnetometer and every magnetic torque on a vehicle read
/// the same instance. A navigator's reference model is another instance,
/// and their difference is the model error it has to live with.
pub trait MagneticField: Send + Sync + core::fmt::Debug {
    /// The field at `fixed` on `decimal_year`, body-fixed nT.
    ///
    /// # Errors
    ///
    /// [`MagneticFieldError`] outside the model's dates or domain.
    fn field_fixed_nt(&self, fixed: DVec3, decimal_year: f64) -> Result<DVec3, MagneticFieldError>;

    /// The field's gradient `∂B_i/∂x_j` at `fixed`, nT/m, or `None` for a
    /// field that cannot state one.
    ///
    /// # Errors
    ///
    /// As [`Self::field_fixed_nt`].
    fn field_gradient_fixed_nt_per_m(
        &self,
        fixed: DVec3,
        decimal_year: f64,
    ) -> Result<Option<DMat3>, MagneticFieldError> {
        self.field_fixed_nt(fixed, decimal_year).map(|_| None)
    }
}
