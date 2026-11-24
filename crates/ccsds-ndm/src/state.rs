//! Blocks the OPM and OMM share: the Cartesian state and the spacecraft
//! parameters.

use crate::Epoch;
use crate::block::{BlockWriter, Entry, Fields};
use crate::error::Result;
use crate::unit::{Kg, Km, KmPerS, M2, Quantity, Vector3};

/// Keywords of a state vector block.
pub(crate) const STATE_KEYWORDS: &[&str] = &["EPOCH", "X", "Y", "Z", "X_DOT", "Y_DOT", "Z_DOT"];

/// Keywords of a spacecraft parameters block.
pub(crate) const SPACECRAFT_KEYWORDS: &[&str] = &[
    "MASS",
    "SOLAR_RAD_AREA",
    "SOLAR_RAD_COEFF",
    "DRAG_AREA",
    "DRAG_COEFF",
];

/// A Cartesian position and velocity at an epoch.
#[derive(Debug, Clone, PartialEq)]
pub struct StateVector {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `EPOCH`.
    pub epoch: Epoch,
    /// `X`, `Y`, `Z`.
    pub position: Vector3<Km>,
    /// `X_DOT`, `Y_DOT`, `Z_DOT`.
    pub velocity: Vector3<KmPerS>,
}

impl StateVector {
    pub(crate) fn read(comments: Vec<String>, fields: &mut Fields) -> Result<Self> {
        Ok(Self {
            comments,
            epoch: fields.required("EPOCH", Entry::epoch)?,
            position: [
                fields.required("X", Entry::quantity)?,
                fields.required("Y", Entry::quantity)?,
                fields.required("Z", Entry::quantity)?,
            ],
            velocity: [
                fields.required("X_DOT", Entry::quantity)?,
                fields.required("Y_DOT", Entry::quantity)?,
                fields.required("Z_DOT", Entry::quantity)?,
            ],
        })
    }

    pub(crate) fn write(&self, out: &mut BlockWriter) -> Result<()> {
        out.text("EPOCH", &self.epoch);
        write_vector(out, ["X", "Y", "Z"], &self.position)?;
        write_vector(out, ["X_DOT", "Y_DOT", "Z_DOT"], &self.velocity)
    }
}

pub(crate) fn write_vector<U: crate::unit::Unit>(
    out: &mut BlockWriter,
    keywords: [&str; 3],
    vector: &Vector3<U>,
) -> Result<()> {
    for (keyword, value) in keywords.into_iter().zip(vector) {
        out.quantity(keyword, *value)?;
    }
    Ok(())
}

/// Mass and the areas and coefficients of the solar pressure and drag
/// models. Every term is optional.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SpacecraftParameters {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `MASS`.
    pub mass: Option<Quantity<Kg>>,
    /// `SOLAR_RAD_AREA`.
    pub solar_rad_area: Option<Quantity<M2>>,
    /// `SOLAR_RAD_COEFF`.
    pub solar_rad_coeff: Option<f64>,
    /// `DRAG_AREA`.
    pub drag_area: Option<Quantity<M2>>,
    /// `DRAG_COEFF`.
    pub drag_coeff: Option<f64>,
}

impl SpacecraftParameters {
    pub(crate) fn read(comments: Vec<String>, fields: &mut Fields) -> Result<Self> {
        Ok(Self {
            comments,
            mass: fields.optional("MASS", Entry::quantity)?,
            solar_rad_area: fields.optional("SOLAR_RAD_AREA", Entry::quantity)?,
            solar_rad_coeff: fields.optional("SOLAR_RAD_COEFF", Entry::real)?,
            drag_area: fields.optional("DRAG_AREA", Entry::quantity)?,
            drag_coeff: fields.optional("DRAG_COEFF", Entry::real)?,
        })
    }

    pub(crate) fn write(&self, out: &mut BlockWriter) -> Result<()> {
        out.quantity_opt("MASS", self.mass)?;
        out.quantity_opt("SOLAR_RAD_AREA", self.solar_rad_area)?;
        out.real_opt("SOLAR_RAD_COEFF", self.solar_rad_coeff)?;
        out.quantity_opt("DRAG_AREA", self.drag_area)?;
        out.real_opt("DRAG_COEFF", self.drag_coeff)
    }
}
