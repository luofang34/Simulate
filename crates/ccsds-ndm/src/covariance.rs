//! The 6×6 position/velocity covariance, stored as its lower triangle.

use crate::block::{BlockWriter, Entry, Fields};
use crate::error::Result;
use crate::frame::RefFrame;

/// Keywords of the lower triangle, row by row.
pub(crate) const KEYWORDS: [&str; 21] = [
    "CX_X",
    "CY_X",
    "CY_Y",
    "CZ_X",
    "CZ_Y",
    "CZ_Z",
    "CX_DOT_X",
    "CX_DOT_Y",
    "CX_DOT_Z",
    "CX_DOT_X_DOT",
    "CY_DOT_X",
    "CY_DOT_Y",
    "CY_DOT_Z",
    "CY_DOT_X_DOT",
    "CY_DOT_Y_DOT",
    "CZ_DOT_X",
    "CZ_DOT_Y",
    "CZ_DOT_Z",
    "CZ_DOT_X_DOT",
    "CZ_DOT_Y_DOT",
    "CZ_DOT_Z_DOT",
];

/// Keywords of an OPM or OMM covariance block.
pub(crate) const BLOCK_KEYWORDS: [&str; 22] = [
    "COV_REF_FRAME",
    "CX_X",
    "CY_X",
    "CY_Y",
    "CZ_X",
    "CZ_Y",
    "CZ_Z",
    "CX_DOT_X",
    "CX_DOT_Y",
    "CX_DOT_Z",
    "CX_DOT_X_DOT",
    "CY_DOT_X",
    "CY_DOT_Y",
    "CY_DOT_Z",
    "CY_DOT_X_DOT",
    "CY_DOT_Y_DOT",
    "CZ_DOT_X",
    "CZ_DOT_Y",
    "CZ_DOT_Z",
    "CZ_DOT_X_DOT",
    "CZ_DOT_Y_DOT",
    "CZ_DOT_Z_DOT",
];

/// Unit of the lower-triangle term at `index`.
pub(crate) fn unit_of(index: usize) -> &'static str {
    match index {
        0..=5 => "km**2",
        6..=8 | 10..=12 | 15..=17 => "km**2/s",
        _ => "km**2/s**2",
    }
}

/// A symmetric 6×6 covariance of position (km) and velocity (km/s).
///
/// Terms are in km², km²/s and km²/s² by block; the lower triangle is kept in
/// the row order the standard writes it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Covariance6 {
    lower: [f64; 21],
}

impl Covariance6 {
    /// Builds a covariance from its lower triangle, row by row
    /// (`[1,1]`, `[2,1]`, `[2,2]`, `[3,1]`, …).
    pub const fn from_lower(lower: [f64; 21]) -> Self {
        Self { lower }
    }

    /// The lower triangle, row by row.
    pub const fn lower(&self) -> &[f64; 21] {
        &self.lower
    }

    /// The term at zero-based `row` and `column`, in either triangle.
    pub fn get(&self, row: usize, column: usize) -> Option<f64> {
        if row > 5 || column > 5 {
            return None;
        }
        let (r, c) = if row >= column {
            (row, column)
        } else {
            (column, row)
        };
        let index = r.saturating_mul(r.saturating_add(1)) / 2 + c;
        self.lower.get(index).copied()
    }

    /// The full symmetric matrix.
    pub fn to_matrix(&self) -> [[f64; 6]; 6] {
        let mut matrix = [[0.0; 6]; 6];
        for (r, row) in matrix.iter_mut().enumerate() {
            for (c, term) in row.iter_mut().enumerate() {
                *term = self.get(r, c).unwrap_or(0.0);
            }
        }
        matrix
    }
}

/// An OPM or OMM covariance block.
#[derive(Debug, Clone, PartialEq)]
pub struct CovarianceBlock {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `COV_REF_FRAME`; absent means the metadata `REF_FRAME`.
    pub ref_frame: Option<RefFrame>,
    /// The matrix.
    pub matrix: Covariance6,
}

pub(crate) fn read_matrix(fields: &mut Fields) -> Result<Covariance6> {
    let mut lower = [0.0; 21];
    for (index, (term, keyword)) in lower.iter_mut().zip(KEYWORDS).enumerate() {
        *term = fields.required(keyword, |entry: &Entry, mode| {
            entry.real_in(unit_of(index), mode)
        })?;
    }
    Ok(Covariance6::from_lower(lower))
}

pub(crate) fn write_matrix(out: &mut BlockWriter, matrix: &Covariance6) -> Result<()> {
    for (index, (term, keyword)) in matrix.lower().iter().zip(KEYWORDS).enumerate() {
        out.real_in(keyword, *term, unit_of(index))?;
    }
    Ok(())
}

impl CovarianceBlock {
    pub(crate) fn read(fields: &mut Fields) -> Result<(Option<RefFrame>, Covariance6)> {
        let ref_frame = fields.optional("COV_REF_FRAME", Entry::frame)?;
        Ok((ref_frame, read_matrix(fields)?))
    }

    pub(crate) fn write(&self, out: &mut BlockWriter) -> Result<()> {
        out.text_opt("COV_REF_FRAME", self.ref_frame.as_ref());
        write_matrix(out, &self.matrix)
    }
}

#[cfg(test)]
mod tests;
