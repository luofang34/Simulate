//! `DAF/CK` pointing kernels: segments, quaternions, angular velocity, and evaluation.

#[cfg(feature = "std")]
mod file_io;
mod quaternion;
mod segment;
#[cfg(test)]
mod tests;
mod type2;
mod type3;

use alloc::string::String;
use alloc::vec::Vec;

pub use quaternion::{matrix_to_quaternion, quaternion_to_matrix};
pub use segment::{CkData, CkSegment};
pub use type2::{CkInterval, CkType2};
pub use type3::{CkPointingInstance, CkType3};

use crate::daf::Daf;
use crate::error::Error;

const KERNEL_TYPE: &str = "CK";
const SUMMARY_DOUBLES: usize = 2;
const SUMMARY_INTEGERS: usize = 6;

/// Evaluated spacecraft/instrument attitude and optional angular velocity from a CK file.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CkPointing {
    /// Encoded SCLK timestamp at which pointing was evaluated (`clkout`).
    pub sclk: f64,
    /// NAIF reference frame ID of the attitude and angular velocity.
    pub frame: i32,
    /// Normalized SPICE scalar-first quaternion `[q0, q1, q2, q3]` representing
    /// the C-matrix `R_{ref -> inst}` (`v_inst = Q2M(q) * v_ref`).
    pub quaternion: [f64; 4],
    /// Angular velocity of `inst` relative to `frame`, expressed in `frame` (`rad/s`),
    /// when `avflag == 1`.
    pub angular_velocity: Option<[f64; 3]>,
}

impl CkPointing {
    /// 3x3 row-major C-matrix `R_{ref -> inst}` (`v_inst = C * v_ref`) from `self.quaternion`.
    #[must_use]
    pub fn c_matrix(&self) -> [[f64; 3]; 3] {
        quaternion_to_matrix(self.quaternion)
    }
}

/// A `DAF/CK` pointing kernel: ordered list of segments with internal name and comments.
#[derive(Clone, Debug, PartialEq)]
pub struct CkFile {
    internal_name: String,
    comment_lines: Vec<String>,
    spare_comment_records: usize,
    segments: Vec<CkSegment>,
}

impl CkFile {
    /// Starts an empty CK kernel (`LOCIFN` at most 60 printable ASCII characters).
    pub fn new(internal_name: &str) -> Result<Self, Error> {
        let daf = Daf::new(
            KERNEL_TYPE,
            SUMMARY_DOUBLES,
            SUMMARY_INTEGERS,
            internal_name,
        )?;
        Ok(Self {
            internal_name: daf.internal_name().into(),
            comment_lines: Vec::new(),
            spare_comment_records: 0,
            segments: Vec::new(),
        })
    }

    /// Parses a `DAF/CK` kernel of either IEEE byte order.
    pub fn parse(bytes: &[u8]) -> Result<Self, Error> {
        Self::from_daf(&Daf::parse(bytes)?)
    }

    /// Reads CK segments out of a parsed [`Daf`].
    pub fn from_daf(daf: &Daf) -> Result<Self, Error> {
        if daf.kernel_type() != KERNEL_TYPE {
            return Err(Error::WrongKernelType {
                expected: "DAF/CK",
                found: alloc::format!("DAF/{}", daf.kernel_type()),
            });
        }
        if daf.nd() != SUMMARY_DOUBLES || daf.ni() != SUMMARY_INTEGERS {
            return Err(Error::InvalidSummaryFormat {
                nd: i64::try_from(daf.nd()).unwrap_or(i64::MAX),
                ni: i64::try_from(daf.ni()).unwrap_or(i64::MAX),
            });
        }
        let mut segments = Vec::with_capacity(daf.arrays().len());
        for array in daf.arrays() {
            segments.push(CkSegment::from_array(array)?);
        }
        Ok(Self {
            internal_name: daf.internal_name().into(),
            comment_lines: daf.comment_lines().to_vec(),
            spare_comment_records: daf.spare_comment_records(),
            segments,
        })
    }

    /// Lays the CK kernel out as a [`Daf`].
    pub fn to_daf(&self) -> Result<Daf, Error> {
        let mut daf = Daf::new(
            KERNEL_TYPE,
            SUMMARY_DOUBLES,
            SUMMARY_INTEGERS,
            &self.internal_name,
        )?;
        daf.set_comment_lines(self.comment_lines.clone())?;
        daf.set_spare_comment_records(self.spare_comment_records);
        for segment in &self.segments {
            daf.push(segment.to_array()?)?;
        }
        Ok(daf)
    }

    /// Encodes the CK kernel deterministically, little-endian.
    pub fn to_bytes(&self) -> Result<Vec<u8>, Error> {
        self.to_daf()?.to_bytes()
    }

    /// Internal file name (`LOCIFN`).
    #[must_use]
    pub fn internal_name(&self) -> &str {
        &self.internal_name
    }

    /// Comment area text, one entry per line.
    #[must_use]
    pub fn comment_lines(&self) -> &[String] {
        &self.comment_lines
    }

    /// Replaces the comment area text.
    pub fn set_comment_lines(&mut self, lines: Vec<String>) -> Result<(), Error> {
        let mut daf = Daf::new(KERNEL_TYPE, SUMMARY_DOUBLES, SUMMARY_INTEGERS, "")?;
        daf.set_comment_lines(lines)?;
        self.comment_lines = daf.comment_lines().to_vec();
        Ok(())
    }

    /// Sets the number of empty comment records written after the comment text.
    pub const fn set_spare_comment_records(&mut self, records: usize) {
        self.spare_comment_records = records;
    }

    /// Segments in file order.
    #[must_use]
    pub fn segments(&self) -> &[CkSegment] {
        &self.segments
    }

    /// Appends a segment. Later segments take precedence over earlier ones on overlap.
    pub fn push(&mut self, segment: CkSegment) {
        self.segments.push(segment);
    }

    /// Evaluates attitude (and angular velocity if stored) for `instrument` at encoded
    /// `sclk` within `tolerance` ticks, searching segments in reverse file order.
    pub fn pointing_and_angular_velocity(
        &self,
        instrument: i32,
        sclk: f64,
        tolerance: f64,
    ) -> Result<CkPointing, Error> {
        for segment in self.segments.iter().rev() {
            if segment.instrument() != instrument {
                continue;
            }
            if let Some(pointing) = segment.pointing(sclk, tolerance)? {
                return Ok(pointing);
            }
        }
        Err(Error::NoCkCoverage {
            instrument,
            sclk,
            tolerance,
        })
    }

    /// Alias for [`Self::pointing_and_angular_velocity`].
    pub fn attitude_at(
        &self,
        instrument: i32,
        sclk: f64,
        tolerance: f64,
    ) -> Result<CkPointing, Error> {
        self.pointing_and_angular_velocity(instrument, sclk, tolerance)
    }
}
