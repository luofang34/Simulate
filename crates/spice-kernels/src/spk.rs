//! `DAF/SPK` ephemeris kernels: segments, their stored form, and evaluation.

#[cfg(feature = "std")]
mod file_io;
mod interpolation;
mod sampled;
mod segment;
#[cfg(test)]
mod tests;

use alloc::string::String;
use alloc::vec::Vec;

pub use sampled::SampledStates;
pub use segment::{SpkData, SpkSegment};

use crate::daf::Daf;
use crate::error::Error;

/// NAIF frame code of the J2000 (ICRF-aligned) inertial frame.
pub const FRAME_J2000: i32 = 1;

/// Largest type 9 window: SPICE interpolates with degree at most 27.
pub const MAX_LAGRANGE_WINDOW: usize = 28;

/// Largest type 13 window: `2 * window - 1` is at most SPICE's degree 27.
pub const MAX_HERMITE_WINDOW: usize = 14;

const KERNEL_TYPE: &str = "SPK";
/// An SPK summary: two doubles (coverage) and six integers (target, centre,
/// frame, data type, and the two array addresses).
const SUMMARY_DOUBLES: usize = 2;
const SUMMARY_INTEGERS: usize = 6;

/// An SPK kernel: an ordered list of segments with the file's name and
/// comments.
#[derive(Clone, Debug, PartialEq)]
pub struct SpkFile {
    internal_name: String,
    comment_lines: Vec<String>,
    spare_comment_records: usize,
    segments: Vec<SpkSegment>,
}

impl SpkFile {
    /// Starts an empty kernel. `internal_name` is the DAF internal file name,
    /// at most 60 printable ASCII characters.
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

    /// Parses an SPK kernel of either IEEE byte order.
    pub fn parse(bytes: &[u8]) -> Result<Self, Error> {
        Self::from_daf(&Daf::parse(bytes)?)
    }

    /// Reads the segments out of a parsed DAF.
    pub fn from_daf(daf: &Daf) -> Result<Self, Error> {
        if daf.kernel_type() != KERNEL_TYPE {
            return Err(Error::WrongKernelType {
                expected: "DAF/SPK",
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
            segments.push(SpkSegment::from_array(array)?);
        }
        Ok(Self {
            internal_name: daf.internal_name().into(),
            comment_lines: daf.comment_lines().to_vec(),
            spare_comment_records: daf.spare_comment_records(),
            segments,
        })
    }

    /// Lays the kernel out as a DAF.
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

    /// Encodes the kernel, little-endian. Equal kernels give equal bytes.
    pub fn to_bytes(&self) -> Result<Vec<u8>, Error> {
        self.to_daf()?.to_bytes()
    }

    /// Internal file name.
    #[must_use]
    pub fn internal_name(&self) -> &str {
        &self.internal_name
    }

    /// Comment area text, one entry per line.
    #[must_use]
    pub fn comment_lines(&self) -> &[String] {
        &self.comment_lines
    }

    /// Replaces the comment area text; see [`Daf::set_comment_lines`] for the
    /// characters a line may hold.
    pub fn set_comment_lines(&mut self, lines: Vec<String>) -> Result<(), Error> {
        let mut daf = Daf::new(KERNEL_TYPE, SUMMARY_DOUBLES, SUMMARY_INTEGERS, "")?;
        daf.set_comment_lines(lines)?;
        self.comment_lines = daf.comment_lines().to_vec();
        Ok(())
    }

    /// Sets the number of empty comment records written after the comment
    /// text; see [`Daf::set_spare_comment_records`].
    pub const fn set_spare_comment_records(&mut self, records: usize) {
        self.spare_comment_records = records;
    }

    /// Segments in file order.
    #[must_use]
    pub fn segments(&self) -> &[SpkSegment] {
        &self.segments
    }

    /// Appends a segment. Where segments overlap, a later one takes
    /// precedence over an earlier one, as in SPICE.
    pub fn push(&mut self, segment: SpkSegment) {
        self.segments.push(segment);
    }

    /// The segment SPICE would use for `target` relative to `center` at `et`:
    /// the last one in the file that covers the epoch.
    #[must_use]
    pub fn segment_for(&self, target: i32, center: i32, et: f64) -> Option<&SpkSegment> {
        self.segments.iter().rev().find(|segment| {
            segment.target() == target && segment.center() == center && segment.covers(et)
        })
    }

    /// State of `target` relative to `center` at `et`, in the frame of the
    /// segment that supplies it. Only a segment giving that pair directly is
    /// used: states are neither chained through intermediate bodies nor
    /// rotated between frames.
    pub fn state(&self, target: i32, center: i32, et: f64) -> Result<[f64; 6], Error> {
        self.segment_for(target, center, et)
            .ok_or(Error::NoCoverage { target, center, et })?
            .state(et)
    }
}
