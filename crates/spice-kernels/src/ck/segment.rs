//! One CK segment: its descriptor, its pointing data, and DAF array conversion.

use alloc::string::String;
use alloc::vec::Vec;

use super::CkPointing;
use super::type2::CkType2;
use super::type3::CkType3;
use crate::daf::{self, DafArray};
use crate::error::Error;

/// Characters a CK segment identifier holds.
const NAME_CHARS: usize = 40;

const TYPE_CONSTANT_RATE: i32 = 2;
const TYPE_LINEAR_SLERP: i32 = 3;

/// What a CK segment stores.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum CkData {
    /// CK type 2: piecewise-constant angular velocity intervals (`avflag = 1`).
    Type2(CkType2),
    /// CK type 3: discrete pointing instances with linear SCLK interval SLERP.
    Type3(CkType3),
    /// A CK segment of a type this crate does not model, preserved verbatim.
    Raw {
        /// CK data type code.
        data_type: i32,
        /// Angular velocity presence flag (`ICD[3]`).
        avflag: i32,
        /// The segment's stored words.
        words: Vec<f64>,
    },
}

impl CkData {
    /// CK data type code.
    #[must_use]
    pub const fn data_type(&self) -> i32 {
        match self {
            Self::Type2(_) => TYPE_CONSTANT_RATE,
            Self::Type3(_) => TYPE_LINEAR_SLERP,
            Self::Raw { data_type, .. } => *data_type,
        }
    }

    /// Descriptor angular velocity flag (`ICD[3]`: `1` if angular velocity is present).
    #[must_use]
    pub const fn avflag(&self) -> i32 {
        match self {
            Self::Type2(_) => 1,
            Self::Type3(t3) => {
                if t3.has_angular_velocity() {
                    1
                } else {
                    0
                }
            }
            Self::Raw { avflag, .. } => *avflag,
        }
    }
}

/// A CK segment: attitude of one instrument/structure relative to a base frame.
#[derive(Clone, Debug, PartialEq)]
pub struct CkSegment {
    name: String,
    instrument: i32,
    frame: i32,
    coverage: [f64; 2],
    data: CkData,
}

impl CkSegment {
    /// Builds a CK type 2 segment covering its intervals' full SCLK span.
    pub fn type2(name: &str, instrument: i32, frame: i32, data: CkType2) -> Result<Self, Error> {
        daf::check_text("segment name", name, NAME_CHARS)?;
        let coverage = data.span();
        Self {
            name: name.into(),
            instrument,
            frame,
            coverage,
            data: CkData::Type2(data),
        }
        .with_coverage(coverage)
    }

    /// Builds a CK type 3 segment covering its instances' full SCLK span.
    pub fn type3(name: &str, instrument: i32, frame: i32, data: CkType3) -> Result<Self, Error> {
        daf::check_text("segment name", name, NAME_CHARS)?;
        let coverage = data.span();
        Self {
            name: name.into(),
            instrument,
            frame,
            coverage,
            data: CkData::Type3(data),
        }
        .with_coverage(coverage)
    }

    /// Reconstructs a CK segment from descriptor fields and raw DAF array words.
    pub fn from_stored(
        name: &str,
        instrument: i32,
        frame: i32,
        coverage: [f64; 2],
        data_type: i32,
        avflag: i32,
        words: Vec<f64>,
    ) -> Result<Self, Error> {
        daf::check_text("segment name", name, NAME_CHARS)?;
        let malformed = |reason| Error::MalformedCkSegment {
            name: name.into(),
            data_type,
            reason,
        };
        let data = match data_type {
            TYPE_CONSTANT_RATE => {
                if avflag != 1 {
                    return Err(malformed("CK type 2 requires avflag = 1"));
                }
                CkData::Type2(CkType2::from_words(&words).map_err(malformed)??)
            }
            TYPE_LINEAR_SLERP => {
                let has_av = match avflag {
                    0 => false,
                    1 => true,
                    _ => return Err(malformed("CK type 3 avflag must be 0 or 1")),
                };
                CkData::Type3(CkType3::from_words(&words, has_av).map_err(malformed)??)
            }
            _ if words.is_empty() => return Err(malformed("no words")),
            _ => CkData::Raw {
                data_type,
                avflag,
                words,
            },
        };
        let segment = Self {
            name: name.into(),
            instrument,
            frame,
            coverage,
            data,
        };
        match &segment.data {
            CkData::Type2(t2) => {
                let [data_start, data_end] = t2.span();
                let [start, end] = coverage;
                if !(start.is_finite() && end.is_finite() && start < end) {
                    return Err(Error::InvalidCoverage {
                        start,
                        end,
                        data_start,
                        data_end,
                    });
                }
                Ok(segment)
            }
            CkData::Type3(t3) => {
                let [data_start, data_end] = t3.span();
                let [start, end] = coverage;
                if !(start.is_finite() && end.is_finite() && start <= end) {
                    return Err(Error::InvalidCoverage {
                        start,
                        end,
                        data_start,
                        data_end,
                    });
                }
                Ok(segment)
            }
            CkData::Raw { .. } => Ok(segment),
        }
    }

    /// Narrows or restates the encoded SCLK coverage interval `[begtim, endtim]`.
    pub fn with_coverage(mut self, coverage: [f64; 2]) -> Result<Self, Error> {
        let [start, end] = coverage;
        match &self.data {
            CkData::Type2(t2) => {
                let [data_start, data_end] = t2.span();
                if !(start < end && data_start <= start && end <= data_end) {
                    return Err(Error::InvalidCoverage {
                        start,
                        end,
                        data_start,
                        data_end,
                    });
                }
            }
            CkData::Type3(t3) => {
                let [data_start, data_end] = t3.span();
                if !(start <= end && data_start <= start && end <= data_end) {
                    return Err(Error::InvalidCoverage {
                        start,
                        end,
                        data_start,
                        data_end,
                    });
                }
            }
            CkData::Raw { .. } => {}
        }
        self.coverage = coverage;
        Ok(self)
    }

    pub(super) fn from_array(array: &DafArray) -> Result<Self, Error> {
        let malformed = |reason| Error::MalformedCkSegment {
            name: array.name().into(),
            data_type: array.integers().get(2).copied().unwrap_or(0),
            reason,
        };
        let [start, end] = *array.doubles() else {
            return Err(malformed("summary does not hold two doubles"));
        };
        let [instrument, frame, data_type, avflag] = *array.integers() else {
            return Err(malformed("summary does not hold six integers"));
        };
        Self::from_stored(
            array.name(),
            instrument,
            frame,
            [start, end],
            data_type,
            avflag,
            array.data().to_vec(),
        )
    }

    pub(super) fn to_array(&self) -> Result<DafArray, Error> {
        let words = match &self.data {
            CkData::Type2(t2) => t2.to_words(),
            CkData::Type3(t3) => t3.to_words(),
            CkData::Raw { words, .. } => words.clone(),
        };
        DafArray::new(
            &self.name,
            self.coverage.to_vec(),
            alloc::vec![
                self.instrument,
                self.frame,
                self.data.data_type(),
                self.data.avflag(),
            ],
            words,
        )
    }

    /// Segment identifier (`SEGID`).
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// NAIF instrument or spacecraft structure ID (`ICD[0]`).
    #[must_use]
    pub const fn instrument(&self) -> i32 {
        self.instrument
    }

    /// Base reference frame NAIF ID (`ICD[1]`).
    #[must_use]
    pub const fn frame(&self) -> i32 {
        self.frame
    }

    /// Segment SCLK coverage `[begtim, endtim]` in encoded SCLK ticks.
    #[must_use]
    pub const fn coverage(&self) -> [f64; 2] {
        self.coverage
    }

    /// Whether the segment carries angular velocity data (`avflag == 1`).
    #[must_use]
    pub const fn has_angular_velocity(&self) -> bool {
        self.data.avflag() == 1
    }

    /// Segment payload.
    #[must_use]
    pub const fn data(&self) -> &CkData {
        &self.data
    }

    /// Whether `sclk` falls within `[begtim - tolerance, endtim + tolerance]`.
    #[must_use]
    pub fn covers_with_tolerance(&self, sclk: f64, tolerance: f64) -> bool {
        if !sclk.is_finite() || !tolerance.is_finite() || tolerance < 0.0 {
            return false;
        }
        let [start, end] = self.coverage;
        start - tolerance <= sclk && sclk <= end + tolerance
    }

    /// Evaluates pointing inside this segment at `sclk` within `tolerance` ticks.
    ///
    /// Returns `Ok(None)` if `sclk` lies outside coverage or inside an internal gap
    /// exceeding `tolerance`.
    pub fn pointing(&self, sclk: f64, tolerance: f64) -> Result<Option<CkPointing>, Error> {
        if !self.covers_with_tolerance(sclk, tolerance) {
            return Ok(None);
        }
        let [cov_start, cov_end] = self.coverage;
        let in_coverage =
            |p: &CkPointing| cov_start - tolerance <= p.sclk && p.sclk <= cov_end + tolerance;
        let found = match &self.data {
            CkData::Type2(t2) => t2.evaluate(sclk, tolerance, self.frame).filter(in_coverage),
            CkData::Type3(t3) => t3.evaluate(sclk, tolerance, self.frame).filter(in_coverage),
            CkData::Raw { data_type, .. } => {
                return Err(Error::UnsupportedCkDataType {
                    data_type: *data_type,
                });
            }
        };
        Ok(found)
    }
}
