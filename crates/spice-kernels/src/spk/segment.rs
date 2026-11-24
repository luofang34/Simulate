//! One SPK segment: its descriptor, its data, and conversion to and from the
//! DAF array that stores it.

use alloc::string::String;
use alloc::vec::Vec;

use super::sampled::SampledStates;
use super::{MAX_HERMITE_WINDOW, MAX_LAGRANGE_WINDOW};
use crate::daf::{self, DafArray};
use crate::error::Error;

/// Characters an SPK segment name holds.
const NAME_CHARS: usize = 40;

const TYPE_LAGRANGE: i32 = 9;
const TYPE_HERMITE: i32 = 13;

/// What a segment stores.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum SpkData {
    /// SPK type 9: Lagrange interpolation of states at unequal time steps.
    Lagrange(SampledStates),
    /// SPK type 13: Hermite interpolation of states at unequal time steps.
    Hermite(SampledStates),
    /// A segment of a type this crate does not model, kept word for word so
    /// that the kernel still reads and writes without loss.
    Raw {
        /// SPK data type code.
        data_type: i32,
        /// The segment's stored words.
        words: Vec<f64>,
    },
}

impl SpkData {
    /// SPK data type code.
    #[must_use]
    pub const fn data_type(&self) -> i32 {
        match self {
            Self::Lagrange(_) => TYPE_LAGRANGE,
            Self::Hermite(_) => TYPE_HERMITE,
            Self::Raw { data_type, .. } => *data_type,
        }
    }
}

/// An SPK segment: the motion of one body relative to another over one
/// interval, in one frame.
#[derive(Clone, Debug, PartialEq)]
pub struct SpkSegment {
    name: String,
    target: i32,
    center: i32,
    frame: i32,
    coverage: [f64; 2],
    data: SpkData,
}

impl SpkSegment {
    /// A type 9 segment covering every sample. `window` of the samples must
    /// be 2 to [`MAX_LAGRANGE_WINDOW`].
    pub fn lagrange(
        name: &str,
        target: i32,
        center: i32,
        frame: i32,
        samples: SampledStates,
    ) -> Result<Self, Error> {
        check_window(TYPE_LAGRANGE, samples.window(), 2, MAX_LAGRANGE_WINDOW)?;
        Self::sampled(name, [target, center, frame], SpkData::Lagrange(samples))
    }

    /// A type 13 segment covering every sample. `window` of the samples must
    /// be 1 to [`MAX_HERMITE_WINDOW`].
    pub fn hermite(
        name: &str,
        target: i32,
        center: i32,
        frame: i32,
        samples: SampledStates,
    ) -> Result<Self, Error> {
        check_window(TYPE_HERMITE, samples.window(), 1, MAX_HERMITE_WINDOW)?;
        Self::sampled(name, [target, center, frame], SpkData::Hermite(samples))
    }

    fn sampled(
        name: &str,
        [target, center, frame]: [i32; 3],
        data: SpkData,
    ) -> Result<Self, Error> {
        daf::check_text("segment name", name, NAME_CHARS)?;
        let coverage = match &data {
            SpkData::Lagrange(samples) | SpkData::Hermite(samples) => samples.span(),
            SpkData::Raw { .. } => [f64::NAN; 2],
        };
        Self {
            name: name.into(),
            target,
            center,
            frame,
            coverage,
            data,
        }
        .with_coverage(coverage)
    }

    /// A segment from its stored form: the descriptor fields and the words of
    /// its array. Types 9 and 13 are decoded and checked; any other type is
    /// kept as [`SpkData::Raw`].
    pub fn from_stored(
        name: &str,
        target: i32,
        center: i32,
        frame: i32,
        coverage: [f64; 2],
        data_type: i32,
        words: Vec<f64>,
    ) -> Result<Self, Error> {
        daf::check_text("segment name", name, NAME_CHARS)?;
        let malformed = |reason| Error::MalformedSegment {
            name: name.into(),
            data_type,
            reason,
        };
        let data = match data_type {
            TYPE_LAGRANGE => {
                let samples = SampledStates::from_words(&words).map_err(malformed)??;
                check_window(data_type, samples.window(), 2, MAX_LAGRANGE_WINDOW)?;
                SpkData::Lagrange(samples)
            }
            TYPE_HERMITE => {
                let samples = SampledStates::from_words(&words).map_err(malformed)??;
                check_window(data_type, samples.window(), 1, MAX_HERMITE_WINDOW)?;
                SpkData::Hermite(samples)
            }
            _ if words.is_empty() => return Err(malformed("no words")),
            _ => SpkData::Raw { data_type, words },
        };
        Self {
            name: name.into(),
            target,
            center,
            frame,
            coverage,
            data,
        }
        .with_coverage(coverage)
    }

    /// Narrows or restates the coverage interval `[start, end]`.
    ///
    /// For sampled data the interval must be non-empty and lie within the
    /// samples, as SPICE requires. A verbatim segment's interval is taken as
    /// given, because its data is not interpreted.
    pub fn with_coverage(mut self, coverage: [f64; 2]) -> Result<Self, Error> {
        let [start, end] = coverage;
        if let SpkData::Lagrange(samples) | SpkData::Hermite(samples) = &self.data {
            let [data_start, data_end] = samples.span();
            if !(start < end && data_start <= start && end <= data_end) {
                return Err(Error::InvalidCoverage {
                    start,
                    end,
                    data_start,
                    data_end,
                });
            }
        }
        self.coverage = coverage;
        Ok(self)
    }

    pub(super) fn from_array(array: &DafArray) -> Result<Self, Error> {
        let malformed = |reason| Error::MalformedSegment {
            name: array.name().into(),
            data_type: array.integers().get(3).copied().unwrap_or(0),
            reason,
        };
        let [start, end] = *array.doubles() else {
            return Err(malformed("summary does not hold two doubles"));
        };
        let [target, center, frame, data_type] = *array.integers() else {
            return Err(malformed("summary does not hold six integers"));
        };
        Self::from_stored(
            array.name(),
            target,
            center,
            frame,
            [start, end],
            data_type,
            array.data().to_vec(),
        )
    }

    pub(super) fn to_array(&self) -> Result<DafArray, Error> {
        let words = match &self.data {
            SpkData::Lagrange(samples) | SpkData::Hermite(samples) => samples.to_words(),
            SpkData::Raw { words, .. } => words.clone(),
        };
        DafArray::new(
            &self.name,
            self.coverage.to_vec(),
            alloc::vec![self.target, self.center, self.frame, self.data.data_type()],
            words,
        )
    }

    /// Segment name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// NAIF code of the body whose motion the segment gives.
    #[must_use]
    pub const fn target(&self) -> i32 {
        self.target
    }

    /// NAIF code of the body the motion is relative to.
    #[must_use]
    pub const fn center(&self) -> i32 {
        self.center
    }

    /// NAIF code of the reference frame of the states.
    #[must_use]
    pub const fn frame(&self) -> i32 {
        self.frame
    }

    /// Coverage interval `[start, end]`, TDB seconds past J2000.
    #[must_use]
    pub const fn coverage(&self) -> [f64; 2] {
        self.coverage
    }

    /// The segment's data.
    #[must_use]
    pub const fn data(&self) -> &SpkData {
        &self.data
    }

    /// Whether `et` lies in the coverage interval, ends included.
    #[must_use]
    pub fn covers(&self, et: f64) -> bool {
        let [start, end] = self.coverage;
        start <= et && et <= end
    }

    /// State at `et`: position in kilometres then velocity in kilometres per
    /// second, in the segment's frame, relative to its centre.
    pub fn state(&self, et: f64) -> Result<[f64; 6], Error> {
        if !self.covers(et) {
            let [start, end] = self.coverage;
            return Err(Error::OutOfCoverage { et, start, end });
        }
        let state = match &self.data {
            SpkData::Lagrange(samples) => samples.lagrange(et),
            SpkData::Hermite(samples) => samples.hermite(et),
            SpkData::Raw { data_type, .. } => {
                return Err(Error::UnsupportedDataType {
                    data_type: *data_type,
                });
            }
        };
        state.ok_or_else(|| Error::MalformedSegment {
            name: self.name.clone(),
            data_type: self.data.data_type(),
            reason: "samples do not fill one interpolation window",
        })
    }
}

fn check_window(data_type: i32, window: usize, min: usize, max: usize) -> Result<(), Error> {
    if (min..=max).contains(&window) {
        Ok(())
    } else {
        Err(Error::InvalidWindow {
            data_type,
            window,
            min,
            max,
        })
    }
}
