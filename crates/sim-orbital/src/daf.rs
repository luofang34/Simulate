//! Binary NAIF Double Precision Array File (`DAF/SPK` and `DAF/PCK`) subset reader.
//!
//! All `DAF` binary header, summary chain, endianness (`LTL-IEEE` / `BIG-IEEE`),
//! and array extraction go through [`spice_kernels::Daf`].

pub(crate) mod bpc;
pub(crate) mod spk;

pub use bpc::{BpcFile, BpcSegment};
pub use spice_kernels::Endian as DafEndian;
pub use spk::{SpkDataType, SpkFile, SpkSegment};

use spice_kernels::{Daf, Error as SpiceError};

/// Why parsing or evaluating a binary NAIF `DAF` kernel failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum DafError {
    /// File was shorter than a single 1,024-byte DAF record or truncated mid-record.
    #[error("DAF file truncated at {len} bytes (needed at least {needed} bytes)")]
    Truncated {
        /// Actual byte slice length.
        len: usize,
        /// Minimum required byte length.
        needed: usize,
    },
    /// File identifier word (`LOCIDW`) did not match expected `DAF/SPK ` or `DAF/PCK `.
    #[error("unexpected DAF identifier `{found}`, expected `{expected}`")]
    UnexpectedIdWord {
        /// Expected 8-byte ASCII signature.
        expected: &'static str,
        /// Found 8-byte ASCII signature (lossy).
        found: String,
    },
    /// Unsupported binary floating-point format (`LOCFMT`, e.g. VAX `VAX-GFLT`).
    #[error("unsupported DAF floating-point format `{0}`")]
    UnsupportedBinaryFormat(String),
    /// Header `ND`/`NI` summary counts did not match the SPK `(2, 6)` or BPC `(2, 5)` layout.
    #[error("unexpected DAF summary layout ND={nd}, NI={ni}")]
    InvalidSummaryFormat {
        /// Number of `f64` components per summary.
        nd: i32,
        /// Number of `i32` components per summary.
        ni: i32,
    },
    /// Summary linked-list pointer was out of bounds or cyclic.
    #[error("corrupt DAF summary record chain at record {record}")]
    CorruptSummaryChain {
        /// 1-based DAF record number.
        record: usize,
    },
    /// Segment double-word address range `[start_word, end_word]` was invalid.
    #[error("invalid DAF segment word range {start_word}..={end_word}")]
    InvalidAddressRange {
        /// 1-based start `f64` word index.
        start_word: i32,
        /// 1-based inclusive end `f64` word index.
        end_word: i32,
    },
    /// SPK segment used a data type outside the documented Type 2 / Type 3 subset.
    #[error("unsupported SPK segment data type {0} (supported: Type 2 and Type 3)")]
    UnsupportedSpkDataType(i32),
    /// Binary PCK segment used a data type outside the documented Type 2 subset.
    #[error("unsupported binary PCK data type {0} (supported: Type 2)")]
    UnsupportedPckDataType(i32),
    /// Segment inertial reference frame was outside the supported `J2000 (1)` / `ECLIPJ2000 (17)` set.
    #[error("unsupported segment base frame {0}")]
    UnsupportedBaseFrame(i32),
    /// Malformed Type 2 / Type 3 directory footer `(INIT, INTLEN, RSIZE, N)`.
    #[error("malformed Chebyshev directory footer in segment `{0}`")]
    MalformedDirectory(String),
    /// Underlying Chebyshev evaluation failed.
    #[error("Chebyshev evaluation error: {0}")]
    Chebyshev(#[from] crate::chebyshev::ChebyshevError),
}

impl From<DafError> for sim_spacetime::EphemerisError {
    fn from(err: DafError) -> Self {
        Self::KernelData(err.to_string())
    }
}

impl From<DafError> for sim_spacetime::OrientationError {
    fn from(err: DafError) -> Self {
        Self::KernelData(err.to_string())
    }
}

impl From<navigate_astrodynamics::spk::SpkError> for DafError {
    fn from(err: navigate_astrodynamics::spk::SpkError) -> Self {
        use navigate_astrodynamics::spk::SpkError;
        match err {
            SpkError::Truncated { len, needed } => Self::Truncated { len, needed },
            SpkError::UnexpectedIdWord { found } => Self::UnexpectedIdWord {
                expected: "DAF/SPK ",
                found,
            },
            SpkError::UnsupportedBinaryFormat(fmt) => Self::UnsupportedBinaryFormat(fmt),
            SpkError::InvalidSummaryFormat { nd, ni } => Self::InvalidSummaryFormat { nd, ni },
            SpkError::CorruptSummaryChain { record } => Self::CorruptSummaryChain { record },
            SpkError::InvalidAddressRange {
                start_word,
                end_word,
            } => Self::InvalidAddressRange {
                start_word,
                end_word,
            },
            SpkError::UnsupportedDataType(dt) => Self::UnsupportedSpkDataType(dt),
            SpkError::UnsupportedBaseFrame(frame) => Self::UnsupportedBaseFrame(frame),
            SpkError::MalformedDirectory(name) | SpkError::InvalidChebyshevRecord(name) => {
                Self::MalformedDirectory(name)
            }
            SpkError::Chebyshev(c) => Self::Chebyshev(c),
            other => Self::MalformedDirectory(other.to_string()),
        }
    }
}

/// Parses and validates a binary DAF container using [`spice_kernels::Daf`].
pub(crate) fn parse_daf(
    bytes: &[u8],
    expected_kernel_type: &str,
    expected_id_word: &'static str,
    expected_nd: usize,
    expected_ni: usize,
) -> Result<Daf, DafError> {
    let daf = Daf::parse(bytes).map_err(|err| map_spice_daf_err(err, expected_id_word))?;
    if daf.kernel_type() != expected_kernel_type {
        return Err(DafError::UnexpectedIdWord {
            expected: expected_id_word,
            found: format!("DAF/{}", daf.kernel_type()),
        });
    }
    if daf.nd() != expected_nd || daf.ni() != expected_ni {
        return Err(DafError::InvalidSummaryFormat {
            nd: i32::try_from(daf.nd()).unwrap_or(i32::MAX),
            ni: i32::try_from(daf.ni()).unwrap_or(i32::MAX),
        });
    }
    Ok(daf)
}

fn map_spice_daf_err(err: SpiceError, expected: &'static str) -> DafError {
    match err {
        SpiceError::Truncated { len, needed } => DafError::Truncated { len, needed },
        SpiceError::NotDaf { found } | SpiceError::WrongKernelType { found, .. } => {
            DafError::UnexpectedIdWord { expected, found }
        }
        SpiceError::UnsupportedBinaryFormat { found } => DafError::UnsupportedBinaryFormat(found),
        SpiceError::InvalidSummaryFormat { nd, ni } => DafError::InvalidSummaryFormat {
            nd: i32::try_from(nd).unwrap_or(i32::MAX),
            ni: i32::try_from(ni).unwrap_or(i32::MAX),
        },
        SpiceError::CorruptSummaryChain { record } => DafError::CorruptSummaryChain { record },
        SpiceError::InvalidAddressRange { start, end, .. } => DafError::InvalidAddressRange {
            start_word: i32::try_from(start).unwrap_or(i32::MAX),
            end_word: i32::try_from(end).unwrap_or(i32::MAX),
        },
        SpiceError::EmptyArray { name } => DafError::MalformedDirectory(name),
        other => DafError::MalformedDirectory(other.to_string()),
    }
}
