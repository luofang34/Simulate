//! Binary NAIF Double Precision Array File (`DAF/SPK` and `DAF/PCK`) subset reader.
//!
//! Supports standard IEEE-754 little-endian (`LTL-IEEE`) and big-endian (`BIG-IEEE`)
//! DAF containers with linked 1,024-byte summary records.

pub(crate) mod bpc;
pub(crate) mod spk;

pub use bpc::{BpcFile, BpcSegment};
pub use spk::{SpkDataType, SpkFile, SpkSegment};

use crate::hash::ContentHash;

const RECORD_BYTES: usize = 1024;
const DOUBLES_PER_RECORD: usize = 128;

/// Byte-order format declared in a NAIF `DAF` file header (`LOCFMT`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DafEndian {
    /// IEEE-754 little-endian (`LTL-IEEE`).
    Little,
    /// IEEE-754 big-endian (`BIG-IEEE`).
    Big,
}

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

/// Parsed raw summary entry from a `DAF` container.
#[derive(Clone, Debug)]
pub(crate) struct RawDafSummary {
    pub name: String,
    pub dc: Vec<f64>,
    pub ic: Vec<i32>,
}

/// Validated view over a raw `DAF` byte buffer.
#[derive(Clone, Debug)]
pub(crate) struct DafContainer<'a> {
    bytes: &'a [u8],
    endian: DafEndian,
    nd: usize,
    ni: usize,
    fward: usize,
    hash: ContentHash,
}

impl<'a> DafContainer<'a> {
    pub(crate) fn parse(
        bytes: &'a [u8],
        expected_id: &'static str,
        expected_nd: i32,
        expected_ni: i32,
    ) -> Result<Self, DafError> {
        if bytes.len() < RECORD_BYTES {
            return Err(DafError::Truncated {
                len: bytes.len(),
                needed: RECORD_BYTES,
            });
        }
        let id_bytes = bytes.get(0..8).ok_or(DafError::Truncated {
            len: bytes.len(),
            needed: 8,
        })?;
        let found_id = String::from_utf8_lossy(id_bytes).into_owned();
        if found_id != expected_id {
            return Err(DafError::UnexpectedIdWord {
                expected: expected_id,
                found: found_id,
            });
        }
        let endian = detect_endian(bytes)?;
        let nd = read_i32(bytes, 8, endian)?;
        let ni = read_i32(bytes, 12, endian)?;
        if nd != expected_nd || ni != expected_ni {
            return Err(DafError::InvalidSummaryFormat { nd, ni });
        }
        let fward_i32 = read_i32(bytes, 76, endian)?;
        if fward_i32 <= 0 {
            return Err(DafError::CorruptSummaryChain { record: 0 });
        }
        Ok(Self {
            bytes,
            endian,
            nd: nd as usize,
            ni: ni as usize,
            fward: fward_i32 as usize,
            hash: ContentHash::of_bytes(bytes),
        })
    }

    pub(crate) const fn content_hash(&self) -> ContentHash {
        self.hash
    }

    pub(crate) fn summaries(&self) -> Result<Vec<RawDafSummary>, DafError> {
        let ss_words = self.nd.wrapping_add(self.ni.wrapping_add(1) / 2);
        let entry_bytes = ss_words.wrapping_mul(8);
        let max_records = self.bytes.len() / RECORD_BYTES;
        let mut out = Vec::new();
        let mut rec = self.fward;
        let mut visited = 0_usize;

        while rec != 0 {
            visited = visited.wrapping_add(1);
            if rec > max_records || visited > max_records {
                return Err(DafError::CorruptSummaryChain { record: rec });
            }
            let sum_off = rec.wrapping_sub(1).wrapping_mul(RECORD_BYTES);
            let name_off = rec.wrapping_mul(RECORD_BYTES);
            let next_f = read_f64(self.bytes, sum_off, self.endian)?;
            let nsum_f = read_f64(self.bytes, sum_off.wrapping_add(16), self.endian)?;
            if !next_f.is_finite() || next_f < 0.0 || !nsum_f.is_finite() || nsum_f < 0.0 {
                return Err(DafError::CorruptSummaryChain { record: rec });
            }
            let nsum = nsum_f as usize;
            let mut idx = 0_usize;
            while idx < nsum {
                let e_off = sum_off
                    .wrapping_add(24)
                    .wrapping_add(idx.wrapping_mul(entry_bytes));
                let n_off = name_off.wrapping_add(idx.wrapping_mul(entry_bytes));
                out.push(self.parse_one_summary(e_off, n_off, entry_bytes)?);
                idx = idx.wrapping_add(1);
            }
            rec = next_f as usize;
        }
        Ok(out)
    }

    fn parse_one_summary(
        &self,
        e_off: usize,
        n_off: usize,
        entry_bytes: usize,
    ) -> Result<RawDafSummary, DafError> {
        let mut dc = Vec::with_capacity(self.nd);
        let mut d_idx = 0_usize;
        while d_idx < self.nd {
            dc.push(read_f64(
                self.bytes,
                e_off.wrapping_add(d_idx.wrapping_mul(8)),
                self.endian,
            )?);
            d_idx = d_idx.wrapping_add(1);
        }
        let ic_base = e_off.wrapping_add(self.nd.wrapping_mul(8));
        let mut ic = Vec::with_capacity(self.ni);
        let mut i_idx = 0_usize;
        while i_idx < self.ni {
            ic.push(read_i32(
                self.bytes,
                ic_base.wrapping_add(i_idx.wrapping_mul(4)),
                self.endian,
            )?);
            i_idx = i_idx.wrapping_add(1);
        }
        let name = self
            .bytes
            .get(n_off..n_off.wrapping_add(entry_bytes))
            .map(|s| String::from_utf8_lossy(s).trim().to_owned())
            .unwrap_or_default();
        Ok(RawDafSummary { name, dc, ic })
    }

    /// Reads 1-based inclusive DAF double words `start_word..=end_word` into a `Vec<f64>`.
    pub(crate) fn read_f64_words(
        &self,
        start_word: i32,
        end_word: i32,
    ) -> Result<Vec<f64>, DafError> {
        if start_word <= 0 || end_word < start_word {
            return Err(DafError::InvalidAddressRange {
                start_word,
                end_word,
            });
        }
        let start_idx = (start_word as usize).wrapping_sub(1);
        let count = (end_word as usize).wrapping_sub(start_idx).wrapping_add(0);
        let byte_start = start_idx.wrapping_mul(8);
        let byte_end = byte_start.wrapping_add(count.wrapping_mul(8));
        if byte_end > self.bytes.len() || count > DOUBLES_PER_RECORD.saturating_mul(2_000_000) {
            return Err(DafError::Truncated {
                len: self.bytes.len(),
                needed: byte_end,
            });
        }
        let mut out = Vec::with_capacity(count);
        let mut k = 0_usize;
        while k < count {
            out.push(read_f64(
                self.bytes,
                byte_start.wrapping_add(k.wrapping_mul(8)),
                self.endian,
            )?);
            k = k.wrapping_add(1);
        }
        Ok(out)
    }
}

fn detect_endian(bytes: &[u8]) -> Result<DafEndian, DafError> {
    let fmt_slice = bytes.get(88..96).ok_or(DafError::Truncated {
        len: bytes.len(),
        needed: 96,
    })?;
    let fmt_str = String::from_utf8_lossy(fmt_slice);
    let trimmed = fmt_str.trim();
    if trimmed.eq_ignore_ascii_case("LTL-IEEE") {
        return Ok(DafEndian::Little);
    }
    if trimmed.eq_ignore_ascii_case("BIG-IEEE") {
        return Ok(DafEndian::Big);
    }
    // Older NAIF DAF headers without LOCFMT store small positive ND (2) at offset 8.
    let nd_le = read_i32(bytes, 8, DafEndian::Little)?;
    if (1..=124).contains(&nd_le) {
        return Ok(DafEndian::Little);
    }
    let nd_be = read_i32(bytes, 8, DafEndian::Big)?;
    if (1..=124).contains(&nd_be) {
        return Ok(DafEndian::Big);
    }
    Err(DafError::UnsupportedBinaryFormat(trimmed.to_owned()))
}

fn read_i32(bytes: &[u8], offset: usize, endian: DafEndian) -> Result<i32, DafError> {
    let end = offset.wrapping_add(4);
    let slice = bytes.get(offset..end).ok_or(DafError::Truncated {
        len: bytes.len(),
        needed: end,
    })?;
    let mut arr = [0_u8; 4];
    arr.copy_from_slice(slice);
    Ok(match endian {
        DafEndian::Little => i32::from_le_bytes(arr),
        DafEndian::Big => i32::from_be_bytes(arr),
    })
}

fn read_f64(bytes: &[u8], offset: usize, endian: DafEndian) -> Result<f64, DafError> {
    let end = offset.wrapping_add(8);
    let slice = bytes.get(offset..end).ok_or(DafError::Truncated {
        len: bytes.len(),
        needed: end,
    })?;
    let mut arr = [0_u8; 8];
    arr.copy_from_slice(slice);
    Ok(match endian {
        DafEndian::Little => f64::from_le_bytes(arr),
        DafEndian::Big => f64::from_be_bytes(arr),
    })
}
