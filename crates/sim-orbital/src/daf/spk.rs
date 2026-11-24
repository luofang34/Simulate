//! DAF/SPK Type 2 & Type 3 Chebyshev ephemeris parsing via [`spice_kernels::Daf`]
//! and evaluation via [`navigate_astrodynamics::spk`].

use glam::DVec3;
pub use navigate_astrodynamics::spk::SpkDataType;
use navigate_astrodynamics::spk::{
    ChebyshevPva3, SpkFile as NavSpkFile, eval_type2_record_3d, eval_type3_record_3d,
};
use spice_kernels::DafArray;

use crate::{
    hash::ContentHash,
    ids::{FrameCode, NaifId},
    time::{TdbEpoch, TdbSpanNs},
};

use super::{DafError, bpc::rot_axis_1, parse_daf};

/// SPICE IAU 1976 mean obliquity of the ecliptic at J2000 (`84,381.448"` in radians).
pub(crate) const SPICE_OBLIQUITY_J2000_RAD: f64 =
    84_381.448 * (core::f64::consts::PI / (180.0 * 3_600.0));

const KM_TO_M: f64 = 1_000.0;

/// One validated fixed-interval Chebyshev segment from a binary `DAF/SPK` file.
#[derive(Clone, Debug)]
pub struct SpkSegment {
    name: String,
    target: NaifId,
    center: NaifId,
    base_frame: FrameCode,
    data_type: SpkDataType,
    start_s: f64,
    end_s: f64,
    span_ns: TdbSpanNs,
    init_s: f64,
    intlen_s: f64,
    rsize: usize,
    n_records: usize,
    records: Vec<f64>,
}

impl SpkSegment {
    fn from_array(array: &DafArray) -> Result<Self, DafError> {
        let name = array.name().to_owned();
        let start_s = array.doubles().first().copied().unwrap_or(0.0);
        let end_s = array.doubles().get(1).copied().unwrap_or(0.0);
        let target = NaifId(array.integers().first().copied().unwrap_or(0));
        let center = NaifId(array.integers().get(1).copied().unwrap_or(0));
        let base_raw = array.integers().get(2).copied().unwrap_or(0);
        let dtype_raw = array.integers().get(3).copied().unwrap_or(0);

        let base_frame = FrameCode(base_raw);
        if base_frame != FrameCode::J2000 && base_frame != FrameCode::ECLIPJ2000 {
            return Err(DafError::UnsupportedBaseFrame(base_raw));
        }
        let data_type = match dtype_raw {
            2 => SpkDataType::Type2ChebyshevPosition,
            3 => SpkDataType::Type3ChebyshevPosVel,
            other => return Err(DafError::UnsupportedSpkDataType(other)),
        };

        let words = array.data();
        if words.len() < 4 || !start_s.is_finite() || !end_s.is_finite() || end_s < start_s {
            return Err(DafError::MalformedDirectory(name));
        }
        let footer_start = words.len().wrapping_sub(4);
        let init_s = words.get(footer_start).copied().unwrap_or(0.0);
        let intlen_s = words
            .get(footer_start.wrapping_add(1))
            .copied()
            .unwrap_or(0.0);
        let rsize_f = words
            .get(footer_start.wrapping_add(2))
            .copied()
            .unwrap_or(0.0);
        let n_rec_f = words
            .get(footer_start.wrapping_add(3))
            .copied()
            .unwrap_or(0.0);
        if !init_s.is_finite()
            || !intlen_s.is_finite()
            || intlen_s <= 0.0
            || rsize_f < 5.0
            || n_rec_f < 1.0
        {
            return Err(DafError::MalformedDirectory(name));
        }
        let rsize = rsize_f as usize;
        let n_records = n_rec_f as usize;
        let expected_payload = rsize.saturating_mul(n_records);
        if expected_payload.saturating_add(4) != words.len() {
            return Err(DafError::MalformedDirectory(name));
        }
        let records = words
            .get(..expected_payload)
            .ok_or_else(|| DafError::MalformedDirectory(name.clone()))?
            .to_vec();
        let span_ns = TdbSpanNs::from_j2000_seconds_inner(start_s, end_s)
            .ok_or_else(|| DafError::MalformedDirectory(name.clone()))?;

        Ok(Self {
            name,
            target,
            center,
            base_frame,
            data_type,
            start_s,
            end_s,
            span_ns,
            init_s,
            intlen_s,
            rsize,
            n_records,
            records,
        })
    }

    /// Segment descriptor name (`40` ASCII characters trimmed).
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Target body or barycenter identifier.
    #[must_use]
    pub const fn target(&self) -> NaifId {
        self.target
    }

    /// Center-of-motion body or barycenter identifier.
    #[must_use]
    pub const fn center(&self) -> NaifId {
        self.center
    }

    /// Segment inertial orientation code (`J2000` or `ECLIPJ2000`).
    #[must_use]
    pub const fn base_frame(&self) -> FrameCode {
        self.base_frame
    }

    /// SPK polynomial representation (`Type 2` or `Type 3`).
    #[must_use]
    pub const fn data_type(&self) -> SpkDataType {
        self.data_type
    }

    /// Closed TDB coverage span of this segment.
    #[must_use]
    pub const fn span_ns(&self) -> TdbSpanNs {
        self.span_ns
    }

    /// Fixed granule interval length `INTLEN` in seconds.
    #[must_use]
    pub const fn granule_length_s(&self) -> f64 {
        self.intlen_s
    }

    /// Whether `epoch` lies inside this segment's valid TDB interval.
    #[must_use]
    pub const fn contains(&self, epoch: TdbEpoch) -> bool {
        self.span_ns.contains(epoch)
    }

    pub(crate) fn evaluate_pva_split_j2000(
        &self,
        t_whole_s: f64,
        t_sub_s: f64,
    ) -> Result<ChebyshevPva3, DafError> {
        let rel_s = (t_whole_s - self.init_s) + t_sub_s;
        let raw_idx = (rel_s / self.intlen_s).floor();
        let rec_idx = if raw_idx <= 0.0 {
            0_usize
        } else {
            (raw_idx as usize).min(self.n_records.saturating_sub(1))
        };
        let rec_start = rec_idx.wrapping_mul(self.rsize);
        let rec_end = rec_start.wrapping_add(self.rsize);
        let record = self
            .records
            .get(rec_start..rec_end)
            .ok_or_else(|| DafError::MalformedDirectory(self.name.clone()))?;

        let (p_arr, v_arr, a_arr) = match self.data_type {
            SpkDataType::Type2ChebyshevPosition => {
                eval_type2_record_3d(record, t_whole_s, t_sub_s, KM_TO_M)?
            }
            SpkDataType::Type3ChebyshevPosVel => {
                eval_type3_record_3d(record, t_whole_s, t_sub_s, KM_TO_M)?
            }
        };
        if self.base_frame == FrameCode::ECLIPJ2000 {
            let rot = rot_axis_1(-SPICE_OBLIQUITY_J2000_RAD);
            return Ok((
                (rot * DVec3::from_array(p_arr)).to_array(),
                (rot * DVec3::from_array(v_arr)).to_array(),
                (rot * DVec3::from_array(a_arr)).to_array(),
            ));
        }
        Ok((p_arr, v_arr, a_arr))
    }

    /// Evaluates `(pos_m, vel_m_s, acc_m_s2)` of `target` relative to `center` in `ICRF / J2000` axes.
    pub fn evaluate_icrf(&self, epoch: TdbEpoch) -> Result<(DVec3, DVec3, DVec3), DafError> {
        let (t_whole_s, t_sub_s) = epoch.split_j2000_seconds();
        let (p, v, a) = self.evaluate_pva_split_j2000(t_whole_s, t_sub_s)?;
        Ok((
            DVec3::from_array(p),
            DVec3::from_array(v),
            DVec3::from_array(a),
        ))
    }

    /// Appends interior Chebyshev granule boundary epochs lying strictly inside `(start, end)`.
    pub fn collect_interior_knots_in(
        &self,
        start: TdbEpoch,
        end: TdbEpoch,
        out: &mut Vec<TdbEpoch>,
    ) {
        let start_s = start.as_j2000_seconds_f64();
        let end_s = end.as_j2000_seconds_f64();
        if end_s <= start_s {
            return;
        }
        let lo_s = start_s.max(self.start_s);
        let hi_s = end_s.min(self.end_s);
        if hi_s <= lo_s {
            return;
        }
        let first_k = (((lo_s - self.init_s) / self.intlen_s).floor() as isize).max(1) as usize;
        let last_k = (((hi_s - self.init_s) / self.intlen_s).ceil() as isize)
            .max(0)
            .min(self.n_records as isize) as usize;
        let mut k = first_k;
        while k < last_k {
            let knot_s = self.init_s + (k as f64) * self.intlen_s;
            if knot_s > start_s
                && knot_s < end_s
                && knot_s >= self.start_s
                && knot_s <= self.end_s
                && let Some(knot) = TdbEpoch::from_j2000_seconds_f64(knot_s)
                && knot > start
                && knot < end
                && self.contains(knot)
            {
                out.push(knot);
            }
            k = k.wrapping_add(1);
        }
    }
}

/// Parsed in-memory `DAF/SPK` file with segment order preserved.
#[derive(Clone, Debug)]
pub struct SpkFile {
    hash: ContentHash,
    inner: NavSpkFile,
    segments: Vec<SpkSegment>,
}

impl SpkFile {
    /// Parses a binary `DAF/SPK` byte slice (`Type 2` and `Type 3` Chebyshev subset) via [`spice_kernels::Daf`].
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, DafError> {
        let daf = parse_daf(bytes, "SPK", "DAF/SPK ", 2, 6)?;
        let mut segments = Vec::with_capacity(daf.arrays().len());
        for array in daf.arrays() {
            segments.push(SpkSegment::from_array(array)?);
        }
        let inner = NavSpkFile::from_bytes(bytes)?;
        Ok(Self {
            hash: ContentHash::of_bytes(bytes),
            inner,
            segments,
        })
    }

    /// Converts an in-memory [`spice_kernels::SpkFile`] containing Type 2/3 segments into [`Self`].
    pub fn from_spice_kernels(spk: &spice_kernels::SpkFile) -> Result<Self, DafError> {
        let bytes = spk
            .to_bytes()
            .map_err(|err| DafError::MalformedDirectory(err.to_string()))?;
        Self::from_bytes(&bytes)
    }

    pub(crate) fn into_nav(self) -> NavSpkFile {
        self.inner
    }

    /// SHA-256 digest of the source `.bsp` bytes.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.hash
    }

    /// Segments in file summary order (later segments override earlier segments on overlap).
    #[must_use]
    pub fn segments(&self) -> &[SpkSegment] {
        &self.segments
    }
}
