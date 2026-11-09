//! DAF/SPK Type 2 & Type 3 Chebyshev ephemeris segment evaluation.

use glam::{DMat3, DVec3};

use crate::{
    chebyshev::{eval_type2_record_3d, eval_type3_record_3d},
    hash::ContentHash,
    ids::{FrameCode, NaifId},
    scalar_math::sin_cos,
    time::{TdbEpoch, TdbSpanNs},
};

use super::{DafContainer, DafError, RawDafSummary};

/// SPICE kilometers-to-meters scale factor (`1,000 m / km`).
const KM_TO_M: f64 = 1_000.0;

/// SPICE IAU 1976 mean obliquity of the ecliptic at J2000 (`84,381.448"` in radians).
pub(crate) const SPICE_OBLIQUITY_J2000_RAD: f64 =
    84_381.448 * (core::f64::consts::PI / (180.0 * 3_600.0));

/// Supported NAIF SPK Chebyshev segment data types.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SpkDataType {
    /// SPK Type 2: fixed-interval Chebyshev polynomials for position `(x, y, z)`.
    Type2ChebyshevPosition,
    /// SPK Type 3: fixed-interval Chebyshev polynomials for position and velocity `(x, y, z, vx, vy, vz)`.
    Type3ChebyshevPosVel,
}

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
    fn from_raw(container: &DafContainer<'_>, raw: RawDafSummary) -> Result<Self, DafError> {
        let start_s = raw.dc.first().copied().unwrap_or(0.0);
        let end_s = raw.dc.get(1).copied().unwrap_or(0.0);
        let target = NaifId(raw.ic.first().copied().unwrap_or(0));
        let center = NaifId(raw.ic.get(1).copied().unwrap_or(0));
        let frame_raw = raw.ic.get(2).copied().unwrap_or(0);
        let dtype_raw = raw.ic.get(3).copied().unwrap_or(0);
        let start_word = raw.ic.get(4).copied().unwrap_or(0);
        let end_word = raw.ic.get(5).copied().unwrap_or(0);

        let base_frame = FrameCode(frame_raw);
        if base_frame != FrameCode::J2000 && base_frame != FrameCode::ECLIPJ2000 {
            return Err(DafError::UnsupportedBaseFrame(frame_raw));
        }
        let data_type = match dtype_raw {
            2 => SpkDataType::Type2ChebyshevPosition,
            3 => SpkDataType::Type3ChebyshevPosVel,
            other => return Err(DafError::UnsupportedSpkDataType(other)),
        };

        let words = container.read_f64_words(start_word, end_word)?;
        if words.len() < 4 {
            return Err(DafError::MalformedDirectory(raw.name));
        }
        let footer_start = words.len() - 4;
        let init_s = words.get(footer_start).copied().unwrap_or(0.0);
        let intlen_s = words.get(footer_start + 1).copied().unwrap_or(0.0);
        let rsize_f = words.get(footer_start + 2).copied().unwrap_or(0.0);
        let n_rec_f = words.get(footer_start + 3).copied().unwrap_or(0.0);
        if !init_s.is_finite()
            || !intlen_s.is_finite()
            || intlen_s <= 0.0
            || rsize_f < 5.0
            || n_rec_f < 1.0
        {
            return Err(DafError::MalformedDirectory(raw.name));
        }
        let rsize = rsize_f as usize;
        let n_records = n_rec_f as usize;
        let expected_payload = rsize.saturating_mul(n_records);
        if expected_payload.saturating_add(4) != words.len() {
            return Err(DafError::MalformedDirectory(raw.name));
        }
        let mut records = words;
        records.truncate(expected_payload);
        let span_ns = TdbSpanNs::from_j2000_seconds_inner(start_s, end_s)
            .ok_or_else(|| DafError::MalformedDirectory(raw.name.clone()))?;

        Ok(Self {
            name: raw.name,
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

    /// Evaluates `(pos_m, vel_m_s, acc_m_s2)` of `target` relative to `center` in `ICRF / J2000` axes.
    ///
    /// Uses half-open granule ownership `[t_k, t_{k+1})` for interior knots and clamps
    /// only the exact upper boundary `t_N` to the last granule `n_records - 1`.
    pub fn evaluate_icrf(&self, epoch: TdbEpoch) -> Result<(DVec3, DVec3, DVec3), DafError> {
        let (t_whole_s, t_sub_s) = epoch.split_j2000_seconds();
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
        let mut pos = DVec3::from_array(p_arr);
        let mut vel = DVec3::from_array(v_arr);
        let mut acc = DVec3::from_array(a_arr);

        if self.base_frame == FrameCode::ECLIPJ2000 {
            let r_eclip_to_j2000 = eclipj2000_to_j2000_matrix();
            pos = r_eclip_to_j2000 * pos;
            vel = r_eclip_to_j2000 * vel;
            acc = r_eclip_to_j2000 * acc;
        }
        Ok((pos, vel, acc))
    }

    /// Appends interior Chebyshev granule boundary epochs lying strictly inside `(start, end)`.
    pub fn collect_interior_knots_in(
        &self,
        start: TdbEpoch,
        end: TdbEpoch,
        out: &mut Vec<TdbEpoch>,
    ) {
        if end <= start {
            return;
        }
        let lo_s = start.as_j2000_seconds_f64().max(self.start_s);
        let hi_s = end.as_j2000_seconds_f64().min(self.end_s);
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
            if let Some(knot) = TdbEpoch::from_j2000_seconds_f64(knot_s)
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
    segments: Vec<SpkSegment>,
}

impl SpkFile {
    /// Parses a binary `DAF/SPK` byte slice (`Type 2` and `Type 3` Chebyshev subset).
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, DafError> {
        let container = DafContainer::parse(bytes, "DAF/SPK ", 2, 6)?;
        let raw_list = container.summaries()?;
        let mut segments = Vec::with_capacity(raw_list.len());
        for raw in raw_list {
            segments.push(SpkSegment::from_raw(&container, raw)?);
        }
        Ok(Self {
            hash: container.content_hash(),
            segments,
        })
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

/// Constant rotation matrix transforming `ECLIPJ2000 (17)` vectors into `J2000 / ICRF (1)`.
#[must_use]
pub(crate) fn eclipj2000_to_j2000_matrix() -> DMat3 {
    // R_J2000_to_ECLIP = R_1(+eps0), so R_ECLIP_to_J2000 = R_1(+eps0)^T.
    let (s, c) = sin_cos(SPICE_OBLIQUITY_J2000_RAD);
    DMat3::from_cols(
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(0.0, c, s),
        DVec3::new(0.0, -s, c),
    )
}
