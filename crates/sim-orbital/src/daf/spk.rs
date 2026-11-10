//! DAF/SPK Type 2 & Type 3 Chebyshev ephemeris segment evaluation backed by [`navigate_astrodynamics::spk`].

use glam::DVec3;
pub use navigate_astrodynamics::spk::SpkDataType;
use navigate_astrodynamics::spk::{SpkFile as NavSpkFile, SpkSegment as NavSpkSegment};

use crate::{
    hash::ContentHash,
    ids::{FrameCode, NaifId},
    time::{TdbEpoch, TdbSpanNs},
};

use super::DafError;

/// SPICE IAU 1976 mean obliquity of the ecliptic at J2000 (`84,381.448"` in radians).
pub(crate) const SPICE_OBLIQUITY_J2000_RAD: f64 =
    84_381.448 * (core::f64::consts::PI / (180.0 * 3_600.0));

/// One validated fixed-interval Chebyshev segment from a binary `DAF/SPK` file.
#[derive(Clone, Debug)]
pub struct SpkSegment {
    inner: NavSpkSegment,
    span_ns: TdbSpanNs,
}

impl SpkSegment {
    fn from_nav(inner: NavSpkSegment) -> Result<Self, DafError> {
        let (start_s, end_s) = inner.span_tdb_s();
        let span_ns = TdbSpanNs::from_j2000_seconds_inner(start_s, end_s)
            .ok_or_else(|| DafError::MalformedDirectory(inner.name().to_owned()))?;
        Ok(Self { inner, span_ns })
    }

    pub(crate) const fn inner(&self) -> &NavSpkSegment {
        &self.inner
    }

    /// Segment descriptor name (`40` ASCII characters trimmed).
    #[must_use]
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    /// Target body or barycenter identifier.
    #[must_use]
    pub const fn target(&self) -> NaifId {
        NaifId(self.inner.target())
    }

    /// Center-of-motion body or barycenter identifier.
    #[must_use]
    pub const fn center(&self) -> NaifId {
        NaifId(self.inner.center())
    }

    /// Segment inertial orientation code (`J2000` or `ECLIPJ2000`).
    #[must_use]
    pub const fn base_frame(&self) -> FrameCode {
        FrameCode(self.inner.base_frame())
    }

    /// SPK polynomial representation (`Type 2` or `Type 3`).
    #[must_use]
    pub const fn data_type(&self) -> SpkDataType {
        self.inner.data_type()
    }

    /// Closed TDB coverage span of this segment.
    #[must_use]
    pub const fn span_ns(&self) -> TdbSpanNs {
        self.span_ns
    }

    /// Fixed granule interval length `INTLEN` in seconds.
    #[must_use]
    pub const fn granule_length_s(&self) -> f64 {
        self.inner.granule_length_s()
    }

    /// Whether `epoch` lies inside this segment's valid TDB interval.
    #[must_use]
    pub const fn contains(&self, epoch: TdbEpoch) -> bool {
        self.span_ns.contains(epoch)
    }

    /// Evaluates `(pos_m, vel_m_s, acc_m_s2)` of `target` relative to `center` in `ICRF / J2000` axes.
    pub fn evaluate_icrf(&self, epoch: TdbEpoch) -> Result<(DVec3, DVec3, DVec3), DafError> {
        let (t_whole_s, t_sub_s) = epoch.split_j2000_seconds();
        let (p, v, a) = self.inner.evaluate_pva_split_j2000(t_whole_s, t_sub_s)?;
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
        for knot_s in self
            .inner
            .interior_knot_seconds_in(start.as_j2000_seconds_f64(), end.as_j2000_seconds_f64())
        {
            if let Some(knot) = TdbEpoch::from_j2000_seconds_f64(knot_s)
                && knot > start
                && knot < end
                && self.contains(knot)
            {
                out.push(knot);
            }
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
    /// Parses a binary `DAF/SPK` byte slice (`Type 2` and `Type 3` Chebyshev subset).
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, DafError> {
        let inner = NavSpkFile::from_bytes(bytes)?;
        let mut segments = Vec::with_capacity(inner.segments().len());
        for seg in inner.segments() {
            segments.push(SpkSegment::from_nav(seg.clone())?);
        }
        Ok(Self {
            hash: ContentHash::of_bytes(bytes),
            inner,
            segments,
        })
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
