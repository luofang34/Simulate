//! Binary NAIF `DAF/PCK` Type 2 Euler-angle orientation segment evaluation.

use glam::{DMat3, DVec3};

use crate::{
    chebyshev::eval_type2_record_3d,
    hash::ContentHash,
    ids::FrameCode,
    scalar_math::sin_cos,
    time::{TdbEpoch, TdbSpanNs},
};

use super::{DafContainer, DafError, RawDafSummary, spk::SPICE_OBLIQUITY_J2000_RAD};

/// Evaluated `(R_base_to_body, omega_body_rad_s, alpha_body_rad_s2)` from a `3-1-3` Euler series.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawEuler313State {
    /// Coordinate transformation matrix from `base_frame` into the `frame_class_id` body frame.
    pub r_base_to_body: DMat3,
    /// Body angular velocity relative to `base_frame`, expressed in body-fixed coordinates (`rad/s`).
    pub omega_body_rad_s: DVec3,
    /// Body angular acceleration relative to `base_frame`, expressed in body-fixed coordinates (`rad/s²`).
    pub alpha_body_rad_s2: DVec3,
}

/// One validated Type-2 Chebyshev `3-1-3` Euler segment from a binary `DAF/PCK` (`.bpc`) file.
#[derive(Clone, Debug)]
pub struct BpcSegment {
    name: String,
    frame_class_id: FrameCode,
    base_frame: FrameCode,
    span_ns: TdbSpanNs,
    init_s: f64,
    intlen_s: f64,
    rsize: usize,
    n_records: usize,
    records: Vec<f64>,
}

impl BpcSegment {
    fn from_raw(container: &DafContainer<'_>, raw: RawDafSummary) -> Result<Self, DafError> {
        let start_s = raw.dc.first().copied().unwrap_or(0.0);
        let end_s = raw.dc.get(1).copied().unwrap_or(0.0);
        let class_id = FrameCode(raw.ic.first().copied().unwrap_or(0));
        let base_raw = raw.ic.get(1).copied().unwrap_or(0);
        let pck_type = raw.ic.get(2).copied().unwrap_or(0);
        let start_word = raw.ic.get(3).copied().unwrap_or(0);
        let end_word = raw.ic.get(4).copied().unwrap_or(0);

        let base_frame = FrameCode(base_raw);
        if base_frame != FrameCode::J2000 && base_frame != FrameCode::ECLIPJ2000 {
            return Err(DafError::UnsupportedBaseFrame(base_raw));
        }
        if pck_type != 2 {
            return Err(DafError::UnsupportedPckDataType(pck_type));
        }

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
            frame_class_id: class_id,
            base_frame,
            span_ns,
            init_s,
            intlen_s,
            rsize,
            n_records,
            records,
        })
    }

    /// Segment descriptor name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Binary PCK class identifier (e.g., `3000` for `ITRF93`, `31008` for `MOON_PA_DE440`).
    #[must_use]
    pub const fn frame_class_id(&self) -> FrameCode {
        self.frame_class_id
    }

    /// Inertial base frame (`J2000` or `ECLIPJ2000`).
    #[must_use]
    pub const fn base_frame(&self) -> FrameCode {
        self.base_frame
    }

    /// Closed TDB coverage span of this segment.
    #[must_use]
    pub const fn span_ns(&self) -> TdbSpanNs {
        self.span_ns
    }

    /// Whether `epoch` lies inside this segment's valid TDB interval.
    #[must_use]
    pub const fn contains(&self, epoch: TdbEpoch) -> bool {
        self.span_ns.contains(epoch)
    }

    /// Evaluates `(R_icrf_to_body, omega_body_rad_s, alpha_body_rad_s2)` at `epoch`.
    ///
    /// Automatically right-multiplies `R_J2000_to_ECLIPJ2000 = R_1(eps_0)` when the segment's
    /// base frame is `ECLIPJ2000 (17)` (as in NAIF's `earth_*_combined.bpc` `ITRF93` kernels).
    pub fn evaluate_icrf_to_body(&self, epoch: TdbEpoch) -> Result<RawEuler313State, DafError> {
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

        let ([p, t, s], [dp, dt, ds], [ddp, ddt, dds]) =
            eval_type2_record_3d(record, t_whole_s, t_sub_s, 1.0)?;
        let mut state = euler_313_kinematics((p, dp, ddp), (t, dt, ddt), (s, ds, dds));
        if self.base_frame == FrameCode::ECLIPJ2000 {
            state.r_base_to_body *= j2000_to_eclipj2000_matrix();
        }
        Ok(state)
    }
}

/// Parsed in-memory `DAF/PCK` (`.bpc`) file with segment precedence preserved.
#[derive(Clone, Debug)]
pub struct BpcFile {
    hash: ContentHash,
    segments: Vec<BpcSegment>,
}

impl BpcFile {
    /// Parses a binary `DAF/PCK` byte slice (`Type 2` Chebyshev Euler-angle subset).
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, DafError> {
        let container = DafContainer::parse(bytes, "DAF/PCK ", 2, 5)?;
        let raw_list = container.summaries()?;
        let mut segments = Vec::with_capacity(raw_list.len());
        for raw in raw_list {
            segments.push(BpcSegment::from_raw(&container, raw)?);
        }
        Ok(Self {
            hash: container.content_hash(),
            segments,
        })
    }

    /// SHA-256 digest of the source `.bpc` bytes.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.hash
    }

    /// Segments in file summary order (later segments override earlier segments on overlap).
    #[must_use]
    pub fn segments(&self) -> &[BpcSegment] {
        &self.segments
    }
}

/// Evaluates `R = R_3(psi) R_1(theta) R_3(phi)` and exact body-frame `(omega, alpha)`.
#[must_use]
pub(crate) fn euler_313_kinematics(
    phi: (f64, f64, f64),
    theta: (f64, f64, f64),
    psi: (f64, f64, f64),
) -> RawEuler313State {
    let (p, dp, ddp) = phi;
    let (t, dt, ddt) = theta;
    let (s, ds, dds) = psi;

    let r_base_to_body = rot_axis_3(s) * rot_axis_1(t) * rot_axis_3(p);
    let (st, ct) = sin_cos(t);
    let (ss, cs) = sin_cos(s);

    let wx = dp * st * ss + dt * cs;
    let wy = dp * st * cs - dt * ss;
    let wz = dp * ct + ds;

    let ax = ddp * st * ss + dp * (dt * ct * ss + ds * st * cs) + ddt * cs - dt * ds * ss;
    let ay = ddp * st * cs + dp * (dt * ct * cs - ds * st * ss) - ddt * ss - dt * ds * cs;
    let az = ddp * ct - dp * dt * st + dds;

    RawEuler313State {
        r_base_to_body,
        omega_body_rad_s: DVec3::new(wx, wy, wz),
        alpha_body_rad_s2: DVec3::new(ax, ay, az),
    }
}

/// Coordinate frame rotation about `+X` by `angle_rad` (`v_new = R_1(angle_rad) * v_old`).
#[must_use]
pub(crate) fn rot_axis_1(angle_rad: f64) -> DMat3 {
    let (s, c) = sin_cos(angle_rad);
    DMat3::from_cols(
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(0.0, c, -s),
        DVec3::new(0.0, s, c),
    )
}

/// Coordinate frame rotation about `+Y` by `angle_rad` (`v_new = R_2(angle_rad) * v_old`).
#[must_use]
pub(crate) fn rot_axis_2(angle_rad: f64) -> DMat3 {
    let (s, c) = sin_cos(angle_rad);
    DMat3::from_cols(
        DVec3::new(c, 0.0, s),
        DVec3::new(0.0, 1.0, 0.0),
        DVec3::new(-s, 0.0, c),
    )
}

/// Coordinate frame rotation about `+Z` by `angle_rad` (`v_new = R_3(angle_rad) * v_old`).
#[must_use]
pub(crate) fn rot_axis_3(angle_rad: f64) -> DMat3 {
    let (s, c) = sin_cos(angle_rad);
    DMat3::from_cols(
        DVec3::new(c, -s, 0.0),
        DVec3::new(s, c, 0.0),
        DVec3::new(0.0, 0.0, 1.0),
    )
}

/// Constant coordinate rotation `R_1(+eps_0)` transforming `J2000 (1)` into `ECLIPJ2000 (17)`.
#[must_use]
fn j2000_to_eclipj2000_matrix() -> DMat3 {
    rot_axis_1(SPICE_OBLIQUITY_J2000_RAD)
}
