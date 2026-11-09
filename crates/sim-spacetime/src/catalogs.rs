//! Spacetime environment catalog traits (`EphemerisCatalog`, `BodyOrientationCatalog`).

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    hash::ContentHash,
    ids::{FrameCode, NaifId},
    kpl::KplError,
    time::{CoverageSet, TdbEpoch, TdbSpanNs},
};

pub use crate::synodic::{
    CollinearLagrangePoint, SynodicError, SynodicFrame, solve_collinear_lagrange_gamma,
};

/// Highest time derivative required and certified for a geometric state query.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum DerivativeOrder {
    /// Geometric position only (`pos_m`).
    Position,
    /// Geometric position and velocity (`pos_m, vel_m_s`).
    Velocity,
    /// Geometric position, velocity, and acceleration (`pos_m, vel_m_s, acc_m_s2`).
    Acceleration,
}

/// Simultaneous geometric state of `target` relative to `center` in `ICRF / J2000` axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicStateIcrf {
    /// Relative position vector in meters (`ICRF / J2000`).
    pub pos_m: DVec3,
    /// Relative coordinate velocity in meters per `TDB` second (`ICRF / J2000`).
    pub vel_m_s: DVec3,
    /// Relative coordinate acceleration in meters per `TDB` second squared (`ICRF / J2000`).
    pub acc_m_s2: DVec3,
}

impl KinematicStateIcrf {
    /// Identical zero state (for `target == center`).
    pub const ZERO: Self = Self {
        pos_m: DVec3::ZERO,
        vel_m_s: DVec3::ZERO,
        acc_m_s2: DVec3::ZERO,
    };

    /// Negates all state derivatives (`center -> target` to `target -> center`).
    #[must_use]
    pub fn negated(self) -> Self {
        Self {
            pos_m: -self.pos_m,
            vel_m_s: -self.vel_m_s,
            acc_m_s2: -self.acc_m_s2,
        }
    }

    /// Component-wise vector sum of two relative states.
    #[must_use]
    pub fn plus(self, rhs: Self) -> Self {
        Self {
            pos_m: self.pos_m + rhs.pos_m,
            vel_m_s: self.vel_m_s + rhs.vel_m_s,
            acc_m_s2: self.acc_m_s2 + rhs.acc_m_s2,
        }
    }
}

/// Policy when Mars body center (`499`) is requested from a catalog that only contains
/// Mars System Barycenter (`4`, such as raw `de440s.bsp` without `mar099s.bsp`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum MarsCenterApproximation {
    /// Require an explicit `499 <-> 4` (or `499 <-> 0`) SPK segment; fail closed otherwise.
    StrictRequire499,
    /// Explicitly approximate Mars body center (`499`) by Mars System Barycenter (`4`),
    /// recording the bounded offset (`~0.20 m` from Phobos + Deimos) in the manifest digest.
    UseBarycenter4 {
        /// Documented upper bound on `||x_499 - x_4||` in meters (e.g. `0.25 m`).
        documented_bound_m: f64,
    },
}

/// Why resolving a geometric ephemeris state failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum EphemerisError {
    /// Failed to parse or evaluate a binary `DAF/SPK` kernel.
    #[error("invalid SPK kernel data: {0}")]
    KernelData(String),
    /// Requested planetary body center (`499`) was absent while its system barycenter (`4`) existed.
    #[error(
        "missing body center {requested}; kernel only provides system barycenter {available_barycenter}"
    )]
    MissingBodyCenter {
        /// Requested planetary body-center ID (e.g. `499`).
        requested: NaifId,
        /// Available planetary system barycenter ID (e.g. `4`).
        available_barycenter: NaifId,
    },
    /// No segment path connected `target` and `center` in the loaded SPK graph.
    #[error("no ephemeris segment chain connects {target} to {center}")]
    UnconnectedBodies {
        /// Requested target body.
        target: NaifId,
        /// Requested center body.
        center: NaifId,
    },
    /// Segment chain existed, but at least one link had no valid segment covering `epoch`.
    #[error(
        "epoch {epoch_j2000_s:.6} s TDB lies outside ephemeris coverage for {target} -> {center}"
    )]
    OutOfCoverage {
        /// Requested target body.
        target: NaifId,
        /// Requested center body.
        center: NaifId,
        /// Requested epoch in `TDB` seconds past J2000.
        epoch_j2000_s: f64,
    },
    /// Observer frame orientation did not match expected `J2000 (1)` in a pure SPK adapter.
    #[error("unsupported observer frame orientation {0} in pure SPK catalog (expected J2000 = 1)")]
    UnsupportedObserverOrientation(i32),
}

/// Geometric planetary state catalog (`ICRF / J2000` axes; never applies light-time or aberration).
pub trait EphemerisCatalog: Send + Sync {
    /// SHA-256 content hash of the loaded segment collection, precedence order, and policies.
    fn content_hash(&self) -> ContentHash;

    /// Exact disjoint valid `TDB` intervals for resolving `target` relative to `center` up to `order`.
    fn coverage(&self, target: NaifId, center: NaifId, order: DerivativeOrder) -> Vec<TdbSpanNs>;

    /// Evaluates simultaneous geometric `(pos_m, vel_m_s, acc_m_s2)` of `target` relative to `center` in `ICRF`.
    fn geometric_state_icrf(
        &self,
        target: NaifId,
        center: NaifId,
        epoch: TdbEpoch,
        order: DerivativeOrder,
    ) -> Result<KinematicStateIcrf, EphemerisError>;

    /// Returns sorted, deduplicated interior Chebyshev granule boundary epochs in `(start, end)`
    /// across all active links between `target` and `center`.
    fn segment_knots_in(
        &self,
        target: NaifId,
        center: NaifId,
        start: TdbEpoch,
        end: TdbEpoch,
    ) -> Vec<TdbEpoch>;
}

/// Accuracy grade of a planetary body-fixed orientation model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SurfaceNavGrade {
    /// Approximate IAU trigonometric series (`IAU_EARTH`, `IAU_MOON`, `IAU_MARS` from `pck00011.tpc`).
    ApproximateIau,
    /// High-precision binary PCK (`ITRF93` for Earth, `MOON_PA_DE440` + `MOON_ME` FK for Moon).
    PrecisionGeodetic,
}

/// Policy when precision binary-PCK coverage is absent for a body at the queried epoch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OrientationPolicy {
    /// Require [`SurfaceNavGrade::PrecisionGeodetic`]; fail closed if outside binary-PCK coverage.
    RequirePrecision,
    /// Allow fallback to [`SurfaceNavGrade::ApproximateIau`], recording the grade in [`BodyRotationState`].
    AllowApproximateIau,
}

/// Simultaneous rotation matrix, angular velocity, and angular acceleration of a body-fixed frame
/// relative to `ICRF / J2000`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyRotationState {
    /// Orthogonal matrix $R_{\text{I}\to\text{F}}$ mapping `ICRF` vectors into body-fixed coordinates.
    pub r_icrf_to_fixed: DMat3,
    /// Angular velocity $\boldsymbol{\omega}_{\text{body}}$ of the body-fixed frame relative to `ICRF`,
    /// expressed in body-fixed coordinates (`rad/s`).
    pub omega_body_rad_s: DVec3,
    /// Angular acceleration $\boldsymbol{\alpha}_{\text{body}} = \dot{\boldsymbol{\omega}}_{\text{body}}$
    /// in body-fixed coordinates (`rad/s^2`).
    pub alpha_body_rad_s2: DVec3,
    /// Concrete frame identifier (`FrameCode::ITRF93`, `MOON_ME`, `IAU_MARS`, etc.).
    pub frame_code: FrameCode,
    /// Certified accuracy grade of this rotation state.
    pub grade: SurfaceNavGrade,
}

impl BodyRotationState {
    /// Transforms `(pos_icrf_m, vel_icrf_m_s)` relative to the body center into body-fixed `(pos_fixed_m, vel_fixed_m_s)`.
    #[must_use]
    pub fn icrf_to_fixed_pv(&self, pos_icrf_m: DVec3, vel_icrf_m_s: DVec3) -> (DVec3, DVec3) {
        let p_f = self.r_icrf_to_fixed * pos_icrf_m;
        let v_f = self.r_icrf_to_fixed * vel_icrf_m_s - self.omega_body_rad_s.cross(p_f);
        (p_f, v_f)
    }

    /// Transforms body-fixed `(pos_fixed_m, vel_fixed_m_s)` into `ICRF` `(pos_icrf_m, vel_icrf_m_s)` relative to the body center.
    #[must_use]
    pub fn fixed_to_icrf_pv(&self, pos_fixed_m: DVec3, vel_fixed_m_s: DVec3) -> (DVec3, DVec3) {
        let r_f_to_i = self.r_icrf_to_fixed.transpose();
        let p_i = r_f_to_i * pos_fixed_m;
        let v_i = r_f_to_i * (vel_fixed_m_s + self.omega_body_rad_s.cross(pos_fixed_m));
        (p_i, v_i)
    }

    /// Transforms body-fixed `(pos, vel, acc)` into `ICRF` `(pos, vel, acc)` including Euler, Coriolis, and centrifugal terms.
    #[must_use]
    pub fn fixed_to_icrf_pva(
        &self,
        pos_fixed_m: DVec3,
        vel_fixed_m_s: DVec3,
        acc_fixed_m_s2: DVec3,
    ) -> (DVec3, DVec3, DVec3) {
        let r_f_to_i = self.r_icrf_to_fixed.transpose();
        let w = self.omega_body_rad_s;
        let w_cross_r = w.cross(pos_fixed_m);
        let p_i = r_f_to_i * pos_fixed_m;
        let v_i = r_f_to_i * (vel_fixed_m_s + w_cross_r);
        let a_f_tot = acc_fixed_m_s2
            + 2.0 * w.cross(vel_fixed_m_s)
            + self.alpha_body_rad_s2.cross(pos_fixed_m)
            + w.cross(w_cross_r);
        let a_i = r_f_to_i * a_f_tot;
        (p_i, v_i, a_i)
    }
}

/// Errors produced when loading or evaluating planetary orientation models.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum OrientationError {
    /// Failed to parse a text `PCK` or `FK` kernel.
    #[error("text PCK/FK error: {0}")]
    Kpl(#[from] KplError),
    /// Failed to parse or evaluate a binary `PCK` (`.bpc`) file.
    #[error("binary PCK error: {0}")]
    KernelData(String),
    /// Requested body had no orientation model in the catalog.
    #[error("no orientation model available for body {0}")]
    MissingBodyModel(NaifId),
    /// Requested epoch fell outside precision binary-PCK coverage under [`OrientationPolicy::RequirePrecision`].
    #[error(
        "epoch {epoch_j2000_s:.6} s TDB is outside precision orientation coverage for body {body}"
    )]
    OutOfPrecisionCoverage {
        /// Target body.
        body: NaifId,
        /// Requested epoch in `TDB` seconds past J2000.
        epoch_j2000_s: f64,
    },
    /// Malformed lunar fixed-offset frame kernel (`TKFRAME_*`).
    #[error("malformed frame kernel specification: {0}")]
    MalformedFrameKernel(String),
}

/// Catalog of planetary body-fixed orientation frames (`ICRF -> body-fixed`).
pub trait BodyOrientationCatalog: Send + Sync {
    /// SHA-256 content hash of all loaded orientation kernels and policies.
    fn content_hash(&self) -> ContentHash;

    /// Disjoint valid `TDB` intervals for `body` meeting or exceeding `min_grade`.
    fn coverage(&self, body: NaifId, min_grade: SurfaceNavGrade) -> Vec<TdbSpanNs>;

    /// Evaluates the body-fixed rotation state of `body` at `epoch`.
    fn body_rotation(
        &self,
        body: NaifId,
        epoch: TdbEpoch,
    ) -> Result<BodyRotationState, OrientationError>;
}

/// Default NAIF `IAU_<BODY>` class-2 frame code (`10010` Sun, `10013` Earth, `10014` Mars, `10020` Moon).
#[must_use]
pub const fn default_iau_frame_code(body: NaifId) -> FrameCode {
    match body.0 {
        10 => FrameCode(10_010),
        199 => FrameCode(10_011),
        299 => FrameCode(10_012),
        399 => FrameCode(10_013),
        499 => FrameCode(10_014),
        599 => FrameCode(10_015),
        699 => FrameCode(10_016),
        799 => FrameCode(10_017),
        899 => FrameCode(10_018),
        999 => FrameCode(10_019),
        301 => FrameCode(10_020),
        _ => FrameCode(10_000_i32.saturating_add(body.0)),
    }
}

/// Helper for computing coverage spans for a requested [`SurfaceNavGrade`].
#[must_use]
pub fn coverage_for_grade(
    precision_cov: &CoverageSet,
    has_approximate_iau: bool,
    min_grade: SurfaceNavGrade,
) -> Vec<TdbSpanNs> {
    match min_grade {
        SurfaceNavGrade::PrecisionGeodetic => precision_cov.spans().to_vec(),
        SurfaceNavGrade::ApproximateIau => {
            if has_approximate_iau {
                CoverageSet::unbounded().spans().to_vec()
            } else {
                precision_cov.spans().to_vec()
            }
        }
    }
}
