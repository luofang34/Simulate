//! Shared Attitude Data Messages (ADM 504.0-B-2 / 504.0-B-1) primitives:
//! quaternions, Euler rotation sequences, attitude direction, and AEM state types.

use std::fmt;

use crate::block::Entry;
use crate::error::{ErrorKind, Result};
use crate::header::Version;
use crate::open_enum::open_enum;
use crate::unit::{PerS, Quantity};
use crate::{ParseMode, number};

/// Attitude quaternion `(Q1, Q2, Q3, QC)` with scalar part `qc` and vector part `(q1, q2, q3)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// First vector component `Q1`.
    pub q1: f64,
    /// Second vector component `Q2`.
    pub q2: f64,
    /// Third vector component `Q3`.
    pub q3: f64,
    /// Scalar (cosine) component `QC`.
    pub qc: f64,
}

impl Quaternion {
    /// Builds a quaternion from `(qc, q1, q2, q3)` (scalar first).
    pub const fn new(qc: f64, q1: f64, q2: f64, q3: f64) -> Self {
        Self { q1, q2, q3, qc }
    }

    /// Builds a quaternion from `[q1, q2, q3, qc]` (scalar last).
    pub const fn from_scalar_last(xyzw: [f64; 4]) -> Self {
        Self {
            q1: xyzw[0],
            q2: xyzw[1],
            q3: xyzw[2],
            qc: xyzw[3],
        }
    }

    /// Components in scalar-last order `[q1, q2, q3, qc]`.
    pub const fn to_scalar_last(self) -> [f64; 4] {
        [self.q1, self.q2, self.q3, self.qc]
    }

    /// Components in scalar-first order `[qc, q1, q2, q3]`.
    pub const fn to_scalar_first(self) -> [f64; 4] {
        [self.qc, self.q1, self.q2, self.q3]
    }
}

/// First time derivative of an attitude quaternion `(Q1_DOT, Q2_DOT, Q3_DOT, QC_DOT)` in `1/s`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuaternionDot {
    /// Derivative of `Q1` (`1/s`).
    pub q1_dot: Quantity<PerS>,
    /// Derivative of `Q2` (`1/s`).
    pub q2_dot: Quantity<PerS>,
    /// Derivative of `Q3` (`1/s`).
    pub q3_dot: Quantity<PerS>,
    /// Derivative of `QC` (`1/s`).
    pub qc_dot: Quantity<PerS>,
}

/// Direction of an attitude rotation (`A2B` or `B2A`) used in ADM 504.0-B-1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AttitudeDirection {
    /// Rotation from `REF_FRAME_A` to `REF_FRAME_B`.
    A2B,
    /// Rotation from `REF_FRAME_B` to `REF_FRAME_A`.
    B2A,
}

impl AttitudeDirection {
    /// Keyword spelling as written in ADM 504.0-B-1.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::A2B => "A2B",
            Self::B2A => "B2A",
        }
    }

    /// Parses `A2B` or `B2A`.
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "A2B" => Some(Self::A2B),
            "B2A" => Some(Self::B2A),
            _ => None,
        }
    }
}

impl fmt::Display for AttitudeDirection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Placement of the scalar component `QC` in AEM 504.0-B-1 quaternion lines (`QUATERNION_TYPE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum QuaternionType {
    /// Scalar component first (`QC Q1 Q2 Q3`).
    First,
    /// Scalar component last (`Q1 Q2 Q3 QC`).
    Last,
}

impl QuaternionType {
    /// Keyword spelling (`FIRST` or `LAST`).
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::First => "FIRST",
            Self::Last => "LAST",
        }
    }

    /// Parses `FIRST` or `LAST`.
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "FIRST" => Some(Self::First),
            "LAST" => Some(Self::Last),
            _ => None,
        }
    }
}

impl fmt::Display for QuaternionType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

open_enum! {
    /// Euler angle rotation sequence (`EULER_ROT_SEQ`), supporting both ADM 504.0-B-1
    /// numeric codes (`121`..`323`) and ADM 504.0-B-2 axis letters (`XYX`..`ZYZ`).
    EulerRotSeq {
        /// Numeric 1-2-1 sequence.
        Seq121 => "121",
        /// Numeric 1-2-3 sequence.
        Seq123 => "123",
        /// Numeric 1-3-1 sequence.
        Seq131 => "131",
        /// Numeric 1-3-2 sequence.
        Seq132 => "132",
        /// Numeric 2-1-2 sequence.
        Seq212 => "212",
        /// Numeric 2-1-3 sequence.
        Seq213 => "213",
        /// Numeric 2-3-1 sequence.
        Seq231 => "231",
        /// Numeric 2-3-2 sequence.
        Seq232 => "232",
        /// Numeric 3-1-2 sequence.
        Seq312 => "312",
        /// Numeric 3-1-3 sequence.
        Seq313 => "313",
        /// Numeric 3-2-1 sequence.
        Seq321 => "321",
        /// Numeric 3-2-3 sequence.
        Seq323 => "323",
        /// Axis-letter X-Y-X sequence.
        Xyx => "XYX",
        /// Axis-letter X-Y-Z sequence.
        Xyz => "XYZ",
        /// Axis-letter X-Z-X sequence.
        Xzx => "XZX",
        /// Axis-letter X-Z-Y sequence.
        Xzy => "XZY",
        /// Axis-letter Y-X-Y sequence.
        Yxy => "YXY",
        /// Axis-letter Y-X-Z sequence.
        Yxz => "YXZ",
        /// Axis-letter Y-Z-X sequence.
        Yzx => "YZX",
        /// Axis-letter Y-Z-Y sequence.
        Yzy => "YZY",
        /// Axis-letter Z-X-Y sequence.
        Zxy => "ZXY",
        /// Axis-letter Z-X-Z sequence.
        Zxz => "ZXZ",
        /// Axis-letter Z-Y-X sequence.
        Zyx => "ZYX",
        /// Axis-letter Z-Y-Z sequence.
        Zyz => "ZYZ",
    }
}

impl EulerRotSeq {
    /// Whether this sequence is one of the 24 standard CCSDS sequences.
    pub const fn is_standard(&self) -> bool {
        !matches!(self, Self::Other(_))
    }
}

/// Attitude parameterization of an AEM segment (`ATTITUDE_TYPE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AttitudeType {
    /// `QUATERNION`: `(Q1, Q2, Q3, QC)`.
    Quaternion,
    /// `QUATERNION/DERIVATIVE`: quaternion plus `(Q1_DOT, Q2_DOT, Q3_DOT, QC_DOT)`.
    QuaternionDerivative,
    /// `QUATERNION/ANGVEL`: quaternion plus `(ANGVEL_X, ANGVEL_Y, ANGVEL_Z)` (ADM 504.0-B-2).
    QuaternionAngVel,
    /// `QUATERNION/RATE`: quaternion plus Euler rates `(X_RATE, Y_RATE, Z_RATE)` (ADM 504.0-B-1).
    QuaternionRate,
    /// `QUATERNION/EULER_RATES`: quaternion plus Euler rates (alias form).
    QuaternionEulerRates,
    /// `EULER_ANGLE`: three Euler angles `(ANGLE_1, ANGLE_2, ANGLE_3)`.
    EulerAngle,
    /// `EULER_ANGLE/DERIVATIVE`: Euler angles plus angle derivatives.
    EulerAngleDerivative,
    /// `EULER_ANGLE/ANGVEL`: Euler angles plus angular velocity (ADM 504.0-B-2).
    EulerAngleAngVel,
    /// `EULER_ANGLE/RATE`: Euler angles plus Euler rates (ADM 504.0-B-1).
    EulerAngleRate,
    /// `SPIN`: `(SPIN_ALPHA, SPIN_DELTA, SPIN_ANGLE, SPIN_ANGLE_VEL)`.
    Spin,
    /// `SPIN/NUTATION`: spin plus `(NUTATION, NUTATION_PER, NUTATION_PHASE)`.
    SpinNutation,
    /// `SPIN/NUTATION_MOM`: spin plus `(MOMENTUM_ALPHA, MOMENTUM_DELTA, NUTATION_VEL)`.
    SpinNutationMom,
}

impl AttitudeType {
    /// Keyword value as written in CCSDS ADM.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Quaternion => "QUATERNION",
            Self::QuaternionDerivative => "QUATERNION/DERIVATIVE",
            Self::QuaternionAngVel => "QUATERNION/ANGVEL",
            Self::QuaternionRate => "QUATERNION/RATE",
            Self::QuaternionEulerRates => "QUATERNION/EULER_RATES",
            Self::EulerAngle => "EULER_ANGLE",
            Self::EulerAngleDerivative => "EULER_ANGLE/DERIVATIVE",
            Self::EulerAngleAngVel => "EULER_ANGLE/ANGVEL",
            Self::EulerAngleRate => "EULER_ANGLE/RATE",
            Self::Spin => "SPIN",
            Self::SpinNutation => "SPIN/NUTATION",
            Self::SpinNutationMom => "SPIN/NUTATION_MOM",
        }
    }

    /// Parses an exact `ATTITUDE_TYPE` value.
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "QUATERNION" => Some(Self::Quaternion),
            "QUATERNION/DERIVATIVE" => Some(Self::QuaternionDerivative),
            "QUATERNION/ANGVEL" => Some(Self::QuaternionAngVel),
            "QUATERNION/RATE" => Some(Self::QuaternionRate),
            "QUATERNION/EULER_RATES" => Some(Self::QuaternionEulerRates),
            "EULER_ANGLE" => Some(Self::EulerAngle),
            "EULER_ANGLE/DERIVATIVE" => Some(Self::EulerAngleDerivative),
            "EULER_ANGLE/ANGVEL" => Some(Self::EulerAngleAngVel),
            "EULER_ANGLE/RATE" => Some(Self::EulerAngleRate),
            "SPIN" => Some(Self::Spin),
            "SPIN/NUTATION" => Some(Self::SpinNutation),
            "SPIN/NUTATION_MOM" => Some(Self::SpinNutationMom),
            _ => None,
        }
    }

    /// Number of numeric data columns after `EPOCH` in KVN ephemeris lines.
    pub const fn column_count(self) -> usize {
        match self {
            Self::Quaternion | Self::Spin => 4,
            Self::EulerAngle => 3,
            Self::EulerAngleDerivative | Self::EulerAngleAngVel | Self::EulerAngleRate => 6,
            Self::QuaternionAngVel
            | Self::QuaternionRate
            | Self::QuaternionEulerRates
            | Self::SpinNutation
            | Self::SpinNutationMom => 7,
            Self::QuaternionDerivative => 8,
        }
    }
}

impl fmt::Display for AttitudeType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

pub(crate) fn check_adm_version(entry: &Entry, mode: ParseMode) -> Result<Version> {
    let version = Version::parse(entry, mode)?;
    if version == Version::V3 {
        return Err(entry.error(ErrorKind::UnsupportedVersion {
            text: entry.value.clone(),
        }));
    }
    Ok(version)
}

pub(crate) fn parse_euler_seq(entry: &Entry, mode: ParseMode) -> Result<EulerRotSeq> {
    let seq = EulerRotSeq::from_name(&entry.value);
    if mode == ParseMode::Strict && !seq.is_standard() {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "a valid 3-axis Euler rotation sequence (e.g., 321 or ZYX)",
        }));
    }
    Ok(seq)
}

pub(crate) fn parse_direction(entry: &Entry, _: ParseMode) -> Result<AttitudeDirection> {
    AttitudeDirection::from_name(&entry.value).ok_or_else(|| {
        entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "A2B or B2A",
        })
    })
}

pub(crate) fn format_num(keyword: &str, value: f64) -> Result<String> {
    number::format_f64(value)
        .ok_or_else(|| crate::error::Error::new(ErrorKind::NonFiniteNumber).with_keyword(keyword))
}
