//! Attitude Ephemeris Message (ADM 504.0-B-2 / 504.0-B-1): time-history
//! attitude ephemeris segments in KVN and XML.

mod kvn;
mod layout;
mod xml;
mod xml_write;

use crate::adm::{AttitudeDirection, AttitudeType, EulerRotSeq, Quaternion, QuaternionType};
use crate::error::Result;
use crate::extension::Extension;
use crate::frame::{CenterName, Interpolation, RefFrame};
use crate::header::{Header, Version};
use crate::time_system::TimeSystem;
use crate::{Epoch, ParseMode, WriteOptions, xml as xml_codec};

/// An Attitude Ephemeris Message.
#[derive(Debug, Clone, PartialEq)]
pub struct Aem {
    /// `CCSDS_AEM_VERS` (`1.0` or `2.0`).
    pub version: Version,
    /// The header.
    pub header: Header,
    /// Metadata + attitude ephemeris segments.
    pub segments: Vec<AemSegment>,
    /// Undefined keywords kept in lenient mode.
    pub extensions: Vec<Extension>,
}

/// One metadata/data segment of an [`Aem`].
#[derive(Debug, Clone, PartialEq)]
pub struct AemSegment {
    /// Segment metadata (`META_START` .. `META_STOP`).
    pub metadata: AemMetadata,
    /// Segment ephemeris states (`DATA_START` .. `DATA_STOP`).
    pub data: AemData,
}

/// Metadata for one AEM segment.
#[derive(Debug, Clone, PartialEq)]
pub struct AemMetadata {
    /// Comments inside the metadata block.
    pub comments: Vec<String>,
    /// `OBJECT_NAME`.
    pub object_name: String,
    /// `OBJECT_ID`.
    pub object_id: String,
    /// Optional `CENTER_NAME`.
    pub center_name: Option<CenterName>,
    /// `REF_FRAME_A`.
    pub ref_frame_a: RefFrame,
    /// `REF_FRAME_B`.
    pub ref_frame_b: RefFrame,
    /// `ATTITUDE_DIR` (mandatory in ADM v1, omitted/optional in ADM v2).
    pub attitude_dir: Option<AttitudeDirection>,
    /// `TIME_SYSTEM`.
    pub time_system: TimeSystem,
    /// `START_TIME`.
    pub start_time: Epoch,
    /// Optional `USEABLE_START_TIME`.
    pub useable_start_time: Option<Epoch>,
    /// Optional `USEABLE_STOP_TIME`.
    pub useable_stop_time: Option<Epoch>,
    /// `STOP_TIME`.
    pub stop_time: Epoch,
    /// `ATTITUDE_TYPE`.
    pub attitude_type: AttitudeType,
    /// `QUATERNION_TYPE` (`FIRST` or `LAST`, used in ADM v1 quaternion segments).
    pub quaternion_type: Option<QuaternionType>,
    /// `EULER_ROT_SEQ` (for Euler angle or Euler rate types).
    pub euler_rot_seq: Option<EulerRotSeq>,
    /// `RATE_FRAME` or `ANGVEL_FRAME`.
    pub rate_frame: Option<RefFrame>,
    /// `INTERPOLATION_METHOD`.
    pub interpolation_method: Option<Interpolation>,
    /// `INTERPOLATION_DEGREE`.
    pub interpolation_degree: Option<u32>,
}

/// Ephemeris data section of an [`AemSegment`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AemData {
    /// Comments inside the data section.
    pub comments: Vec<String>,
    /// Time-ordered attitude states.
    pub states: Vec<AemState>,
    /// Optional attitude covariance matrices (`COVARIANCE_START` .. `COVARIANCE_STOP`).
    pub covariances: Vec<AemCovariance>,
}

/// One epoch-stamped attitude ephemeris sample.
#[derive(Debug, Clone, PartialEq)]
pub struct AemState {
    /// Sample epoch.
    pub epoch: Epoch,
    /// Attitude parameters matching `ATTITUDE_TYPE`.
    pub state: AemAttitudeState,
}

/// Attitude sample payload for one epoch of an [`AemData`].
#[derive(Debug, Clone, PartialEq)]
pub enum AemAttitudeState {
    /// `QUATERNION`.
    Quaternion {
        /// Unit attitude quaternion `(q1, q2, q3, qc)`.
        quaternion: Quaternion,
    },
    /// `QUATERNION/DERIVATIVE`.
    QuaternionDerivative {
        /// Unit attitude quaternion.
        quaternion: Quaternion,
        /// Quaternion rates `[q1_dot, q2_dot, q3_dot, qc_dot]` (`1/s`).
        q_dot: [f64; 4],
    },
    /// `QUATERNION/ANGVEL`.
    QuaternionAngVel {
        /// Unit attitude quaternion.
        quaternion: Quaternion,
        /// Angular velocity `[x, y, z]` (`deg/s`).
        ang_vel_deg_s: [f64; 3],
    },
    /// `QUATERNION/RATE` or `QUATERNION/EULER_RATES`.
    QuaternionRate {
        /// Unit attitude quaternion.
        quaternion: Quaternion,
        /// Euler angle rates `[rate_1, rate_2, rate_3]` (`deg/s`).
        rates_deg_s: [f64; 3],
    },
    /// `EULER_ANGLE`.
    EulerAngle {
        /// Euler rotation angles `[angle_1, angle_2, angle_3]` (`deg`).
        angles_deg: [f64; 3],
    },
    /// `EULER_ANGLE/DERIVATIVE`.
    EulerAngleDerivative {
        /// Euler rotation angles `[angle_1, angle_2, angle_3]` (`deg`).
        angles_deg: [f64; 3],
        /// Euler angle derivatives `[d_1, d_2, d_3]` (`deg/s`).
        derivatives_deg_s: [f64; 3],
    },
    /// `EULER_ANGLE/ANGVEL`.
    EulerAngleAngVel {
        /// Euler rotation angles `[angle_1, angle_2, angle_3]` (`deg`).
        angles_deg: [f64; 3],
        /// Angular velocity `[x, y, z]` (`deg/s`).
        ang_vel_deg_s: [f64; 3],
    },
    /// `EULER_ANGLE/RATE`.
    EulerAngleRate {
        /// Euler rotation angles `[angle_1, angle_2, angle_3]` (`deg`).
        angles_deg: [f64; 3],
        /// Euler rotation rates `[r_1, r_2, r_3]` (`deg/s`).
        rates_deg_s: [f64; 3],
    },
    /// `SPIN`.
    Spin {
        /// Right ascension of spin axis (`deg`).
        spin_alpha_deg: f64,
        /// Declination of spin axis (`deg`).
        spin_delta_deg: f64,
        /// Spin phase angle (`deg`).
        spin_angle_deg: f64,
        /// Spin angular velocity (`deg/s`).
        spin_angle_vel_deg_s: f64,
    },
    /// `SPIN/NUTATION`.
    SpinNutation {
        /// Right ascension of spin axis (`deg`).
        spin_alpha_deg: f64,
        /// Declination of spin axis (`deg`).
        spin_delta_deg: f64,
        /// Spin phase angle (`deg`).
        spin_angle_deg: f64,
        /// Spin angular velocity (`deg/s`).
        spin_angle_vel_deg_s: f64,
        /// Nutation angle (`deg`).
        nutation_deg: f64,
        /// Nutation period (`s`).
        nutation_per_s: f64,
        /// Nutation phase (`deg`).
        nutation_phase_deg: f64,
    },
    /// `SPIN/NUTATION_MOM`.
    SpinNutationMom {
        /// Right ascension of spin axis (`deg`).
        spin_alpha_deg: f64,
        /// Declination of spin axis (`deg`).
        spin_delta_deg: f64,
        /// Spin phase angle (`deg`).
        spin_angle_deg: f64,
        /// Spin angular velocity (`deg/s`).
        spin_angle_vel_deg_s: f64,
        /// Angular momentum right ascension (`deg`).
        momentum_alpha_deg: f64,
        /// Angular momentum declination (`deg`).
        momentum_delta_deg: f64,
        /// Nutation angular velocity (`deg/s`).
        nutation_vel_deg_s: f64,
    },
}

/// Optional lower-triangular attitude covariance block inside an AEM segment.
#[derive(Debug, Clone, PartialEq)]
pub struct AemCovariance {
    /// Comments attached to this covariance block.
    pub comments: Vec<String>,
    /// Covariance epoch.
    pub epoch: Option<Epoch>,
    /// Reference frame (`COV_REF_FRAME`).
    pub ref_frame: Option<RefFrame>,
    /// Lower-triangular covariance entries in row-major order (e.g., 6 entries for 3x3 or 21 for 6x6).
    pub lower_triangle: Vec<f64>,
}

impl Aem {
    /// Parses an AEM from KVN text.
    pub fn from_kvn(text: &str, mode: ParseMode) -> Result<Self> {
        kvn::read(text, mode)
    }

    /// Parses an AEM from XML text.
    pub fn from_xml(text: &str, mode: ParseMode) -> Result<Self> {
        let root = xml_codec::tree::parse(text)?;
        Self::from_element(&root, mode)
    }

    pub(crate) fn from_element(root: &xml_codec::tree::Element, mode: ParseMode) -> Result<Self> {
        xml::read(root, mode)
    }

    /// Formats the AEM as KVN text.
    pub fn to_kvn(&self, options: WriteOptions) -> Result<String> {
        kvn::write(self, options)
    }

    /// Formats the AEM as a standalone XML document.
    pub fn to_xml(&self, options: WriteOptions) -> Result<String> {
        let mut out = xml_codec::writer::XmlWriter::document();
        self.write_element(&mut out, options, true)?;
        Ok(out.finish())
    }

    pub(crate) fn write_element(
        &self,
        out: &mut xml_codec::writer::XmlWriter,
        options: WriteOptions,
        standalone: bool,
    ) -> Result<()> {
        xml_write::write(self, out, options, standalone)
    }
}
