//! Attitude Parameter Message (ADM 504.0-B-2 / 504.0-B-1): instantaneous
//! attitude state (quaternion, Euler angles, angular velocity, spin), inertia
//! tensor, and attitude maneuvers in KVN and XML.

mod kvn;
mod layout;
mod params;
mod xml;

use crate::adm::{AttitudeDirection, EulerRotSeq, Quaternion, QuaternionDot};
use crate::error::Result;
use crate::extension::{Extension, UserDefined};
use crate::frame::{CenterName, RefFrame};
use crate::header::{Header, Version};
use crate::time_system::TimeSystem;
use crate::unit::{Deg, DegPerS, Kg, KgM2, Nm, Quantity, Seconds, Vector3};
use crate::{Epoch, ParseMode, WriteOptions};

/// An Attitude Parameter Message.
#[derive(Debug, Clone, PartialEq)]
pub struct Apm {
    /// `CCSDS_APM_VERS` (`1.0` or `2.0`).
    pub version: Version,
    /// The header.
    pub header: Header,
    /// The metadata.
    pub metadata: ApmMetadata,
    /// The attitude data blocks.
    pub data: ApmData,
    /// Undefined keywords kept in lenient mode.
    pub extensions: Vec<Extension>,
}

/// APM metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct ApmMetadata {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `OBJECT_NAME`.
    pub object_name: String,
    /// `OBJECT_ID`.
    pub object_id: String,
    /// `CENTER_NAME`.
    pub center_name: Option<CenterName>,
    /// `TIME_SYSTEM`.
    pub time_system: TimeSystem,
}

/// APM data.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ApmData {
    /// Comments at the start of the data section.
    pub comments: Vec<String>,
    /// Message-level `EPOCH` (mandatory in ADM v1, optional in v2 if blocks carry epochs).
    pub epoch: Option<Epoch>,
    /// Quaternion attitude state block.
    pub quaternion: Option<ApmQuaternion>,
    /// Euler angle attitude state block.
    pub euler: Option<ApmEuler>,
    /// Angular velocity vector block.
    pub ang_vel: Option<ApmAngularVelocity>,
    /// Spin-stabilized state block.
    pub spin: Option<ApmSpin>,
    /// Spacecraft inertia tensor block.
    pub inertia: Option<ApmInertia>,
    /// Planned or executed attitude maneuvers, in message order.
    pub maneuvers: Vec<ApmManeuver>,
    /// User-defined parameters.
    pub user_defined: Option<UserDefined>,
}

/// APM quaternion block (`<quaternionState>`).
#[derive(Debug, Clone, PartialEq)]
pub struct ApmQuaternion {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// Block `EPOCH` (ADM v2).
    pub epoch: Option<Epoch>,
    /// `REF_FRAME_A` / `Q_FRAME_A`.
    pub ref_frame_a: RefFrame,
    /// `REF_FRAME_B` / `Q_FRAME_B`.
    pub ref_frame_b: RefFrame,
    /// `Q_DIR` (ADM v1).
    pub dir: Option<AttitudeDirection>,
    /// `Q1`, `Q2`, `Q3`, `QC`.
    pub quaternion: Quaternion,
    /// Optional `Q1_DOT`, `Q2_DOT`, `Q3_DOT`, `QC_DOT` (`1/s`).
    pub rate: Option<QuaternionDot>,
}

/// APM Euler angle block (`<eulerAngleState>` / `<eulerElementsThree>`).
#[derive(Debug, Clone, PartialEq)]
pub struct ApmEuler {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// Block `EPOCH` (ADM v2).
    pub epoch: Option<Epoch>,
    /// `REF_FRAME_A` / `EULER_FRAME_A`.
    pub ref_frame_a: RefFrame,
    /// `REF_FRAME_B` / `EULER_FRAME_B`.
    pub ref_frame_b: RefFrame,
    /// `EULER_DIR` (ADM v1).
    pub dir: Option<AttitudeDirection>,
    /// `EULER_ROT_SEQ`.
    pub rot_seq: EulerRotSeq,
    /// `RATE_FRAME` (ADM v1/v2).
    pub rate_frame: Option<RefFrame>,
    /// `ANGLE_1..3` or `X_ANGLE`/`Y_ANGLE`/`Z_ANGLE` (`deg`).
    pub angles: [Quantity<Deg>; 3],
    /// Optional `ANGLE_1_DOT..3_DOT` or `X_RATE`/`Y_RATE`/`Z_RATE` (`deg/s`).
    pub rates: Option<[Quantity<DegPerS>; 3]>,
}

/// APM angular velocity vector block (`<angVel>`).
#[derive(Debug, Clone, PartialEq)]
pub struct ApmAngularVelocity {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// Block `EPOCH` (ADM v2).
    pub epoch: Option<Epoch>,
    /// `REF_FRAME_A` / `ANGVEL_FRAME_A`.
    pub ref_frame_a: Option<RefFrame>,
    /// `REF_FRAME_B` / `ANGVEL_FRAME_B`.
    pub ref_frame_b: Option<RefFrame>,
    /// `ANGVEL_FRAME`.
    pub frame: RefFrame,
    /// `ANGVEL_X`, `ANGVEL_Y`, `ANGVEL_Z` (`deg/s`).
    pub vector: Vector3<DegPerS>,
}

/// APM spin-stabilized attitude block (`<spin>` / `<eulerElementsSpin>`).
#[derive(Debug, Clone, PartialEq)]
pub struct ApmSpin {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// Block `EPOCH` (ADM v2).
    pub epoch: Option<Epoch>,
    /// `REF_FRAME_A` / `SPIN_FRAME_A`.
    pub ref_frame_a: RefFrame,
    /// `REF_FRAME_B` / `SPIN_FRAME_B`.
    pub ref_frame_b: RefFrame,
    /// `SPIN_DIR` (ADM v1).
    pub dir: Option<AttitudeDirection>,
    /// `SPIN_ALPHA` (`deg`).
    pub spin_alpha: Quantity<Deg>,
    /// `SPIN_DELTA` (`deg`).
    pub spin_delta: Quantity<Deg>,
    /// `SPIN_ANGLE` (`deg`).
    pub spin_angle: Quantity<Deg>,
    /// `SPIN_ANGLE_VEL` (`deg/s`).
    pub spin_angle_vel: Quantity<DegPerS>,
    /// `NUTATION` (`deg`).
    pub nutation: Option<Quantity<Deg>>,
    /// `NUTATION_PER` (`s`).
    pub nutation_per: Option<Quantity<Seconds>>,
    /// `NUTATION_PHASE` (`deg`).
    pub nutation_phase: Option<Quantity<Deg>>,
    /// `MOMENTUM_ALPHA` (`deg`).
    pub momentum_alpha: Option<Quantity<Deg>>,
    /// `MOMENTUM_DELTA` (`deg`).
    pub momentum_delta: Option<Quantity<Deg>>,
    /// `NUTATION_VEL` (`deg/s`).
    pub nutation_vel: Option<Quantity<DegPerS>>,
}

/// APM spacecraft inertia tensor block (`<inertia>` / `<spacecraftParameters>`).
#[derive(Debug, Clone, PartialEq)]
pub struct ApmInertia {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `INERTIA_REF_FRAME`.
    pub ref_frame: Option<RefFrame>,
    /// `IXX` / `I11` (`kg*m**2`).
    pub ixx: Quantity<KgM2>,
    /// `IYY` / `I22` (`kg*m**2`).
    pub iyy: Quantity<KgM2>,
    /// `IZZ` / `I33` (`kg*m**2`).
    pub izz: Quantity<KgM2>,
    /// `IXY` / `I12` (`kg*m**2`).
    pub ixy: Quantity<KgM2>,
    /// `IXZ` / `I13` (`kg*m**2`).
    pub ixz: Quantity<KgM2>,
    /// `IYZ` / `I23` (`kg*m**2`).
    pub iyz: Quantity<KgM2>,
}

/// One attitude maneuver (`<maneuverParameters>`).
#[derive(Debug, Clone, PartialEq)]
pub struct ApmManeuver {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `MAN_EPOCH_START`.
    pub epoch_start: Epoch,
    /// `MAN_DURATION` (`s`).
    pub duration: Quantity<Seconds>,
    /// `MAN_REF_FRAME`.
    pub ref_frame: RefFrame,
    /// `MAN_TOR_1`, `MAN_TOR_2`, `MAN_TOR_3` (`N*m`).
    pub torque: Vector3<Nm>,
    /// Optional `MAN_DELTA_MASS` (`kg`).
    pub delta_mass: Option<Quantity<Kg>>,
}

impl Apm {
    /// Reads a KVN APM.
    pub fn from_kvn(text: &str, mode: ParseMode) -> Result<Self> {
        kvn::read(text, mode)
    }

    /// Reads an XML APM (`<apm>` root).
    pub fn from_xml(text: &str, mode: ParseMode) -> Result<Self> {
        Self::from_element(&crate::xml::tree::parse(text)?, mode)
    }

    pub(crate) fn from_element(root: &crate::xml::tree::Element, mode: ParseMode) -> Result<Self> {
        xml::read(root, mode)
    }

    /// Writes the message as KVN.
    pub fn to_kvn(&self, options: WriteOptions) -> Result<String> {
        kvn::write(self, options)
    }

    /// Writes the message as a standalone XML document.
    pub fn to_xml(&self, options: WriteOptions) -> Result<String> {
        let mut out = crate::xml::writer::XmlWriter::document();
        self.write_element(&mut out, options, true)?;
        Ok(out.finish())
    }

    pub(crate) fn write_element(
        &self,
        out: &mut crate::xml::writer::XmlWriter,
        options: WriteOptions,
        standalone: bool,
    ) -> Result<()> {
        xml::write(self, out, options, standalone)
    }
}
