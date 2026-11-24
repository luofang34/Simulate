//! Orbit Ephemeris Message: time-tagged states, in one or more segments, with
//! optional covariances.

mod kvn;
mod layout;
mod xml;

use crate::covariance::Covariance6;
use crate::error::Result;
use crate::extension::Extension;
use crate::frame::{CenterName, Interpolation, RefFrame};
use crate::header::{Header, Version};
use crate::time_system::TimeSystem;
use crate::unit::{Km, KmPerS, KmPerS2, Vector3};
use crate::{Epoch, ParseMode, WriteOptions};

/// An Orbit Ephemeris Message.
#[derive(Debug, Clone, PartialEq)]
pub struct Oem {
    /// `CCSDS_OEM_VERS`.
    pub version: Version,
    /// The header.
    pub header: Header,
    /// Segments in message order.
    pub segments: Vec<OemSegment>,
    /// Undefined keywords a lenient read kept.
    pub extensions: Vec<Extension>,
}

/// One segment: metadata and the ephemeris it describes.
#[derive(Debug, Clone, PartialEq)]
pub struct OemSegment {
    /// The metadata.
    pub metadata: OemMetadata,
    /// The ephemeris and covariances.
    pub data: OemData,
}

/// OEM segment metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct OemMetadata {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `OBJECT_NAME`.
    pub object_name: String,
    /// `OBJECT_ID`.
    pub object_id: String,
    /// `CENTER_NAME`.
    pub center_name: CenterName,
    /// `REF_FRAME`.
    pub ref_frame: RefFrame,
    /// `REF_FRAME_EPOCH`.
    pub ref_frame_epoch: Option<Epoch>,
    /// `TIME_SYSTEM`.
    pub time_system: TimeSystem,
    /// `START_TIME`.
    pub start_time: Epoch,
    /// `USEABLE_START_TIME`.
    pub useable_start_time: Option<Epoch>,
    /// `USEABLE_STOP_TIME`.
    pub useable_stop_time: Option<Epoch>,
    /// `STOP_TIME`.
    pub stop_time: Epoch,
    /// `INTERPOLATION`.
    pub interpolation: Option<Interpolation>,
    /// `INTERPOLATION_DEGREE`.
    pub interpolation_degree: Option<u32>,
}

/// The ephemeris of one segment.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct OemData {
    /// Comments before the first state.
    pub comments: Vec<String>,
    /// States in message order.
    pub states: Vec<EphemerisState>,
    /// Covariances in message order.
    pub covariances: Vec<EphemerisCovariance>,
}

/// One ephemeris line.
#[derive(Debug, Clone, PartialEq)]
pub struct EphemerisState {
    /// The epoch.
    pub epoch: Epoch,
    /// Position.
    pub position: Vector3<Km>,
    /// Velocity.
    pub velocity: Vector3<KmPerS>,
    /// Acceleration, when the segment carries it.
    pub acceleration: Option<Vector3<KmPerS2>>,
}

/// One covariance of the ephemeris.
#[derive(Debug, Clone, PartialEq)]
pub struct EphemerisCovariance {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `EPOCH`.
    pub epoch: Epoch,
    /// `COV_REF_FRAME`; absent means the metadata `REF_FRAME`.
    pub ref_frame: Option<RefFrame>,
    /// The matrix.
    pub matrix: Covariance6,
}

impl Oem {
    /// Reads a KVN OEM.
    pub fn from_kvn(text: &str, mode: ParseMode) -> Result<Self> {
        kvn::read(text, mode)
    }

    /// Reads an XML OEM (`<oem>` root).
    pub fn from_xml(text: &str, mode: ParseMode) -> Result<Self> {
        xml::read(&crate::xml::tree::parse(text)?, mode)
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
