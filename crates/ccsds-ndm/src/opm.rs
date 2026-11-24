//! Orbit Parameter Message: one state, optionally with osculating Keplerian
//! elements, spacecraft parameters, a covariance and planned maneuvers.

mod layout;

use crate::covariance::CovarianceBlock;
use crate::error::Result;
use crate::extension::{Extension, UserDefined};
use crate::frame::{CenterName, RefFrame};
use crate::header::{Header, Version};
use crate::kvn;
use crate::state::{SpacecraftParameters, StateVector};
use crate::time_system::TimeSystem;
use crate::unit::{Deg, Kg, Km, Km3PerS2, KmPerS, Quantity, Seconds, Vector3};
use crate::xml;
use crate::{Epoch, ParseMode, WriteOptions};

pub(crate) use layout::SCHEMA;

/// An Orbit Parameter Message.
#[derive(Debug, Clone, PartialEq)]
pub struct Opm {
    /// `CCSDS_OPM_VERS`.
    pub version: Version,
    /// The header.
    pub header: Header,
    /// The metadata.
    pub metadata: OpmMetadata,
    /// The data blocks.
    pub data: OpmData,
    /// Undefined keywords a lenient read kept.
    pub extensions: Vec<Extension>,
}

/// OPM metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct OpmMetadata {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `OBJECT_NAME`.
    pub object_name: String,
    /// `OBJECT_ID`, normally the international designator.
    pub object_id: String,
    /// `CENTER_NAME`.
    pub center_name: CenterName,
    /// `REF_FRAME`.
    pub ref_frame: RefFrame,
    /// `REF_FRAME_EPOCH`, for frames whose epoch is not intrinsic.
    pub ref_frame_epoch: Option<Epoch>,
    /// `TIME_SYSTEM`.
    pub time_system: TimeSystem,
}

/// OPM data.
#[derive(Debug, Clone, PartialEq)]
pub struct OpmData {
    /// The state vector.
    pub state: StateVector,
    /// Osculating Keplerian elements.
    pub keplerian: Option<KeplerianElements>,
    /// Spacecraft parameters.
    pub spacecraft: Option<SpacecraftParameters>,
    /// The position/velocity covariance.
    pub covariance: Option<CovarianceBlock>,
    /// Maneuvers, in message order.
    pub maneuvers: Vec<Maneuver>,
    /// User-defined parameters.
    pub user_defined: Option<UserDefined>,
}

/// Osculating Keplerian elements.
#[derive(Debug, Clone, PartialEq)]
pub struct KeplerianElements {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `SEMI_MAJOR_AXIS`.
    pub semi_major_axis: Quantity<Km>,
    /// `ECCENTRICITY`.
    pub eccentricity: f64,
    /// `INCLINATION`.
    pub inclination: Quantity<Deg>,
    /// `RA_OF_ASC_NODE`.
    pub ra_of_asc_node: Quantity<Deg>,
    /// `ARG_OF_PERICENTER`.
    pub arg_of_pericenter: Quantity<Deg>,
    /// `TRUE_ANOMALY` or `MEAN_ANOMALY`.
    pub anomaly: Anomaly,
    /// `GM`.
    pub gm: Quantity<Km3PerS2>,
}

/// The anomaly that fixes the position on the orbit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Anomaly {
    /// `TRUE_ANOMALY`.
    True(Quantity<Deg>),
    /// `MEAN_ANOMALY`.
    Mean(Quantity<Deg>),
}

/// One maneuver.
#[derive(Debug, Clone, PartialEq)]
pub struct Maneuver {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `MAN_EPOCH_IGNITION`.
    pub epoch_ignition: Epoch,
    /// `MAN_DURATION`; zero for an impulsive maneuver.
    pub duration: Quantity<Seconds>,
    /// `MAN_DELTA_MASS`, zero or negative.
    pub delta_mass: Quantity<Kg>,
    /// `MAN_REF_FRAME`.
    pub ref_frame: RefFrame,
    /// `MAN_DV_1`, `MAN_DV_2`, `MAN_DV_3`.
    pub delta_v: Vector3<KmPerS>,
}

impl Opm {
    /// Reads a KVN OPM.
    pub fn from_kvn(text: &str, mode: ParseMode) -> Result<Self> {
        let lines = kvn::lexer::lex(text, mode);
        layout::from_flat(kvn::flat::read(lines, &SCHEMA, mode)?, mode)
    }

    /// Reads an XML OPM (`<opm>` root).
    pub fn from_xml(text: &str, mode: ParseMode) -> Result<Self> {
        let root = xml::tree::parse(text)?;
        Self::from_element(&root, mode)
    }

    pub(crate) fn from_element(root: &xml::tree::Element, mode: ParseMode) -> Result<Self> {
        layout::from_flat(xml::flat::read(root, &SCHEMA, mode)?, mode)
    }

    /// Writes the message as KVN.
    pub fn to_kvn(&self, options: WriteOptions) -> Result<String> {
        kvn::flat::write(&layout::to_flat(self, options)?, &SCHEMA)
    }

    /// Writes the message as a standalone XML document.
    pub fn to_xml(&self, options: WriteOptions) -> Result<String> {
        let mut out = xml::writer::XmlWriter::document();
        self.write_element(&mut out, options, true)?;
        Ok(out.finish())
    }

    pub(crate) fn write_element(
        &self,
        out: &mut xml::writer::XmlWriter,
        options: WriteOptions,
        standalone: bool,
    ) -> Result<()> {
        let flat = layout::to_flat(self, options)?;
        let attributes = xml::root_attributes(self.version, standalone);
        xml::flat::write(out, &flat, &SCHEMA, &attributes);
        Ok(())
    }
}
