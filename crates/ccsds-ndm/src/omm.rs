//! Orbit Mean-Elements Message: mean elements of an orbit theory (usually
//! SGP4), with the TLE-derived parameters that theory needs.

pub(crate) mod layout;

use crate::covariance::CovarianceBlock;
use crate::error::{Error, Result};
use crate::extension::{Extension, UserDefined};
use crate::frame::{CenterName, MeanElementTheory, RefFrame};
use crate::header::{Header, Version};
use crate::state::SpacecraftParameters;
use crate::time_system::TimeSystem;
use crate::unit::{
    Deg, Km, Km3PerS2, M2PerKg, PerEarthRadius, Quantity, RevPerDay, RevPerDay2, RevPerDay3,
};
use crate::{Epoch, ParseMode, WriteOptions, kvn, xml};

pub(crate) use layout::SCHEMA;

/// An Orbit Mean-Elements Message.
///
/// Several fields the standard makes mandatory are optional here, because
/// the JSON layout CelesTrak serves omits them (the version, the header, the
/// centre, frame, time system and theory) and its KVN leaves the header
/// blank. Strict KVN and XML reads still require them.
#[derive(Debug, Clone, PartialEq)]
pub struct Omm {
    /// `CCSDS_OMM_VERS`; absent only in JSON.
    pub version: Option<Version>,
    /// The header.
    pub header: Header,
    /// The metadata.
    pub metadata: OmmMetadata,
    /// The data blocks.
    pub data: OmmData,
    /// Undefined keywords a lenient read kept.
    pub extensions: Vec<Extension>,
}

/// OMM metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct OmmMetadata {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `OBJECT_NAME`.
    pub object_name: String,
    /// `OBJECT_ID`, normally the international designator.
    pub object_id: String,
    /// `CENTER_NAME`.
    pub center_name: Option<CenterName>,
    /// `REF_FRAME`.
    pub ref_frame: Option<RefFrame>,
    /// `REF_FRAME_EPOCH`.
    pub ref_frame_epoch: Option<Epoch>,
    /// `TIME_SYSTEM`.
    pub time_system: Option<TimeSystem>,
    /// `MEAN_ELEMENT_THEORY`.
    pub mean_element_theory: Option<MeanElementTheory>,
}

/// OMM data.
#[derive(Debug, Clone, PartialEq)]
pub struct OmmData {
    /// The mean elements.
    pub mean_elements: MeanElements,
    /// Spacecraft parameters.
    pub spacecraft: Option<SpacecraftParameters>,
    /// TLE-related parameters.
    pub tle: Option<TleParameters>,
    /// The position/velocity covariance.
    pub covariance: Option<CovarianceBlock>,
    /// User-defined parameters.
    pub user_defined: Option<UserDefined>,
}

/// Mean Keplerian elements in the message's theory.
#[derive(Debug, Clone, PartialEq)]
pub struct MeanElements {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `EPOCH`.
    pub epoch: Epoch,
    /// `MEAN_MOTION` or `SEMI_MAJOR_AXIS`.
    pub size: OrbitSize,
    /// `ECCENTRICITY`.
    pub eccentricity: f64,
    /// `INCLINATION`.
    pub inclination: Quantity<Deg>,
    /// `RA_OF_ASC_NODE`.
    pub ra_of_asc_node: Quantity<Deg>,
    /// `ARG_OF_PERICENTER`.
    pub arg_of_pericenter: Quantity<Deg>,
    /// `MEAN_ANOMALY`.
    pub mean_anomaly: Quantity<Deg>,
    /// `GM`.
    pub gm: Option<Quantity<Km3PerS2>>,
}

/// How an OMM gives the size of the orbit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OrbitSize {
    /// `MEAN_MOTION`, which SGP4 uses.
    MeanMotion(Quantity<RevPerDay>),
    /// `SEMI_MAJOR_AXIS`, which other theories use.
    SemiMajorAxis(Quantity<Km>),
}

/// TLE-related parameters.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TleParameters {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// `EPHEMERIS_TYPE`.
    pub ephemeris_type: Option<i32>,
    /// `CLASSIFICATION_TYPE`.
    pub classification_type: Option<String>,
    /// `NORAD_CAT_ID`.
    pub norad_cat_id: Option<u32>,
    /// `ELEMENT_SET_NO`.
    pub element_set_no: Option<u32>,
    /// `REV_AT_EPOCH`.
    pub rev_at_epoch: Option<u32>,
    /// `BSTAR` or `BTERM`.
    pub drag: Option<TleDrag>,
    /// `MEAN_MOTION_DOT`.
    pub mean_motion_dot: Option<Quantity<RevPerDay2>>,
    /// `MEAN_MOTION_DDOT` or `AGOM`.
    pub second_term: Option<TleSecondTerm>,
}

/// The drag term: SGP4's BSTAR or SGP4-XP's BTERM.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TleDrag {
    /// `BSTAR`.
    Bstar(Quantity<PerEarthRadius>),
    /// `BTERM`.
    Bterm(Quantity<M2PerKg>),
}

/// SGP4's second derivative of mean motion, or SGP4-XP's solar pressure term.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TleSecondTerm {
    /// `MEAN_MOTION_DDOT`.
    MeanMotionDdot(Quantity<RevPerDay3>),
    /// `AGOM`.
    Agom(Quantity<M2PerKg>),
}

impl Omm {
    /// Reads one KVN OMM.
    pub fn from_kvn(text: &str, mode: ParseMode) -> Result<Self> {
        let lines = kvn::lexer::lex(text, mode);
        layout::from_flat(kvn::flat::read(lines, &SCHEMA, mode)?, mode)
    }

    /// Reads concatenated KVN OMMs, as CelesTrak serves a group.
    pub fn from_kvn_all(text: &str, mode: ParseMode) -> Result<Vec<Self>> {
        let lines = kvn::lexer::lex(text, mode);
        let groups = kvn::lexer::split_messages(lines, SCHEMA.version_key);
        if groups.is_empty() {
            return Err(Error::new(crate::ErrorKind::Empty));
        }
        groups
            .into_iter()
            .enumerate()
            .map(|(index, lines)| {
                kvn::flat::read(lines, &SCHEMA, mode)
                    .and_then(|flat| layout::from_flat(flat, mode))
                    .map_err(|e| e.in_message(index))
            })
            .collect()
    }

    /// Reads an XML OMM (`<omm>` root).
    pub fn from_xml(text: &str, mode: ParseMode) -> Result<Self> {
        Self::from_element(&xml::tree::parse(text)?, mode)
    }

    /// Reads every OMM of an XML document: one `<omm>`, or an `<ndm>`
    /// holding only OMMs, as CelesTrak serves a group.
    pub fn from_xml_all(text: &str, mode: ParseMode) -> Result<Vec<Self>> {
        let root = xml::tree::parse(text)?;
        if root.name != "ndm" {
            return Self::from_element(&root, mode).map(|omm| vec![omm]);
        }
        root.children
            .iter()
            .filter(|child| child.name != "COMMENT")
            .enumerate()
            .map(|(index, child)| Self::from_element(child, mode).map_err(|e| e.in_message(index)))
            .collect()
    }

    pub(crate) fn from_element(root: &xml::tree::Element, mode: ParseMode) -> Result<Self> {
        layout::from_flat(xml::flat::read(root, &SCHEMA, mode)?, mode)
    }

    /// Writes the message as KVN. Fails if the version is absent.
    pub fn to_kvn(&self, options: WriteOptions) -> Result<String> {
        kvn::flat::write(&layout::to_flat(self, options)?, &SCHEMA)
    }

    /// Writes the message as a standalone XML document. Fails if the version
    /// is absent.
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
        let version = layout::require_version(self)?;
        let attributes = xml::root_attributes(version, standalone);
        xml::flat::write(out, &flat, &SCHEMA, &attributes);
        Ok(())
    }
}
