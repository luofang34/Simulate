//! The OMM's blocks and their mapping to the typed message.

use super::{
    MeanElements, Omm, OmmData, OmmMetadata, OrbitSize, TleDrag, TleParameters, TleSecondTerm,
};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::covariance::{self, CovarianceBlock};
use crate::error::{Error, ErrorKind, Result};
use crate::extension::{self, Extension};
use crate::flat::{FlatMessage, FlatSchema, HEADER, METADATA, Section, optional_block};
use crate::header::{HEADER_KEYWORDS, Header, Version};
use crate::state::{SPACECRAFT_KEYWORDS, SpacecraftParameters};
use crate::{Location, ParseMode, WriteOptions};

const MEAN: usize = 2;
const SPACECRAFT: usize = 3;
const TLE: usize = 4;
const COVARIANCE: usize = 5;
const USER: usize = 6;
const EXTENSIONS: usize = 7;

const METADATA_KEYWORDS: &[&str] = &[
    "OBJECT_NAME",
    "OBJECT_ID",
    "CENTER_NAME",
    "REF_FRAME",
    "REF_FRAME_EPOCH",
    "TIME_SYSTEM",
    "MEAN_ELEMENT_THEORY",
];

const MEAN_KEYWORDS: &[&str] = &[
    "EPOCH",
    "SEMI_MAJOR_AXIS",
    "MEAN_MOTION",
    "ECCENTRICITY",
    "INCLINATION",
    "RA_OF_ASC_NODE",
    "ARG_OF_PERICENTER",
    "MEAN_ANOMALY",
    "GM",
];

const TLE_KEYWORDS: &[&str] = &[
    "EPHEMERIS_TYPE",
    "CLASSIFICATION_TYPE",
    "NORAD_CAT_ID",
    "ELEMENT_SET_NO",
    "REV_AT_EPOCH",
    "BSTAR",
    "BTERM",
    "MEAN_MOTION_DOT",
    "MEAN_MOTION_DDOT",
    "AGOM",
];

const fn section(element: &'static str, keywords: &'static [&'static str]) -> Section {
    Section {
        element,
        keywords,
        prefix: None,
        repeats_at: None,
    }
}

/// The OMM layout, in the order of ODM 502.0-B-3 table 4-3.
pub(crate) const SCHEMA: FlatSchema = FlatSchema {
    name: "OMM",
    root: "omm",
    version_key: "CCSDS_OMM_VERS",
    sections: &[
        section("header", HEADER_KEYWORDS),
        section("metadata", METADATA_KEYWORDS),
        section("meanElements", MEAN_KEYWORDS),
        section("spacecraftParameters", SPACECRAFT_KEYWORDS),
        section("tleParameters", TLE_KEYWORDS),
        section("covarianceMatrix", &covariance::BLOCK_KEYWORDS),
        Section {
            prefix: Some(extension::PREFIX),
            ..section("userDefinedParameters", &[])
        },
    ],
};

pub(crate) fn from_flat(mut flat: FlatMessage, mode: ParseMode) -> Result<Omm> {
    let version = flat
        .version
        .take()
        .map(|entry| Version::parse(&entry, mode).map(|v| (v, entry.at)))
        .transpose()?;
    let start = version.and_then(|(_, at)| at);
    let mut extensions = Vec::new();
    let header = Header::read(flat.take(HEADER).unwrap_or_default(), mode, &mut extensions)?;
    let metadata = read_metadata(
        flat.take(METADATA).unwrap_or(Block::starting_at(start)),
        mode,
        &mut extensions,
    )?;
    let (comments, mut fields) =
        Fields::new(flat.take(MEAN).unwrap_or(Block::starting_at(start)), mode)?;
    let mean_elements = read_mean(comments, &mut fields)?;
    fields.finish(&mut extensions)?;
    let spacecraft = optional_block(&mut flat, SPACECRAFT, mode, &mut extensions, |c, f| {
        SpacecraftParameters::read(c, f)
    })?;
    let tle = optional_block(&mut flat, TLE, mode, &mut extensions, read_tle)?;
    let covariance = optional_block(&mut flat, COVARIANCE, mode, &mut extensions, |c, f| {
        let (ref_frame, matrix) = CovarianceBlock::read(f)?;
        Ok(CovarianceBlock {
            comments: c,
            ref_frame,
            matrix,
        })
    })?;
    let user_defined = flat
        .take(USER)
        .map(|block| extension::read(block, mode, &mut extensions))
        .transpose()?;
    for block in flat.take_all(EXTENSIONS) {
        Fields::new(block, mode)?.1.finish(&mut extensions)?;
    }
    Ok(Omm {
        version: version.map(|(v, _)| v),
        header,
        metadata,
        data: OmmData {
            mean_elements,
            spacecraft,
            tle,
            covariance,
            user_defined,
        },
        extensions,
    })
}

fn read_metadata(
    block: Block,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<OmmMetadata> {
    let (comments, mut fields) = Fields::new(block, mode)?;
    let metadata = OmmMetadata {
        comments,
        object_name: fields.required("OBJECT_NAME", Entry::string)?,
        object_id: fields.required("OBJECT_ID", Entry::string)?,
        center_name: fields.expected("CENTER_NAME", Entry::center)?,
        ref_frame: fields.expected("REF_FRAME", Entry::frame)?,
        ref_frame_epoch: fields.optional("REF_FRAME_EPOCH", Entry::epoch)?,
        time_system: fields.expected("TIME_SYSTEM", Entry::time_system)?,
        mean_element_theory: fields.expected("MEAN_ELEMENT_THEORY", |e, _| {
            Ok(crate::MeanElementTheory::from_name(&e.value))
        })?,
    };
    fields.finish(extensions)?;
    Ok(metadata)
}

/// Reads one of two alternative keywords; both present is an error.
fn either<A, B>(
    fields: &mut Fields,
    first: &'static str,
    read_first: impl FnOnce(&Entry, ParseMode) -> Result<A>,
    second: &'static str,
    read_second: impl FnOnce(&Entry, ParseMode) -> Result<B>,
) -> Result<Option<Result<A, B>>> {
    if fields.has(first) && fields.has(second) {
        return Err(fields.error_for(second, ErrorKind::DuplicateKeyword));
    }
    if fields.has(second) {
        return Ok(fields.optional(second, read_second)?.map(Err));
    }
    Ok(fields.optional(first, read_first)?.map(Ok))
}

fn read_mean(comments: Vec<String>, fields: &mut Fields) -> Result<MeanElements> {
    let epoch = fields.required("EPOCH", Entry::epoch)?;
    let size = match either(
        fields,
        "MEAN_MOTION",
        Entry::quantity,
        "SEMI_MAJOR_AXIS",
        Entry::quantity,
    )? {
        Some(Ok(motion)) => OrbitSize::MeanMotion(motion),
        Some(Err(axis)) => OrbitSize::SemiMajorAxis(axis),
        None => return Err(fields.missing("MEAN_MOTION")),
    };
    Ok(MeanElements {
        comments,
        epoch,
        size,
        eccentricity: fields.required("ECCENTRICITY", Entry::real)?,
        inclination: fields.required("INCLINATION", Entry::quantity)?,
        ra_of_asc_node: fields.required("RA_OF_ASC_NODE", Entry::quantity)?,
        arg_of_pericenter: fields.required("ARG_OF_PERICENTER", Entry::quantity)?,
        mean_anomaly: fields.required("MEAN_ANOMALY", Entry::quantity)?,
        gm: fields.optional("GM", Entry::quantity)?,
    })
}

fn read_tle(comments: Vec<String>, fields: &mut Fields) -> Result<TleParameters> {
    let ephemeris_type = fields.optional("EPHEMERIS_TYPE", Entry::integer)?;
    let classification_type = fields.optional("CLASSIFICATION_TYPE", Entry::string)?;
    let norad_cat_id = fields.optional("NORAD_CAT_ID", Entry::integer)?;
    let element_set_no = fields.optional("ELEMENT_SET_NO", Entry::integer)?;
    let rev_at_epoch = fields.optional("REV_AT_EPOCH", Entry::integer)?;
    let drag = either(fields, "BSTAR", Entry::quantity, "BTERM", Entry::quantity)?
        .map(|term| term.map_or_else(TleDrag::Bterm, TleDrag::Bstar));
    let mean_motion_dot = fields.optional("MEAN_MOTION_DOT", Entry::quantity)?;
    let second_term = either(
        fields,
        "MEAN_MOTION_DDOT",
        Entry::quantity,
        "AGOM",
        Entry::quantity,
    )?
    .map(|term| term.map_or_else(TleSecondTerm::Agom, TleSecondTerm::MeanMotionDdot));
    Ok(TleParameters {
        comments,
        ephemeris_type,
        classification_type,
        norad_cat_id,
        element_set_no,
        rev_at_epoch,
        drag,
        mean_motion_dot,
        second_term,
    })
}

/// The version, which KVN and XML cannot do without.
pub(crate) fn require_version(omm: &Omm) -> Result<Version> {
    omm.version
        .ok_or_else(|| Error::new(ErrorKind::MissingKeyword).with_keyword(SCHEMA.version_key))
}

pub(crate) fn to_flat(omm: &Omm, options: WriteOptions) -> Result<FlatMessage> {
    let version = require_version(omm)?;
    Ok(FlatMessage {
        version: Some(Entry::text(
            SCHEMA.version_key,
            version.as_str(),
            Location::new(1, 1),
        )),
        blocks: to_blocks(omm, options)?,
    })
}

/// Every block of the message, tagged with its section.
pub(crate) fn to_blocks(omm: &Omm, options: WriteOptions) -> Result<Vec<(usize, Block)>> {
    let mut blocks = vec![(HEADER, omm.header.write(options))];
    let meta = &omm.metadata;
    let mut out = BlockWriter::new(&meta.comments, options);
    out.text("OBJECT_NAME", &meta.object_name);
    out.text("OBJECT_ID", &meta.object_id);
    out.text_opt("CENTER_NAME", meta.center_name.as_ref());
    out.text_opt("REF_FRAME", meta.ref_frame.as_ref());
    out.text_opt("REF_FRAME_EPOCH", meta.ref_frame_epoch.as_ref());
    out.text_opt("TIME_SYSTEM", meta.time_system);
    out.text_opt("MEAN_ELEMENT_THEORY", meta.mean_element_theory.as_ref());
    blocks.push((METADATA, out.finish()));
    let data = &omm.data;
    blocks.push((MEAN, write_mean(&data.mean_elements, options)?));
    if let Some(spacecraft) = &data.spacecraft {
        let mut out = BlockWriter::new(&spacecraft.comments, options);
        spacecraft.write(&mut out)?;
        blocks.push((SPACECRAFT, out.finish()));
    }
    if let Some(tle) = &data.tle {
        blocks.push((TLE, write_tle(tle, options)?));
    }
    if let Some(covariance) = &data.covariance {
        let mut out = BlockWriter::new(&covariance.comments, options);
        covariance.write(&mut out)?;
        blocks.push((COVARIANCE, out.finish()));
    }
    if let Some(user) = &data.user_defined {
        blocks.push((USER, extension::write(user, options)));
    }
    if !omm.extensions.is_empty() {
        blocks.push((EXTENSIONS, extension::write_extensions(&omm.extensions)));
    }
    Ok(blocks)
}

fn write_mean(mean: &MeanElements, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&mean.comments, options);
    out.text("EPOCH", &mean.epoch);
    match mean.size {
        OrbitSize::MeanMotion(motion) => out.quantity("MEAN_MOTION", motion)?,
        OrbitSize::SemiMajorAxis(axis) => out.quantity("SEMI_MAJOR_AXIS", axis)?,
    }
    out.real("ECCENTRICITY", mean.eccentricity)?;
    out.quantity("INCLINATION", mean.inclination)?;
    out.quantity("RA_OF_ASC_NODE", mean.ra_of_asc_node)?;
    out.quantity("ARG_OF_PERICENTER", mean.arg_of_pericenter)?;
    out.quantity("MEAN_ANOMALY", mean.mean_anomaly)?;
    out.quantity_opt("GM", mean.gm)?;
    Ok(out.finish())
}

fn write_tle(tle: &TleParameters, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&tle.comments, options);
    out.integer_opt("EPHEMERIS_TYPE", tle.ephemeris_type);
    out.text_opt("CLASSIFICATION_TYPE", tle.classification_type.as_ref());
    out.integer_opt("NORAD_CAT_ID", tle.norad_cat_id);
    out.integer_opt("ELEMENT_SET_NO", tle.element_set_no);
    out.integer_opt("REV_AT_EPOCH", tle.rev_at_epoch);
    match tle.drag {
        Some(TleDrag::Bstar(value)) => out.quantity("BSTAR", value)?,
        Some(TleDrag::Bterm(value)) => out.quantity("BTERM", value)?,
        None => {}
    }
    out.quantity_opt("MEAN_MOTION_DOT", tle.mean_motion_dot)?;
    match tle.second_term {
        Some(TleSecondTerm::MeanMotionDdot(value)) => out.quantity("MEAN_MOTION_DDOT", value)?,
        Some(TleSecondTerm::Agom(value)) => out.quantity("AGOM", value)?,
        None => {}
    }
    Ok(out.finish())
}
