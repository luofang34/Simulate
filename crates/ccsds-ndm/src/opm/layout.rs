//! The OPM's blocks and their mapping to the typed message.

use super::{Anomaly, KeplerianElements, Maneuver, Opm, OpmData, OpmMetadata};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::covariance::{self, CovarianceBlock};
use crate::error::{Error, ErrorKind, Result};
use crate::extension::{self, Extension};
use crate::flat::{FlatMessage, FlatSchema, HEADER, METADATA, Section, optional_block};
use crate::header::{HEADER_KEYWORDS, Header, Version};
use crate::state::{self, SPACECRAFT_KEYWORDS, STATE_KEYWORDS, SpacecraftParameters, StateVector};
use crate::{ParseMode, WriteOptions};

const STATE: usize = 2;
const KEPLERIAN: usize = 3;
const SPACECRAFT: usize = 4;
const COVARIANCE: usize = 5;
const MANEUVER: usize = 6;
const USER: usize = 7;
const EXTENSIONS: usize = 8;

const METADATA_KEYWORDS: &[&str] = &[
    "OBJECT_NAME",
    "OBJECT_ID",
    "CENTER_NAME",
    "REF_FRAME",
    "REF_FRAME_EPOCH",
    "TIME_SYSTEM",
];

const KEPLERIAN_KEYWORDS: &[&str] = &[
    "SEMI_MAJOR_AXIS",
    "ECCENTRICITY",
    "INCLINATION",
    "RA_OF_ASC_NODE",
    "ARG_OF_PERICENTER",
    "TRUE_ANOMALY",
    "MEAN_ANOMALY",
    "GM",
];

const MANEUVER_KEYWORDS: &[&str] = &[
    "MAN_EPOCH_IGNITION",
    "MAN_DURATION",
    "MAN_DELTA_MASS",
    "MAN_REF_FRAME",
    "MAN_DV_1",
    "MAN_DV_2",
    "MAN_DV_3",
];

const fn section(element: &'static str, keywords: &'static [&'static str]) -> Section {
    Section {
        element,
        keywords,
        prefix: None,
        repeats_at: None,
    }
}

/// The OPM layout, in the order of ODM 502.0-B-3 table 3-3.
pub(crate) const SCHEMA: FlatSchema = FlatSchema {
    name: "OPM",
    root: "opm",
    version_key: "CCSDS_OPM_VERS",
    sections: &[
        section("header", HEADER_KEYWORDS),
        section("metadata", METADATA_KEYWORDS),
        section("stateVector", STATE_KEYWORDS),
        section("keplerianElements", KEPLERIAN_KEYWORDS),
        section("spacecraftParameters", SPACECRAFT_KEYWORDS),
        section("covarianceMatrix", &covariance::BLOCK_KEYWORDS),
        Section {
            repeats_at: Some("MAN_EPOCH_IGNITION"),
            ..section("maneuverParameters", MANEUVER_KEYWORDS)
        },
        Section {
            prefix: Some(extension::PREFIX),
            ..section("userDefinedParameters", &[])
        },
    ],
};

pub(super) fn from_flat(mut flat: FlatMessage, mode: ParseMode) -> Result<Opm> {
    let version_entry = flat
        .version
        .take()
        .ok_or_else(|| Error::new(ErrorKind::MissingKeyword).with_keyword(SCHEMA.version_key))?;
    let version = Version::parse(&version_entry, mode)?;
    let start = version_entry.at;
    let mut extensions = Vec::new();
    let header = Header::read(flat.take(HEADER).unwrap_or_default(), mode, &mut extensions)?;
    let metadata = read_metadata(
        flat.take(METADATA).unwrap_or(Block::starting_at(start)),
        mode,
        &mut extensions,
    )?;
    let (comments, mut fields) =
        Fields::new(flat.take(STATE).unwrap_or(Block::starting_at(start)), mode)?;
    let state = StateVector::read(comments, &mut fields)?;
    fields.finish(&mut extensions)?;
    let keplerian = optional_block(&mut flat, KEPLERIAN, mode, &mut extensions, read_keplerian)?;
    let spacecraft = optional_block(&mut flat, SPACECRAFT, mode, &mut extensions, |c, f| {
        SpacecraftParameters::read(c, f)
    })?;
    let covariance = optional_block(&mut flat, COVARIANCE, mode, &mut extensions, |c, f| {
        let (ref_frame, matrix) = CovarianceBlock::read(f)?;
        Ok(CovarianceBlock {
            comments: c,
            ref_frame,
            matrix,
        })
    })?;
    let mut maneuvers = Vec::new();
    for block in flat.take_all(MANEUVER) {
        let (comments, mut fields) = Fields::new(block, mode)?;
        maneuvers.push(read_maneuver(comments, &mut fields)?);
        fields.finish(&mut extensions)?;
    }
    let user_defined = flat
        .take(USER)
        .map(|block| extension::read(block, mode, &mut extensions))
        .transpose()?;
    let data = OpmData {
        state,
        keplerian,
        spacecraft,
        covariance,
        maneuvers,
        user_defined,
    };
    Ok(Opm {
        version,
        header,
        metadata,
        data,
        extensions,
    })
}

fn read_metadata(
    block: Block,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<OpmMetadata> {
    let (comments, mut fields) = Fields::new(block, mode)?;
    let metadata = OpmMetadata {
        comments,
        object_name: fields.required("OBJECT_NAME", Entry::string)?,
        object_id: fields.required("OBJECT_ID", Entry::string)?,
        center_name: fields.required("CENTER_NAME", Entry::center)?,
        ref_frame: fields.required("REF_FRAME", Entry::frame)?,
        ref_frame_epoch: fields.optional("REF_FRAME_EPOCH", Entry::epoch)?,
        time_system: fields.required("TIME_SYSTEM", Entry::time_system)?,
    };
    fields.finish(extensions)?;
    Ok(metadata)
}

fn read_keplerian(comments: Vec<String>, fields: &mut Fields) -> Result<KeplerianElements> {
    let semi_major_axis = fields.required("SEMI_MAJOR_AXIS", Entry::quantity)?;
    let eccentricity = fields.required("ECCENTRICITY", Entry::real)?;
    let inclination = fields.required("INCLINATION", Entry::quantity)?;
    let ra_of_asc_node = fields.required("RA_OF_ASC_NODE", Entry::quantity)?;
    let arg_of_pericenter = fields.required("ARG_OF_PERICENTER", Entry::quantity)?;
    let anomaly = if fields.has("MEAN_ANOMALY") && !fields.has("TRUE_ANOMALY") {
        Anomaly::Mean(fields.required("MEAN_ANOMALY", Entry::quantity)?)
    } else {
        Anomaly::True(fields.required("TRUE_ANOMALY", Entry::quantity)?)
    };
    Ok(KeplerianElements {
        comments,
        semi_major_axis,
        eccentricity,
        inclination,
        ra_of_asc_node,
        arg_of_pericenter,
        anomaly,
        gm: fields.required("GM", Entry::quantity)?,
    })
}

fn read_maneuver(comments: Vec<String>, fields: &mut Fields) -> Result<Maneuver> {
    Ok(Maneuver {
        comments,
        epoch_ignition: fields.required("MAN_EPOCH_IGNITION", Entry::epoch)?,
        duration: fields.required("MAN_DURATION", Entry::quantity)?,
        delta_mass: fields.required("MAN_DELTA_MASS", Entry::quantity)?,
        ref_frame: fields.required("MAN_REF_FRAME", Entry::frame)?,
        delta_v: [
            fields.required("MAN_DV_1", Entry::quantity)?,
            fields.required("MAN_DV_2", Entry::quantity)?,
            fields.required("MAN_DV_3", Entry::quantity)?,
        ],
    })
}

pub(super) fn to_flat(opm: &Opm, options: WriteOptions) -> Result<FlatMessage> {
    let mut blocks = vec![(HEADER, opm.header.write(options))];
    let meta = &opm.metadata;
    let mut out = BlockWriter::new(&meta.comments, options);
    out.text("OBJECT_NAME", &meta.object_name);
    out.text("OBJECT_ID", &meta.object_id);
    out.text("CENTER_NAME", &meta.center_name);
    out.text("REF_FRAME", &meta.ref_frame);
    out.text_opt("REF_FRAME_EPOCH", meta.ref_frame_epoch.as_ref());
    out.text("TIME_SYSTEM", meta.time_system);
    blocks.push((METADATA, out.finish()));
    let data = &opm.data;
    let mut out = BlockWriter::new(&data.state.comments, options);
    data.state.write(&mut out)?;
    blocks.push((STATE, out.finish()));
    if let Some(kepler) = &data.keplerian {
        blocks.push((KEPLERIAN, write_keplerian(kepler, options)?));
    }
    if let Some(spacecraft) = &data.spacecraft {
        let mut out = BlockWriter::new(&spacecraft.comments, options);
        spacecraft.write(&mut out)?;
        blocks.push((SPACECRAFT, out.finish()));
    }
    if let Some(covariance) = &data.covariance {
        let mut out = BlockWriter::new(&covariance.comments, options);
        covariance.write(&mut out)?;
        blocks.push((COVARIANCE, out.finish()));
    }
    for maneuver in &data.maneuvers {
        let mut out = BlockWriter::new(&maneuver.comments, options);
        out.text("MAN_EPOCH_IGNITION", &maneuver.epoch_ignition);
        out.quantity("MAN_DURATION", maneuver.duration)?;
        out.quantity("MAN_DELTA_MASS", maneuver.delta_mass)?;
        out.text("MAN_REF_FRAME", &maneuver.ref_frame);
        state::write_vector(
            &mut out,
            ["MAN_DV_1", "MAN_DV_2", "MAN_DV_3"],
            &maneuver.delta_v,
        )?;
        blocks.push((MANEUVER, out.finish()));
    }
    if let Some(user) = &data.user_defined {
        blocks.push((USER, extension::write(user, options)));
    }
    if !opm.extensions.is_empty() {
        blocks.push((EXTENSIONS, extension::write_extensions(&opm.extensions)));
    }
    Ok(FlatMessage {
        version: Some(Entry::text(
            SCHEMA.version_key,
            opm.version.as_str(),
            crate::Location::new(1, 1),
        )),
        blocks,
    })
}

fn write_keplerian(kepler: &KeplerianElements, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&kepler.comments, options);
    out.quantity("SEMI_MAJOR_AXIS", kepler.semi_major_axis)?;
    out.real("ECCENTRICITY", kepler.eccentricity)?;
    out.quantity("INCLINATION", kepler.inclination)?;
    out.quantity("RA_OF_ASC_NODE", kepler.ra_of_asc_node)?;
    out.quantity("ARG_OF_PERICENTER", kepler.arg_of_pericenter)?;
    match kepler.anomaly {
        Anomaly::True(value) => out.quantity("TRUE_ANOMALY", value)?,
        Anomaly::Mean(value) => out.quantity("MEAN_ANOMALY", value)?,
    }
    out.quantity("GM", kepler.gm)?;
    Ok(out.finish())
}
