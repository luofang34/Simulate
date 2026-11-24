//! Shared CDM header, relative-metadata, and state-vector helpers.

use super::{CdmHeader, CdmObject, CdmRelativeMetadata, CdmRelativeStateVector, CdmStateVector};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::{ErrorKind, Result};
use crate::extension::Extension;
use crate::header::Version;
use crate::state::write_vector;
use crate::{ParseMode, WriteOptions};

pub(super) const VERSION_KEY: &str = "CCSDS_CDM_VERS";

pub(super) const HEADER_KEYWORDS: &[&str] =
    &["CREATION_DATE", "ORIGINATOR", "MESSAGE_FOR", "MESSAGE_ID"];

pub(super) const RELATIVE_KEYWORDS: &[&str] = &[
    "TCA",
    "MISS_DISTANCE",
    "RELATIVE_SPEED",
    "RELATIVE_POSITION_R",
    "RELATIVE_POSITION_T",
    "RELATIVE_POSITION_N",
    "RELATIVE_VELOCITY_R",
    "RELATIVE_VELOCITY_T",
    "RELATIVE_VELOCITY_N",
    "START_SCREEN_PERIOD",
    "STOP_SCREEN_PERIOD",
    "SCREEN_VOLUME_FRAME",
    "SCREEN_VOLUME_SHAPE",
    "SCREEN_VOLUME_X",
    "SCREEN_VOLUME_Y",
    "SCREEN_VOLUME_Z",
    "SCREEN_ENTRY_TIME",
    "SCREEN_EXIT_TIME",
    "COLLISION_PROBABILITY",
    "COLLISION_PROBABILITY_METHOD",
];

pub(super) const REL_STATE_KEYS: [&str; 6] = [
    "RELATIVE_POSITION_R",
    "RELATIVE_POSITION_T",
    "RELATIVE_POSITION_N",
    "RELATIVE_VELOCITY_R",
    "RELATIVE_VELOCITY_T",
    "RELATIVE_VELOCITY_N",
];

pub(super) const STATE_VECTOR_KEYWORDS: &[&str] = &["X", "Y", "Z", "X_DOT", "Y_DOT", "Z_DOT"];

pub(super) fn check_cdm_version(entry: &Entry, mode: ParseMode) -> Result<Version> {
    let version = Version::parse(entry, mode)?;
    if version != Version::V1 {
        return Err(entry.error(ErrorKind::UnsupportedVersion {
            text: entry.value.clone(),
        }));
    }
    Ok(version)
}

pub(super) fn read_header(
    block: Block,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<CdmHeader> {
    let (comments, mut fields) = Fields::new(block, mode)?;
    let header = CdmHeader {
        comments,
        creation_date: fields.required("CREATION_DATE", Entry::epoch)?,
        originator: fields.required("ORIGINATOR", Entry::string)?,
        message_for: fields.optional("MESSAGE_FOR", Entry::string)?,
        message_id: fields.required("MESSAGE_ID", Entry::string)?,
    };
    fields.finish(extensions)?;
    Ok(header)
}

pub(super) fn write_header(header: &CdmHeader, options: WriteOptions) -> Block {
    let mut out = BlockWriter::new(&header.comments, options);
    out.text("CREATION_DATE", &header.creation_date);
    out.text("ORIGINATOR", &header.originator);
    out.text_opt("MESSAGE_FOR", header.message_for.as_ref());
    out.text("MESSAGE_ID", &header.message_id);
    out.finish()
}

pub(super) fn read_relative(
    comments: Vec<String>,
    fields: &mut Fields,
    mode: ParseMode,
) -> Result<CdmRelativeMetadata> {
    let tca = fields.required("TCA", Entry::epoch)?;
    let miss_distance = fields.required("MISS_DISTANCE", Entry::quantity)?;
    let relative_speed = fields.optional("RELATIVE_SPEED", Entry::quantity)?;
    let relative_state = if fields.has_any(&REL_STATE_KEYS) {
        Some(CdmRelativeStateVector {
            position: [
                fields.required("RELATIVE_POSITION_R", Entry::quantity)?,
                fields.required("RELATIVE_POSITION_T", Entry::quantity)?,
                fields.required("RELATIVE_POSITION_N", Entry::quantity)?,
            ],
            velocity: [
                fields.required("RELATIVE_VELOCITY_R", Entry::quantity)?,
                fields.required("RELATIVE_VELOCITY_T", Entry::quantity)?,
                fields.required("RELATIVE_VELOCITY_N", Entry::quantity)?,
            ],
        })
    } else {
        None
    };
    let collision_probability = fields.optional("COLLISION_PROBABILITY", Entry::real)?;
    if mode == ParseMode::Strict
        && let Some(p) = collision_probability
        && !(0.0..=1.0).contains(&p)
    {
        return Err(fields.error_for(
            "COLLISION_PROBABILITY",
            ErrorKind::InvalidValue {
                text: p.to_string(),
                expected: "a probability in 0..=1",
            },
        ));
    }
    Ok(CdmRelativeMetadata {
        comments,
        tca,
        miss_distance,
        relative_speed,
        relative_state,
        start_screen_period: fields.optional("START_SCREEN_PERIOD", Entry::epoch)?,
        stop_screen_period: fields.optional("STOP_SCREEN_PERIOD", Entry::epoch)?,
        screen_volume_frame: fields.optional("SCREEN_VOLUME_FRAME", Entry::frame)?,
        screen_volume_shape: fields.optional("SCREEN_VOLUME_SHAPE", Entry::string)?,
        screen_volume_x: fields.optional("SCREEN_VOLUME_X", Entry::quantity)?,
        screen_volume_y: fields.optional("SCREEN_VOLUME_Y", Entry::quantity)?,
        screen_volume_z: fields.optional("SCREEN_VOLUME_Z", Entry::quantity)?,
        screen_entry_time: fields.optional("SCREEN_ENTRY_TIME", Entry::epoch)?,
        screen_exit_time: fields.optional("SCREEN_EXIT_TIME", Entry::epoch)?,
        collision_probability,
        collision_probability_method: fields
            .optional("COLLISION_PROBABILITY_METHOD", Entry::string)?,
    })
}

pub(super) fn write_relative_head(rel: &CdmRelativeMetadata, out: &mut BlockWriter) -> Result<()> {
    out.text("TCA", &rel.tca);
    out.quantity("MISS_DISTANCE", rel.miss_distance)?;
    out.quantity_opt("RELATIVE_SPEED", rel.relative_speed)?;
    Ok(())
}

pub(super) fn write_relative_state(
    state: &CdmRelativeStateVector,
    out: &mut BlockWriter,
) -> Result<()> {
    write_vector(
        out,
        [
            "RELATIVE_POSITION_R",
            "RELATIVE_POSITION_T",
            "RELATIVE_POSITION_N",
        ],
        &state.position,
    )?;
    write_vector(
        out,
        [
            "RELATIVE_VELOCITY_R",
            "RELATIVE_VELOCITY_T",
            "RELATIVE_VELOCITY_N",
        ],
        &state.velocity,
    )
}

pub(super) fn write_relative_tail(rel: &CdmRelativeMetadata, out: &mut BlockWriter) -> Result<()> {
    out.text_opt("START_SCREEN_PERIOD", rel.start_screen_period.as_ref());
    out.text_opt("STOP_SCREEN_PERIOD", rel.stop_screen_period.as_ref());
    out.text_opt("SCREEN_VOLUME_FRAME", rel.screen_volume_frame.as_ref());
    out.text_opt("SCREEN_VOLUME_SHAPE", rel.screen_volume_shape.as_ref());
    out.quantity_opt("SCREEN_VOLUME_X", rel.screen_volume_x)?;
    out.quantity_opt("SCREEN_VOLUME_Y", rel.screen_volume_y)?;
    out.quantity_opt("SCREEN_VOLUME_Z", rel.screen_volume_z)?;
    out.text_opt("SCREEN_ENTRY_TIME", rel.screen_entry_time.as_ref());
    out.text_opt("SCREEN_EXIT_TIME", rel.screen_exit_time.as_ref());
    out.real_opt("COLLISION_PROBABILITY", rel.collision_probability)?;
    out.text_opt(
        "COLLISION_PROBABILITY_METHOD",
        rel.collision_probability_method.as_ref(),
    );
    Ok(())
}

pub(super) fn read_state_vector(
    comments: Vec<String>,
    fields: &mut Fields,
) -> Result<CdmStateVector> {
    Ok(CdmStateVector {
        comments,
        position: [
            fields.required("X", Entry::quantity)?,
            fields.required("Y", Entry::quantity)?,
            fields.required("Z", Entry::quantity)?,
        ],
        velocity: [
            fields.required("X_DOT", Entry::quantity)?,
            fields.required("Y_DOT", Entry::quantity)?,
            fields.required("Z_DOT", Entry::quantity)?,
        ],
    })
}

pub(super) fn write_state_vector(sv: &CdmStateVector, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&sv.comments, options);
    write_vector(&mut out, ["X", "Y", "Z"], &sv.position)?;
    write_vector(&mut out, ["X_DOT", "Y_DOT", "Z_DOT"], &sv.velocity)?;
    Ok(out.finish())
}

pub(super) fn parse_object(entry: &Entry, _: ParseMode) -> Result<CdmObject> {
    CdmObject::from_name(&entry.value).ok_or_else(|| {
        entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "OBJECT1 or OBJECT2",
        })
    })
}
