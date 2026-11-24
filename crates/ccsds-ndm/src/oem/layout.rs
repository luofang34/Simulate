//! OEM blocks shared by both notations: metadata, and the keyword form of
//! states and covariances that XML uses.

use super::{EphemerisCovariance, EphemerisState, OemMetadata};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::covariance;
use crate::error::Result;
use crate::extension::Extension;
use crate::frame::Interpolation;
use crate::state::write_vector;
use crate::{ParseMode, WriteOptions};

/// The `CCSDS_OEM_VERS` keyword.
pub(super) const VERSION_KEY: &str = "CCSDS_OEM_VERS";

pub(super) fn read_metadata(
    block: Block,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<OemMetadata> {
    let (comments, mut fields) = Fields::new(block, mode)?;
    let metadata = OemMetadata {
        comments,
        object_name: fields.required("OBJECT_NAME", Entry::string)?,
        object_id: fields.required("OBJECT_ID", Entry::string)?,
        center_name: fields.required("CENTER_NAME", Entry::center)?,
        ref_frame: fields.required("REF_FRAME", Entry::frame)?,
        ref_frame_epoch: fields.optional("REF_FRAME_EPOCH", Entry::epoch)?,
        time_system: fields.required("TIME_SYSTEM", Entry::time_system)?,
        start_time: fields.required("START_TIME", Entry::epoch)?,
        useable_start_time: fields.optional("USEABLE_START_TIME", Entry::epoch)?,
        useable_stop_time: fields.optional("USEABLE_STOP_TIME", Entry::epoch)?,
        stop_time: fields.required("STOP_TIME", Entry::epoch)?,
        interpolation: fields.optional("INTERPOLATION", |e, _| {
            Ok(Interpolation::from_name(&e.value))
        })?,
        interpolation_degree: fields.optional("INTERPOLATION_DEGREE", Entry::integer)?,
    };
    fields.finish(extensions)?;
    Ok(metadata)
}

pub(super) fn write_metadata(meta: &OemMetadata, options: WriteOptions) -> Block {
    let mut out = BlockWriter::new(&meta.comments, options);
    out.text("OBJECT_NAME", &meta.object_name);
    out.text("OBJECT_ID", &meta.object_id);
    out.text("CENTER_NAME", &meta.center_name);
    out.text("REF_FRAME", &meta.ref_frame);
    out.text_opt("REF_FRAME_EPOCH", meta.ref_frame_epoch.as_ref());
    out.text("TIME_SYSTEM", meta.time_system);
    out.text("START_TIME", &meta.start_time);
    out.text_opt("USEABLE_START_TIME", meta.useable_start_time.as_ref());
    out.text_opt("USEABLE_STOP_TIME", meta.useable_stop_time.as_ref());
    out.text("STOP_TIME", &meta.stop_time);
    out.text_opt("INTERPOLATION", meta.interpolation.as_ref());
    out.integer_opt("INTERPOLATION_DEGREE", meta.interpolation_degree);
    out.finish()
}

const ACCELERATION: [&str; 3] = ["X_DDOT", "Y_DDOT", "Z_DDOT"];

/// A state in keyword form (`<stateVector>`).
pub(super) fn read_state(fields: &mut Fields) -> Result<EphemerisState> {
    let state = crate::state::StateVector::read(Vec::new(), fields)?;
    let acceleration = if ACCELERATION.iter().any(|key| fields.has(key)) {
        Some([
            fields.required("X_DDOT", Entry::quantity)?,
            fields.required("Y_DDOT", Entry::quantity)?,
            fields.required("Z_DDOT", Entry::quantity)?,
        ])
    } else {
        None
    };
    Ok(EphemerisState {
        epoch: state.epoch,
        position: state.position,
        velocity: state.velocity,
        acceleration,
    })
}

pub(super) fn write_state(state: &EphemerisState, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&[], options);
    out.text("EPOCH", &state.epoch);
    write_vector(&mut out, ["X", "Y", "Z"], &state.position)?;
    write_vector(&mut out, ["X_DOT", "Y_DOT", "Z_DOT"], &state.velocity)?;
    if let Some(acceleration) = &state.acceleration {
        write_vector(&mut out, ACCELERATION, acceleration)?;
    }
    Ok(out.finish())
}

/// A covariance in keyword form (`<covarianceMatrix>`).
pub(super) fn read_covariance(
    comments: Vec<String>,
    fields: &mut Fields,
) -> Result<EphemerisCovariance> {
    let epoch = fields.required("EPOCH", Entry::epoch)?;
    let ref_frame = fields.optional("COV_REF_FRAME", Entry::frame)?;
    Ok(EphemerisCovariance {
        comments,
        epoch,
        ref_frame,
        matrix: covariance::read_matrix(fields)?,
    })
}

pub(super) fn write_covariance(
    covariance: &EphemerisCovariance,
    options: WriteOptions,
) -> Result<Block> {
    let mut out = BlockWriter::new(&covariance.comments, options);
    out.text("EPOCH", &covariance.epoch);
    out.text_opt("COV_REF_FRAME", covariance.ref_frame.as_ref());
    covariance::write_matrix(&mut out, &covariance.matrix)?;
    Ok(out.finish())
}
