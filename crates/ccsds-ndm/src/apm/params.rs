//! APM inertia, maneuver, and message validation helpers shared by KVN and XML.

use super::{ApmData, ApmInertia, ApmManeuver};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::{Error, ErrorKind, Location, Result};
use crate::header::Version;
use crate::state::write_vector;
use crate::{ParseMode, WriteOptions};

pub(super) fn read_inertia(comments: Vec<String>, fields: &mut Fields) -> Result<ApmInertia> {
    let ref_frame = fields.optional("INERTIA_REF_FRAME", Entry::frame)?;
    let (kxx, kyy, kzz, kxy, kxz, kyz) = if fields.has_any(&["I11", "I22", "I33"]) {
        ("I11", "I22", "I33", "I12", "I13", "I23")
    } else {
        ("IXX", "IYY", "IZZ", "IXY", "IXZ", "IYZ")
    };
    Ok(ApmInertia {
        comments,
        ref_frame,
        ixx: fields.required(kxx, Entry::quantity)?,
        iyy: fields.required(kyy, Entry::quantity)?,
        izz: fields.required(kzz, Entry::quantity)?,
        ixy: fields.required(kxy, Entry::quantity)?,
        ixz: fields.required(kxz, Entry::quantity)?,
        iyz: fields.required(kyz, Entry::quantity)?,
    })
}

pub(super) fn write_inertia(
    inertia: &ApmInertia,
    version: Version,
    options: WriteOptions,
) -> Result<Block> {
    let mut out = BlockWriter::new(&inertia.comments, options);
    out.text_opt("INERTIA_REF_FRAME", inertia.ref_frame.as_ref());
    let (kxx, kyy, kzz, kxy, kxz, kyz) = if version == Version::V1 {
        ("I11", "I22", "I33", "I12", "I13", "I23")
    } else {
        ("IXX", "IYY", "IZZ", "IXY", "IXZ", "IYZ")
    };
    out.quantity(kxx, inertia.ixx)?;
    out.quantity(kyy, inertia.iyy)?;
    out.quantity(kzz, inertia.izz)?;
    out.quantity(kxy, inertia.ixy)?;
    out.quantity(kxz, inertia.ixz)?;
    out.quantity(kyz, inertia.iyz)?;
    Ok(out.finish())
}

pub(super) fn read_maneuver(comments: Vec<String>, fields: &mut Fields) -> Result<ApmManeuver> {
    let duration = fields.required("MAN_DURATION", Entry::quantity)?;
    if duration.value() < 0.0 {
        return Err(fields.error_for(
            "MAN_DURATION",
            ErrorKind::InvalidValue {
                text: duration.value().to_string(),
                expected: "a non-negative duration",
            },
        ));
    }
    Ok(ApmManeuver {
        comments,
        epoch_start: fields.required("MAN_EPOCH_START", Entry::epoch)?,
        duration,
        ref_frame: fields.required("MAN_REF_FRAME", Entry::frame)?,
        torque: [
            fields.required("MAN_TOR_1", Entry::quantity)?,
            fields.required("MAN_TOR_2", Entry::quantity)?,
            fields.required("MAN_TOR_3", Entry::quantity)?,
        ],
        delta_mass: fields.optional("MAN_DELTA_MASS", Entry::quantity)?,
    })
}

pub(super) fn write_maneuver(man: &ApmManeuver, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&man.comments, options);
    out.text("MAN_EPOCH_START", &man.epoch_start);
    out.quantity("MAN_DURATION", man.duration)?;
    out.text("MAN_REF_FRAME", &man.ref_frame);
    write_vector(
        &mut out,
        ["MAN_TOR_1", "MAN_TOR_2", "MAN_TOR_3"],
        &man.torque,
    )?;
    out.quantity_opt("MAN_DELTA_MASS", man.delta_mass)?;
    Ok(out.finish())
}

pub(super) fn validate_data(
    data: &ApmData,
    version: Version,
    at: Location,
    mode: ParseMode,
) -> Result<()> {
    if mode == ParseMode::Lenient {
        return Ok(());
    }
    let has_state = data.quaternion.is_some()
        || data.euler.is_some()
        || data.ang_vel.is_some()
        || data.spin.is_some();
    if !has_state {
        return Err(Error::new(ErrorKind::MissingKeyword)
            .with_keyword("Q1")
            .at(at));
    }
    let has_epoch = data.epoch.is_some()
        || data.quaternion.as_ref().is_some_and(|q| q.epoch.is_some())
        || data.euler.as_ref().is_some_and(|e| e.epoch.is_some())
        || data.ang_vel.as_ref().is_some_and(|a| a.epoch.is_some())
        || data.spin.as_ref().is_some_and(|s| s.epoch.is_some());
    if !has_epoch || (version == Version::V1 && data.epoch.is_none()) {
        return Err(Error::new(ErrorKind::MissingKeyword)
            .with_keyword("EPOCH")
            .at(at));
    }
    Ok(())
}
