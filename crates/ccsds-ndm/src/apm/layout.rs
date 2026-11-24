//! Shared APM block readers and writers for both KVN and XML.

use super::{ApmAngularVelocity, ApmEuler, ApmMetadata, ApmQuaternion, ApmSpin};
use crate::adm::{Quaternion, QuaternionDot, parse_direction, parse_euler_seq};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::Result;
use crate::extension::Extension;
use crate::header::Version;
use crate::state::write_vector;
use crate::{ParseMode, WriteOptions};

pub(super) const VERSION_KEY: &str = "CCSDS_APM_VERS";

pub(super) const METADATA_KEYWORDS: &[&str] =
    &["OBJECT_NAME", "OBJECT_ID", "CENTER_NAME", "TIME_SYSTEM"];

pub(super) const QUAT_DISCRIMINATORS: &[&str] = &[
    "Q_FRAME_A",
    "Q_FRAME_B",
    "Q_DIR",
    "Q1",
    "Q2",
    "Q3",
    "QC",
    "Q1_DOT",
    "Q2_DOT",
    "Q3_DOT",
    "QC_DOT",
];

pub(super) const EULER_DISCRIMINATORS: &[&str] = &[
    "EULER_FRAME_A",
    "EULER_FRAME_B",
    "EULER_DIR",
    "EULER_ROT_SEQ",
    "RATE_FRAME",
    "ANGLE_1",
    "ANGLE_2",
    "ANGLE_3",
    "ANGLE_1_DOT",
    "ANGLE_2_DOT",
    "ANGLE_3_DOT",
    "X_ANGLE",
    "Y_ANGLE",
    "Z_ANGLE",
    "X_RATE",
    "Y_RATE",
    "Z_RATE",
];

pub(super) const ANGVEL_DISCRIMINATORS: &[&str] = &[
    "ANGVEL_FRAME_A",
    "ANGVEL_FRAME_B",
    "ANGVEL_FRAME",
    "ANGVEL_X",
    "ANGVEL_Y",
    "ANGVEL_Z",
];

pub(super) const SPIN_DISCRIMINATORS: &[&str] = &[
    "SPIN_FRAME_A",
    "SPIN_FRAME_B",
    "SPIN_DIR",
    "SPIN_ALPHA",
    "SPIN_DELTA",
    "SPIN_ANGLE",
    "SPIN_ANGLE_VEL",
    "NUTATION",
    "NUTATION_PER",
    "NUTATION_PHASE",
    "MOMENTUM_ALPHA",
    "MOMENTUM_DELTA",
    "NUTATION_VEL",
];

pub(super) const INERTIA_KEYWORDS: &[&str] = &[
    "INERTIA_REF_FRAME",
    "IXX",
    "IYY",
    "IZZ",
    "IXY",
    "IXZ",
    "IYZ",
    "I11",
    "I22",
    "I33",
    "I12",
    "I13",
    "I23",
];

pub(super) const MANEUVER_KEYWORDS: &[&str] = &[
    "MAN_EPOCH_START",
    "MAN_DURATION",
    "MAN_REF_FRAME",
    "MAN_TOR_1",
    "MAN_TOR_2",
    "MAN_TOR_3",
    "MAN_DELTA_MASS",
];

pub(super) fn read_metadata(
    block: Block,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<ApmMetadata> {
    let (comments, mut fields) = Fields::new(block, mode)?;
    let metadata = ApmMetadata {
        comments,
        object_name: fields.required("OBJECT_NAME", Entry::string)?,
        object_id: fields.required("OBJECT_ID", Entry::string)?,
        center_name: fields.optional("CENTER_NAME", Entry::center)?,
        time_system: fields.required("TIME_SYSTEM", Entry::time_system)?,
    };
    fields.finish(extensions)?;
    Ok(metadata)
}

pub(super) fn write_metadata(meta: &ApmMetadata, options: WriteOptions) -> Block {
    let mut out = BlockWriter::new(&meta.comments, options);
    out.text("OBJECT_NAME", &meta.object_name);
    out.text("OBJECT_ID", &meta.object_id);
    out.text_opt("CENTER_NAME", meta.center_name.as_ref());
    out.text("TIME_SYSTEM", meta.time_system);
    out.finish()
}

pub(super) fn read_quaternion(comments: Vec<String>, fields: &mut Fields) -> Result<ApmQuaternion> {
    let epoch = fields.optional("EPOCH", Entry::epoch)?;
    let ref_frame_a = if fields.has("Q_FRAME_A") {
        fields.required("Q_FRAME_A", Entry::frame)?
    } else {
        fields.required("REF_FRAME_A", Entry::frame)?
    };
    let ref_frame_b = if fields.has("Q_FRAME_B") {
        fields.required("Q_FRAME_B", Entry::frame)?
    } else {
        fields.required("REF_FRAME_B", Entry::frame)?
    };
    let dir = fields.optional("Q_DIR", parse_direction)?;
    let quaternion = Quaternion {
        q1: fields.required("Q1", Entry::real)?,
        q2: fields.required("Q2", Entry::real)?,
        q3: fields.required("Q3", Entry::real)?,
        qc: fields.required("QC", Entry::real)?,
    };
    let rate_keys = ["Q1_DOT", "Q2_DOT", "Q3_DOT", "QC_DOT"];
    let rate = if fields.has_any(&rate_keys) {
        Some(QuaternionDot {
            q1_dot: fields.required("Q1_DOT", Entry::quantity)?,
            q2_dot: fields.required("Q2_DOT", Entry::quantity)?,
            q3_dot: fields.required("Q3_DOT", Entry::quantity)?,
            qc_dot: fields.required("QC_DOT", Entry::quantity)?,
        })
    } else {
        None
    };
    Ok(ApmQuaternion {
        comments,
        epoch,
        ref_frame_a,
        ref_frame_b,
        dir,
        quaternion,
        rate,
    })
}

pub(super) fn write_quaternion_fields(
    out: &mut BlockWriter,
    q: &ApmQuaternion,
    version: Version,
) -> Result<()> {
    out.text_opt("EPOCH", q.epoch.as_ref());
    let (fa, fb) = if version == Version::V1 {
        ("Q_FRAME_A", "Q_FRAME_B")
    } else {
        ("REF_FRAME_A", "REF_FRAME_B")
    };
    out.text(fa, &q.ref_frame_a);
    out.text(fb, &q.ref_frame_b);
    out.text_opt("Q_DIR", q.dir);
    out.real("Q1", q.quaternion.q1)?;
    out.real("Q2", q.quaternion.q2)?;
    out.real("Q3", q.quaternion.q3)?;
    out.real("QC", q.quaternion.qc)?;
    if let Some(rate) = q.rate {
        out.quantity("Q1_DOT", rate.q1_dot)?;
        out.quantity("Q2_DOT", rate.q2_dot)?;
        out.quantity("Q3_DOT", rate.q3_dot)?;
        out.quantity("QC_DOT", rate.qc_dot)?;
    }
    Ok(())
}

pub(super) fn read_euler(comments: Vec<String>, fields: &mut Fields) -> Result<ApmEuler> {
    let epoch = fields.optional("EPOCH", Entry::epoch)?;
    let ref_frame_a = if fields.has("EULER_FRAME_A") {
        fields.required("EULER_FRAME_A", Entry::frame)?
    } else {
        fields.required("REF_FRAME_A", Entry::frame)?
    };
    let ref_frame_b = if fields.has("EULER_FRAME_B") {
        fields.required("EULER_FRAME_B", Entry::frame)?
    } else {
        fields.required("REF_FRAME_B", Entry::frame)?
    };
    let dir = fields.optional("EULER_DIR", parse_direction)?;
    let rot_seq = fields.required("EULER_ROT_SEQ", parse_euler_seq)?;
    let rate_frame = fields.optional("RATE_FRAME", Entry::frame)?;
    let angles = read_angle_triple(fields)?;
    let rates = read_rate_triple(fields)?;
    Ok(ApmEuler {
        comments,
        epoch,
        ref_frame_a,
        ref_frame_b,
        dir,
        rot_seq,
        rate_frame,
        angles,
        rates,
    })
}

fn read_angle_triple(fields: &mut Fields) -> Result<[crate::unit::Quantity<crate::Deg>; 3]> {
    if fields.has_any(&["X_ANGLE", "Y_ANGLE", "Z_ANGLE"]) {
        Ok([
            fields.required("X_ANGLE", Entry::quantity)?,
            fields.required("Y_ANGLE", Entry::quantity)?,
            fields.required("Z_ANGLE", Entry::quantity)?,
        ])
    } else {
        Ok([
            fields.required("ANGLE_1", Entry::quantity)?,
            fields.required("ANGLE_2", Entry::quantity)?,
            fields.required("ANGLE_3", Entry::quantity)?,
        ])
    }
}

fn read_rate_triple(
    fields: &mut Fields,
) -> Result<Option<[crate::unit::Quantity<crate::DegPerS>; 3]>> {
    if fields.has_any(&["X_RATE", "Y_RATE", "Z_RATE"]) {
        Ok(Some([
            fields.required("X_RATE", Entry::quantity)?,
            fields.required("Y_RATE", Entry::quantity)?,
            fields.required("Z_RATE", Entry::quantity)?,
        ]))
    } else if fields.has_any(&["ANGLE_1_DOT", "ANGLE_2_DOT", "ANGLE_3_DOT"]) {
        Ok(Some([
            fields.required("ANGLE_1_DOT", Entry::quantity)?,
            fields.required("ANGLE_2_DOT", Entry::quantity)?,
            fields.required("ANGLE_3_DOT", Entry::quantity)?,
        ]))
    } else {
        Ok(None)
    }
}

pub(super) fn write_euler_fields(
    out: &mut BlockWriter,
    euler: &ApmEuler,
    version: Version,
) -> Result<()> {
    out.text_opt("EPOCH", euler.epoch.as_ref());
    let (fa, fb) = if version == Version::V1 {
        ("EULER_FRAME_A", "EULER_FRAME_B")
    } else {
        ("REF_FRAME_A", "REF_FRAME_B")
    };
    out.text(fa, &euler.ref_frame_a);
    out.text(fb, &euler.ref_frame_b);
    out.text_opt("EULER_DIR", euler.dir);
    out.text("EULER_ROT_SEQ", &euler.rot_seq);
    out.text_opt("RATE_FRAME", euler.rate_frame.as_ref());
    let angle_keys = if version == Version::V1 {
        ["X_ANGLE", "Y_ANGLE", "Z_ANGLE"]
    } else {
        ["ANGLE_1", "ANGLE_2", "ANGLE_3"]
    };
    write_vector(out, angle_keys, &euler.angles)?;
    if let Some(rates) = &euler.rates {
        let rate_keys = if version == Version::V1 {
            ["X_RATE", "Y_RATE", "Z_RATE"]
        } else {
            ["ANGLE_1_DOT", "ANGLE_2_DOT", "ANGLE_3_DOT"]
        };
        write_vector(out, rate_keys, rates)?;
    }
    Ok(())
}

pub(super) fn read_ang_vel(
    comments: Vec<String>,
    fields: &mut Fields,
) -> Result<ApmAngularVelocity> {
    let epoch = fields.optional("EPOCH", Entry::epoch)?;
    let ref_frame_a = if fields.has("ANGVEL_FRAME_A") {
        fields.optional("ANGVEL_FRAME_A", Entry::frame)?
    } else {
        fields.optional("REF_FRAME_A", Entry::frame)?
    };
    let ref_frame_b = if fields.has("ANGVEL_FRAME_B") {
        fields.optional("ANGVEL_FRAME_B", Entry::frame)?
    } else {
        fields.optional("REF_FRAME_B", Entry::frame)?
    };
    Ok(ApmAngularVelocity {
        comments,
        epoch,
        ref_frame_a,
        ref_frame_b,
        frame: fields.required("ANGVEL_FRAME", Entry::frame)?,
        vector: [
            fields.required("ANGVEL_X", Entry::quantity)?,
            fields.required("ANGVEL_Y", Entry::quantity)?,
            fields.required("ANGVEL_Z", Entry::quantity)?,
        ],
    })
}

pub(super) fn write_ang_vel(
    ang_vel: &ApmAngularVelocity,
    version: Version,
    options: WriteOptions,
) -> Result<Block> {
    let mut out = BlockWriter::new(&ang_vel.comments, options);
    out.text_opt("EPOCH", ang_vel.epoch.as_ref());
    let (fa, fb) = if version == Version::V1 {
        ("ANGVEL_FRAME_A", "ANGVEL_FRAME_B")
    } else {
        ("REF_FRAME_A", "REF_FRAME_B")
    };
    out.text_opt(fa, ang_vel.ref_frame_a.as_ref());
    out.text_opt(fb, ang_vel.ref_frame_b.as_ref());
    out.text("ANGVEL_FRAME", &ang_vel.frame);
    write_vector(
        &mut out,
        ["ANGVEL_X", "ANGVEL_Y", "ANGVEL_Z"],
        &ang_vel.vector,
    )?;
    Ok(out.finish())
}

pub(super) fn read_spin(comments: Vec<String>, fields: &mut Fields) -> Result<ApmSpin> {
    let epoch = fields.optional("EPOCH", Entry::epoch)?;
    let ref_frame_a = if fields.has("SPIN_FRAME_A") {
        fields.required("SPIN_FRAME_A", Entry::frame)?
    } else {
        fields.required("REF_FRAME_A", Entry::frame)?
    };
    let ref_frame_b = if fields.has("SPIN_FRAME_B") {
        fields.required("SPIN_FRAME_B", Entry::frame)?
    } else {
        fields.required("REF_FRAME_B", Entry::frame)?
    };
    Ok(ApmSpin {
        comments,
        epoch,
        ref_frame_a,
        ref_frame_b,
        dir: fields.optional("SPIN_DIR", parse_direction)?,
        spin_alpha: fields.required("SPIN_ALPHA", Entry::quantity)?,
        spin_delta: fields.required("SPIN_DELTA", Entry::quantity)?,
        spin_angle: fields.required("SPIN_ANGLE", Entry::quantity)?,
        spin_angle_vel: fields.required("SPIN_ANGLE_VEL", Entry::quantity)?,
        nutation: fields.optional("NUTATION", Entry::quantity)?,
        nutation_per: fields.optional("NUTATION_PER", Entry::quantity)?,
        nutation_phase: fields.optional("NUTATION_PHASE", Entry::quantity)?,
        momentum_alpha: fields.optional("MOMENTUM_ALPHA", Entry::quantity)?,
        momentum_delta: fields.optional("MOMENTUM_DELTA", Entry::quantity)?,
        nutation_vel: fields.optional("NUTATION_VEL", Entry::quantity)?,
    })
}

pub(super) fn write_spin(spin: &ApmSpin, version: Version, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&spin.comments, options);
    out.text_opt("EPOCH", spin.epoch.as_ref());
    let (fa, fb) = if version == Version::V1 {
        ("SPIN_FRAME_A", "SPIN_FRAME_B")
    } else {
        ("REF_FRAME_A", "REF_FRAME_B")
    };
    out.text(fa, &spin.ref_frame_a);
    out.text(fb, &spin.ref_frame_b);
    out.text_opt("SPIN_DIR", spin.dir);
    out.quantity("SPIN_ALPHA", spin.spin_alpha)?;
    out.quantity("SPIN_DELTA", spin.spin_delta)?;
    out.quantity("SPIN_ANGLE", spin.spin_angle)?;
    out.quantity("SPIN_ANGLE_VEL", spin.spin_angle_vel)?;
    out.quantity_opt("NUTATION", spin.nutation)?;
    out.quantity_opt("NUTATION_PER", spin.nutation_per)?;
    out.quantity_opt("NUTATION_PHASE", spin.nutation_phase)?;
    out.quantity_opt("MOMENTUM_ALPHA", spin.momentum_alpha)?;
    out.quantity_opt("MOMENTUM_DELTA", spin.momentum_delta)?;
    out.quantity_opt("NUTATION_VEL", spin.nutation_vel)?;
    Ok(out.finish())
}

pub(super) use super::params::{
    read_inertia, read_maneuver, validate_data, write_inertia, write_maneuver,
};
