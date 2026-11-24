//! Shared AEM metadata & state helpers for KVN and XML.

use super::{AemAttitudeState, AemMetadata, AemState};
use crate::adm::{
    AttitudeType, Quaternion, QuaternionType, format_num, parse_direction, parse_euler_seq,
};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::{Error, ErrorKind, Location, Result};
use crate::extension::Extension;
use crate::header::Version;
use crate::{ParseMode, WriteOptions, number};

pub(super) const VERSION_KEY: &str = "CCSDS_AEM_VERS";

pub(super) fn read_metadata(
    block: Block,
    version: Version,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<AemMetadata> {
    let block_at = block.at;
    let (comments, mut fields) = Fields::new(block, mode)?;
    let attitude_type = fields.required("ATTITUDE_TYPE", parse_attitude_type)?;
    let meta = AemMetadata {
        comments,
        object_name: fields.required("OBJECT_NAME", Entry::string)?,
        object_id: fields.required("OBJECT_ID", Entry::string)?,
        center_name: fields.optional("CENTER_NAME", Entry::center)?,
        ref_frame_a: fields.required("REF_FRAME_A", Entry::frame)?,
        ref_frame_b: fields.required("REF_FRAME_B", Entry::frame)?,
        attitude_dir: fields.optional("ATTITUDE_DIR", parse_direction)?,
        time_system: fields.required("TIME_SYSTEM", Entry::time_system)?,
        start_time: fields.required("START_TIME", Entry::epoch)?,
        useable_start_time: fields.optional("USEABLE_START_TIME", Entry::epoch)?,
        useable_stop_time: fields.optional("USEABLE_STOP_TIME", Entry::epoch)?,
        stop_time: fields.required("STOP_TIME", Entry::epoch)?,
        attitude_type,
        quaternion_type: fields.optional("QUATERNION_TYPE", parse_quaternion_type)?,
        euler_rot_seq: fields.optional("EULER_ROT_SEQ", parse_euler_seq)?,
        rate_frame: read_rate_frame(&mut fields)?,
        interpolation_method: fields.optional("INTERPOLATION_METHOD", |e, _| {
            Ok(crate::Interpolation::from_name(&e.value))
        })?,
        interpolation_degree: fields.optional("INTERPOLATION_DEGREE", parse_degree)?,
    };
    fields.finish(extensions)?;
    validate_metadata(&meta, version, block_at, mode)?;
    Ok(meta)
}

fn read_rate_frame(fields: &mut Fields) -> Result<Option<crate::RefFrame>> {
    if fields.has("RATE_FRAME") {
        fields.optional("RATE_FRAME", Entry::frame)
    } else {
        fields.optional("ANGVEL_FRAME", Entry::frame)
    }
}

fn validate_metadata(
    meta: &AemMetadata,
    version: Version,
    at: Option<Location>,
    mode: ParseMode,
) -> Result<()> {
    if mode == ParseMode::Lenient {
        return Ok(());
    }
    if version == Version::V1 && meta.attitude_dir.is_none() {
        return Err(missing("ATTITUDE_DIR", at));
    }
    let uses_quat = matches!(
        meta.attitude_type,
        AttitudeType::Quaternion
            | AttitudeType::QuaternionDerivative
            | AttitudeType::QuaternionAngVel
            | AttitudeType::QuaternionRate
            | AttitudeType::QuaternionEulerRates
    );
    if version == Version::V1 && uses_quat && meta.quaternion_type.is_none() {
        return Err(missing("QUATERNION_TYPE", at));
    }
    let needs_euler_seq = matches!(
        meta.attitude_type,
        AttitudeType::EulerAngle
            | AttitudeType::EulerAngleDerivative
            | AttitudeType::EulerAngleAngVel
            | AttitudeType::EulerAngleRate
            | AttitudeType::QuaternionRate
            | AttitudeType::QuaternionEulerRates
    );
    if needs_euler_seq && meta.euler_rot_seq.is_none() {
        return Err(missing("EULER_ROT_SEQ", at));
    }
    Ok(())
}

fn missing(keyword: &'static str, at: Option<Location>) -> Error {
    let err = Error::new(ErrorKind::MissingKeyword).with_keyword(keyword);
    match at {
        Some(loc) => err.at(loc),
        None => err,
    }
}

pub(super) fn write_metadata(meta: &AemMetadata, version: Version, options: WriteOptions) -> Block {
    let mut out = BlockWriter::new(&meta.comments, options);
    out.text("OBJECT_NAME", &meta.object_name);
    out.text("OBJECT_ID", &meta.object_id);
    out.text_opt("CENTER_NAME", meta.center_name.as_ref());
    out.text("REF_FRAME_A", &meta.ref_frame_a);
    out.text("REF_FRAME_B", &meta.ref_frame_b);
    out.text_opt("ATTITUDE_DIR", meta.attitude_dir);
    out.text("TIME_SYSTEM", meta.time_system);
    out.text("START_TIME", &meta.start_time);
    out.text_opt("USEABLE_START_TIME", meta.useable_start_time.as_ref());
    out.text_opt("USEABLE_STOP_TIME", meta.useable_stop_time.as_ref());
    out.text("STOP_TIME", &meta.stop_time);
    out.text("ATTITUDE_TYPE", meta.attitude_type);
    out.text_opt("QUATERNION_TYPE", meta.quaternion_type);
    out.text_opt("EULER_ROT_SEQ", meta.euler_rot_seq.as_ref());
    if let Some(rf) = &meta.rate_frame {
        let key = if version == Version::V2
            && matches!(
                meta.attitude_type,
                AttitudeType::QuaternionAngVel | AttitudeType::EulerAngleAngVel
            ) {
            "ANGVEL_FRAME"
        } else {
            "RATE_FRAME"
        };
        out.text(key, rf);
    }
    out.text_opt("INTERPOLATION_METHOD", meta.interpolation_method.as_ref());
    out.text_opt("INTERPOLATION_DEGREE", meta.interpolation_degree);
    out.finish()
}

fn parse_attitude_type(entry: &Entry, _: ParseMode) -> Result<AttitudeType> {
    AttitudeType::from_name(&entry.value).ok_or_else(|| {
        entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "a valid CCSDS ATTITUDE_TYPE",
        })
    })
}

fn parse_quaternion_type(entry: &Entry, _: ParseMode) -> Result<QuaternionType> {
    QuaternionType::from_name(&entry.value).ok_or_else(|| {
        entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "FIRST or LAST",
        })
    })
}

fn parse_degree(entry: &Entry, _: ParseMode) -> Result<u32> {
    let deg: u32 = entry.value.trim().parse().map_err(|_| {
        entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "a positive integer interpolation degree",
        })
    })?;
    if deg == 0 {
        return Err(entry.error(ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "an integer >= 1",
        }));
    }
    Ok(deg)
}

pub(super) fn parse_state_columns(
    epoch: crate::Epoch,
    cols: &[f64],
    meta: &AemMetadata,
    at: Option<Location>,
) -> Result<AemState> {
    let q_first = meta.quaternion_type == Some(QuaternionType::First);
    let state = match meta.attitude_type {
        AttitudeType::Quaternion => AemAttitudeState::Quaternion {
            quaternion: col_quat(cols, 0, q_first, at)?,
        },
        AttitudeType::QuaternionDerivative => {
            let dot = col_quat(cols, 4, q_first, at)?;
            AemAttitudeState::QuaternionDerivative {
                quaternion: col_quat(cols, 0, q_first, at)?,
                q_dot: [dot.q1, dot.q2, dot.q3, dot.qc],
            }
        }
        AttitudeType::QuaternionAngVel => AemAttitudeState::QuaternionAngVel {
            quaternion: col_quat(cols, 0, q_first, at)?,
            ang_vel_deg_s: col_v3(cols, 4, at)?,
        },
        AttitudeType::QuaternionRate | AttitudeType::QuaternionEulerRates => {
            AemAttitudeState::QuaternionRate {
                quaternion: col_quat(cols, 0, q_first, at)?,
                rates_deg_s: col_v3(cols, 4, at)?,
            }
        }
        AttitudeType::EulerAngle
        | AttitudeType::EulerAngleDerivative
        | AttitudeType::EulerAngleAngVel
        | AttitudeType::EulerAngleRate => parse_euler_cols(cols, meta.attitude_type, at)?,
        AttitudeType::Spin | AttitudeType::SpinNutation | AttitudeType::SpinNutationMom => {
            parse_spin_cols(cols, meta.attitude_type, at)?
        }
    };
    Ok(AemState { epoch, state })
}

fn col_val(cols: &[f64], idx: usize, at: Option<Location>) -> Result<f64> {
    cols.get(idx).copied().ok_or_else(|| unexpected(at))
}

fn col_v3(cols: &[f64], offset: usize, at: Option<Location>) -> Result<[f64; 3]> {
    Ok([
        col_val(cols, offset, at)?,
        col_val(cols, offset.wrapping_add(1), at)?,
        col_val(cols, offset.wrapping_add(2), at)?,
    ])
}

fn col_quat(
    cols: &[f64],
    offset: usize,
    q_first: bool,
    at: Option<Location>,
) -> Result<Quaternion> {
    let a = col_val(cols, offset, at)?;
    let b = col_val(cols, offset.wrapping_add(1), at)?;
    let c = col_val(cols, offset.wrapping_add(2), at)?;
    let d = col_val(cols, offset.wrapping_add(3), at)?;
    Ok(if q_first {
        Quaternion::new(a, b, c, d)
    } else {
        Quaternion::new(d, a, b, c)
    })
}

fn parse_euler_cols(
    cols: &[f64],
    kind: AttitudeType,
    at: Option<Location>,
) -> Result<AemAttitudeState> {
    let angles_deg = col_v3(cols, 0, at)?;
    Ok(match kind {
        AttitudeType::EulerAngle => AemAttitudeState::EulerAngle { angles_deg },
        AttitudeType::EulerAngleDerivative => AemAttitudeState::EulerAngleDerivative {
            angles_deg,
            derivatives_deg_s: col_v3(cols, 3, at)?,
        },
        AttitudeType::EulerAngleAngVel => AemAttitudeState::EulerAngleAngVel {
            angles_deg,
            ang_vel_deg_s: col_v3(cols, 3, at)?,
        },
        AttitudeType::EulerAngleRate => AemAttitudeState::EulerAngleRate {
            angles_deg,
            rates_deg_s: col_v3(cols, 3, at)?,
        },
        _ => return Err(unexpected(at)),
    })
}

fn parse_spin_cols(
    cols: &[f64],
    kind: AttitudeType,
    at: Option<Location>,
) -> Result<AemAttitudeState> {
    let spin_alpha_deg = col_val(cols, 0, at)?;
    let spin_delta_deg = col_val(cols, 1, at)?;
    let spin_angle_deg = col_val(cols, 2, at)?;
    let spin_angle_vel_deg_s = col_val(cols, 3, at)?;
    Ok(match kind {
        AttitudeType::Spin => AemAttitudeState::Spin {
            spin_alpha_deg,
            spin_delta_deg,
            spin_angle_deg,
            spin_angle_vel_deg_s,
        },
        AttitudeType::SpinNutation => AemAttitudeState::SpinNutation {
            spin_alpha_deg,
            spin_delta_deg,
            spin_angle_deg,
            spin_angle_vel_deg_s,
            nutation_deg: col_val(cols, 4, at)?,
            nutation_per_s: col_val(cols, 5, at)?,
            nutation_phase_deg: col_val(cols, 6, at)?,
        },
        AttitudeType::SpinNutationMom => AemAttitudeState::SpinNutationMom {
            spin_alpha_deg,
            spin_delta_deg,
            spin_angle_deg,
            spin_angle_vel_deg_s,
            momentum_alpha_deg: col_val(cols, 4, at)?,
            momentum_delta_deg: col_val(cols, 5, at)?,
            nutation_vel_deg_s: col_val(cols, 6, at)?,
        },
        _ => return Err(unexpected(at)),
    })
}

fn unexpected(at: Option<Location>) -> Error {
    let err = Error::new(ErrorKind::ColumnCount {
        expected: "columns matching ATTITUDE_TYPE",
        found: 0,
    });
    match at {
        Some(loc) => err.at(loc),
        None => err,
    }
}

pub(super) fn state_columns(
    state: &AemAttitudeState,
    q_type: Option<QuaternionType>,
) -> Result<Vec<String>> {
    let q_first = q_type == Some(QuaternionType::First);
    let mut vals = Vec::with_capacity(8);
    match *state {
        AemAttitudeState::Quaternion { quaternion } => push_quat(&mut vals, &quaternion, q_first),
        AemAttitudeState::QuaternionDerivative { quaternion, q_dot } => {
            push_quat(&mut vals, &quaternion, q_first);
            let [q1, q2, q3, qc] = q_dot;
            let dq = Quaternion { q1, q2, q3, qc };
            push_quat(&mut vals, &dq, q_first);
        }
        AemAttitudeState::QuaternionAngVel {
            quaternion,
            ang_vel_deg_s: w,
        }
        | AemAttitudeState::QuaternionRate {
            quaternion,
            rates_deg_s: w,
        } => {
            push_quat(&mut vals, &quaternion, q_first);
            vals.extend(w);
        }
        AemAttitudeState::EulerAngle { angles_deg: a } => vals.extend(a),
        AemAttitudeState::EulerAngleDerivative {
            angles_deg: a,
            derivatives_deg_s: d,
        }
        | AemAttitudeState::EulerAngleAngVel {
            angles_deg: a,
            ang_vel_deg_s: d,
        }
        | AemAttitudeState::EulerAngleRate {
            angles_deg: a,
            rates_deg_s: d,
        } => {
            vals.extend(a);
            vals.extend(d);
        }
        AemAttitudeState::Spin {
            spin_alpha_deg: a,
            spin_delta_deg: d,
            spin_angle_deg: ph,
            spin_angle_vel_deg_s: w,
        } => vals.extend([a, d, ph, w]),
        AemAttitudeState::SpinNutation {
            spin_alpha_deg: a,
            spin_delta_deg: d,
            spin_angle_deg: ph,
            spin_angle_vel_deg_s: w,
            nutation_deg: n,
            nutation_per_s: p,
            nutation_phase_deg: np,
        } => vals.extend([a, d, ph, w, n, p, np]),
        AemAttitudeState::SpinNutationMom {
            spin_alpha_deg: a,
            spin_delta_deg: d,
            spin_angle_deg: ph,
            spin_angle_vel_deg_s: w,
            momentum_alpha_deg: ma,
            momentum_delta_deg: md,
            nutation_vel_deg_s: nv,
        } => vals.extend([a, d, ph, w, ma, md, nv]),
    }
    vals.into_iter()
        .map(|v| format_num("AEM_STATE", v))
        .collect()
}

fn push_quat(out: &mut Vec<f64>, q: &Quaternion, q_first: bool) {
    if q_first {
        out.extend(q.to_scalar_first());
    } else {
        out.extend(q.to_scalar_last());
    }
}

pub(super) fn parse_real_col(
    token: &str,
    keyword: &'static str,
    at: Location,
    mode: ParseMode,
) -> Result<f64> {
    number::parse_f64(token, mode).ok_or_else(|| {
        Error::new(ErrorKind::InvalidNumber {
            text: token.to_owned(),
        })
        .with_keyword(keyword)
        .at(at)
    })
}
