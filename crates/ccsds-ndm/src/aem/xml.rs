//! AEM XML reader and writer (`<aem>`).

use super::layout::{self, VERSION_KEY};
use super::{Aem, AemAttitudeState, AemCovariance, AemData, AemMetadata, AemSegment, AemState};
use crate::adm::{AttitudeType, Quaternion, check_adm_version};
use crate::block::{Block, Entry, Fields};
use crate::error::{ErrorKind, Result};
use crate::extension::{Extension, ExtensionValue};
use crate::header::Version;
use crate::xml::flat::version_entry;
use crate::xml::tree::Element;
use crate::{ParseMode, header::Header};

pub(super) fn read(root: &Element, mode: ParseMode) -> Result<Aem> {
    if root.name != "aem" {
        return Err(root.error(ErrorKind::WrongMessage {
            expected: "AEM",
            found: root.name.clone(),
        }));
    }
    let version = check_adm_version(&version_entry(root, VERSION_KEY, mode)?, mode)?;
    let mut extensions = Vec::new();
    let header = Header::read(
        root.required_child("header")?.block()?,
        mode,
        &mut extensions,
    )?;
    let body = root.required_child("body")?;
    let mut segments = Vec::new();
    for child in &body.children {
        if child.name == "segment" {
            segments.push(read_segment(child, version, mode, &mut extensions)?);
        } else if mode == ParseMode::Lenient && child.children.is_empty() {
            let e = child.entry(child.name.clone());
            extensions.push(Extension {
                keyword: e.keyword,
                value: ExtensionValue::Text(e.value),
                unit: e.unit,
            });
        } else {
            return Err(child.unexpected());
        }
    }
    if mode == ParseMode::Strict && segments.is_empty() {
        return Err(body.error(ErrorKind::UnexpectedEnd {
            expected: "segment",
        }));
    }
    Ok(Aem {
        version,
        header,
        segments,
        extensions,
    })
}

fn read_segment(
    seg: &Element,
    version: Version,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<AemSegment> {
    let metadata = layout::read_metadata(
        seg.required_child("metadata")?.block()?,
        version,
        mode,
        extensions,
    )?;
    let data = read_data(seg.required_child("data")?, &metadata, mode, extensions)?;
    Ok(AemSegment { metadata, data })
}

fn read_data(
    el: &Element,
    meta: &AemMetadata,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<AemData> {
    let mut data = AemData::default();
    for child in &el.children {
        match child.name.as_str() {
            "COMMENT" if child.children.is_empty() => {
                data.comments.push(child.text.trim().to_owned());
            }
            "attitudeState" => {
                data.states
                    .push(read_attitude_state(child, meta, mode, extensions)?);
            }
            "covarianceMatrix" => {
                data.covariances
                    .push(read_covariance(child, mode, extensions)?);
            }
            _ if mode == ParseMode::Lenient && child.children.is_empty() => {
                let mut b = Block::starting_at(Some(child.at));
                b.entries.push(child.entry(child.name.clone()));
                Fields::new(b, mode)?.1.finish(extensions)?;
            }
            _ => return Err(child.unexpected()),
        }
    }
    if mode == ParseMode::Strict && data.states.is_empty() {
        return Err(el.error(ErrorKind::UnexpectedEnd {
            expected: "attitudeState",
        }));
    }
    Ok(data)
}

fn read_attitude_state(
    el: &Element,
    meta: &AemMetadata,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<AemState> {
    let mut block = Block::starting_at(Some(el.at));
    collect_state_entries(el, &mut block)?;
    let (_, mut fields) = Fields::new(block, mode)?;
    let epoch = fields.required("EPOCH", Entry::epoch)?;
    let state = parse_state_fields(&mut fields, meta.attitude_type)?;
    fields.finish(extensions)?;
    Ok(AemState { epoch, state })
}

fn collect_state_entries(el: &Element, block: &mut Block) -> Result<()> {
    for child in &el.children {
        if child.name == "COMMENT" && child.children.is_empty() {
            block.comments.push(child.text.trim().to_owned());
        } else if !child.children.is_empty() {
            collect_state_entries(child, block)?;
        } else {
            let key = if let Some(k) = child.attribute("angle") {
                k.to_owned()
            } else if let Some(k) = child.attribute("rate") {
                k.to_owned()
            } else {
                child.name.clone()
            };
            block.entries.push(child.entry(key));
        }
    }
    Ok(())
}

fn parse_state_fields(fields: &mut Fields, kind: AttitudeType) -> Result<AemAttitudeState> {
    match kind {
        AttitudeType::Quaternion => Ok(AemAttitudeState::Quaternion {
            quaternion: read_quat(fields)?,
        }),
        AttitudeType::QuaternionDerivative => Ok(AemAttitudeState::QuaternionDerivative {
            quaternion: read_quat(fields)?,
            q_dot: [
                fields.required("Q1_DOT", Entry::real)?,
                fields.required("Q2_DOT", Entry::real)?,
                fields.required("Q3_DOT", Entry::real)?,
                fields.required("QC_DOT", Entry::real)?,
            ],
        }),
        AttitudeType::QuaternionAngVel => Ok(AemAttitudeState::QuaternionAngVel {
            quaternion: read_quat(fields)?,
            ang_vel_deg_s: read_v3(fields, ["ANGVEL_X", "ANGVEL_Y", "ANGVEL_Z"])?,
        }),
        AttitudeType::QuaternionRate | AttitudeType::QuaternionEulerRates => {
            Ok(AemAttitudeState::QuaternionRate {
                quaternion: read_quat(fields)?,
                rates_deg_s: read_rates(fields)?,
            })
        }
        AttitudeType::EulerAngle => Ok(AemAttitudeState::EulerAngle {
            angles_deg: read_angles(fields)?,
        }),
        AttitudeType::EulerAngleDerivative => Ok(AemAttitudeState::EulerAngleDerivative {
            angles_deg: read_angles(fields)?,
            derivatives_deg_s: read_rates(fields)?,
        }),
        AttitudeType::EulerAngleAngVel => Ok(AemAttitudeState::EulerAngleAngVel {
            angles_deg: read_angles(fields)?,
            ang_vel_deg_s: read_v3(fields, ["ANGVEL_X", "ANGVEL_Y", "ANGVEL_Z"])?,
        }),
        AttitudeType::EulerAngleRate => Ok(AemAttitudeState::EulerAngleRate {
            angles_deg: read_angles(fields)?,
            rates_deg_s: read_rates(fields)?,
        }),
        AttitudeType::Spin | AttitudeType::SpinNutation | AttitudeType::SpinNutationMom => {
            parse_spin_fields(fields, kind)
        }
    }
}

fn parse_spin_fields(fields: &mut Fields, kind: AttitudeType) -> Result<AemAttitudeState> {
    let spin_alpha_deg = fields.required("SPIN_ALPHA", Entry::real)?;
    let spin_delta_deg = fields.required("SPIN_DELTA", Entry::real)?;
    let spin_angle_deg = fields.required("SPIN_ANGLE", Entry::real)?;
    let spin_angle_vel_deg_s = fields.required("SPIN_ANGLE_VEL", Entry::real)?;
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
            nutation_deg: fields.required("NUTATION", Entry::real)?,
            nutation_per_s: fields.required("NUTATION_PER", Entry::real)?,
            nutation_phase_deg: fields.required("NUTATION_PHASE", Entry::real)?,
        },
        AttitudeType::SpinNutationMom => AemAttitudeState::SpinNutationMom {
            spin_alpha_deg,
            spin_delta_deg,
            spin_angle_deg,
            spin_angle_vel_deg_s,
            momentum_alpha_deg: fields.required("MOMENTUM_ALPHA", Entry::real)?,
            momentum_delta_deg: fields.required("MOMENTUM_DELTA", Entry::real)?,
            nutation_vel_deg_s: fields.required("NUTATION_VEL", Entry::real)?,
        },
        _ => return Err(fields.error_for("SPIN_ALPHA", ErrorKind::MissingKeyword)),
    })
}

fn read_quat(fields: &mut Fields) -> Result<Quaternion> {
    Ok(Quaternion {
        q1: fields.required("Q1", Entry::real)?,
        q2: fields.required("Q2", Entry::real)?,
        q3: fields.required("Q3", Entry::real)?,
        qc: fields.required("QC", Entry::real)?,
    })
}

fn read_v3(fields: &mut Fields, [k0, k1, k2]: [&str; 3]) -> Result<[f64; 3]> {
    Ok([
        fields.required(k0, Entry::real)?,
        fields.required(k1, Entry::real)?,
        fields.required(k2, Entry::real)?,
    ])
}

fn read_angles(fields: &mut Fields) -> Result<[f64; 3]> {
    if fields.has_any(&["X_ANGLE", "Y_ANGLE", "Z_ANGLE"]) {
        read_v3(fields, ["X_ANGLE", "Y_ANGLE", "Z_ANGLE"])
    } else {
        read_v3(fields, ["ANGLE_1", "ANGLE_2", "ANGLE_3"])
    }
}

fn read_rates(fields: &mut Fields) -> Result<[f64; 3]> {
    if fields.has_any(&["X_RATE", "Y_RATE", "Z_RATE"]) {
        read_v3(fields, ["X_RATE", "Y_RATE", "Z_RATE"])
    } else {
        read_v3(fields, ["ANGLE_1_DOT", "ANGLE_2_DOT", "ANGLE_3_DOT"])
    }
}

fn read_covariance(
    el: &Element,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<AemCovariance> {
    let mut comments = Vec::new();
    let mut epoch = None;
    let mut ref_frame = None;
    let mut values = Vec::new();
    for child in &el.children {
        match child.name.as_str() {
            "COMMENT" if child.children.is_empty() => comments.push(child.text.trim().to_owned()),
            "EPOCH" => epoch = Some(child.entry("EPOCH".to_owned()).epoch(mode)?),
            "COV_REF_FRAME" => {
                ref_frame = Some(child.entry("COV_REF_FRAME".to_owned()).frame(mode)?);
            }
            "VALUE" | "C" => values.push(child.entry(child.name.clone()).real(mode)?),
            _ if mode == ParseMode::Lenient => {
                let mut b = Block::starting_at(Some(child.at));
                b.entries.push(child.entry(child.name.clone()));
                Fields::new(b, mode)?.1.finish(extensions)?;
            }
            _ => return Err(child.unexpected()),
        }
    }
    Ok(AemCovariance {
        comments,
        epoch,
        ref_frame,
        lower_triangle: values,
    })
}
