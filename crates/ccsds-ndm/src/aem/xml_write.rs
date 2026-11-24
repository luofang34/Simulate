//! AEM XML writer (`<aem>`).

use super::layout::VERSION_KEY;
use super::{Aem, AemAttitudeState, AemCovariance, AemState};
use crate::adm::{AttitudeType, Quaternion, format_num};
use crate::error::Result;
use crate::extension;
use crate::header::Version;
use crate::xml::writer::XmlWriter;
use crate::{WriteOptions, xml};

pub(super) fn write(
    aem: &Aem,
    out: &mut XmlWriter,
    options: WriteOptions,
    standalone: bool,
) -> Result<()> {
    let mut attributes = xml::root_attributes(aem.version, standalone);
    attributes.push(("id", VERSION_KEY));
    attributes.push(("version", aem.version.as_str()));
    out.open("aem", &attributes);
    out.block("header", &aem.header.write(options), None);
    out.open("body", &[]);
    for seg in &aem.segments {
        out.open("segment", &[]);
        out.block(
            "metadata",
            &super::layout::write_metadata(&seg.metadata, aem.version, options),
            None,
        );
        out.open("data", &[]);
        out.comments(&seg.data.comments);
        for st in &seg.data.states {
            write_state_xml(out, st, seg.metadata.attitude_type, aem.version)?;
        }
        for cov in &seg.data.covariances {
            write_cov_xml(out, cov)?;
        }
        out.close("data");
        out.close("segment");
    }
    if !aem.extensions.is_empty() {
        out.entries(&extension::write_extensions(&aem.extensions), None);
    }
    out.close("body");
    out.close("aem");
    Ok(())
}

fn write_state_xml(
    out: &mut XmlWriter,
    st: &AemState,
    kind: AttitudeType,
    version: Version,
) -> Result<()> {
    out.open("attitudeState", &[]);
    let tag = state_wrapper_tag(kind);
    out.open(tag, &[]);
    out.leaf("EPOCH", &[], &st.epoch.to_string());
    write_state_body(out, &st.state, version)?;
    out.close(tag);
    out.close("attitudeState");
    Ok(())
}

fn write_state_body(out: &mut XmlWriter, state: &AemAttitudeState, version: Version) -> Result<()> {
    match *state {
        AemAttitudeState::Quaternion { quaternion } => write_quat_leaves(out, &quaternion)?,
        AemAttitudeState::QuaternionDerivative { quaternion, q_dot } => {
            write_quat_leaves(out, &quaternion)?;
            let [d1, d2, d3, dc] = q_dot;
            write_num_leaf(out, "Q1_DOT", d1)?;
            write_num_leaf(out, "Q2_DOT", d2)?;
            write_num_leaf(out, "Q3_DOT", d3)?;
            write_num_leaf(out, "QC_DOT", dc)?;
        }
        AemAttitudeState::QuaternionAngVel {
            quaternion,
            ang_vel_deg_s: [wx, wy, wz],
        } => {
            write_quat_leaves(out, &quaternion)?;
            write_num_leaf(out, "ANGVEL_X", wx)?;
            write_num_leaf(out, "ANGVEL_Y", wy)?;
            write_num_leaf(out, "ANGVEL_Z", wz)?;
        }
        AemAttitudeState::QuaternionRate {
            quaternion,
            rates_deg_s: r,
        } => {
            write_quat_leaves(out, &quaternion)?;
            write_rate_leaves(out, r, version)?;
        }
        AemAttitudeState::EulerAngle { angles_deg: a } => write_angle_leaves(out, a, version)?,
        AemAttitudeState::EulerAngleDerivative {
            angles_deg: a,
            derivatives_deg_s: d,
        }
        | AemAttitudeState::EulerAngleRate {
            angles_deg: a,
            rates_deg_s: d,
        } => {
            write_angle_leaves(out, a, version)?;
            write_rate_leaves(out, d, version)?;
        }
        AemAttitudeState::EulerAngleAngVel {
            angles_deg: a,
            ang_vel_deg_s: [wx, wy, wz],
        } => {
            write_angle_leaves(out, a, version)?;
            write_num_leaf(out, "ANGVEL_X", wx)?;
            write_num_leaf(out, "ANGVEL_Y", wy)?;
            write_num_leaf(out, "ANGVEL_Z", wz)?;
        }
        _ => write_spin_body(out, state)?,
    }
    Ok(())
}

fn write_spin_body(out: &mut XmlWriter, state: &AemAttitudeState) -> Result<()> {
    match *state {
        AemAttitudeState::Spin {
            spin_alpha_deg,
            spin_delta_deg,
            spin_angle_deg,
            spin_angle_vel_deg_s,
        } => {
            write_num_leaf(out, "SPIN_ALPHA", spin_alpha_deg)?;
            write_num_leaf(out, "SPIN_DELTA", spin_delta_deg)?;
            write_num_leaf(out, "SPIN_ANGLE", spin_angle_deg)?;
            write_num_leaf(out, "SPIN_ANGLE_VEL", spin_angle_vel_deg_s)?;
        }
        AemAttitudeState::SpinNutation {
            spin_alpha_deg,
            spin_delta_deg,
            spin_angle_deg,
            spin_angle_vel_deg_s,
            nutation_deg,
            nutation_per_s,
            nutation_phase_deg,
        } => {
            write_num_leaf(out, "SPIN_ALPHA", spin_alpha_deg)?;
            write_num_leaf(out, "SPIN_DELTA", spin_delta_deg)?;
            write_num_leaf(out, "SPIN_ANGLE", spin_angle_deg)?;
            write_num_leaf(out, "SPIN_ANGLE_VEL", spin_angle_vel_deg_s)?;
            write_num_leaf(out, "NUTATION", nutation_deg)?;
            write_num_leaf(out, "NUTATION_PER", nutation_per_s)?;
            write_num_leaf(out, "NUTATION_PHASE", nutation_phase_deg)?;
        }
        AemAttitudeState::SpinNutationMom {
            spin_alpha_deg,
            spin_delta_deg,
            spin_angle_deg,
            spin_angle_vel_deg_s,
            momentum_alpha_deg,
            momentum_delta_deg,
            nutation_vel_deg_s,
        } => {
            write_num_leaf(out, "SPIN_ALPHA", spin_alpha_deg)?;
            write_num_leaf(out, "SPIN_DELTA", spin_delta_deg)?;
            write_num_leaf(out, "SPIN_ANGLE", spin_angle_deg)?;
            write_num_leaf(out, "SPIN_ANGLE_VEL", spin_angle_vel_deg_s)?;
            write_num_leaf(out, "MOMENTUM_ALPHA", momentum_alpha_deg)?;
            write_num_leaf(out, "MOMENTUM_DELTA", momentum_delta_deg)?;
            write_num_leaf(out, "NUTATION_VEL", nutation_vel_deg_s)?;
        }
        _ => {}
    }
    Ok(())
}

const fn state_wrapper_tag(kind: AttitudeType) -> &'static str {
    match kind {
        AttitudeType::Quaternion => "quaternionEphemeris",
        AttitudeType::QuaternionDerivative => "quaternionDerivative",
        AttitudeType::QuaternionAngVel => "quaternionAngVel",
        AttitudeType::QuaternionRate | AttitudeType::QuaternionEulerRates => "quaternionEulerRate",
        AttitudeType::EulerAngle => "eulerAngle",
        AttitudeType::EulerAngleDerivative => "eulerAngleDerivative",
        AttitudeType::EulerAngleAngVel => "eulerAngleAngVel",
        AttitudeType::EulerAngleRate => "eulerAngleRate",
        AttitudeType::Spin => "spin",
        AttitudeType::SpinNutation => "spinNutation",
        AttitudeType::SpinNutationMom => "spinNutationMom",
    }
}

fn write_quat_leaves(out: &mut XmlWriter, q: &Quaternion) -> Result<()> {
    write_num_leaf(out, "Q1", q.q1)?;
    write_num_leaf(out, "Q2", q.q2)?;
    write_num_leaf(out, "Q3", q.q3)?;
    write_num_leaf(out, "QC", q.qc)
}

fn write_angle_leaves(out: &mut XmlWriter, [a1, a2, a3]: [f64; 3], version: Version) -> Result<()> {
    let [k1, k2, k3] = if version == Version::V1 {
        ["X_ANGLE", "Y_ANGLE", "Z_ANGLE"]
    } else {
        ["ANGLE_1", "ANGLE_2", "ANGLE_3"]
    };
    write_num_leaf(out, k1, a1)?;
    write_num_leaf(out, k2, a2)?;
    write_num_leaf(out, k3, a3)
}

fn write_rate_leaves(out: &mut XmlWriter, [r1, r2, r3]: [f64; 3], version: Version) -> Result<()> {
    let [k1, k2, k3] = if version == Version::V1 {
        ["X_RATE", "Y_RATE", "Z_RATE"]
    } else {
        ["ANGLE_1_DOT", "ANGLE_2_DOT", "ANGLE_3_DOT"]
    };
    write_num_leaf(out, k1, r1)?;
    write_num_leaf(out, k2, r2)?;
    write_num_leaf(out, k3, r3)
}

fn write_num_leaf(out: &mut XmlWriter, key: &str, val: f64) -> Result<()> {
    let s = format_num(key, val)?;
    out.leaf(key, &[], &s);
    Ok(())
}

fn write_cov_xml(out: &mut XmlWriter, cov: &AemCovariance) -> Result<()> {
    out.open("covarianceMatrix", &[]);
    out.comments(&cov.comments);
    if let Some(ep) = &cov.epoch {
        out.leaf("EPOCH", &[], &ep.to_string());
    }
    if let Some(rf) = &cov.ref_frame {
        out.leaf("COV_REF_FRAME", &[], rf.as_str());
    }
    for &v in &cov.lower_triangle {
        write_num_leaf(out, "C", v)?;
    }
    out.close("covarianceMatrix");
    Ok(())
}
