//! ADM (`APM`, `AEM`) bidirectional attitude adapters for `sim-orbital`.

use core::f64::consts::PI;

use ccsds_ndm::{
    Aem, AemAttitudeState, AemData, AemMetadata, AemSegment, AemState, Apm, ApmAngularVelocity,
    ApmData, ApmInertia, ApmManeuver, ApmMetadata, ApmQuaternion, AttitudeType, EulerRotSeq,
    Interpolation, PerS, Quantity, Quaternion, QuaternionDot, Version, vector3,
};
use glam::{DMat3, DQuat, DVec3};
use sim_spacetime::{AttitudeSample, AttitudeTrajectorySource, TdbEpoch, TimeScaleCatalog};

use super::{CcsdsError, CcsdsMeta, build_header, epoch_to_tdb, tdb_to_epoch};
use crate::ContinuousPlantState;

const RAD_TO_DEG: f64 = 180.0 / PI;
const DEG_TO_RAD: f64 = PI / 180.0;

/// Attitude maneuver torque specification for [`CcsdsAdm::plant_to_apm`](crate::CcsdsAdm::plant_to_apm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApmBurn {
    /// Maneuver start epoch.
    pub epoch_start: TdbEpoch,
    /// Maneuver duration (`s`).
    pub duration_s: f64,
    /// Body-frame torque vector `[tx, ty, tz]` (`N*m`).
    pub torque_nm: DVec3,
    /// Optional propellant mass change (`kg`, typically negative).
    pub delta_mass_kg: Option<f64>,
}

/// Imported `APM` attitude, angular velocity, inertia tensor, and maneuver list.
#[derive(Clone, Debug, PartialEq)]
pub struct ApmImport {
    /// Sample epoch converted to [`TdbEpoch`].
    pub epoch: TdbEpoch,
    /// Attitude quaternion transforming body vectors into `REF_FRAME_A` (`GCRF`/`ICRF`).
    pub q_body_to_icrf: DQuat,
    /// Optional spacecraft body angular velocity (`rad/s`).
    pub omega_body_rad_s: Option<DVec3>,
    /// Optional spacecraft inertia tensor (`kg*m^2`) in `INERTIA_REF_FRAME`.
    pub inertia_kg_m2: Option<DMat3>,
    /// Raw CCSDS attitude maneuver blocks preserved from the `APM`.
    pub maneuvers: Vec<ApmManeuver>,
}

/// Exports a [`ContinuousPlantState`], optional inertia tensor, and maneuvers into a CCSDS 2.0 `APM`.
pub fn plant_state_to_apm(
    meta: &CcsdsMeta,
    epoch: TdbEpoch,
    plant: &ContinuousPlantState,
    inertia_kg_m2: Option<DMat3>,
    maneuvers: &[ApmBurn],
    scales: &TimeScaleCatalog,
) -> Result<Apm, CcsdsError> {
    let ep = tdb_to_epoch(epoch, meta.time_system, scales)?;
    let q = dquat_to_ccsds(plant.q_body_to_icrf);
    let q_dot = quaternion_derivative(plant.q_body_to_icrf, plant.omega_body_rad_s);
    let w_deg = plant.omega_body_rad_s * RAD_TO_DEG;
    let inertia = inertia_kg_m2.map(|m| ApmInertia {
        comments: Vec::new(),
        ref_frame: Some(meta.body_frame.clone()),
        ixx: Quantity::new(m.x_axis.x),
        iyy: Quantity::new(m.y_axis.y),
        izz: Quantity::new(m.z_axis.z),
        ixy: Quantity::new(m.x_axis.y),
        ixz: Quantity::new(m.x_axis.z),
        iyz: Quantity::new(m.y_axis.z),
    });
    let mut man_blocks = Vec::with_capacity(maneuvers.len());
    for man in maneuvers {
        man_blocks.push(ApmManeuver {
            comments: Vec::new(),
            epoch_start: tdb_to_epoch(man.epoch_start, meta.time_system, scales)?,
            duration: Quantity::new(man.duration_s),
            ref_frame: meta.body_frame.clone(),
            torque: vector3([man.torque_nm.x, man.torque_nm.y, man.torque_nm.z]),
            delta_mass: man.delta_mass_kg.map(Quantity::new),
        });
    }
    Ok(Apm {
        version: Version::V2,
        header: build_header(
            meta.creation_date.clone(),
            &meta.originator,
            meta.message_id.as_deref(),
        ),
        metadata: ApmMetadata {
            comments: Vec::new(),
            object_name: meta.object_name.clone(),
            object_id: meta.object_id.clone(),
            center_name: Some(meta.center_name.clone()),
            time_system: meta.time_system,
        },
        data: ApmData {
            comments: Vec::new(),
            epoch: Some(ep.clone()),
            quaternion: Some(ApmQuaternion {
                comments: Vec::new(),
                epoch: Some(ep.clone()),
                ref_frame_a: meta.ref_frame.clone(),
                ref_frame_b: meta.body_frame.clone(),
                dir: None,
                quaternion: q,
                rate: Some(q_dot),
            }),
            euler: None,
            ang_vel: Some(ApmAngularVelocity {
                comments: Vec::new(),
                epoch: Some(ep),
                ref_frame_a: Some(meta.ref_frame.clone()),
                ref_frame_b: Some(meta.body_frame.clone()),
                frame: meta.body_frame.clone(),
                vector: vector3([w_deg.x, w_deg.y, w_deg.z]),
            }),
            spin: None,
            inertia,
            maneuvers: man_blocks,
            user_defined: None,
        },
        extensions: Vec::new(),
    })
}

/// Imports a CCSDS `APM` into `sim-orbital` attitude quaternion, body rates, and inertia tensor.
pub fn apm_to_attitude(apm: &Apm, scales: &TimeScaleCatalog) -> Result<ApmImport, CcsdsError> {
    let ep = apm
        .data
        .epoch
        .as_ref()
        .or_else(|| apm.data.quaternion.as_ref().and_then(|q| q.epoch.as_ref()))
        .or_else(|| apm.data.euler.as_ref().and_then(|e| e.epoch.as_ref()))
        .or_else(|| apm.data.ang_vel.as_ref().and_then(|a| a.epoch.as_ref()))
        .ok_or(CcsdsError::MissingData("APM epoch"))?;
    let epoch = epoch_to_tdb(ep, apm.metadata.time_system, scales)?;
    let (q_body_to_icrf, q_dot_omega) = if let Some(q) = &apm.data.quaternion {
        let dq = ccsds_to_dquat(q.quaternion);
        let from_dot = q.rate.map(|r| omega_from_q_dot(dq, r));
        (dq, from_dot)
    } else if let Some(e) = &apm.data.euler {
        let [a1, a2, a3] = e.angles;
        (
            euler_seq_to_dquat(
                &e.rot_seq,
                [
                    a1.value() * DEG_TO_RAD,
                    a2.value() * DEG_TO_RAD,
                    a3.value() * DEG_TO_RAD,
                ],
            )?,
            None,
        )
    } else {
        return Err(CcsdsError::MissingData("APM quaternion or Euler state"));
    };
    let omega_body_rad_s = apm
        .data
        .ang_vel
        .as_ref()
        .map(|av| {
            let [wx, wy, wz] = av.vector;
            DVec3::new(wx.value(), wy.value(), wz.value()) * DEG_TO_RAD
        })
        .or(q_dot_omega);
    let inertia_kg_m2 = apm.data.inertia.as_ref().map(|i| {
        let (xx, yy, zz) = (i.ixx.value(), i.iyy.value(), i.izz.value());
        let (xy, xz, yz) = (i.ixy.value(), i.ixz.value(), i.iyz.value());
        DMat3::from_cols(
            DVec3::new(xx, xy, xz),
            DVec3::new(xy, yy, yz),
            DVec3::new(xz, yz, zz),
        )
    });
    Ok(ApmImport {
        epoch,
        q_body_to_icrf,
        omega_body_rad_s,
        inertia_kg_m2,
        maneuvers: apm.data.maneuvers.clone(),
    })
}

/// Exports `sim-spacetime` [`AttitudeSample`] points into a CCSDS 2.0 `AEM`.
pub fn attitude_samples_to_aem(
    meta: &CcsdsMeta,
    samples: &[AttitudeSample],
    include_ang_vel: bool,
    interpolation: Option<(Interpolation, u32)>,
    scales: &TimeScaleCatalog,
) -> Result<Aem, CcsdsError> {
    let Some(first) = samples.first() else {
        return Err(CcsdsError::MissingData("at least one AEM sample"));
    };
    let Some(last) = samples.last() else {
        return Err(CcsdsError::MissingData("at least one AEM sample"));
    };
    let attitude_type = if include_ang_vel {
        AttitudeType::QuaternionAngVel
    } else {
        AttitudeType::Quaternion
    };
    let mut states = Vec::with_capacity(samples.len());
    for s in samples {
        let ep = tdb_to_epoch(s.epoch, meta.time_system, scales)?;
        let q = dquat_to_ccsds(s.q_body_to_icrf);
        let payload = if include_ang_vel {
            let w = s.omega_body_rad_s * RAD_TO_DEG;
            AemAttitudeState::QuaternionAngVel {
                quaternion: q,
                ang_vel_deg_s: [w.x, w.y, w.z],
            }
        } else {
            AemAttitudeState::Quaternion { quaternion: q }
        };
        states.push(AemState {
            epoch: ep,
            state: payload,
        });
    }
    let (interp, degree) = match interpolation {
        Some((m, d)) => (Some(m), Some(d)),
        None => (None, None),
    };
    Ok(Aem {
        version: Version::V2,
        header: build_header(
            meta.creation_date.clone(),
            &meta.originator,
            meta.message_id.as_deref(),
        ),
        segments: vec![AemSegment {
            metadata: AemMetadata {
                comments: Vec::new(),
                object_name: meta.object_name.clone(),
                object_id: meta.object_id.clone(),
                center_name: Some(meta.center_name.clone()),
                ref_frame_a: meta.ref_frame.clone(),
                ref_frame_b: meta.body_frame.clone(),
                attitude_dir: None,
                time_system: meta.time_system,
                start_time: tdb_to_epoch(first.epoch, meta.time_system, scales)?,
                useable_start_time: None,
                useable_stop_time: None,
                stop_time: tdb_to_epoch(last.epoch, meta.time_system, scales)?,
                attitude_type,
                quaternion_type: None,
                euler_rot_seq: None,
                rate_frame: include_ang_vel.then(|| meta.body_frame.clone()),
                interpolation_method: interp,
                interpolation_degree: degree,
            },
            data: AemData {
                comments: Vec::new(),
                states,
                covariances: Vec::new(),
            },
        }],
        extensions: Vec::new(),
    })
}

/// Samples any [`AttitudeTrajectorySource`] at `epochs` into a CCSDS 2.0 `AEM`.
pub fn attitude_source_to_aem(
    meta: &CcsdsMeta,
    source: &dyn AttitudeTrajectorySource,
    epochs: &[TdbEpoch],
    include_ang_vel: bool,
    interpolation: Option<(Interpolation, u32)>,
    scales: &TimeScaleCatalog,
) -> Result<Aem, CcsdsError> {
    let mut samples = Vec::with_capacity(epochs.len());
    for &epoch in epochs {
        let sample = source
            .sample_attitude_icrf(epoch)
            .map_err(|e| CcsdsError::Sgp4(e.to_string()))?;
        samples.push(sample);
    }
    attitude_samples_to_aem(meta, &samples, include_ang_vel, interpolation, scales)
}

/// Imports all quaternion/Euler attitude samples across segments of a CCSDS `AEM`.
pub fn aem_to_attitude_samples(
    aem: &Aem,
    scales: &TimeScaleCatalog,
) -> Result<Vec<AttitudeSample>, CcsdsError> {
    let mut out = Vec::new();
    for seg in &aem.segments {
        for st in &seg.data.states {
            let epoch = epoch_to_tdb(&st.epoch, seg.metadata.time_system, scales)?;
            let (q_body_to_icrf, omega_body_rad_s) =
                map_aem_state(&st.state, seg.metadata.euler_rot_seq.as_ref())?;
            out.push(AttitudeSample {
                epoch,
                q_body_to_icrf,
                omega_body_rad_s,
            });
        }
    }
    Ok(out)
}

fn map_aem_state(
    state: &AemAttitudeState,
    euler_seq: Option<&EulerRotSeq>,
) -> Result<(DQuat, DVec3), CcsdsError> {
    match *state {
        AemAttitudeState::Quaternion { quaternion } => {
            Ok((ccsds_to_dquat(quaternion), DVec3::ZERO))
        }
        AemAttitudeState::QuaternionAngVel {
            quaternion,
            ang_vel_deg_s: [wx, wy, wz],
        } => Ok((
            ccsds_to_dquat(quaternion),
            DVec3::new(wx, wy, wz) * DEG_TO_RAD,
        )),
        AemAttitudeState::QuaternionDerivative {
            quaternion,
            q_dot: [d1, d2, d3, dc],
        } => {
            let dq = ccsds_to_dquat(quaternion);
            let omega = omega_from_q_dot(
                dq,
                QuaternionDot {
                    q1_dot: Quantity::new(d1),
                    q2_dot: Quantity::new(d2),
                    q3_dot: Quantity::new(d3),
                    qc_dot: Quantity::new(dc),
                },
            );
            Ok((dq, omega))
        }
        AemAttitudeState::QuaternionRate { quaternion, .. } => {
            Ok((ccsds_to_dquat(quaternion), DVec3::ZERO))
        }
        AemAttitudeState::EulerAngle {
            angles_deg: [a, b, c],
        }
        | AemAttitudeState::EulerAngleDerivative {
            angles_deg: [a, b, c],
            ..
        }
        | AemAttitudeState::EulerAngleRate {
            angles_deg: [a, b, c],
            ..
        } => {
            let seq = euler_seq.ok_or(CcsdsError::MissingData("EULER_ROT_SEQ"))?;
            Ok((
                euler_seq_to_dquat(seq, [a * DEG_TO_RAD, b * DEG_TO_RAD, c * DEG_TO_RAD])?,
                DVec3::ZERO,
            ))
        }
        AemAttitudeState::EulerAngleAngVel {
            angles_deg: [a, b, c],
            ang_vel_deg_s: [wx, wy, wz],
        } => {
            let seq = euler_seq.ok_or(CcsdsError::MissingData("EULER_ROT_SEQ"))?;
            Ok((
                euler_seq_to_dquat(seq, [a * DEG_TO_RAD, b * DEG_TO_RAD, c * DEG_TO_RAD])?,
                DVec3::new(wx, wy, wz) * DEG_TO_RAD,
            ))
        }
        AemAttitudeState::Spin { .. }
        | AemAttitudeState::SpinNutation { .. }
        | AemAttitudeState::SpinNutationMom { .. } => Err(CcsdsError::MissingData(
            "quaternion or Euler attitude state",
        )),
    }
}

fn dquat_to_ccsds(q: DQuat) -> Quaternion {
    Quaternion::new(q.w, q.x, q.y, q.z)
}

fn ccsds_to_dquat(q: Quaternion) -> DQuat {
    DQuat::from_xyzw(q.q1, q.q2, q.q3, q.qc).normalize()
}

fn quaternion_derivative(q: DQuat, omega_body_rad_s: DVec3) -> QuaternionDot {
    let wq = DQuat::from_xyzw(
        omega_body_rad_s.x,
        omega_body_rad_s.y,
        omega_body_rad_s.z,
        0.0,
    );
    let q_dot = (q * wq) * 0.5;
    QuaternionDot {
        q1_dot: Quantity::<PerS>::new(q_dot.x),
        q2_dot: Quantity::<PerS>::new(q_dot.y),
        q3_dot: Quantity::<PerS>::new(q_dot.z),
        qc_dot: Quantity::<PerS>::new(q_dot.w),
    }
}

fn omega_from_q_dot(q: DQuat, dot: QuaternionDot) -> DVec3 {
    let dq = DQuat::from_xyzw(
        dot.q1_dot.value(),
        dot.q2_dot.value(),
        dot.q3_dot.value(),
        dot.qc_dot.value(),
    );
    let wq = (q.conjugate() * dq) * 2.0;
    DVec3::new(wq.x, wq.y, wq.z)
}

fn euler_seq_to_dquat(seq: &EulerRotSeq, [a, b, c]: [f64; 3]) -> Result<DQuat, CcsdsError> {
    let mut bytes = seq.as_str().bytes();
    let (Some(c1), Some(c2), Some(c3), None) =
        (bytes.next(), bytes.next(), bytes.next(), bytes.next())
    else {
        return Err(CcsdsError::MissingData("3-axis EULER_ROT_SEQ"));
    };
    let q = axis_quat(c1, a)? * axis_quat(c2, b)? * axis_quat(c3, c)?;
    Ok(q.normalize())
}

fn axis_quat(code: u8, angle_rad: f64) -> Result<DQuat, CcsdsError> {
    let half = 0.5 * angle_rad;
    let s = libm::sin(half);
    let c = libm::cos(half);
    match code {
        b'1' | b'X' => Ok(DQuat::from_xyzw(s, 0.0, 0.0, c)),
        b'2' | b'Y' => Ok(DQuat::from_xyzw(0.0, s, 0.0, c)),
        b'3' | b'Z' => Ok(DQuat::from_xyzw(0.0, 0.0, s, c)),
        _ => Err(CcsdsError::MissingData("valid EULER_ROT_SEQ axis")),
    }
}
