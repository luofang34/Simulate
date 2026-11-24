//! Internal grid generation, sampling, and SPK/CK segment builders for `spice_export`.

use glam::{DQuat, DVec3};
use sim_spacetime::{AttitudeSample, FrameAxesSpec, TdbEpoch, TranslationalTrajectorySource};
use spice_kernels::{
    ChebyshevPositions, ChebyshevStates, CkFile, CkInterval, CkPointing, CkPointingInstance,
    CkSegment, CkType2, CkType3, FRAME_J2000, SampledStates, SpkFile, SpkSegment,
};

use super::{
    CkExportConfig, CkExportKind, KM_TO_M, LinearSclkMapping, M_TO_KM, MAX_GRID_RECORDS,
    SpiceExportError, SpkExportConfig, SpkExportKind,
};
use crate::{
    KinematicStateIcrf, plant::attitude_math::quat_rotate_vec_portable, scalar_math::sqrt,
};

pub(super) fn kinematic_to_spice(state: &KinematicStateIcrf) -> [f64; 6] {
    [
        state.pos_m.x * M_TO_KM,
        state.pos_m.y * M_TO_KM,
        state.pos_m.z * M_TO_KM,
        state.vel_m_s.x * M_TO_KM,
        state.vel_m_s.y * M_TO_KM,
        state.vel_m_s.z * M_TO_KM,
    ]
}

pub(super) fn spice_to_kinematic(state: &[f64; 6]) -> KinematicStateIcrf {
    let [x, y, z, vx, vy, vz] = *state;
    KinematicStateIcrf {
        pos_m: DVec3::new(x, y, z) * KM_TO_M,
        vel_m_s: DVec3::new(vx, vy, vz) * KM_TO_M,
        acc_m_s2: DVec3::ZERO,
    }
}

pub(super) fn attitude_to_ck_instance(
    epoch: TdbEpoch,
    q_body_to_icrf: DQuat,
    omega_body_rad_s: Option<DVec3>,
    sclk: &LinearSclkMapping,
) -> Result<CkPointingInstance, SpiceExportError> {
    let q_unit = normalize_finite_quat(q_body_to_icrf)?;
    let angular_velocity = match omega_body_rad_s {
        Some(omega_body) => {
            if !omega_body.is_finite() {
                return Err(SpiceExportError::InvalidConfiguration(
                    "angular velocity must be finite",
                ));
            }
            Some(quat_rotate_vec_portable(q_unit, omega_body).to_array())
        }
        None => None,
    };
    Ok(CkPointingInstance {
        sclk: sclk.epoch_to_sclk(epoch)?,
        quaternion: [q_unit.w, -q_unit.x, -q_unit.y, -q_unit.z],
        angular_velocity,
    })
}

pub(super) fn ck_pointing_to_attitude(epoch: TdbEpoch, pointing: CkPointing) -> AttitudeSample {
    let [w, x, y, z] = pointing.quaternion;
    let q_raw = DQuat::from_xyzw(-x, -y, -z, w);
    let q_body_to_icrf = normalize_finite_quat(q_raw).unwrap_or(DQuat::IDENTITY);
    let omega_body_rad_s = match pointing.angular_velocity {
        Some(av_ref) => {
            quat_rotate_vec_portable(q_body_to_icrf.conjugate(), DVec3::from_array(av_ref))
        }
        None => DVec3::ZERO,
    };
    AttitudeSample {
        epoch,
        q_body_to_icrf,
        omega_body_rad_s,
    }
}

pub(super) fn build_spk_from_source(
    source: &dyn TranslationalTrajectorySource,
    config: &SpkExportConfig<'_>,
) -> Result<SpkFile, SpiceExportError> {
    let valid = source.valid_interval();
    let (init_s, end_s, span_s) = validate_span(valid.start_epoch, valid.end_epoch)?;
    let segment = match config.kind {
        SpkExportKind::Type2Chebyshev {
            granule_duration_s,
            degree,
        } => {
            let (n_records, intlen) = chebyshev_grid(init_s, end_s, span_s, granule_duration_s)?;
            let pos = ChebyshevPositions::try_fit_nodes(init_s, intlen, n_records, degree, |t| {
                let [x, y, z, ..] =
                    sample_source_state_km(source, valid.start_epoch, valid.end_epoch, t)?;
                Ok::<[f64; 3], SpiceExportError>([x, y, z])
            })??;
            SpkSegment::type2(
                config.segment_name,
                config.target.0,
                config.center.0,
                FRAME_J2000,
                pos,
            )?
            .with_coverage([init_s, end_s])?
        }
        SpkExportKind::Type3Chebyshev {
            granule_duration_s,
            degree,
        } => {
            let (n_records, intlen) = chebyshev_grid(init_s, end_s, span_s, granule_duration_s)?;
            let states = ChebyshevStates::try_fit_nodes(init_s, intlen, n_records, degree, |t| {
                sample_source_state_km(source, valid.start_epoch, valid.end_epoch, t)
            })??;
            SpkSegment::type3(
                config.segment_name,
                config.target.0,
                config.center.0,
                FRAME_J2000,
                states,
            )?
            .with_coverage([init_s, end_s])?
        }
        SpkExportKind::Type9Lagrange { step_s, window }
        | SpkExportKind::Type13Hermite { step_s, window } => {
            let samples = sample_source_grid(source, valid.start_epoch, valid.end_epoch, step_s)?;
            let (epochs, states) = split_epoch_states(&samples);
            build_sampled_spk_segment(config, epochs, states, window, config.kind)?
        }
    };
    let mut spk = SpkFile::new(config.internal_name)?;
    spk.push(segment);
    Ok(spk)
}

pub(super) fn build_spk_from_samples(
    samples: &[(TdbEpoch, KinematicStateIcrf)],
    config: &SpkExportConfig<'_>,
) -> Result<SpkFile, SpiceExportError> {
    let first = samples
        .first()
        .ok_or(SpiceExportError::InvalidConfiguration("empty sample slice"))?;
    let last = samples
        .last()
        .ok_or(SpiceExportError::InvalidConfiguration("empty sample slice"))?;
    let (init_s, end_s, span_s) = validate_span(first.0, last.0)?;
    let (epochs, states) = convert_sampled_states(samples)?;
    let segment = match config.kind {
        SpkExportKind::Type2Chebyshev {
            granule_duration_s,
            degree,
        } => {
            let (n_records, intlen) = chebyshev_grid(init_s, end_s, span_s, granule_duration_s)?;
            let pos_km: Vec<[f64; 3]> = states.iter().map(|&[x, y, z, ..]| [x, y, z]).collect();
            let fitted = ChebyshevPositions::fit_samples(
                init_s, intlen, n_records, degree, &epochs, &pos_km,
            )?;
            SpkSegment::type2(
                config.segment_name,
                config.target.0,
                config.center.0,
                FRAME_J2000,
                fitted,
            )?
            .with_coverage([init_s, end_s])?
        }
        SpkExportKind::Type3Chebyshev {
            granule_duration_s,
            degree,
        } => {
            let (n_records, intlen) = chebyshev_grid(init_s, end_s, span_s, granule_duration_s)?;
            let fitted =
                ChebyshevStates::fit_samples(init_s, intlen, n_records, degree, &epochs, &states)?;
            SpkSegment::type3(
                config.segment_name,
                config.target.0,
                config.center.0,
                FRAME_J2000,
                fitted,
            )?
            .with_coverage([init_s, end_s])?
        }
        SpkExportKind::Type9Lagrange { window, .. }
        | SpkExportKind::Type13Hermite { window, .. } => {
            build_sampled_spk_segment(config, epochs, states, window, config.kind)?
        }
    };
    let mut spk = SpkFile::new(config.internal_name)?;
    spk.push(segment);
    Ok(spk)
}

fn build_sampled_spk_segment(
    config: &SpkExportConfig<'_>,
    epochs: Vec<f64>,
    states: Vec<[f64; 6]>,
    window: usize,
    kind: SpkExportKind,
) -> Result<SpkSegment, SpiceExportError> {
    let sampled = SampledStates::new(epochs, states, window)?;
    let segment = match kind {
        SpkExportKind::Type9Lagrange { .. } => SpkSegment::lagrange(
            config.segment_name,
            config.target.0,
            config.center.0,
            FRAME_J2000,
            sampled,
        )?,
        SpkExportKind::Type13Hermite { .. } => SpkSegment::hermite(
            config.segment_name,
            config.target.0,
            config.center.0,
            FRAME_J2000,
            sampled,
        )?,
        SpkExportKind::Type2Chebyshev { .. } | SpkExportKind::Type3Chebyshev { .. } => {
            return Err(SpiceExportError::InvalidConfiguration(
                "unexpected Chebyshev kind in sampled builder",
            ));
        }
    };
    Ok(segment)
}

pub(super) fn build_ck_file(
    instances: &[CkPointingInstance],
    config: &CkExportConfig<'_>,
) -> Result<CkFile, SpiceExportError> {
    let segment = match config.kind {
        CkExportKind::Type2ConstantRate { .. } => {
            if instances.len() < 2 {
                return Err(SpiceExportError::InvalidConfiguration(
                    "CK Type 2 export requires at least two attitude samples",
                ));
            }
            let seconds_per_tick = config.sclk.seconds_per_tick()?;
            let mut intervals = Vec::with_capacity(instances.len().saturating_sub(1));
            for pair in instances.windows(2) {
                let (Some(curr), Some(next)) = (pair.first(), pair.last()) else {
                    continue;
                };
                intervals.push(CkInterval {
                    start_sclk: curr.sclk,
                    stop_sclk: next.sclk,
                    seconds_per_tick,
                    quaternion: curr.quaternion,
                    angular_velocity: curr.angular_velocity.unwrap_or([0.0, 0.0, 0.0]),
                });
            }
            CkSegment::type2(
                config.segment_name,
                config.instrument_id,
                FRAME_J2000,
                CkType2::new(intervals)?,
            )?
        }
        CkExportKind::Type3Slerp {
            include_angular_velocity,
            ..
        } => CkSegment::type3(
            config.segment_name,
            config.instrument_id,
            FRAME_J2000,
            CkType3::single_interval(instances.to_vec(), include_angular_velocity)?,
        )?,
    };
    let mut ck = CkFile::new(config.internal_name)?;
    ck.push(segment);
    Ok(ck)
}

pub(super) fn uniform_epoch_grid(
    start: TdbEpoch,
    end: TdbEpoch,
    step_s: f64,
) -> Result<Vec<TdbEpoch>, SpiceExportError> {
    if !step_s.is_finite() || step_s < 1.0e-6 {
        return Err(SpiceExportError::InvalidConfiguration(
            "step_s must be finite and >= 1e-6 s",
        ));
    }
    let span_s = end.signed_offset_seconds_from(start);
    if !span_s.is_finite() || span_s <= 0.0 {
        return Err(SpiceExportError::EmptySpan);
    }
    let raw_steps = libm::ceil(span_s / step_s);
    if !raw_steps.is_finite() || raw_steps > MAX_GRID_RECORDS {
        return Err(SpiceExportError::InvalidConfiguration(
            "sampling grid count exceeds limit",
        ));
    }
    let n_steps = (raw_steps as usize).max(1);
    let start_ns = start.to_j2000_nanos_floor();
    let end_ns = end.to_j2000_nanos_floor();
    let span_ns = end_ns.saturating_sub(start_ns);
    let mut out = Vec::with_capacity(n_steps.saturating_add(1));
    out.push(start);
    for k in 1..n_steps {
        let off_ns = (span_ns.saturating_mul(k as i128)) / (n_steps as i128);
        let ep = TdbEpoch::from_j2000_nanos(start_ns.saturating_add(off_ns));
        if ep > *out.last().unwrap_or(&start) && ep < end {
            out.push(ep);
        }
    }
    if end > *out.last().unwrap_or(&start) {
        out.push(end);
    }
    Ok(out)
}

fn normalize_finite_quat(q: DQuat) -> Result<DQuat, SpiceExportError> {
    let arr = q.to_array();
    if arr.iter().any(|x| !x.is_finite()) {
        return Err(SpiceExportError::InvalidUnitQuaternion);
    }
    let norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if !norm_sq.is_finite() || norm_sq <= 0.0 {
        return Err(SpiceExportError::InvalidUnitQuaternion);
    }
    let inv = 1.0 / sqrt(norm_sq);
    Ok(DQuat::from_xyzw(q.x * inv, q.y * inv, q.z * inv, q.w * inv))
}

fn validate_span(start: TdbEpoch, end: TdbEpoch) -> Result<(f64, f64, f64), SpiceExportError> {
    let span_s = end.signed_offset_seconds_from(start);
    if !span_s.is_finite() || span_s <= 0.0 {
        return Err(SpiceExportError::EmptySpan);
    }
    let init_s = start.as_j2000_seconds_f64();
    let end_s = end.as_j2000_seconds_f64();
    if !init_s.is_finite() || !end_s.is_finite() || end_s <= init_s {
        return Err(SpiceExportError::EmptySpan);
    }
    Ok((init_s, end_s, span_s))
}

fn chebyshev_grid(
    init_s: f64,
    end_s: f64,
    span_s: f64,
    granule_duration_s: f64,
) -> Result<(usize, f64), SpiceExportError> {
    if !granule_duration_s.is_finite() || granule_duration_s <= 0.0 {
        return Err(SpiceExportError::InvalidConfiguration(
            "granule_duration_s must be finite and positive",
        ));
    }
    let raw_count = libm::ceil(span_s / granule_duration_s);
    if !raw_count.is_finite() || raw_count > MAX_GRID_RECORDS {
        return Err(SpiceExportError::InvalidConfiguration(
            "Chebyshev granule count exceeds limit",
        ));
    }
    let n_records = (raw_count as usize).max(1);
    let mut intlen = (end_s - init_s) / (n_records as f64);
    while init_s + (n_records as f64) * intlen < end_s {
        intlen = intlen.next_up();
    }
    Ok((n_records, intlen))
}

fn sample_source_state_km(
    source: &dyn TranslationalTrajectorySource,
    start: TdbEpoch,
    end: TdbEpoch,
    et_f64: f64,
) -> Result<[f64; 6], SpiceExportError> {
    let raw = TdbEpoch::from_j2000_seconds_f64(et_f64)
        .ok_or(SpiceExportError::InvalidConfiguration("non-finite epoch"))?;
    let clamped = raw.clamp(start, end);
    let framed = source.sample_translational_icrf(clamped)?;
    if framed.spec.axes != FrameAxesSpec::IcrfInertial {
        return Err(SpiceExportError::NonIcrfAxes {
            found: framed.spec.axes,
        });
    }
    let si = framed.to_si_metric();
    Ok([
        si.pos.x * M_TO_KM,
        si.pos.y * M_TO_KM,
        si.pos.z * M_TO_KM,
        si.vel.x * M_TO_KM,
        si.vel.y * M_TO_KM,
        si.vel.z * M_TO_KM,
    ])
}

fn sample_source_grid(
    source: &dyn TranslationalTrajectorySource,
    start: TdbEpoch,
    end: TdbEpoch,
    step_s: f64,
) -> Result<Vec<(f64, [f64; 6])>, SpiceExportError> {
    let grid = uniform_epoch_grid(start, end, step_s)?;
    let mut out = Vec::with_capacity(grid.len());
    for epoch in grid {
        let framed = source.sample_translational_icrf(epoch)?;
        if framed.spec.axes != FrameAxesSpec::IcrfInertial {
            return Err(SpiceExportError::NonIcrfAxes {
                found: framed.spec.axes,
            });
        }
        let si = framed.to_si_metric();
        out.push((
            epoch.as_j2000_seconds_f64(),
            [
                si.pos.x * M_TO_KM,
                si.pos.y * M_TO_KM,
                si.pos.z * M_TO_KM,
                si.vel.x * M_TO_KM,
                si.vel.y * M_TO_KM,
                si.vel.z * M_TO_KM,
            ],
        ));
    }
    Ok(out)
}

fn split_epoch_states(samples: &[(f64, [f64; 6])]) -> (Vec<f64>, Vec<[f64; 6]>) {
    let mut epochs = Vec::with_capacity(samples.len());
    let mut states = Vec::with_capacity(samples.len());
    for &(ep, st) in samples {
        epochs.push(ep);
        states.push(st);
    }
    (epochs, states)
}

fn convert_sampled_states(
    samples: &[(TdbEpoch, KinematicStateIcrf)],
) -> Result<(Vec<f64>, Vec<[f64; 6]>), SpiceExportError> {
    let mut epochs = Vec::with_capacity(samples.len());
    let mut states = Vec::with_capacity(samples.len());
    for &(epoch, ref kin) in samples {
        if !kin.pos_m.is_finite() || !kin.vel_m_s.is_finite() {
            return Err(SpiceExportError::InvalidConfiguration(
                "non-finite kinematic sample",
            ));
        }
        epochs.push(epoch.as_j2000_seconds_f64());
        states.push(kinematic_to_spice(kin));
    }
    Ok((epochs, states))
}
