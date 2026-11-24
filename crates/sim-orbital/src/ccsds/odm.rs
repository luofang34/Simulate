//! ODM (`OPM`, `OMM`, `OEM`) bidirectional adapters for `sim-orbital`.

use std::sync::Arc;

use ccsds_ndm::{
    CenterName, CovarianceBlock, EphemerisCovariance, EphemerisState, Interpolation, Maneuver,
    MeanElementTheory, MeanElements, Oem, OemData, OemMetadata, OemSegment, Omm, OmmData,
    OmmMetadata, Opm, OpmData, OpmMetadata, OrbitSize, PerEarthRadius, Quantity, RefFrame,
    RevPerDay, RevPerDay2, RevPerDay3, StateVector, TimeSystem, TleDrag, TleParameters,
    TleSecondTerm, Version, vector3,
};
use glam::DVec3;
use sim_spacetime::{
    TdbEpoch, TimeScaleCatalog, TranslationalTrajectorySource, ValidEpochInterval,
};

use super::{
    COV_KM2_TO_M2, COV_M2_TO_KM2, CcsdsError, CcsdsMeta, build_header, cov_ccsds_to_si,
    cov_si_to_ccsds, dvec3_km_to_m, dvec3_kms_to_ms, dvec3_m_to_km, dvec3_ms_to_kms, epoch_to_tdb,
    epoch_to_unix_secs_nanos, is_icrf_frame, is_rtn_frame, tdb_to_epoch, unix_secs_nanos_to_epoch,
};
use crate::{
    FiniteBurnPlan, KinematicStateIcrf, Sgp4CatalogTrajectory, Sgp4Mode, StateCovariance6,
};

const M_TO_KM: f64 = 1.0e-3;
const KM_TO_M: f64 = 1_000.0;

/// Imported `OPM` translational state, frame, optional ICRF covariance, and maneuvers.
#[derive(Clone, Debug, PartialEq)]
pub struct OpmImport {
    /// State epoch converted to [`TdbEpoch`].
    pub epoch: TdbEpoch,
    /// Translational state vector in SI meters and `m/s` (`acc_m_s2 = ZERO`).
    pub state: KinematicStateIcrf,
    /// `REF_FRAME` reported by the `OPM`.
    pub ref_frame: RefFrame,
    /// Optional 6x6 covariance mapped into ICRF SI (`m^2`, `m^2/s`, `m^2/s^2`).
    pub covariance_icrf: Option<StateCovariance6>,
    /// Optional spacecraft mass (`kg`).
    pub mass_kg: Option<f64>,
    /// Raw CCSDS maneuver entries preserved from the `OPM`.
    pub maneuvers: Vec<Maneuver>,
}

/// Exports an ICRF state, optional covariance, and maneuvers to a CCSDS 3.0 `OPM`.
pub fn state_to_opm(
    meta: &CcsdsMeta,
    epoch: TdbEpoch,
    state: &KinematicStateIcrf,
    cov_icrf: Option<&StateCovariance6>,
    cov_in_rtn: bool,
    maneuvers: &[Maneuver],
    scales: &TimeScaleCatalog,
) -> Result<Opm, CcsdsError> {
    let epoch_ccsds = tdb_to_epoch(epoch, meta.time_system, scales)?;
    let covariance = match cov_icrf {
        Some(cov) if cov_in_rtn => {
            let ric = cov.in_ric_frame(state.pos_m, state.vel_m_s)?;
            Some(CovarianceBlock {
                comments: Vec::new(),
                ref_frame: Some(RefFrame::Rtn),
                matrix: cov_si_to_ccsds(&ric, COV_M2_TO_KM2),
            })
        }
        Some(cov) => Some(CovarianceBlock {
            comments: Vec::new(),
            ref_frame: Some(meta.ref_frame.clone()),
            matrix: cov_si_to_ccsds(cov, COV_M2_TO_KM2),
        }),
        None => None,
    };
    Ok(Opm {
        version: Version::V3,
        header: build_header(
            meta.creation_date.clone(),
            &meta.originator,
            meta.message_id.as_deref(),
        ),
        metadata: OpmMetadata {
            comments: Vec::new(),
            object_name: meta.object_name.clone(),
            object_id: meta.object_id.clone(),
            center_name: meta.center_name.clone(),
            ref_frame: meta.ref_frame.clone(),
            ref_frame_epoch: None,
            time_system: meta.time_system,
        },
        data: OpmData {
            state: StateVector {
                comments: Vec::new(),
                epoch: epoch_ccsds,
                position: dvec3_m_to_km(state.pos_m),
                velocity: dvec3_ms_to_kms(state.vel_m_s),
            },
            keplerian: None,
            spacecraft: None,
            covariance,
            maneuvers: maneuvers.to_vec(),
            user_defined: None,
        },
        extensions: Vec::new(),
    })
}

/// Imports a CCSDS `OPM` back into `sim-orbital` [`KinematicStateIcrf`] and [`StateCovariance6`].
pub fn opm_to_state(opm: &Opm, scales: &TimeScaleCatalog) -> Result<OpmImport, CcsdsError> {
    if !is_icrf_frame(&opm.metadata.ref_frame) {
        return Err(CcsdsError::UnsupportedFrame(opm.metadata.ref_frame.clone()));
    }
    let epoch = epoch_to_tdb(&opm.data.state.epoch, opm.metadata.time_system, scales)?;
    let pos_m = dvec3_km_to_m(&opm.data.state.position);
    let vel_m_s = dvec3_kms_to_ms(&opm.data.state.velocity);
    let covariance_icrf = match &opm.data.covariance {
        Some(cb) => {
            let cov_frame = cb.ref_frame.as_ref().unwrap_or(&opm.metadata.ref_frame);
            let raw = cov_ccsds_to_si(&cb.matrix, COV_KM2_TO_M2);
            if is_rtn_frame(cov_frame) {
                Some(StateCovariance6::from_ric_frame(&raw, pos_m, vel_m_s)?)
            } else if is_icrf_frame(cov_frame) {
                Some(raw)
            } else {
                return Err(CcsdsError::UnsupportedFrame(cov_frame.clone()));
            }
        }
        None => None,
    };
    Ok(OpmImport {
        epoch,
        state: KinematicStateIcrf {
            pos_m,
            vel_m_s,
            acc_m_s2: DVec3::ZERO,
        },
        ref_frame: opm.metadata.ref_frame.clone(),
        covariance_icrf,
        mass_kg: opm
            .data
            .spacecraft
            .as_ref()
            .and_then(|s| s.mass)
            .map(Quantity::value),
        maneuvers: opm.data.maneuvers.clone(),
    })
}

/// Converts a `sim-orbital` [`FiniteBurnPlan`] into a CCSDS `OPM` [`Maneuver`].
pub fn burn_plan_to_maneuver(
    ignition_epoch: TdbEpoch,
    time_system: TimeSystem,
    plan: &FiniteBurnPlan,
    delta_mass_kg: f64,
    ref_frame: RefFrame,
    scales: &TimeScaleCatalog,
) -> Result<Maneuver, CcsdsError> {
    let epoch_ignition = tdb_to_epoch(ignition_epoch, time_system, scales)?;
    let dv_km_s = plan.unit_dir_icrf * (plan.delta_v_m_s * M_TO_KM);
    Ok(Maneuver {
        comments: Vec::new(),
        epoch_ignition,
        duration: Quantity::new(plan.duration_s),
        delta_mass: Quantity::new(delta_mass_kg),
        ref_frame,
        delta_v: vector3([dv_km_s.x, dv_km_s.y, dv_km_s.z]),
    })
}

/// Exports [`sgp4::Elements`] into a CCSDS `OMM`.
pub fn sgp4_elements_to_omm(
    object_name: &str,
    international_designator: Option<&str>,
    elements: &sgp4::Elements,
    originator: &str,
    creation_date: Option<ccsds_ndm::Epoch>,
) -> Result<Omm, CcsdsError> {
    let dt_utc = elements.datetime.and_utc();
    let epoch = unix_secs_nanos_to_epoch(dt_utc.timestamp(), dt_utc.timestamp_subsec_nanos())?;
    let object_id = international_designator
        .map(str::to_owned)
        .or_else(|| elements.international_designator.clone())
        .unwrap_or_else(|| format!("NORAD-{}", elements.norad_id));
    let class = match elements.classification {
        sgp4::Classification::Unclassified => "U",
        sgp4::Classification::Classified => "C",
        sgp4::Classification::Secret => "S",
    };
    Ok(Omm {
        version: Some(Version::V3),
        header: build_header(
            creation_date.or_else(|| Some(epoch.clone())),
            originator,
            None,
        ),
        metadata: OmmMetadata {
            comments: Vec::new(),
            object_name: elements
                .object_name
                .clone()
                .unwrap_or_else(|| object_name.to_owned()),
            object_id,
            center_name: Some(CenterName::Earth),
            ref_frame: Some(RefFrame::Teme),
            ref_frame_epoch: None,
            time_system: Some(TimeSystem::Utc),
            mean_element_theory: Some(MeanElementTheory::Sgp4),
        },
        data: OmmData {
            mean_elements: MeanElements {
                comments: Vec::new(),
                epoch,
                size: OrbitSize::MeanMotion(Quantity::<RevPerDay>::new(elements.mean_motion)),
                eccentricity: elements.eccentricity,
                inclination: Quantity::new(elements.inclination),
                ra_of_asc_node: Quantity::new(elements.right_ascension),
                arg_of_pericenter: Quantity::new(elements.argument_of_perigee),
                mean_anomaly: Quantity::new(elements.mean_anomaly),
                gm: None,
            },
            spacecraft: None,
            tle: Some(TleParameters {
                comments: Vec::new(),
                ephemeris_type: Some(i32::from(elements.ephemeris_type)),
                classification_type: Some(class.to_owned()),
                norad_cat_id: u32::try_from(elements.norad_id).ok(),
                element_set_no: u32::try_from(elements.element_set_number).ok(),
                rev_at_epoch: u32::try_from(elements.revolution_number).ok(),
                drag: Some(TleDrag::Bstar(Quantity::<PerEarthRadius>::new(
                    elements.drag_term,
                ))),
                mean_motion_dot: Some(Quantity::<RevPerDay2>::new(elements.mean_motion_dot)),
                second_term: Some(TleSecondTerm::MeanMotionDdot(Quantity::<RevPerDay3>::new(
                    elements.mean_motion_ddot,
                ))),
            }),
            covariance: None,
            user_defined: None,
        },
        extensions: Vec::new(),
    })
}

/// Exports an [`Sgp4CatalogTrajectory`] directly into a CCSDS `OMM`.
pub fn sgp4_trajectory_to_omm(
    traj: &Sgp4CatalogTrajectory,
    originator: &str,
    creation_date: Option<ccsds_ndm::Epoch>,
) -> Result<Omm, CcsdsError> {
    let el = traj.elements();
    let name = el.object_name.as_deref().unwrap_or("SGP4-OBJECT");
    sgp4_elements_to_omm(
        name,
        el.international_designator.as_deref(),
        el,
        originator,
        creation_date,
    )
}

/// Converts a CCSDS `OMM` into [`sgp4::Elements`], preserving `f64` orbital parameters bit-for-bit.
pub fn omm_to_sgp4_elements(omm: &Omm) -> Result<sgp4::Elements, CcsdsError> {
    let me = &omm.data.mean_elements;
    let OrbitSize::MeanMotion(mean_motion) = me.size else {
        return Err(CcsdsError::MissingData("MEAN_MOTION required for SGP4 OMM"));
    };
    let (secs, nanos) = epoch_to_unix_secs_nanos(&me.epoch)?;
    let cal_epoch = unix_secs_nanos_to_epoch(secs, nanos)?;
    let tle = omm.data.tle.as_ref();
    let norad_id = tle.and_then(|t| t.norad_cat_id).unwrap_or(0);
    let rev = tle.and_then(|t| t.rev_at_epoch).unwrap_or(0);
    let el_set = tle.and_then(|t| t.element_set_no).unwrap_or(999);
    let class = tle
        .and_then(|t| t.classification_type.as_deref())
        .unwrap_or("U");
    let eph_type = tle.and_then(|t| t.ephemeris_type).unwrap_or(0);
    let bstar = match tle.and_then(|t| t.drag) {
        Some(TleDrag::Bstar(q)) => q.value(),
        _ => 0.0,
    };
    let ndot = tle
        .and_then(|t| t.mean_motion_dot)
        .map_or(0.0, Quantity::value);
    let nddot = match tle.and_then(|t| t.second_term) {
        Some(TleSecondTerm::MeanMotionDdot(q)) => q.value(),
        _ => 0.0,
    };
    let json = serde_json::json!({
        "OBJECT_NAME": omm.metadata.object_name,
        "OBJECT_ID": omm.metadata.object_id,
        "EPOCH": cal_epoch.to_string(),
        "MEAN_MOTION": mean_motion.value(),
        "ECCENTRICITY": me.eccentricity,
        "INCLINATION": me.inclination.value(),
        "RA_OF_ASC_NODE": me.ra_of_asc_node.value(),
        "ARG_OF_PERICENTER": me.arg_of_pericenter.value(),
        "MEAN_ANOMALY": me.mean_anomaly.value(),
        "EPHEMERIS_TYPE": eph_type,
        "CLASSIFICATION_TYPE": class,
        "NORAD_CAT_ID": norad_id,
        "ELEMENT_SET_NO": el_set,
        "REV_AT_EPOCH": rev,
        "BSTAR": bstar,
        "MEAN_MOTION_DOT": ndot,
        "MEAN_MOTION_DDOT": nddot,
    });
    let mut elements: sgp4::Elements =
        serde_json::from_value(json).map_err(|e| CcsdsError::Sgp4(e.to_string()))?;
    elements.mean_motion = mean_motion.value();
    elements.eccentricity = me.eccentricity;
    elements.inclination = me.inclination.value();
    elements.right_ascension = me.ra_of_asc_node.value();
    elements.argument_of_perigee = me.arg_of_pericenter.value();
    elements.mean_anomaly = me.mean_anomaly.value();
    elements.drag_term = bstar;
    elements.mean_motion_dot = ndot;
    elements.mean_motion_ddot = nddot;
    Ok(elements)
}

/// Constructs an [`Sgp4CatalogTrajectory`] directly from a parsed CCSDS `OMM`.
pub fn omm_to_sgp4_trajectory(
    trajectory_id: u64,
    omm: &Omm,
    mode: Sgp4Mode,
    valid_interval: ValidEpochInterval,
    time_scales: impl Into<Arc<TimeScaleCatalog>>,
) -> Result<Sgp4CatalogTrajectory, CcsdsError> {
    let elements = omm_to_sgp4_elements(omm)?;
    Sgp4CatalogTrajectory::from_elements_with_mode(
        trajectory_id,
        elements,
        mode,
        valid_interval,
        time_scales,
    )
    .map_err(|e| CcsdsError::Sgp4(e.to_string()))
}

/// Exports sampled ICRF states and optional covariances to a single-segment CCSDS 3.0 `OEM`.
pub fn states_to_oem(
    meta: &CcsdsMeta,
    samples: &[(TdbEpoch, KinematicStateIcrf)],
    covariances: &[(TdbEpoch, StateCovariance6)],
    interpolation: Option<(Interpolation, u32)>,
    scales: &TimeScaleCatalog,
) -> Result<Oem, CcsdsError> {
    let (Some((first_epoch, _)), Some((last_epoch, _))) = (samples.first(), samples.last()) else {
        return Err(CcsdsError::MissingData("at least one OEM sample"));
    };
    let mut states = Vec::with_capacity(samples.len());
    for (epoch, st) in samples {
        states.push(build_oem_state(*epoch, st, meta.time_system, scales)?);
    }
    let mut covs = Vec::with_capacity(covariances.len());
    for (epoch, cov) in covariances {
        covs.push(EphemerisCovariance {
            comments: Vec::new(),
            epoch: tdb_to_epoch(*epoch, meta.time_system, scales)?,
            ref_frame: Some(meta.ref_frame.clone()),
            matrix: cov_si_to_ccsds(cov, COV_M2_TO_KM2),
        });
    }
    let (interp, degree) = match interpolation {
        Some((m, d)) => (Some(m), Some(d)),
        None => (None, None),
    };
    Ok(Oem {
        version: Version::V3,
        header: build_header(
            meta.creation_date.clone(),
            &meta.originator,
            meta.message_id.as_deref(),
        ),
        segments: vec![OemSegment {
            metadata: OemMetadata {
                comments: Vec::new(),
                object_name: meta.object_name.clone(),
                object_id: meta.object_id.clone(),
                center_name: meta.center_name.clone(),
                ref_frame: meta.ref_frame.clone(),
                ref_frame_epoch: None,
                time_system: meta.time_system,
                start_time: tdb_to_epoch(*first_epoch, meta.time_system, scales)?,
                useable_start_time: None,
                useable_stop_time: None,
                stop_time: tdb_to_epoch(*last_epoch, meta.time_system, scales)?,
                interpolation: interp,
                interpolation_degree: degree,
            },
            data: OemData {
                comments: Vec::new(),
                states,
                covariances: covs,
            },
        }],
        extensions: Vec::new(),
    })
}

fn build_oem_state(
    epoch: TdbEpoch,
    st: &KinematicStateIcrf,
    time_system: TimeSystem,
    scales: &TimeScaleCatalog,
) -> Result<EphemerisState, CcsdsError> {
    let acceleration = if st.acc_m_s2 != DVec3::ZERO {
        Some(vector3([
            st.acc_m_s2.x * M_TO_KM,
            st.acc_m_s2.y * M_TO_KM,
            st.acc_m_s2.z * M_TO_KM,
        ]))
    } else {
        None
    };
    Ok(EphemerisState {
        epoch: tdb_to_epoch(epoch, time_system, scales)?,
        position: dvec3_m_to_km(st.pos_m),
        velocity: dvec3_ms_to_kms(st.vel_m_s),
        acceleration,
    })
}

/// Samples any [`TranslationalTrajectorySource`] at `epochs` into a CCSDS 3.0 `OEM`.
pub fn translational_source_to_oem(
    meta: &CcsdsMeta,
    source: &dyn TranslationalTrajectorySource,
    epochs: &[TdbEpoch],
    interpolation: Option<(Interpolation, u32)>,
    scales: &TimeScaleCatalog,
) -> Result<Oem, CcsdsError> {
    let mut samples = Vec::with_capacity(epochs.len());
    for &epoch in epochs {
        let framed = source
            .sample_translational_icrf(epoch)
            .map_err(|e| CcsdsError::Sgp4(e.to_string()))?;
        samples.push((
            epoch,
            KinematicStateIcrf {
                pos_m: framed.pos,
                vel_m_s: framed.vel,
                acc_m_s2: DVec3::ZERO,
            },
        ));
    }
    states_to_oem(meta, &samples, &[], interpolation, scales)
}

/// Imports all ICRF state samples across segments of a CCSDS `OEM`.
pub fn oem_to_states(
    oem: &Oem,
    scales: &TimeScaleCatalog,
) -> Result<Vec<(TdbEpoch, KinematicStateIcrf)>, CcsdsError> {
    let mut out = Vec::new();
    for seg in &oem.segments {
        if !is_icrf_frame(&seg.metadata.ref_frame) {
            return Err(CcsdsError::UnsupportedFrame(seg.metadata.ref_frame.clone()));
        }
        for st in &seg.data.states {
            let epoch = epoch_to_tdb(&st.epoch, seg.metadata.time_system, scales)?;
            let acc_m_s2 = match st.acceleration {
                Some([ax, ay, az]) => DVec3::new(
                    ax.value() * KM_TO_M,
                    ay.value() * KM_TO_M,
                    az.value() * KM_TO_M,
                ),
                None => DVec3::ZERO,
            };
            out.push((
                epoch,
                KinematicStateIcrf {
                    pos_m: dvec3_km_to_m(&st.position),
                    vel_m_s: dvec3_kms_to_ms(&st.velocity),
                    acc_m_s2,
                },
            ));
        }
    }
    Ok(out)
}
