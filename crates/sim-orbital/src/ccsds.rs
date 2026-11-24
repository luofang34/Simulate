//! Bidirectional CCSDS NDM (`OPM`, `OMM`, `OEM`, `APM`, `AEM`, `CDM`) interchange adapters
//! converting between `ccsds-ndm` message types and `sim-orbital` / `sim-spacetime` SI states,
//! covariances, epochs, `SGP4` elements, attitude states, and B-plane encounter metrics.

mod adm;
mod cdm;
mod epoch;
mod odm;

use std::sync::Arc;

use ccsds_ndm::{
    Aem, Apm, Cdm, CenterName, Covariance6, Epoch, Header, Interpolation, Maneuver, Oem, Omm, Opm,
    RefFrame, TimeSystem, Vector3, vector3,
};
use glam::{DMat3, DVec3};
use sim_geo::UtcTime;
use sim_spacetime::{
    AttitudeSample, AttitudeTrajectorySource, SimEpoch, TdbEpoch, TimeError, TimeScaleCatalog,
    TranslationalTrajectorySource, ValidEpochInterval,
};

use crate::{
    BPlaneEncounterResult, ContinuousPlantState, FiniteBurnPlan, KinematicStateIcrf,
    Sgp4CatalogTrajectory, Sgp4Mode, StateCovariance6, VariationalError,
};

use epoch::{
    epoch_to_tdb, epoch_to_unix_secs_nanos, epoch_to_utc_time, sim_epoch_start_to_ccsds,
    tdb_to_epoch, unix_secs_nanos_to_epoch, utc_time_to_epoch,
};
pub use odm::OpmImport;
pub use {adm::ApmBurn, adm::ApmImport};
pub use {cdm::CdmImport, cdm::CdmObj, cdm::CdmObjState};

const KM_TO_M: f64 = 1_000.0;
const M_TO_KM: f64 = 1.0e-3;
const COV_M2_TO_KM2: f64 = 1.0e-6;
const COV_KM2_TO_M2: f64 = 1.0e6;

/// Errors returned by `sim-orbital` CCSDS message import and export adapters.
#[derive(Debug, thiserror::Error)]
pub enum CcsdsError {
    /// Underlying `ccsds-ndm` parsing or formatting error.
    #[error("CCSDS NDM error: {0}")]
    Ndm(#[from] ccsds_ndm::Error),
    /// Epoch parsing or formatting error.
    #[error("epoch error: {0}")]
    Epoch(String),
    /// Time-scale conversion error in `sim-spacetime`.
    #[error("time-scale conversion error: {0}")]
    TimeScale(#[from] TimeError),
    /// Variational frame or B-plane calculation error.
    #[error("variational covariance/B-plane error: {0}")]
    Variational(#[from] VariationalError),
    /// Unsupported CCSDS `TIME_SYSTEM`.
    #[error("unsupported CCSDS time system `{0}`")]
    UnsupportedTimeSystem(TimeSystem),
    /// Unsupported CCSDS reference frame for requested state or covariance.
    #[error("unsupported CCSDS reference frame `{0}`")]
    UnsupportedFrame(RefFrame),
    /// Missing required CCSDS data.
    #[error("missing required CCSDS data: {0}")]
    MissingData(&'static str),
    /// Non-finite numerical value encountered during conversion.
    #[error("non-finite floating-point value during CCSDS conversion")]
    NonFinite,
    /// `SGP4` construction or `OMM` element mapping error.
    #[error("SGP4/OMM adapter error: {0}")]
    Sgp4(String),
}

/// Shared object, frame, time-system, and header metadata for CCSDS exports (`OPM`, `OEM`, `APM`, `AEM`, `CDM`).
#[derive(Clone, Debug, PartialEq)]
pub struct CcsdsMeta {
    /// `OBJECT_NAME` (and optional CDM `MESSAGE_FOR`).
    pub object_name: String,
    /// `OBJECT_ID` (international designator).
    pub object_id: String,
    /// `CENTER_NAME` (`CenterName::Earth`, etc.).
    pub center_name: CenterName,
    /// Inertial frame (`REF_FRAME` for ODM, `REF_FRAME_A` for ADM).
    pub ref_frame: RefFrame,
    /// Body-fixed frame (`REF_FRAME_B` / `ANGVEL_FRAME` / `INERTIA_REF_FRAME` for ADM).
    pub body_frame: RefFrame,
    /// `TIME_SYSTEM` (`Utc`, `Tdb`, `Tai`, `Tt`).
    pub time_system: TimeSystem,
    /// Header `ORIGINATOR`.
    pub originator: String,
    /// Optional header `CREATION_DATE`.
    pub creation_date: Option<Epoch>,
    /// Optional `MESSAGE_ID`.
    pub message_id: Option<String>,
}

/// Namespace of CCSDS [`Epoch`] <-> [`TdbEpoch`] / [`UtcTime`] / [`SimEpoch`] conversions.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CcsdsEpoch;

impl CcsdsEpoch {
    /// Converts a CCSDS [`Epoch`] in `time_system` into a [`TdbEpoch`].
    pub fn to_tdb(
        epoch: &Epoch,
        time_system: TimeSystem,
        scales: &TimeScaleCatalog,
    ) -> Result<TdbEpoch, CcsdsError> {
        epoch_to_tdb(epoch, time_system, scales)
    }

    /// Converts a [`TdbEpoch`] into a CCSDS [`Epoch`] expressed in `time_system`.
    pub fn from_tdb(
        tdb: TdbEpoch,
        time_system: TimeSystem,
        scales: &TimeScaleCatalog,
    ) -> Result<Epoch, CcsdsError> {
        tdb_to_epoch(tdb, time_system, scales)
    }

    /// Converts a [`UtcTime`] directly into a CCSDS [`Epoch`].
    pub fn from_utc(utc: UtcTime) -> Result<Epoch, CcsdsError> {
        utc_time_to_epoch(utc)
    }

    /// Converts a CCSDS UTC [`Epoch`] directly into [`UtcTime`].
    pub fn to_utc(epoch: &Epoch) -> Result<UtcTime, CcsdsError> {
        epoch_to_utc_time(epoch)
    }

    /// Exports the [`SimEpoch`] anchor (`SimInstant::START`) as a CCSDS [`Epoch`].
    pub fn from_sim_start(
        sim_epoch: &SimEpoch,
        time_system: TimeSystem,
    ) -> Result<Epoch, CcsdsError> {
        sim_epoch_start_to_ccsds(sim_epoch, time_system)
    }
}

/// Namespace of CCSDS `OPM`, `OMM`, and `OEM` adapters for `sim-orbital`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CcsdsOdm;

impl CcsdsOdm {
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
        odm::state_to_opm(meta, epoch, state, cov_icrf, cov_in_rtn, maneuvers, scales)
    }

    /// Imports a CCSDS `OPM` back into `sim-orbital` state and covariance.
    pub fn opm_to_state(opm: &Opm, scales: &TimeScaleCatalog) -> Result<OpmImport, CcsdsError> {
        odm::opm_to_state(opm, scales)
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
        odm::burn_plan_to_maneuver(
            ignition_epoch,
            time_system,
            plan,
            delta_mass_kg,
            ref_frame,
            scales,
        )
    }

    /// Exports [`sgp4::Elements`] into a CCSDS `OMM`.
    pub fn sgp4_elements_to_omm(
        object_name: &str,
        international_designator: Option<&str>,
        elements: &sgp4::Elements,
        originator: &str,
        creation_date: Option<Epoch>,
    ) -> Result<Omm, CcsdsError> {
        odm::sgp4_elements_to_omm(
            object_name,
            international_designator,
            elements,
            originator,
            creation_date,
        )
    }

    /// Exports an [`Sgp4CatalogTrajectory`] directly into a CCSDS `OMM`.
    pub fn sgp4_trajectory_to_omm(
        traj: &Sgp4CatalogTrajectory,
        originator: &str,
        creation_date: Option<Epoch>,
    ) -> Result<Omm, CcsdsError> {
        odm::sgp4_trajectory_to_omm(traj, originator, creation_date)
    }

    /// Converts a CCSDS `OMM` into [`sgp4::Elements`].
    pub fn omm_to_sgp4_elements(omm: &Omm) -> Result<sgp4::Elements, CcsdsError> {
        odm::omm_to_sgp4_elements(omm)
    }

    /// Constructs an [`Sgp4CatalogTrajectory`] directly from a parsed CCSDS `OMM`.
    pub fn omm_to_sgp4_trajectory(
        trajectory_id: u64,
        omm: &Omm,
        mode: Sgp4Mode,
        valid_interval: ValidEpochInterval,
        time_scales: impl Into<Arc<TimeScaleCatalog>>,
    ) -> Result<Sgp4CatalogTrajectory, CcsdsError> {
        odm::omm_to_sgp4_trajectory(trajectory_id, omm, mode, valid_interval, time_scales)
    }

    /// Exports sampled ICRF states and covariances into a CCSDS 3.0 `OEM`.
    pub fn states_to_oem(
        meta: &CcsdsMeta,
        samples: &[(TdbEpoch, KinematicStateIcrf)],
        covariances: &[(TdbEpoch, StateCovariance6)],
        interpolation: Option<(Interpolation, u32)>,
        scales: &TimeScaleCatalog,
    ) -> Result<Oem, CcsdsError> {
        odm::states_to_oem(meta, samples, covariances, interpolation, scales)
    }

    /// Samples any [`TranslationalTrajectorySource`] at `epochs` into a CCSDS 3.0 `OEM`.
    pub fn source_to_oem(
        meta: &CcsdsMeta,
        source: &dyn TranslationalTrajectorySource,
        epochs: &[TdbEpoch],
        interpolation: Option<(Interpolation, u32)>,
        scales: &TimeScaleCatalog,
    ) -> Result<Oem, CcsdsError> {
        odm::translational_source_to_oem(meta, source, epochs, interpolation, scales)
    }

    /// Imports all ICRF state samples from a CCSDS `OEM`.
    pub fn oem_to_states(
        oem: &Oem,
        scales: &TimeScaleCatalog,
    ) -> Result<Vec<(TdbEpoch, KinematicStateIcrf)>, CcsdsError> {
        odm::oem_to_states(oem, scales)
    }
}

/// Namespace of CCSDS `APM` and `AEM` attitude adapters for `sim-orbital`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CcsdsAdm;

impl CcsdsAdm {
    /// Exports a [`ContinuousPlantState`], optional inertia, and maneuvers into a CCSDS 2.0 `APM`.
    pub fn plant_to_apm(
        meta: &CcsdsMeta,
        epoch: TdbEpoch,
        plant: &ContinuousPlantState,
        inertia_kg_m2: Option<DMat3>,
        maneuvers: &[ApmBurn],
        scales: &TimeScaleCatalog,
    ) -> Result<Apm, CcsdsError> {
        adm::plant_state_to_apm(meta, epoch, plant, inertia_kg_m2, maneuvers, scales)
    }

    /// Imports a CCSDS `APM` into `sim-orbital` attitude quaternion, body rates, and inertia.
    pub fn apm_to_attitude(apm: &Apm, scales: &TimeScaleCatalog) -> Result<ApmImport, CcsdsError> {
        adm::apm_to_attitude(apm, scales)
    }

    /// Exports `sim-spacetime` [`AttitudeSample`] points into a CCSDS 2.0 `AEM`.
    pub fn samples_to_aem(
        meta: &CcsdsMeta,
        samples: &[AttitudeSample],
        include_ang_vel: bool,
        interpolation: Option<(Interpolation, u32)>,
        scales: &TimeScaleCatalog,
    ) -> Result<Aem, CcsdsError> {
        adm::attitude_samples_to_aem(meta, samples, include_ang_vel, interpolation, scales)
    }

    /// Samples any [`AttitudeTrajectorySource`] at `epochs` into a CCSDS 2.0 `AEM`.
    pub fn source_to_aem(
        meta: &CcsdsMeta,
        source: &dyn AttitudeTrajectorySource,
        epochs: &[TdbEpoch],
        include_ang_vel: bool,
        interpolation: Option<(Interpolation, u32)>,
        scales: &TimeScaleCatalog,
    ) -> Result<Aem, CcsdsError> {
        adm::attitude_source_to_aem(meta, source, epochs, include_ang_vel, interpolation, scales)
    }

    /// Imports all quaternion/Euler attitude samples from a CCSDS `AEM`.
    pub fn aem_to_samples(
        aem: &Aem,
        scales: &TimeScaleCatalog,
    ) -> Result<Vec<AttitudeSample>, CcsdsError> {
        adm::aem_to_attitude_samples(aem, scales)
    }
}

/// Namespace of CCSDS `CDM` conjunction adapters for `sim-orbital`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CcsdsCdm;

impl CcsdsCdm {
    /// Exports primary/secondary ICRF states, covariances, and [`BPlaneEncounterResult`] to a `Cdm`.
    pub fn encounter_to_cdm(
        meta: &CcsdsMeta,
        creation_date: TdbEpoch,
        tca: TdbEpoch,
        primary: &CdmObjState<'_>,
        secondary: &CdmObjState<'_>,
        encounter: &BPlaneEncounterResult,
        scales: &TimeScaleCatalog,
    ) -> Result<Cdm, CcsdsError> {
        cdm::encounter_to_cdm(
            meta,
            creation_date,
            tca,
            primary,
            secondary,
            encounter,
            scales,
        )
    }

    /// Imports a CCSDS [`Cdm`] and recomputes B-plane collision probability.
    pub fn cdm_to_encounter(
        cdm: &Cdm,
        time_system: TimeSystem,
        combined_radius_m: f64,
        scales: &TimeScaleCatalog,
    ) -> Result<CdmImport, CcsdsError> {
        cdm::cdm_to_bplane_encounter(cdm, time_system, combined_radius_m, scales)
    }
}

pub(crate) fn dvec3_m_to_km(v: DVec3) -> Vector3<ccsds_ndm::Km> {
    vector3([v.x * M_TO_KM, v.y * M_TO_KM, v.z * M_TO_KM])
}

pub(crate) fn dvec3_km_to_m(v: &Vector3<ccsds_ndm::Km>) -> DVec3 {
    let [x, y, z] = *v;
    DVec3::new(
        x.value() * KM_TO_M,
        y.value() * KM_TO_M,
        z.value() * KM_TO_M,
    )
}

pub(crate) fn dvec3_ms_to_kms(v: DVec3) -> Vector3<ccsds_ndm::KmPerS> {
    vector3([v.x * M_TO_KM, v.y * M_TO_KM, v.z * M_TO_KM])
}

pub(crate) fn dvec3_kms_to_ms(v: &Vector3<ccsds_ndm::KmPerS>) -> DVec3 {
    let [x, y, z] = *v;
    DVec3::new(
        x.value() * KM_TO_M,
        y.value() * KM_TO_M,
        z.value() * KM_TO_M,
    )
}

pub(crate) fn cov_si_to_ccsds(cov: &StateCovariance6, scale: f64) -> Covariance6 {
    let m = cov.as_matrix();
    let mut lower = [0.0; 21];
    let mut idx = 0usize;
    for (r, row) in m.iter().enumerate() {
        for val in row.iter().take(r.wrapping_add(1)) {
            if let Some(slot) = lower.get_mut(idx) {
                *slot = *val * scale;
            }
            idx = idx.wrapping_add(1);
        }
    }
    Covariance6::from_lower(lower)
}

pub(crate) fn cov_ccsds_to_si(cov: &Covariance6, scale: f64) -> StateCovariance6 {
    let mut mat = cov.to_matrix();
    for row in &mut mat {
        for cell in row.iter_mut() {
            *cell *= scale;
        }
    }
    StateCovariance6::from_matrix(mat)
}

pub(crate) fn build_header(
    creation_date: Option<Epoch>,
    originator: &str,
    message_id: Option<&str>,
) -> Header {
    Header {
        comments: Vec::new(),
        classification: None,
        creation_date,
        originator: Some(originator.to_owned()),
        message_id: message_id.map(str::to_owned),
    }
}

pub(crate) fn is_icrf_frame(frame: &RefFrame) -> bool {
    matches!(frame, RefFrame::Icrf | RefFrame::Gcrf | RefFrame::Eme2000)
}

pub(crate) fn is_rtn_frame(frame: &RefFrame) -> bool {
    matches!(frame, RefFrame::Rtn | RefFrame::Rsw)
}
