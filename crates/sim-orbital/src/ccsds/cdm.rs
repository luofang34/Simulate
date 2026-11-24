//! CDM (CCSDS 508.0-B-1) bidirectional adapters mapping `BPlaneEncounterResult`
//! and RTN `StateCovariance6` into and out of `ccsds_ndm::Cdm`.

use ccsds_ndm::{
    Cdm, CdmAdditionalParameters, CdmCovariance, CdmData, CdmHeader, CdmMetadata, CdmObject,
    CdmRelativeMetadata, CdmRelativeStateVector, CdmSegment, CdmStateVector, CenterName, Quantity,
    RefFrame, TimeSystem, Version, vector3,
};
use glam::DVec3;
use sim_spacetime::{TdbEpoch, TimeScaleCatalog};

use super::{
    CcsdsError, CcsdsMeta, dvec3_km_to_m, dvec3_kms_to_ms, dvec3_m_to_km, dvec3_ms_to_kms,
    epoch_to_tdb, is_icrf_frame, tdb_to_epoch,
};
use crate::{
    BPlaneEncounterResult, StateCovariance6, compute_bplane_collision_probability,
    icrf_to_ric_rotation,
};

/// State, covariance, and metadata for one CDM object (`OBJECT1` or `OBJECT2`).
#[derive(Clone, Debug)]
pub struct CdmObjState<'a> {
    /// Object catalog/frame/physical metadata.
    pub config: &'a CdmObj,
    /// Inertial ICRF position at TCA (`m`).
    pub pos_m: DVec3,
    /// Inertial ICRF velocity at TCA (`m/s`).
    pub vel_m_s: DVec3,
    /// 6x6 ICRF covariance (`m^2`, `m^2/s`, `m^2/s^2`).
    pub cov_icrf: &'a StateCovariance6,
}

/// Object metadata and optional physical properties for one CDM participant (`OBJECT1` or `OBJECT2`).
#[derive(Clone, Debug, PartialEq)]
pub struct CdmObj {
    /// `OBJECT_DESIGNATOR` (catalog identifier).
    pub object_designator: String,
    /// `CATALOG_NAME` (e.g., `"SATCAT"`).
    pub catalog_name: String,
    /// `OBJECT_NAME`.
    pub object_name: String,
    /// `INTERNATIONAL_DESIGNATOR`.
    pub international_designator: String,
    /// Optional `OBJECT_TYPE` (`"PAYLOAD"`, `"DEBRIS"`, etc.).
    pub object_type: Option<String>,
    /// `MANEUVERABLE` (`"YES"`, `"NO"`, `"N/A"`).
    pub maneuverable: String,
    /// `REF_FRAME` (`RefFrame::Eme2000`, `RefFrame::Gcrf`, or `RefFrame::Icrf`).
    pub ref_frame: RefFrame,
    /// Optional hard-body collision area (`AREA_PC`, `m**2`).
    pub area_pc_m2: Option<f64>,
    /// Optional object mass (`MASS`, `kg`).
    pub mass_kg: Option<f64>,
}

/// Imported CDM encounter states, ICRF covariances, and recomputed B-plane risk metrics.
#[derive(Clone, Debug, PartialEq)]
pub struct CdmImport {
    /// Time of closest approach converted to [`TdbEpoch`].
    pub tca: TdbEpoch,
    /// Primary (`OBJECT1`) position in ICRF (`m`).
    pub primary_pos_m: DVec3,
    /// Primary (`OBJECT1`) velocity in ICRF (`m/s`).
    pub primary_vel_m_s: DVec3,
    /// Primary (`OBJECT1`) 6x6 covariance reconstructed in ICRF (`m^2`, `m^2/s`, `m^2/s^2`).
    pub primary_cov_icrf: StateCovariance6,
    /// Secondary (`OBJECT2`) position in ICRF (`m`).
    pub secondary_pos_m: DVec3,
    /// Secondary (`OBJECT2`) velocity in ICRF (`m/s`).
    pub secondary_vel_m_s: DVec3,
    /// Secondary (`OBJECT2`) 6x6 covariance reconstructed in ICRF (`m^2`, `m^2/s`, `m^2/s^2`).
    pub secondary_cov_icrf: StateCovariance6,
    /// Miss distance reported in the CDM `MISS_DISTANCE` field (`m`).
    pub reported_miss_distance_m: f64,
    /// Collision probability reported in the CDM `COLLISION_PROBABILITY` field, if present.
    pub reported_probability: Option<f64>,
    /// B-plane collision probability and Mahalanobis metrics recomputed from the imported states.
    pub recomputed: BPlaneEncounterResult,
}

/// Exports primary/secondary ICRF states, ICRF covariances, and [`BPlaneEncounterResult`] to a `Cdm`.
pub fn encounter_to_cdm(
    meta: &CcsdsMeta,
    creation_date: TdbEpoch,
    tca: TdbEpoch,
    primary: &CdmObjState<'_>,
    secondary: &CdmObjState<'_>,
    encounter: &BPlaneEncounterResult,
    scales: &TimeScaleCatalog,
) -> Result<Cdm, CcsdsError> {
    let creation_ep = match &meta.creation_date {
        Some(ep) => ep.clone(),
        None => tdb_to_epoch(creation_date, meta.time_system, scales)?,
    };
    let tca_ep = tdb_to_epoch(tca, meta.time_system, scales)?;
    let r_icrf_to_rtn = icrf_to_ric_rotation(primary.pos_m, primary.vel_m_s)?;
    let rel_pos = r_icrf_to_rtn * (secondary.pos_m - primary.pos_m);
    let rel_vel = r_icrf_to_rtn * (secondary.vel_m_s - primary.vel_m_s);
    let obj1 = build_segment(CdmObject::Object1, primary)?;
    let obj2 = build_segment(CdmObject::Object2, secondary)?;

    Ok(Cdm {
        version: Version::V1,
        header: CdmHeader {
            comments: Vec::new(),
            creation_date: creation_ep,
            originator: meta.originator.clone(),
            message_for: Some(primary.config.object_name.clone()),
            message_id: meta
                .message_id
                .clone()
                .unwrap_or_else(|| "CDM-001".to_owned()),
        },
        relative_metadata: CdmRelativeMetadata {
            comments: Vec::new(),
            tca: tca_ep,
            miss_distance: Quantity::new(encounter.miss_distance_m),
            relative_speed: Some(Quantity::new(rel_vel.length())),
            relative_state: Some(CdmRelativeStateVector {
                position: vector3([rel_pos.x, rel_pos.y, rel_pos.z]),
                velocity: vector3([rel_vel.x, rel_vel.y, rel_vel.z]),
            }),
            start_screen_period: None,
            stop_screen_period: None,
            screen_volume_frame: None,
            screen_volume_shape: None,
            screen_volume_x: None,
            screen_volume_y: None,
            screen_volume_z: None,
            screen_entry_time: None,
            screen_exit_time: None,
            collision_probability: Some(encounter.probability),
            collision_probability_method: Some("BPLANE-FOSTER-2D".to_owned()),
        },
        object1: obj1,
        object2: obj2,
        user_defined: None,
        extensions: Vec::new(),
    })
}

fn build_segment(object: CdmObject, part: &CdmObjState<'_>) -> Result<CdmSegment, CcsdsError> {
    let cfg = part.config;
    let cov_rtn = part.cov_icrf.in_ric_frame(part.pos_m, part.vel_m_s)?;
    let additional_parameters = if cfg.area_pc_m2.is_some() || cfg.mass_kg.is_some() {
        Some(CdmAdditionalParameters {
            comments: Vec::new(),
            area_pc: cfg.area_pc_m2.map(Quantity::new),
            area_drg: None,
            area_srp: None,
            mass: cfg.mass_kg.map(Quantity::new),
            cd_area_over_mass: None,
            cr_area_over_mass: None,
            thrust_acceleration: None,
            sedr: None,
        })
    } else {
        None
    };
    Ok(CdmSegment {
        metadata: CdmMetadata {
            comments: Vec::new(),
            object,
            object_designator: cfg.object_designator.clone(),
            catalog_name: cfg.catalog_name.clone(),
            object_name: cfg.object_name.clone(),
            international_designator: cfg.international_designator.clone(),
            object_type: cfg.object_type.clone(),
            operator_contact_position: None,
            operator_organization: None,
            operator_phone: None,
            operator_email: None,
            ephemeris_name: "SIMULATE".to_owned(),
            covariance_method: "CALCULATED".to_owned(),
            maneuverable: cfg.maneuverable.clone(),
            orbit_center: Some(CenterName::Earth),
            ref_frame: cfg.ref_frame.clone(),
            gravity_model: None,
            atmospheric_model: None,
            n_body_perturbations: None,
            solar_rad_pressure: None,
            earth_tides: None,
            intrack_thrust: None,
        },
        data: CdmData {
            comments: Vec::new(),
            od_parameters: None,
            additional_parameters,
            state_vector: CdmStateVector {
                comments: Vec::new(),
                position: dvec3_m_to_km(part.pos_m),
                velocity: dvec3_ms_to_kms(part.vel_m_s),
            },
            covariance: CdmCovariance::new(super::cov_si_to_ccsds(&cov_rtn, 1.0)),
        },
    })
}

/// Imports a CCSDS [`Cdm`], reconstructs both objects' ICRF covariances, and recomputes B-plane collision probability.
pub fn cdm_to_bplane_encounter(
    cdm: &Cdm,
    time_system: TimeSystem,
    combined_radius_m: f64,
    scales: &TimeScaleCatalog,
) -> Result<CdmImport, CcsdsError> {
    if !is_icrf_frame(&cdm.object1.metadata.ref_frame) {
        return Err(CcsdsError::UnsupportedFrame(
            cdm.object1.metadata.ref_frame.clone(),
        ));
    }
    if !is_icrf_frame(&cdm.object2.metadata.ref_frame) {
        return Err(CcsdsError::UnsupportedFrame(
            cdm.object2.metadata.ref_frame.clone(),
        ));
    }
    let tca = epoch_to_tdb(&cdm.relative_metadata.tca, time_system, scales)?;
    let p1 = dvec3_km_to_m(&cdm.object1.data.state_vector.position);
    let v1 = dvec3_kms_to_ms(&cdm.object1.data.state_vector.velocity);
    let p2 = dvec3_km_to_m(&cdm.object2.data.state_vector.position);
    let v2 = dvec3_kms_to_ms(&cdm.object2.data.state_vector.velocity);

    let cov1_rtn = super::cov_ccsds_to_si(&cdm.object1.data.covariance.matrix, 1.0);
    let cov2_rtn = super::cov_ccsds_to_si(&cdm.object2.data.covariance.matrix, 1.0);
    let primary_cov_icrf = StateCovariance6::from_ric_frame(&cov1_rtn, p1, v1)?;
    let secondary_cov_icrf = StateCovariance6::from_ric_frame(&cov2_rtn, p2, v2)?;
    let recomputed = compute_bplane_collision_probability(
        p1,
        v1,
        &primary_cov_icrf,
        p2,
        v2,
        &secondary_cov_icrf,
        combined_radius_m,
    )?;
    Ok(CdmImport {
        tca,
        primary_pos_m: p1,
        primary_vel_m_s: v1,
        primary_cov_icrf,
        secondary_pos_m: p2,
        secondary_vel_m_s: v2,
        secondary_cov_icrf,
        reported_miss_distance_m: cdm.relative_metadata.miss_distance.value(),
        reported_probability: cdm.relative_metadata.collision_probability,
        recomputed,
    })
}
