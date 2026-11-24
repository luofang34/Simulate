//! Round-trip integration tests for `sim-orbital` CCSDS NDM (`OPM`, `OMM`, `OEM`, `APM`, `AEM`, `CDM`) adapters.

use std::error::Error;
use std::sync::Arc;

use ccsds_ndm::{
    Aem, Apm, Cdm, CenterName, Epoch, Interpolation, Oem, Omm, Opm, ParseMode, RefFrame,
    TimeSystem, WriteOptions,
};
use glam::{DMat3, DQuat, DVec3};
use sim_orbital::{
    ApmBurn, CcsdsAdm, CcsdsCdm, CcsdsEpoch, CcsdsMeta, CcsdsOdm, CdmObj, CdmObjState,
    ContinuousPlantState, FiniteBurnPlan, KinematicStateIcrf, Sgp4CatalogTrajectory, Sgp4Mode,
    StateCovariance6, compute_bplane_collision_probability,
};
use sim_spacetime::{
    AttitudeSample, NaifId, SimEpoch, TdbEpoch, TimeScaleCatalog, TranslationalTrajectorySource,
    ValidEpochInterval,
};

type TestResult = Result<(), Box<dyn Error>>;

fn demo_meta(
    frame: RefFrame,
    body_frame: RefFrame,
    time_system: TimeSystem,
) -> Result<CcsdsMeta, Box<dyn Error>> {
    Ok(CcsdsMeta {
        object_name: "SENTINEL-6A".to_owned(),
        object_id: "2020-086A".to_owned(),
        center_name: CenterName::Earth,
        ref_frame: frame,
        body_frame,
        time_system,
        originator: "SIMULATE".to_owned(),
        creation_date: Some(Epoch::parse("2026-06-01T00:00:00")?),
        message_id: Some("MSG-001".to_owned()),
    })
}

fn sample_cov_icrf() -> StateCovariance6 {
    let m = [
        [120.0, 15.0, 0.0, 0.0, -0.05, 0.0],
        [15.0, 250.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.012, 0.0, 0.0],
        [-0.05, 0.0, 0.0, 0.0, 0.025, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.009],
    ];
    StateCovariance6::from_matrix(m)
}

#[test]
fn epoch_conversions_round_trip_across_all_scales() -> TestResult {
    let scales = TimeScaleCatalog::builtin()?;
    let tdb = TdbEpoch::new(833_600_000, 125_000_000);
    for ts in [
        TimeSystem::Utc,
        TimeSystem::Tai,
        TimeSystem::Tt,
        TimeSystem::Tdb,
    ] {
        let ep = CcsdsEpoch::from_tdb(tdb, ts, &scales)?;
        let back = CcsdsEpoch::to_tdb(&ep, ts, &scales)?;
        let dt = (back.as_j2000_seconds_f64() - tdb.as_j2000_seconds_f64()).abs();
        assert!(dt < 1.0e-5, "scale {ts} drift {dt}");
    }
    let sim = SimEpoch::from_tdb(tdb, Arc::new(scales));
    let ep_start = CcsdsEpoch::from_sim_start(&sim, TimeSystem::Utc)?;
    let utc = CcsdsEpoch::to_utc(&ep_start)?;
    let ep_utc = CcsdsEpoch::from_utc(utc)?;
    assert!((CcsdsEpoch::to_utc(&ep_utc)?.unix_seconds() - utc.unix_seconds()).abs() < 1e-6);
    Ok(())
}

#[test]
fn opm_icrf_and_rtn_covariance_round_trip() -> TestResult {
    let scales = TimeScaleCatalog::builtin()?;
    let epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let state = KinematicStateIcrf {
        pos_m: DVec3::new(6_878_137.0, 125_500.0, -2_100_250.0),
        vel_m_s: DVec3::new(1_250.0, 7_125.0, 2_012.5),
        acc_m_s2: DVec3::ZERO,
    };
    let cov = sample_cov_icrf();
    let burn = FiniteBurnPlan {
        unit_dir_icrf: DVec3::new(0.0, 1.0, 0.0),
        delta_v_m_s: 12.5,
        accel_m_s2: 0.25,
        duration_s: 50.0,
        ignition_lead_s: 0.0,
    };
    let man = CcsdsOdm::burn_plan_to_maneuver(
        epoch,
        TimeSystem::Utc,
        &burn,
        -4.2,
        RefFrame::Icrf,
        &scales,
    )?;
    let meta = demo_meta(
        RefFrame::Icrf,
        RefFrame::Other("SC_BODY_1".to_owned()),
        TimeSystem::Utc,
    )?;
    for in_rtn in [false, true] {
        let opm = CcsdsOdm::state_to_opm(
            &meta,
            epoch,
            &state,
            Some(&cov),
            in_rtn,
            std::slice::from_ref(&man),
            &scales,
        )?;
        let xml = opm.to_xml(WriteOptions::default().with_units(true))?;
        let parsed = Opm::from_xml(&xml, ParseMode::Strict)?;
        let imported = CcsdsOdm::opm_to_state(&parsed, &scales)?;
        assert!((imported.state.pos_m - state.pos_m).length() < 1.0e-6);
        assert!((imported.state.vel_m_s - state.vel_m_s).length() < 1.0e-9);
        let imp_cov_state = imported
            .covariance_icrf
            .ok_or("missing imported covariance")?;
        for (imp_row, exp_row) in imp_cov_state.as_matrix().iter().zip(cov.as_matrix().iter()) {
            for (got, expected) in imp_row.iter().zip(exp_row.iter()) {
                assert!((got - expected).abs() < 1.0e-6);
            }
        }
        assert_eq!(imported.maneuvers.len(), 1);
    }
    Ok(())
}

#[test]
fn sgp4_omm_and_oem_round_trip_match_propagation() -> TestResult {
    let scales = Arc::new(TimeScaleCatalog::builtin()?);
    let l1 = b"1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9001";
    let l2 = b"2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537";
    let start = TdbEpoch::new(820_497_600, 0);
    let interval = ValidEpochInterval::new(start, TdbEpoch::new(820_584_000, 0));
    let traj =
        Sgp4CatalogTrajectory::from_tle(1, Some("ISS"), l1, l2, interval, Arc::clone(&scales))?;
    let omm = CcsdsOdm::sgp4_trajectory_to_omm(&traj, "SIMULATE", None)?;
    let kvn = omm.to_kvn(WriteOptions::default())?;
    let parsed_omm = Omm::from_kvn(&kvn, ParseMode::Strict)?;
    let traj2 = CcsdsOdm::omm_to_sgp4_trajectory(
        2,
        &parsed_omm,
        Sgp4Mode::ModernWgs84Iau,
        interval,
        Arc::clone(&scales),
    )?;
    let probe = TdbEpoch::new(820_504_800, 0);
    let s1 = traj.sample_translational_icrf(probe)?;
    let s2 = traj2.sample_translational_icrf(probe)?;
    assert!((s1.pos - s2.pos).length() < 1.0e-9);
    assert!((s1.vel - s2.vel).length() < 1.0e-12);

    let meta = demo_meta(
        RefFrame::Gcrf,
        RefFrame::Other("SC_BODY_1".to_owned()),
        TimeSystem::Tdb,
    )?;
    let oem = CcsdsOdm::source_to_oem(
        &meta,
        &traj,
        &[start, probe],
        Some((Interpolation::Lagrange, 5)),
        &scales,
    )?;
    let xml = oem.to_xml(WriteOptions::default())?;
    let parsed_oem = Oem::from_xml(&xml, ParseMode::Strict)?;
    let states = CcsdsOdm::oem_to_states(&parsed_oem, &scales)?;
    assert_eq!(states.len(), 2);
    let second = states.get(1).ok_or("missing OEM second state")?;
    assert!((second.1.pos_m - s1.pos).length() < 1.0e-6);
    Ok(())
}

#[test]
fn apm_and_aem_round_trip_attitude_inertia_and_rates() -> TestResult {
    let scales = TimeScaleCatalog::builtin()?;
    let epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let q = DQuat::from_xyzw(0.15, -0.25, 0.35, 0.8888194417315589).normalize();
    let omega = DVec3::new(0.01, -0.02, 0.03);
    let plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(7_000_000.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_546.0, 0.0),
        q_body_to_icrf: q,
        omega_body_rad_s: omega,
        propellant_mass_kg: 120.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let inertia = DMat3::from_cols(
        DVec3::new(1200.0, -5.0, 2.0),
        DVec3::new(-5.0, 2100.0, 3.5),
        DVec3::new(2.0, 3.5, 2800.0),
    );
    let burn = ApmBurn {
        epoch_start: TdbEpoch::new(820_497_660, 0),
        duration_s: 15.0,
        torque_nm: DVec3::new(0.05, -0.02, 0.01),
        delta_mass_kg: Some(-0.05),
    };
    let meta = demo_meta(
        RefFrame::Gcrf,
        RefFrame::Other("SC_BODY_1".to_owned()),
        TimeSystem::Utc,
    )?;
    let apm = CcsdsAdm::plant_to_apm(&meta, epoch, &plant, Some(inertia), &[burn], &scales)?;
    let parsed_apm = Apm::from_kvn(&apm.to_kvn(WriteOptions::default())?, ParseMode::Strict)?;
    let imp = CcsdsAdm::apm_to_attitude(&parsed_apm, &scales)?;
    assert!(imp.q_body_to_icrf.angle_between(q) < 1.0e-12);
    assert!((imp.omega_body_rad_s.ok_or("missing omega")? - omega).length() < 1.0e-12);
    assert_eq!(imp.inertia_kg_m2.ok_or("missing inertia")?, inertia);
    assert_eq!(imp.maneuvers.len(), 1);

    let s1 = AttitudeSample {
        epoch,
        q_body_to_icrf: q,
        omega_body_rad_s: omega,
    };
    let s2 = AttitudeSample {
        epoch: TdbEpoch::new(820_497_720, 0),
        q_body_to_icrf: q,
        omega_body_rad_s: omega,
    };
    let aem = CcsdsAdm::samples_to_aem(
        &meta,
        &[s1, s2],
        true,
        Some((Interpolation::Hermite, 5)),
        &scales,
    )?;
    let parsed_aem = Aem::from_xml(&aem.to_xml(WriteOptions::default())?, ParseMode::Strict)?;
    let back = CcsdsAdm::aem_to_samples(&parsed_aem, &scales)?;
    assert_eq!(back.len(), 2);
    let b1 = back.get(1).ok_or("missing second AEM sample")?;
    assert!(b1.q_body_to_icrf.angle_between(q) < 1.0e-12);
    assert!((b1.omega_body_rad_s - omega).length() < 1.0e-12);
    Ok(())
}

#[test]
fn cdm_bplane_encounter_round_trip_preserves_risk() -> TestResult {
    let scales = TimeScaleCatalog::builtin()?;
    let tca = TdbEpoch::JAN_1_2026_00Z_TDB;
    let p1 = DVec3::new(6_878_137.0, 0.0, 0.0);
    let v1 = DVec3::new(0.0, 7_612.0, 0.0);
    let p2 = DVec3::new(6_878_152.0, 0.0, 20.0);
    let v2 = DVec3::new(0.0, -7_612.0, 500.0);
    let cov = sample_cov_icrf();
    let enc = compute_bplane_collision_probability(p1, v1, &cov, p2, v2, &cov, 15.0)?;
    let meta = demo_meta(
        RefFrame::Eme2000,
        RefFrame::Other("SC_BODY_1".to_owned()),
        TimeSystem::Utc,
    )?;
    let c1 = CdmObj {
        object_designator: "46984".to_owned(),
        catalog_name: "SATCAT".to_owned(),
        object_name: "SENTINEL-6A".to_owned(),
        international_designator: "2020-086A".to_owned(),
        object_type: Some("PAYLOAD".to_owned()),
        maneuverable: "YES".to_owned(),
        ref_frame: RefFrame::Eme2000,
        area_pc_m2: Some(12.0),
        mass_kg: Some(1192.0),
    };
    let c2 = CdmObj {
        object_designator: "99001".to_owned(),
        object_name: "DEB".to_owned(),
        international_designator: "1999-001B".to_owned(),
        object_type: Some("DEBRIS".to_owned()),
        maneuverable: "NO".to_owned(),
        ..c1.clone()
    };
    let cdm = CcsdsCdm::encounter_to_cdm(
        &meta,
        tca,
        tca,
        &CdmObjState {
            config: &c1,
            pos_m: p1,
            vel_m_s: v1,
            cov_icrf: &cov,
        },
        &CdmObjState {
            config: &c2,
            pos_m: p2,
            vel_m_s: v2,
            cov_icrf: &cov,
        },
        &enc,
        &scales,
    )?;
    let kvn = cdm.to_kvn(WriteOptions::default().with_units(true))?;
    let parsed = Cdm::from_kvn(&kvn, ParseMode::Strict)?;
    let imp = CcsdsCdm::cdm_to_encounter(&parsed, TimeSystem::Utc, 15.0, &scales)?;
    assert!((imp.recomputed.probability - enc.probability).abs() < 1.0e-12);
    assert!((imp.recomputed.miss_distance_m - enc.miss_distance_m).abs() < 1.0e-9);
    assert!((imp.recomputed.mahalanobis_distance_sq - enc.mahalanobis_distance_sq).abs() < 1.0e-9);
    Ok(())
}
