//! Verification tests for the catalog-tier `SGP4`/`SDP4` propagator (`Sgp4CatalogTrajectory`).

use std::error::Error;

use glam::{DMat3, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, Dop853Tolerances, EphemerisCatalog,
    ForceModel, MassPropertiesModel, PlanningInputFingerprint, PropellantMode, ReactionWheelMode,
    Sgp4CatalogTrajectory, Sgp4Error, Sgp4Mode, SimulationManifestDigest, TdbEpoch,
    TimeScaleCatalog, VesselConfig, teme_to_icrf_rotation,
};
use sim_spacetime::{
    ContentHash, DynamicalModelIdentity, FrameAxesSpec, NaifId, TrajectoryQueryError,
    TranslationalTrajectorySource, UnitSemantics, ValidEpochInterval,
};

const TLE_00005_1: &[u8] = b"1 00005U 58002B   00179.78495062  .00000023  00000-0  28098-4 0  4753";
const TLE_00005_2: &[u8] = b"2 00005  34.2682 348.7242 1859667 331.7664  19.3264 10.82419157413667";

const TLE_04632_1: &[u8] = b"1 04632U 70093B   04031.91070959 -.00000084  00000-0  10000-3 0  9955";
const TLE_04632_2: &[u8] = b"2 04632  11.4628 273.1101 1450506 207.6000 143.9350  1.20231981 44145";

const TLE_06251_1: &[u8] = b"1 06251U 62025E   06176.82412014  .00008885  00000-0  12808-3 0  3985";
const TLE_06251_2: &[u8] = b"2 06251  58.0579  54.0425 0030035 139.1568 221.1854 15.56387291  6774";

const TLE_23599_1: &[u8] = b"1 23599U 95029B   06171.76535463  .00085586  12891-6  12956-2 0  2905";
const TLE_23599_2: &[u8] = b"2 23599   6.9327   0.2849 5782022 274.4436  25.2425  4.47796565123555";

const TLE_28057_1: &[u8] = b"1 28057U 03049A   06177.78615833  .00000060  00000-0  35940-4 0  1836";
const TLE_28057_2: &[u8] = b"2 28057  98.4283 247.6961 0000884  88.1964 271.9322 14.35478080140550";

fn assert_vallado_points(
    line1: &[u8],
    line2: &[u8],
    cases: &[(f64, [f64; 3], [f64; 3])],
) -> Result<(), Box<dyn Error>> {
    let ts = TimeScaleCatalog::builtin()?;
    let window = ValidEpochInterval::new(TdbEpoch::J2000, TdbEpoch::JAN_1_2028_00Z_TDB);
    let traj = Sgp4CatalogTrajectory::from_tle_with_mode(
        1,
        None,
        line1,
        line2,
        Sgp4Mode::AfspcCompatibilityWgs72,
        window,
        ts,
    )?;
    for &(t_min, ref_pos, ref_vel) in cases {
        let state = traj.propagate_teme_minutes(t_min)?;
        let dp = (state.position_km - DVec3::from_array(ref_pos)).abs();
        let dv = (state.velocity_km_s - DVec3::from_array(ref_vel)).abs();
        assert!(dp.max_element() < 1.0e-6, "pos err {dp:?} at t={t_min}");
        assert!(dv.max_element() < 1.0e-9, "vel err {dv:?} at t={t_min}");
    }
    Ok(())
}

#[test]
fn vallado_near_earth_reference_vectors_match() -> Result<(), Box<dyn Error>> {
    assert_vallado_points(
        TLE_06251_1,
        TLE_06251_2,
        &[
            (
                0.0,
                [3988.31022699, 5498.96657235, 0.90055879],
                [-3.290032738, 2.357652820, 6.496623475],
            ),
            (
                720.0,
                [3692.60030028, -976.24265255, -5623.36447493],
                [3.897257243, 6.415554948, 1.429112190],
            ),
        ],
    )?;
    assert_vallado_points(
        TLE_28057_1,
        TLE_28057_2,
        &[
            (
                0.0,
                [-2715.28237486, -6619.26436889, -0.01341443],
                [-1.008587273, 0.422782003, 7.385272942],
            ),
            (
                720.0,
                [-2090.79884266, -2723.22832193, 6266.13356576],
                [1.992640665, 6.337529519, 3.411803080],
            ),
        ],
    )?;
    Ok(())
}

#[test]
fn vallado_deep_space_reference_vectors_match() -> Result<(), Box<dyn Error>> {
    assert_vallado_points(
        TLE_00005_1,
        TLE_00005_2,
        &[
            (
                0.0,
                [7022.46529266, -1400.08296755, 0.03995155],
                [1.893841015, 6.405893759, 4.534807250],
            ),
            (
                4320.0,
                [-9060.47373569, 4658.70952502, 813.68673153],
                [-2.232832783, -4.110453490, -3.157345433],
            ),
        ],
    )?;
    assert_vallado_points(
        TLE_04632_1,
        TLE_04632_2,
        &[
            (
                0.0,
                [2334.11450085, -41920.44035349, -0.03867437],
                [2.826321032, -0.065091664, 0.570936053],
            ),
            (
                -5184.0,
                [-29020.02587128, 13819.84419063, -5713.33679183],
                [-1.768068390, -3.235371192, -0.395206135],
            ),
        ],
    )?;
    assert_vallado_points(
        TLE_23599_1,
        TLE_23599_2,
        &[(
            100.0,
            [3933.37509798, 23024.07662542, 2798.25966746],
            [-2.542860616, 1.327134966, 0.162450076],
        )],
    )?;
    Ok(())
}

#[test]
fn omm_json_construction_and_modern_mode_succeed() -> Result<(), Box<dyn Error>> {
    let ts = TimeScaleCatalog::builtin()?;
    let window = ValidEpochInterval::new(TdbEpoch::J2000, TdbEpoch::JAN_1_2028_00Z_TDB);
    let omm = r#"{
        "OBJECT_NAME": "ISS (ZARYA)",
        "OBJECT_ID": "1998-067A",
        "EPOCH": "2020-07-12T01:19:07.402656",
        "MEAN_MOTION": 15.49560532,
        "ECCENTRICITY": 0.0001771,
        "INCLINATION": 51.6435,
        "RA_OF_ASC_NODE": 225.4004,
        "ARG_OF_PERICENTER": 44.9625,
        "MEAN_ANOMALY": 5.1087,
        "EPHEMERIS_TYPE": 0,
        "CLASSIFICATION_TYPE": "U",
        "NORAD_CAT_ID": 25544,
        "ELEMENT_SET_NO": 999,
        "REV_AT_EPOCH": 23587,
        "BSTAR": 0.0049645,
        "MEAN_MOTION_DOT": 0.00289036,
        "MEAN_MOTION_DDOT": 0
    }"#;
    let traj = Sgp4CatalogTrajectory::from_omm_json(25544, omm, window, ts)?;
    assert_eq!(traj.elements().norad_id, 25544);
    let q_epoch = TdbEpoch::new(647_790_000, 0);
    let f1 = traj.sample_translational_icrf(q_epoch)?;
    let f2 = traj.sample_translational_icrf(q_epoch)?;
    assert_eq!(f1, f2);
    assert!(f1.pos.length() > 6_500_000.0 && f1.pos.length() < 7_200_000.0);
    assert!(f1.vel.length() > 7_000.0 && f1.vel.length() < 8_000.0);
    Ok(())
}

#[test]
fn invalid_tle_and_elements_fail_cleanly() -> Result<(), Box<dyn Error>> {
    let ts = TimeScaleCatalog::builtin()?;
    let window = ValidEpochInterval::new(TdbEpoch::J2000, TdbEpoch::JAN_1_2028_00Z_TDB);
    let bad = Sgp4CatalogTrajectory::from_tle(1, None, b"bad1", b"bad2", window, ts.clone());
    assert!(matches!(bad, Err(Sgp4Error::InvalidTle(_))));
    let bad_omm = Sgp4CatalogTrajectory::from_omm_json(1, "{}", window, ts);
    assert!(matches!(bad_omm, Err(Sgp4Error::InvalidOmm(_))));
    Ok(())
}

#[test]
fn teme_to_icrf_rotation_is_orthonormal_and_physically_oriented() {
    let epochs = [
        TdbEpoch::J2000,
        TdbEpoch::JAN_1_2026_00Z_TDB,
        TdbEpoch::new(3_155_760_000, 0),
    ];
    for epoch in epochs {
        let q = teme_to_icrf_rotation(epoch);
        let err = (q.transpose() * q - DMat3::IDENTITY).abs_diff_eq(DMat3::ZERO, 1.0e-14);
        assert!(err, "rotation not orthogonal at {epoch:?}");
        assert!((q.determinant() - 1.0).abs() < 1.0e-14);
    }
    let q_j2000 = teme_to_icrf_rotation(TdbEpoch::J2000);
    let d_j2000 = (q_j2000 * DVec3::X - DVec3::X).length();
    assert!(d_j2000 > 1.0e-5 && d_j2000 < 2.0e-4);

    let q_1cy = teme_to_icrf_rotation(TdbEpoch::new(3_155_760_000, 0));
    let shifted_x = q_1cy * DVec3::X;
    assert!((shifted_x.y.abs() - 0.022_36).abs() < 5.0e-4);
    assert!((shifted_x.z.abs() - 0.009_72).abs() < 5.0e-4);
}

#[test]
fn trajectory_source_contract_and_coverage_enforced() -> Result<(), Box<dyn Error>> {
    let ts = TimeScaleCatalog::builtin()?;
    let start = TdbEpoch::new(204_800_000, 0);
    let end = TdbEpoch::new(204_810_000, 0);
    let traj = Sgp4CatalogTrajectory::from_tle(
        77,
        Some("CAL-SAT"),
        TLE_28057_1,
        TLE_28057_2,
        ValidEpochInterval::new(start, end),
        ts.clone(),
    )?;
    let src: &dyn TranslationalTrajectorySource = &traj;
    assert_eq!(src.trajectory_id(), 77);
    assert!(src.model_identity().supports_physical_ephemeris_analysis());
    assert!(src.model_identity().is_catalog_tier());

    let state = src.sample_translational_icrf(start)?;
    assert_eq!(state.spec.origin, NaifId::EARTH);
    assert_eq!(state.spec.axes, FrameAxesSpec::IcrfInertial);
    assert_eq!(state.spec.units, UnitSemantics::SiMetric);

    let dt_elem = traj.elements().datetime.and_utc();
    let elem_s = dt_elem.timestamp() as f64 + f64::from(dt_elem.timestamp_subsec_nanos()) * 1.0e-9;
    let query_min = (ts.tdb_to_utc(start)?.unix_seconds() - elem_s) / 60.0;
    let teme = traj.propagate_teme_minutes(query_min)?;
    assert!((state.pos.length() - teme.position_km.length() * 1_000.0).abs() < 1.0e-7);
    assert!((state.vel.length() - teme.velocity_km_s.length() * 1_000.0).abs() < 1.0e-10);

    let too_late = TdbEpoch::new(end.j2000_seconds() + 1, 0);
    let err = src.sample_translational_icrf(too_late).unwrap_err();
    assert!(matches!(
        err,
        TrajectoryQueryError::EpochOutOfCoverage { .. }
    ));
    Ok(())
}

#[test]
fn truth_digests_exclude_sgp4_catalog_trajectories() -> Result<(), Box<dyn Error>> {
    let solar = BuiltinSolarSystem2026_2028::load_verified()?;
    let force_model = ForceModel::standard_solar_system(solar.constants.clone())?;
    let vessel_cfg = VesselConfig {
        mass_model: MassPropertiesModel::diagonal(
            12_000.0,
            [85_000.0, 18_000.0, 85_000.0],
            [0.0; 3],
        ),
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 5.0e6,
            tau_max_nm: 5.0e4,
        },
        tolerances: Dop853Tolerances::default(),
        bounding_radius_m: 6.0,
        auto_soi_switch: true,
    };
    let truth_model = DynamicalModelIdentity::SolarSystemEphemerisNBody {
        force_model_hash: force_model.content_hash(),
    };
    let digest_before = SimulationManifestDigest::compute(
        BuiltinSolarSystem2026_2028::START_TDB,
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"terrain"),
        force_model.content_hash(),
        vessel_cfg.tolerances.content_hash(),
    );
    let fp_before = PlanningInputFingerprint::compute(
        truth_model,
        &force_model,
        &solar.ephemeris,
        &solar.orientation,
        &vessel_cfg,
    );

    let sgp4_traj = Sgp4CatalogTrajectory::from_tle(
        99,
        Some("TLE"),
        TLE_28057_1,
        TLE_28057_2,
        ValidEpochInterval::new(TdbEpoch::J2000, TdbEpoch::JAN_1_2028_00Z_TDB),
        solar.time_scales.clone(),
    )?;
    let sample = sgp4_traj.sample_translational_icrf(TdbEpoch::JAN_1_2026_00Z_TDB)?;
    assert!(sample.pos.is_finite());

    let digest_after = SimulationManifestDigest::compute(
        BuiltinSolarSystem2026_2028::START_TDB,
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"terrain"),
        force_model.content_hash(),
        vessel_cfg.tolerances.content_hash(),
    );
    let fp_after = PlanningInputFingerprint::compute(
        truth_model,
        &force_model,
        &solar.ephemeris,
        &solar.orientation,
        &vessel_cfg,
    );
    assert_eq!(digest_before, digest_after);
    assert_eq!(fp_before, fp_after);
    assert_ne!(sgp4_traj.model_identity(), truth_model);
    Ok(())
}
