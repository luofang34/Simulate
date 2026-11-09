//! Planetary orientation and time-scale contract verification: tiered planetary orientations (`ITRF93`,
//! `MOON_ME`, `MOON_PA`, `IAU_MARS`), NAIF `SPKEZR` golden vector, and `TimeScaleCatalog` expiration policies.

use std::{collections::BTreeMap, path::Path};

use glam::DVec3;
use sim_geo::UtcTime;
use sim_orbital::{
    BodyOrientationCatalog, BpcFile, BuiltinSolarSystem2026_2028, DerivativeOrder, EarthEopEntry,
    EarthEopTable, EphemerisCatalog, FrameCode, FutureUtcPolicy, IauPckCatalog, LeapSecondTable,
    LunarTargetFrame, NaifId, OrientationError, OrientationPolicy, OutOfEopPolicy,
    SpkEphemerisCatalog, SpkFile, StackedOrientationCatalog, SurfaceNavGrade, TdbEpoch, TimeError,
    TimeScaleCatalog,
};
use sim_spacetime::{TICKS_PER_SECOND, TickSpan};

#[test]
fn orientation_grades_policies_and_angular_velocity_derivatives_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let ep = TdbEpoch::new(852_000_000, 0);

    let earth_rot = solar
        .orientation
        .body_rotation(NaifId::EARTH, ep)
        .expect("earth itrf93");
    assert_eq!(earth_rot.frame_code, FrameCode::ITRF93);
    assert_eq!(earth_rot.grade, SurfaceNavGrade::PrecisionGeodetic);

    let moon_rot = solar
        .orientation
        .body_rotation(NaifId::MOON, ep)
        .expect("moon me");
    assert_eq!(moon_rot.frame_code, FrameCode::MOON_ME);
    assert_eq!(moon_rot.grade, SurfaceNavGrade::PrecisionGeodetic);

    let mars_rot = solar
        .orientation
        .body_rotation(NaifId::MARS, ep)
        .expect("mars iau");
    assert_eq!(mars_rot.grade, SurfaceNavGrade::ApproximateIau);

    // Out-of-window Earth query must fail closed under RequirePrecision:
    let out_ep = TdbEpoch::new(900_000_000, 0);
    assert!(matches!(
        solar.orientation.body_rotation(NaifId::EARTH, out_ep),
        Err(OrientationError::OutOfPrecisionCoverage {
            body: NaifId::EARTH,
            ..
        })
    ));

    // Verify analytical omega_body & alpha_body against finite differences of R_icrf_to_fixed:
    let dt_s = 0.25_f64;
    let span = TickSpan::period_of_hz(4).expect("4 Hz");
    let ep_minus = TdbEpoch::new(851_999_999, (0.75 * TICKS_PER_SECOND as f64) as u64);
    let ep_plus = ep.add_span(span);

    for &body in &[NaifId::EARTH, NaifId::MOON, NaifId::MARS] {
        let r0 = solar.orientation.body_rotation(body, ep).expect("r0");
        let rm = solar.orientation.body_rotation(body, ep_minus).expect("rm");
        let rp = solar.orientation.body_rotation(body, ep_plus).expect("rp");

        let r_dot = (rp.r_icrf_to_fixed - rm.r_icrf_to_fixed) * (1.0 / (2.0 * dt_s));
        // -[omega_body]_x = r_dot * R^T => omega = (skew[1,2], skew[2,0], skew[0,1]) with -[w]_x
        let skew = -(r_dot * r0.r_icrf_to_fixed.transpose());
        let omega_fd = DVec3::new(skew.col(1).z, skew.col(2).x, skew.col(0).y);
        assert!(
            (r0.omega_body_rad_s - omega_fd).length() < 1e-10,
            "omega mismatch on {body}: analytic={:?}, fd={omega_fd:?}",
            r0.omega_body_rad_s
        );

        let alpha_fd = (rp.omega_body_rad_s - rm.omega_body_rad_s) / (2.0 * dt_s);
        assert!(
            (r0.alpha_body_rad_s2 - alpha_fd).length() < 1e-12,
            "alpha mismatch on {body}"
        );
    }
}

#[test]
fn moon_pa_vs_iau_offset_and_optional_naif_golden_spkezr_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let iau = IauPckCatalog::from_pck_bytes(BuiltinSolarSystem2026_2028::PCK00011_TPC_BYTES)
        .expect("iau");
    let ep = TdbEpoch::new(852_000_000, 0);

    let me_prec = solar
        .orientation
        .body_rotation(NaifId::MOON, ep)
        .expect("me");
    let me_iau = iau.evaluate_iau(NaifId::MOON, ep).expect("iau moon");
    let delta_r = me_prec.r_icrf_to_fixed * me_iau.r_icrf_to_fixed.transpose();
    let trace_cos =
        ((delta_r.col(0).x + delta_r.col(1).y + delta_r.col(2).z - 1.0) * 0.5).clamp(-1.0, 1.0);
    let diff_arcsec = trace_cos.acos().to_degrees() * 3_600.0;
    // NAIF moon_de440_250416.tf documents ~0.0024 deg..0.0051 deg (~8.6"..18.4") difference between DE440 ME and IAU_MOON!
    assert!(
        diff_arcsec < 25.0,
        "unexpectedly large MOON_ME vs IAU_MOON angle: {diff_arcsec} arcsec"
    );

    // Optional check against full NASA NAIF source kernels in scratch (when present locally):
    let scratch_dir = Path::new(
        "/Users/fangluo/.gemini/antigravity/brain/0e879125-f0db-4777-aa49-58b098b418dc/scratch/naif_kernels",
    );
    let de440_path = scratch_dir.join("de440s.bsp");
    let moon_bpc_path = scratch_dir.join("moon_pa_de440_200625.bpc");
    if de440_path.exists() && moon_bpc_path.exists() {
        verify_naif_spkezr_golden_vector(&de440_path, &moon_bpc_path, &iau);
    }
}

#[cfg(test)]
fn verify_naif_spkezr_golden_vector(de440_path: &Path, moon_bpc_path: &Path, iau: &IauPckCatalog) {
    let de_bytes = std::fs::read(de440_path).expect("read de440s");
    let bpc_bytes = std::fs::read(moon_bpc_path).expect("read moon bpc");
    let spk = SpkEphemerisCatalog::from_files(
        vec![SpkFile::from_bytes(&de_bytes).expect("spk")],
        sim_orbital::MarsCenterApproximation::UseBarycenter4 {
            documented_bound_m: 0.25,
        },
    );
    let me_cat = StackedOrientationCatalog::build(
        iau.clone(),
        Vec::new(),
        vec![BpcFile::from_bytes(&bpc_bytes).expect("bpc")],
        Some(BuiltinSolarSystem2026_2028::MOON_DE440_FK_BYTES),
        LunarTargetFrame::MeanEarth,
        BTreeMap::new(),
        OrientationPolicy::RequirePrecision,
    )
    .expect("me cat");
    let pa_cat = StackedOrientationCatalog::build(
        iau.clone(),
        Vec::new(),
        vec![BpcFile::from_bytes(&bpc_bytes).expect("bpc")],
        None,
        LunarTargetFrame::PrincipalAxes,
        BTreeMap::new(),
        OrientationPolicy::RequirePrecision,
    )
    .expect("pa cat");

    // Exact NAIF example from moon_de440_250416.tf lines 414-419 (`ET = 717768000.0`, Earth wrt Moon):
    let et = TdbEpoch::new(717_768_000, 0);
    let icrf = spk
        .geometric_state_icrf(NaifId::EARTH, NaifId::MOON, et, DerivativeOrder::Velocity)
        .expect("earth wrt moon");
    let (p_me, v_me) = me_cat
        .body_rotation(NaifId::MOON, et)
        .expect("me")
        .icrf_to_fixed_pv(icrf.pos_m, icrf.vel_m_s);
    let (p_pa, v_pa) = pa_cat
        .body_rotation(NaifId::MOON, et)
        .expect("pa")
        .icrf_to_fixed_pv(icrf.pos_m, icrf.vel_m_s);

    assert!((p_me / 1_000.0 - DVec3::new(374_008.654, -23_435.970, 10_141.334)).length() < 1e-3);
    assert!(
        (v_me / 1_000.0 - DVec3::new(-0.026_381_34, 0.045_327_57, 0.118_941_64)).length() < 2e-8
    );
    assert!((p_pa / 1_000.0 - DVec3::new(373_997.028, -23_558.987, 10_284.057)).length() < 1e-3);
    assert!(
        (v_pa / 1_000.0 - DVec3::new(-0.026_411_81, 0.045_336_42, 0.118_931_51)).length() < 2e-8
    );
}

#[test]
fn time_scale_expiration_policies_verified() {
    let lsk = LeapSecondTable::from_naif_lsk_bytes(BuiltinSolarSystem2026_2028::NAIF0012_TLS_BYTES)
        .expect("lsk");
    let eop = EarthEopTable::new(vec![
        EarthEopEntry {
            utc_unix_s: 1_770_000_000.0,
            ut1_minus_utc_s: 0.040,
            xp_arcsec: 0.10,
            yp_arcsec: 0.25,
        },
        EarthEopEntry {
            utc_unix_s: 1_770_086_400.0,
            ut1_minus_utc_s: 0.042,
            xp_arcsec: 0.12,
            yp_arcsec: 0.27,
        },
    ]);
    let strict_ts = TimeScaleCatalog::new(
        lsk,
        FutureUtcPolicy::RejectAfterExpiration {
            expires_tai_j2000_s: 800_000_000.0,
        },
        eop,
        OutOfEopPolicy::RejectOutOfRange,
    );

    assert!(matches!(
        strict_ts.tdb_to_utc(TdbEpoch::new(830_000_000, 0)),
        Err(TimeError::LeapSecondTableExpired { .. })
    ));
    let mid_utc = UtcTime::from_unix_seconds(1_770_043_200.0);
    let ut1 = strict_ts.utc_to_ut1(mid_utc).expect("inside eop");
    assert!((ut1.ut1_minus_utc_s - 0.041).abs() < 1e-12);
    assert!(!ut1.is_approximate_fallback);
    assert!(matches!(
        strict_ts.utc_to_ut1(UtcTime::from_unix_seconds(1_800_000_000.0)),
        Err(TimeError::OutOfEopCoverage { .. })
    ));
}
