//! SPK/DAF ephemeris contract verification: DAF/SPK subset, Mars 499 vs barycenter 4,
//! disjoint mass hierarchy, coverage gaps, precedence, and derivative/knot accuracy.

use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ConstantsError, ContentHash, DafError,
    DerivativeOrder, EphemerisCatalog, EphemerisError, MarsCenterApproximation, NaifId,
    SimClockAdapter, SimulationManifestDigest, SpkDataType, SpkEphemerisCatalog, SpkFile, TdbEpoch,
};

#[cfg(test)]
type SyntheticSpkSegment<'a> = (&'a str, f64, f64, i32, i32, i32, i32, &'a [f64]);

#[cfg(test)]
fn build_synthetic_daf_spk(segments: &[SyntheticSpkSegment<'_>]) -> Vec<u8> {
    let mut next_word: i32 = 385;
    let mut sum_bytes = Vec::new();
    let mut name_bytes = Vec::new();
    let mut data_bytes = Vec::new();

    for &(name, dc0, dc1, target, center, frame, dtype, words) in segments {
        let start_w = next_word;
        let end_w = start_w + (words.len() as i32) - 1;
        next_word = end_w + 1;

        let mut entry = [0_u8; 40];
        entry[0..8].copy_from_slice(&dc0.to_le_bytes());
        entry[8..16].copy_from_slice(&dc1.to_le_bytes());
        entry[16..20].copy_from_slice(&target.to_le_bytes());
        entry[20..24].copy_from_slice(&center.to_le_bytes());
        entry[24..28].copy_from_slice(&frame.to_le_bytes());
        entry[28..32].copy_from_slice(&dtype.to_le_bytes());
        entry[32..36].copy_from_slice(&start_w.to_le_bytes());
        entry[36..40].copy_from_slice(&end_w.to_le_bytes());
        sum_bytes.extend_from_slice(&entry);

        let mut nbuf = [b' '; 40];
        let nb = name.as_bytes();
        let copy_len = nb.len().min(40);
        nbuf[..copy_len].copy_from_slice(&nb[..copy_len]);
        name_bytes.extend_from_slice(&nbuf);

        for &w in words {
            data_bytes.extend_from_slice(&w.to_le_bytes());
        }
    }

    let mut rec1 = vec![0_u8; 1024];
    rec1[0..8].copy_from_slice(b"DAF/SPK ");
    rec1[8..12].copy_from_slice(&2_i32.to_le_bytes());
    rec1[12..16].copy_from_slice(&6_i32.to_le_bytes());
    rec1[16..28].copy_from_slice(b"SYNTHETICSPK");
    rec1[76..80].copy_from_slice(&2_i32.to_le_bytes());
    rec1[80..84].copy_from_slice(&2_i32.to_le_bytes());
    rec1[84..88].copy_from_slice(&next_word.to_le_bytes());
    rec1[88..96].copy_from_slice(b"LTL-IEEE");

    let mut rec2 = vec![0_u8; 1024];
    rec2[16..24].copy_from_slice(&(segments.len() as f64).to_le_bytes());
    rec2[24..24 + sum_bytes.len()].copy_from_slice(&sum_bytes);

    let mut rec3 = vec![0_u8; 1024];
    rec3[..name_bytes.len()].copy_from_slice(&name_bytes);

    if !data_bytes.len().is_multiple_of(1024) {
        data_bytes.resize(data_bytes.len().next_multiple_of(1024), 0);
    }

    let mut all = rec1;
    all.extend_from_slice(&rec2);
    all.extend_from_slice(&rec3);
    all.extend_from_slice(&data_bytes);
    all
}

#[test]
fn builtin_solar_package_loads_and_verifies_checksums() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("verified load");
    assert_eq!(
        BuiltinSolarSystem2026_2028::total_embedded_bytes(),
        1_399_215
    );

    let cov = solar
        .ephemeris
        .coverage(NaifId::MARS, NaifId::EARTH, DerivativeOrder::Acceleration);
    assert_eq!(cov.len(), 1);
    assert!(cov[0].contains(BuiltinSolarSystem2026_2028::START_TDB));
    assert!(cov[0].contains(BuiltinSolarSystem2026_2028::END_TDB));
}

#[test]
fn mars_499_fails_closed_without_mars_spk_unless_barycenter_approx_declared() {
    let full =
        SpkFile::from_bytes(BuiltinSolarSystem2026_2028::SOLAR_SPK_BYTES).expect("valid spk");
    let raw_without_499 = build_de440s_only_subset();
    let de_only_file = SpkFile::from_bytes(&raw_without_499).expect("de440s subset");
    assert_eq!(full.segments().len(), de_only_file.segments().len() + 1);

    let strict_cat = SpkEphemerisCatalog::from_files(
        vec![de_only_file.clone()],
        MarsCenterApproximation::StrictRequire499,
    );
    let epoch = TdbEpoch::new(830_000_000, 0);
    let err = strict_cat
        .geometric_state_icrf(NaifId::MARS, NaifId::SSB, epoch, DerivativeOrder::Position)
        .expect_err("must reject missing 499");
    assert_eq!(
        err,
        EphemerisError::MissingBodyCenter {
            requested: NaifId::MARS,
            available_barycenter: NaifId::MARS_BARYCENTER,
        }
    );

    let opt_in_cat = SpkEphemerisCatalog::from_files(
        vec![de_only_file],
        MarsCenterApproximation::UseBarycenter4 {
            documented_bound_m: 0.25,
        },
    );
    assert_ne!(strict_cat.content_hash(), opt_in_cat.content_hash());

    let full_cat =
        SpkEphemerisCatalog::from_files(vec![full], MarsCenterApproximation::StrictRequire499);
    let mut max_offset_m = 0.0_f64;
    for step in 0..=200 {
        let t_s = 820_497_600 + step * 300_000;
        let ep = TdbEpoch::new(t_s, 0);
        let approx = opt_in_cat
            .geometric_state_icrf(NaifId::MARS, NaifId::SSB, ep, DerivativeOrder::Position)
            .expect("barycenter approx");
        let exact = full_cat
            .geometric_state_icrf(NaifId::MARS, NaifId::SSB, ep, DerivativeOrder::Position)
            .expect("exact 499");
        max_offset_m = max_offset_m.max((exact.pos_m - approx.pos_m).length());
    }
    assert!(
        max_offset_m > 0.10 && max_offset_m < 0.25,
        "measured ||x_499 - x_4|| max offset {max_offset_m} m outside expected Phobos/Deimos range"
    );
}

#[cfg(test)]
fn build_de440s_only_subset() -> Vec<u8> {
    // Re-pack constant 1-record Type-2 proxies for `(4, 0)` and `(3, 0)` without `499`.
    let mut b4_from_real = Vec::new();
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let ep0 = TdbEpoch::new(830_000_000, 0);
    let s4 = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::MARS_BARYCENTER,
            NaifId::SSB,
            ep0,
            DerivativeOrder::Position,
        )
        .expect("4->0");
    // Build a genuine de440s-only binary slice directly from SOLAR_SPK_BYTES by dropping segment 12 (`MAR099S_499_4`).
    let _ = s4;
    b4_from_real.extend_from_slice(BuiltinSolarSystem2026_2028::SOLAR_SPK_BYTES);
    // Summary record 2 stores `nsum = 12.0` at byte offset `1024 + 16`: decrement to `11.0` to omit segment 12 (`499 -> 4`)!
    b4_from_real[1024 + 16..1024 + 24].copy_from_slice(&11.0_f64.to_le_bytes());
    b4_from_real
}

#[test]
fn hierarchy_validation_rejects_emb_mars_barycenter_and_ssb_double_counting() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let valid_set = [
        NaifId::SUN,
        NaifId::MERCURY_BARYCENTER,
        NaifId::VENUS_BARYCENTER,
        NaifId::EARTH,
        NaifId::MOON,
        NaifId::MARS,
        NaifId::JUPITER_BARYCENTER,
        NaifId::SATURN_BARYCENTER,
    ];
    solar
        .constants
        .validate_attracting_hierarchy(&valid_set)
        .expect("disjoint set is valid");

    let emb_and_earth = [NaifId::SUN, NaifId::EARTH_MOON_BARYCENTER, NaifId::EARTH];
    assert_eq!(
        solar
            .constants
            .validate_attracting_hierarchy(&emb_and_earth),
        Err(ConstantsError::OverlappingBarycenterHierarchy {
            barycenter: NaifId::EARTH_MOON_BARYCENTER,
            child: NaifId::EARTH,
        })
    );

    let mars_bary_and_phobos = [NaifId::SUN, NaifId::MARS_BARYCENTER, NaifId::PHOBOS];
    assert_eq!(
        solar
            .constants
            .validate_attracting_hierarchy(&mars_bary_and_phobos),
        Err(ConstantsError::OverlappingBarycenterHierarchy {
            barycenter: NaifId::MARS_BARYCENTER,
            child: NaifId::PHOBOS,
        })
    );

    assert_eq!(
        solar
            .constants
            .validate_attracting_hierarchy(&[NaifId::SSB, NaifId::EARTH]),
        Err(ConstantsError::SsbInAttractingSet)
    );
}

#[test]
fn coverage_gaps_precedence_type3_and_unsupported_spk_type_contracts() {
    // Segment A: [0, 100], x = 10 km; Segment B: [200, 300], x = 20 km (gap in (100, 200)).
    let words_a = [
        50.0, 50.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 8.0, 1.0,
    ];
    let words_b = [
        250.0, 50.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 200.0, 100.0, 8.0, 1.0,
    ];
    let gapped_bsp = build_synthetic_daf_spk(&[
        ("SEG_A", 0.0, 100.0, 399, 0, 1, 2, &words_a),
        ("SEG_B", 200.0, 300.0, 399, 0, 1, 2, &words_b),
    ]);
    let cat = SpkEphemerisCatalog::from_bsp_slices(
        &[&gapped_bsp],
        MarsCenterApproximation::StrictRequire499,
    )
    .expect("gapped bsp");
    let spans = cat.coverage(NaifId::EARTH, NaifId::SSB, DerivativeOrder::Position);
    assert_eq!(spans.len(), 2);
    assert!(matches!(
        cat.geometric_state_icrf(
            NaifId::EARTH,
            NaifId::SSB,
            TdbEpoch::new(150, 0),
            DerivativeOrder::Position
        ),
        Err(EphemerisError::OutOfCoverage { .. })
    ));

    // Type-3 override segment covering [50, 250] with x = 99 km, vx = 1.5 km/s:
    let words_t3 = [
        150.0, 100.0, 99.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0, 200.0,
        14.0, 1.0,
    ];
    let override_bsp = build_synthetic_daf_spk(&[("OVR_T3", 50.0, 250.0, 399, 0, 1, 3, &words_t3)]);
    let file_ovr = SpkFile::from_bytes(&override_bsp).expect("type3 file");
    assert_eq!(
        file_ovr.segments()[0].data_type(),
        SpkDataType::Type3ChebyshevPosVel
    );

    let merged = SpkEphemerisCatalog::from_bsp_slices(
        &[&gapped_bsp, &override_bsp],
        MarsCenterApproximation::StrictRequire499,
    )
    .expect("merged");
    assert_eq!(
        merged
            .coverage(NaifId::EARTH, NaifId::SSB, DerivativeOrder::Velocity)
            .len(),
        1
    );
    let s75 = merged
        .geometric_state_icrf(
            NaifId::EARTH,
            NaifId::SSB,
            TdbEpoch::new(75, 0),
            DerivativeOrder::Velocity,
        )
        .expect("covered by override");
    assert!((s75.pos_m.x - 99_000.0).abs() < 1e-9);
    assert!((s75.vel_m_s.x - 1_500.0).abs() < 1e-9);

    // Unsupported SPK Type 13 must fail closed:
    let bad_bsp = build_synthetic_daf_spk(&[("BAD_T13", 0.0, 100.0, 399, 0, 1, 13, &words_a)]);
    assert_eq!(
        SpkFile::from_bytes(&bad_bsp).expect_err("reject type 13"),
        DafError::UnsupportedSpkDataType(13)
    );
}

#[test]
fn chebyshev_derivatives_match_fourth_order_finite_differences() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let ep = TdbEpoch::new(845_123_456, 0);
    // At 1 AU (1.5e11 m, ulp ~ 3e-5 m), h = 64 s balances FD cancellation (~5e-7 m/s) and O(h^4) truncation.
    let dt_s = 64.0_f64;
    let ep_m2 = TdbEpoch::new(845_123_456 - 128, 0);
    let ep_m1 = TdbEpoch::new(845_123_456 - 64, 0);
    let ep_p1 = TdbEpoch::new(845_123_456 + 64, 0);
    let ep_p2 = TdbEpoch::new(845_123_456 + 128, 0);

    for &(target, center) in &[
        (NaifId::EARTH, NaifId::SSB),
        (NaifId::MOON, NaifId::EARTH),
        (NaifId::MARS, NaifId::SSB),
    ] {
        let s0 = solar
            .ephemeris
            .geometric_state_icrf(target, center, ep, DerivativeOrder::Acceleration)
            .expect("s0");
        let sm2 = solar
            .ephemeris
            .geometric_state_icrf(target, center, ep_m2, DerivativeOrder::Velocity)
            .expect("sm2");
        let sm1 = solar
            .ephemeris
            .geometric_state_icrf(target, center, ep_m1, DerivativeOrder::Velocity)
            .expect("sm1");
        let sp1 = solar
            .ephemeris
            .geometric_state_icrf(target, center, ep_p1, DerivativeOrder::Velocity)
            .expect("sp1");
        let sp2 = solar
            .ephemeris
            .geometric_state_icrf(target, center, ep_p2, DerivativeOrder::Velocity)
            .expect("sp2");

        let fd_v = (sm2.pos_m - 8.0 * sm1.pos_m + 8.0 * sp1.pos_m - sp2.pos_m) / (12.0 * dt_s);
        let fd_a =
            (sm2.vel_m_s - 8.0 * sm1.vel_m_s + 8.0 * sp1.vel_m_s - sp2.vel_m_s) / (12.0 * dt_s);
        assert!(
            (s0.vel_m_s - fd_v).length() < 5e-6,
            "velocity FD mismatch on {target}->{center}: err={}",
            (s0.vel_m_s - fd_v).length()
        );
        assert!(
            (s0.acc_m_s2 - fd_a).length() < 1e-10,
            "accel FD mismatch on {target}->{center}: err={}",
            (s0.acc_m_s2 - fd_a).length()
        );
    }
}

#[test]
fn lca_cancellation_granule_knots_and_manifest_digest_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let ep = TdbEpoch::new(845_123_456, 0);

    let moon_emb = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::MOON,
            NaifId::EARTH_MOON_BARYCENTER,
            ep,
            DerivativeOrder::Acceleration,
        )
        .expect("301->3");
    let earth_emb = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::EARTH,
            NaifId::EARTH_MOON_BARYCENTER,
            ep,
            DerivativeOrder::Acceleration,
        )
        .expect("399->3");
    let moon_earth = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::MOON,
            NaifId::EARTH,
            ep,
            DerivativeOrder::Acceleration,
        )
        .expect("301->399");
    assert_eq!(moon_earth.pos_m, moon_emb.pos_m - earth_emb.pos_m);

    let t_end = TdbEpoch::new(ep.j2000_seconds() + 30 * 86_400, 0);
    let knots = solar
        .ephemeris
        .segment_knots_in(NaifId::MOON, NaifId::EARTH, ep, t_end);
    assert!(!knots.is_empty());
    for &k in &knots {
        let left = TdbEpoch::new(k.j2000_seconds() - 1, sim_spacetime::TICKS_PER_SECOND - 1);
        let s_l = solar
            .ephemeris
            .geometric_state_icrf(
                NaifId::MOON,
                NaifId::EARTH,
                left,
                DerivativeOrder::Acceleration,
            )
            .expect("left");
        let s_r = solar
            .ephemeris
            .geometric_state_icrf(
                NaifId::MOON,
                NaifId::EARTH,
                k,
                DerivativeOrder::Acceleration,
            )
            .expect("right");
        assert!((s_r.pos_m - s_l.pos_m).length() < 1e-3);
        assert!((s_r.acc_m_s2 - s_l.acc_m_s2).length() < 1e-9);
    }
}

#[test]
fn manifest_digest_and_clock_adapter_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let clock = SimClockAdapter::new(BuiltinSolarSystem2026_2028::START_TDB);
    let inst = sim_spacetime::SimInstant::START.advanced(sim_spacetime::TickSpan::from_ticks_u128(
        123_456_789_012_345,
    ));
    assert_eq!(clock.tdb_to_instant(clock.instant_to_tdb(inst)), Some(inst));
    let d1 = SimulationManifestDigest::compute(
        clock.base_tdb(),
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"terrain-v1"),
        ContentHash::of_bytes(b"force-v1"),
        ContentHash::of_bytes(b"num-v1"),
    );
    let d2 = SimulationManifestDigest::compute(
        clock.base_tdb(),
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"terrain-v2"),
        ContentHash::of_bytes(b"force-v1"),
        ContentHash::of_bytes(b"num-v1"),
    );
    assert_ne!(d1.root_hash, d2.root_hash);
}
