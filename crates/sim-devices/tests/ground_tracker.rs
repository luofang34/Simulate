//! Integration tests for ground tracking measurements (`GroundTracker`):
//! light-time causality, finite-difference range-rate parity, topocentric ENU angles,
//! elevation mask and planetary occultation gating, and deterministic `TrialRng` noise.

#[cfg(test)]
#[path = "ground_tracker/fixtures.rs"]
mod fixtures;
#[cfg(test)]
#[path = "ground_tracker/noise_and_errors.rs"]
mod noise_and_errors;
#[cfg(test)]
#[path = "ground_tracker/pointing_and_occultation.rs"]
mod pointing_and_occultation;

use core::f64::consts::FRAC_PI_2;

use fixtures::{
    AnalyticSolarCatalog, CircularOrbitTrajectory, MU_EARTH, MU_SUN, goldstone_station,
};
use glam::DVec3;
use sim_devices::{GroundTracker, GroundTrackerStationSpec, TrackingErrorSpec, TrackingLinkMode};
use sim_geo::BodyEllipsoid;
use sim_spacetime::{
    ASTRONOMICAL_UNIT_M, DerivativeOrder, EphemerisCatalog, NaifId, SPEED_OF_LIGHT_M_S,
    TICKS_PER_SECOND, TdbEpoch, TranslationalTrajectorySource,
};

#[cfg(test)]
fn assert_range_rate_matches_fd(
    tracker: &GroundTracker,
    traj: &dyn TranslationalTrajectorySource,
    cat: &AnalyticSolarCatalog,
    t_mid: TdbEpoch,
    h_ticks: u64,
    tol_m_s: f64,
) {
    let shift = |n: i64| -> TdbEpoch {
        let total = (t_mid.j2000_seconds() as i128) * (TICKS_PER_SECOND as i128)
            + (t_mid.subsec_ticks() as i128)
            + (n as i128) * (h_ticks as i128);
        let secs = total.div_euclid(TICKS_PER_SECOND as i128) as i64;
        let ticks = total.rem_euclid(TICKS_PER_SECOND as i128) as u64;
        TdbEpoch::new(secs, ticks)
    };
    let (t_m2, t_m1, t_p1, t_p2) = (shift(-2), shift(-1), shift(1), shift(2));
    let h_s = (h_ticks as f64) / (TICKS_PER_SECOND as f64);

    for mode in [
        TrackingLinkMode::OneWayDownlink,
        TrackingLinkMode::TwoWayCoherent,
    ] {
        let m_mid = tracker.measure_truth(traj, t_mid, mode, cat, cat).unwrap();
        let m_m1 = tracker.measure_truth(traj, t_m1, mode, cat, cat).unwrap();
        let m_p1 = tracker.measure_truth(traj, t_p1, mode, cat, cat).unwrap();
        let m_m2 = tracker.measure_truth(traj, t_m2, mode, cat, cat).unwrap();
        let m_p2 = tracker.measure_truth(traj, t_p2, mode, cat, cat).unwrap();

        let fd_4th =
            (-m_p2.range_m + 8.0 * m_p1.range_m - 8.0 * m_m1.range_m + m_m2.range_m) / (12.0 * h_s);
        assert!(
            (m_mid.range_rate_m_s - fd_4th).abs() < tol_m_s,
            "{mode:?}: {} vs {fd_4th}",
            m_mid.range_rate_m_s
        );
        let fd_lt_4th = (-m_p2.light_time_s + 8.0 * m_p1.light_time_s - 8.0 * m_m1.light_time_s
            + m_m2.light_time_s)
            / (12.0 * h_s);
        let expected_doppler = -tracker.station().carrier_frequency_hz * fd_lt_4th;
        assert!((m_mid.doppler_hz - expected_doppler).abs() < 5.0e-3);
    }
}

#[cfg(test)]
fn station_ssb_pos(
    station: &GroundTrackerStationSpec,
    epoch: TdbEpoch,
    cat: &AnalyticSolarCatalog,
) -> DVec3 {
    let body = cat
        .geometric_state_icrf(station.body, NaifId::SSB, epoch, DerivativeOrder::Position)
        .unwrap()
        .pos_m;
    let (rel, _, _) = sim_spacetime::surface_station_state_icrf(
        station.body,
        station.ellipsoid,
        &station.geodetic,
        epoch,
        cat,
    )
    .unwrap();
    body + rel
}

#[test]
fn leo_and_geo_range_rate_match_two_point_and_fourth_order_finite_differences() {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: false,
        include_moon: false,
    };
    let tracker =
        GroundTracker::new(goldstone_station(-FRAC_PI_2), TrackingErrorSpec::IDEAL).unwrap();
    let leo = CircularOrbitTrajectory::new(
        NaifId::EARTH,
        ref_epoch,
        7_100_000.0,
        51.6_f64.to_radians(),
        0.35,
        MU_EARTH,
        600.0,
    );
    let geo = CircularOrbitTrajectory::new(
        NaifId::EARTH,
        ref_epoch,
        42_164_000.0,
        5.0_f64.to_radians(),
        0.2,
        MU_EARTH,
        600.0,
    );

    let t_mid = TdbEpoch::new(ref_epoch.j2000_seconds() + 120, 0);
    for (traj, h_ticks) in [
        (
            &leo as &dyn TranslationalTrajectorySource,
            TICKS_PER_SECOND / 200,
        ),
        (
            &geo as &dyn TranslationalTrajectorySource,
            TICKS_PER_SECOND / 4,
        ),
    ] {
        let t_p = TdbEpoch::new(t_mid.j2000_seconds(), h_ticks);
        let t_m = TdbEpoch::new(t_mid.j2000_seconds() - 1, TICKS_PER_SECOND - h_ticks);
        let two_h = t_p.signed_offset_seconds_from(t_m);
        for mode in [
            TrackingLinkMode::OneWayDownlink,
            TrackingLinkMode::TwoWayCoherent,
        ] {
            let mid = tracker
                .measure_truth(traj, t_mid, mode, &cat, &cat)
                .unwrap();
            let plus = tracker.measure_truth(traj, t_p, mode, &cat, &cat).unwrap();
            let minus = tracker.measure_truth(traj, t_m, mode, &cat, &cat).unwrap();
            let fd_2pt = (plus.range_m - minus.range_m) / two_h;
            assert!((mid.range_rate_m_s - fd_2pt).abs() < 1.0e-7);
        }
        assert_range_rate_matches_fd(&tracker, traj, &cat, t_mid, h_ticks, 1.0e-7);
    }
}

#[test]
fn lunar_and_heliocentric_range_rate_match_finite_differences() {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat_geo = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: false,
        include_moon: true,
    };
    let tracker =
        GroundTracker::new(goldstone_station(-FRAC_PI_2), TrackingErrorSpec::IDEAL).unwrap();
    let t_mid = TdbEpoch::new(ref_epoch.j2000_seconds() + 120, 0);
    let lunar = CircularOrbitTrajectory::new(
        NaifId::MOON,
        ref_epoch,
        2_200_000.0,
        30.0_f64.to_radians(),
        0.7,
        BodyEllipsoid::MOON_IAU.mu_m3_s2,
        600.0,
    );
    assert_range_rate_matches_fd(
        &tracker,
        &lunar,
        &cat_geo,
        t_mid,
        TICKS_PER_SECOND / 2,
        1.0e-7,
    );

    let cat_helio = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: true,
        include_moon: false,
    };
    let helio_probe = CircularOrbitTrajectory::new(
        NaifId::SSB,
        ref_epoch,
        1.524 * ASTRONOMICAL_UNIT_M,
        1.85_f64.to_radians(),
        0.4,
        MU_SUN,
        10_000.0,
    );
    assert_range_rate_matches_fd(
        &tracker,
        &helio_probe,
        &cat_helio,
        t_mid,
        200 * TICKS_PER_SECOND,
        1.0e-7,
    );
}

#[test]
fn two_way_bounce_causality_and_light_time_distinguish_from_one_way() {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: true,
        include_moon: true,
    };
    let station = goldstone_station(-FRAC_PI_2);
    let tracker = GroundTracker::new(station.clone(), TrackingErrorSpec::IDEAL).unwrap();
    let lunar = CircularOrbitTrajectory::new(
        NaifId::MOON,
        ref_epoch,
        2_200_000.0,
        20.0_f64.to_radians(),
        1.1,
        BodyEllipsoid::MOON_IAU.mu_m3_s2,
        600.0,
    );
    let rx_epoch = ref_epoch.advanced_seconds(180.0);

    let one_way = tracker
        .measure_truth(
            &lunar,
            rx_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat,
        )
        .unwrap();
    let two_way = tracker
        .measure_truth(
            &lunar,
            rx_epoch,
            TrackingLinkMode::TwoWayCoherent,
            &cat,
            &cat,
        )
        .unwrap();

    assert!(one_way.bounce_epoch.is_none());
    assert!(one_way.tx_epoch < one_way.rx_epoch);
    let bounce_epoch = two_way.bounce_epoch.expect("two-way bounce epoch");
    assert!(two_way.tx_epoch < bounce_epoch && bounce_epoch < two_way.rx_epoch);

    let sat_rel = lunar.sample_translational_icrf(bounce_epoch).unwrap().pos;
    let moon_pos = cat
        .geometric_state_icrf(
            NaifId::MOON,
            NaifId::SSB,
            bounce_epoch,
            DerivativeOrder::Position,
        )
        .unwrap()
        .pos_m;
    let r_sat_bounce = moon_pos + sat_rel;

    let down_res = (r_sat_bounce - station_ssb_pos(&station, two_way.rx_epoch, &cat)).length()
        - SPEED_OF_LIGHT_M_S * two_way.rx_epoch.signed_offset_seconds_from(bounce_epoch);
    let up_res = (r_sat_bounce - station_ssb_pos(&station, two_way.tx_epoch, &cat)).length()
        - SPEED_OF_LIGHT_M_S * bounce_epoch.signed_offset_seconds_from(two_way.tx_epoch);
    assert!(down_res.abs() < 0.02);
    assert!(up_res.abs() < 0.02);
    assert!((two_way.range_m - 0.5 * SPEED_OF_LIGHT_M_S * two_way.light_time_s).abs() < 1e-6);
    assert!((two_way.range_m - one_way.range_m).abs() > 100.0);
    let expected_2w_doppler =
        -2.0 * station.carrier_frequency_hz * two_way.range_rate_m_s / SPEED_OF_LIGHT_M_S;
    assert!((two_way.doppler_hz - expected_2w_doppler).abs() < 1e-9);
}
