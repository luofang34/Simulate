use core::f64::consts::FRAC_PI_2;

use glam::DVec3;
use sim_devices::{
    GroundTracker, GroundTrackerError, GroundTrackerStationSpec, TrackingErrorSpec,
    TrackingLinkMode, TrackingMeasurement, TrackingTruthMeasurement,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::{NaifId, SPEED_OF_LIGHT_M_S, TdbEpoch, TrajectoryQueryError, UnitSemantics};

use super::fixtures::{
    AnalyticSolarCatalog, CircularOrbitTrajectory, FixedTarget, MU_EARTH, goldstone_station,
};

#[cfg(test)]
fn noise_test_fixture() -> (AnalyticSolarCatalog, GroundTrackerStationSpec, FixedTarget) {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: false,
        include_moon: false,
    };
    let station = GroundTrackerStationSpec::new(
        24,
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
        Geodetic::from_degrees(0.0, 0.0, 0.0),
        0.0,
        8.4e9,
    );
    let target = FixedTarget::at(
        NaifId::EARTH,
        ref_epoch,
        DVec3::new(10_000_000.0, 3_000_000.0, 2_000_000.0),
    );
    (cat, station, target)
}

#[test]
fn ideal_error_spec_matches_truth_exactly() {
    let (cat, station, target) = noise_test_fixture();
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let mut ideal = GroundTracker::new(station, TrackingErrorSpec::IDEAL).unwrap();
    let m = ideal
        .measure_noisy(
            &target,
            ref_epoch,
            TrackingLinkMode::TwoWayCoherent,
            &cat,
            &cat,
        )
        .unwrap();
    assert_eq!(m.range_m, m.truth.range_m);
    assert_eq!(m.range_rate_m_s, m.truth.range_rate_m_s);
    assert_eq!(m.doppler_hz, m.truth.doppler_hz);
    assert_eq!(m.azimuth_rad, m.truth.azimuth_rad);
    assert_eq!(m.elevation_rad, m.truth.elevation_rad);
    assert!(!m.below_mask);
}

#[test]
fn noisy_measurements_match_bias_and_sigma_statistics() {
    let (cat, station, target) = noise_test_fixture();
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let err_spec = TrackingErrorSpec {
        range_bias_m: 12.5,
        range_sigma_m: 3.0,
        range_rate_bias_m_s: -0.04,
        range_rate_sigma_m_s: 0.01,
        azimuth_bias_rad: 2.0e-4,
        azimuth_sigma_rad: 5.0e-5,
        elevation_bias_rad: -1.5e-4,
        elevation_sigma_rad: 4.0e-5,
        rng_seed: 0xDEAD_BEEF_CAFE_0001,
        stream_tag: 77,
    };
    let mut tracker = GroundTracker::new(station.clone(), err_spec).unwrap();
    let n = 2_000_usize;
    let (mut sum, mut sum_sq) = ([0.0_f64; 4], [0.0_f64; 4]);
    for _ in 0..n {
        let a = tracker
            .measure_noisy(
                &target,
                ref_epoch,
                TrackingLinkMode::OneWayDownlink,
                &cat,
                &cat,
            )
            .unwrap();
        let expected_doppler =
            -station.carrier_frequency_hz * a.range_rate_m_s / SPEED_OF_LIGHT_M_S;
        assert!((a.doppler_hz - expected_doppler).abs() < 1e-9);
        let diffs = [
            a.range_m - a.truth.range_m,
            a.range_rate_m_s - a.truth.range_rate_m_s,
            a.azimuth_rad - a.truth.azimuth_rad,
            a.elevation_rad - a.truth.elevation_rad,
        ];
        for k in 0..4 {
            sum[k] += diffs[k];
            sum_sq[k] += diffs[k] * diffs[k];
        }
    }
    let expected = [
        (err_spec.range_bias_m, err_spec.range_sigma_m),
        (err_spec.range_rate_bias_m_s, err_spec.range_rate_sigma_m_s),
        (err_spec.azimuth_bias_rad, err_spec.azimuth_sigma_rad),
        (err_spec.elevation_bias_rad, err_spec.elevation_sigma_rad),
    ];
    for (k, (bias, sigma)) in expected.into_iter().enumerate() {
        let mean = sum[k] / (n as f64);
        let std = ((sum_sq[k] / (n as f64)) - mean * mean).sqrt();
        let se = sigma / (n as f64).sqrt();
        assert!((mean - bias).abs() < 4.5 * se);
        assert!((std / sigma - 1.0).abs() < 0.08);
    }
}

#[test]
fn noisy_measurements_replay_deterministically_across_clones_and_sample_indices() {
    let (cat, station, target) = noise_test_fixture();
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let err_spec = TrackingErrorSpec {
        range_bias_m: 1.0,
        range_sigma_m: 2.5,
        range_rate_bias_m_s: 0.01,
        range_rate_sigma_m_s: 0.02,
        azimuth_bias_rad: 1e-4,
        azimuth_sigma_rad: 1e-4,
        elevation_bias_rad: -1e-4,
        elevation_sigma_rad: 1e-4,
        rng_seed: 999,
        stream_tag: 17,
    };
    let mut tracker_a = GroundTracker::new(station, err_spec).unwrap();
    let mut tracker_b = tracker_a.clone();
    for idx in 0..20_u64 {
        let a = tracker_a
            .measure_noisy(
                &target,
                ref_epoch,
                TrackingLinkMode::OneWayDownlink,
                &cat,
                &cat,
            )
            .unwrap();
        let b = tracker_b
            .measure_noisy(
                &target,
                ref_epoch,
                TrackingLinkMode::OneWayDownlink,
                &cat,
                &cat,
            )
            .unwrap();
        let c = tracker_a
            .measure_noisy_at_sample(
                idx,
                &target,
                ref_epoch,
                TrackingLinkMode::OneWayDownlink,
                &cat,
                &cat,
            )
            .unwrap();
        assert_eq!(a.range_m.to_bits(), b.range_m.to_bits());
        assert_eq!(a.range_m.to_bits(), c.range_m.to_bits());
        assert_eq!(a.doppler_hz.to_bits(), c.doppler_hz.to_bits());
    }
    tracker_a.set_sample_counter(7);
    let seq_7 = tracker_a
        .measure_noisy(
            &target,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat,
        )
        .unwrap();
    let rand_7 = tracker_b
        .measure_noisy_at_sample(
            7,
            &target,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat,
        )
        .unwrap();
    assert_eq!(seq_7.range_m.to_bits(), rand_7.range_m.to_bits());
}

#[test]
fn trajectory_transform_and_orientation_errors_propagate() {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: false,
        include_moon: false,
    };
    let tracker =
        GroundTracker::new(goldstone_station(-FRAC_PI_2), TrackingErrorSpec::IDEAL).unwrap();

    let mut traj = CircularOrbitTrajectory::new(
        NaifId::EARTH,
        ref_epoch,
        7_000_000.0,
        0.5,
        0.0,
        MU_EARTH,
        10.0,
    );
    let out_of_cov = tracker.measure_truth(
        &traj,
        ref_epoch.advanced_seconds(100.0),
        TrackingLinkMode::OneWayDownlink,
        &cat,
        &cat,
    );
    assert!(matches!(
        out_of_cov,
        Err(GroundTrackerError::Trajectory(
            TrajectoryQueryError::EpochOutOfCoverage { .. }
        ))
    ));

    traj.units = UnitSemantics::Cr3bpNormalized {
        l_star_m: 384_400_000.0,
        t_star_s: 375_200.0,
        mu_star: 0.01215,
    };
    assert!(matches!(
        tracker.measure_truth(
            &traj,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat
        ),
        Err(GroundTrackerError::Transform(_))
    ));

    let mars_station = GroundTrackerStationSpec::new(
        99,
        NaifId::MARS,
        BodyEllipsoid::MARS_IAU,
        Geodetic::from_degrees(0.0, 0.0, 0.0),
        0.0,
        8.4e9,
    );
    let mars_tracker = GroundTracker::new(mars_station, TrackingErrorSpec::IDEAL).unwrap();
    let ok_traj = CircularOrbitTrajectory::new(
        NaifId::EARTH,
        ref_epoch,
        7_000_000.0,
        0.5,
        0.0,
        MU_EARTH,
        100.0,
    );
    assert!(matches!(
        mars_tracker.measure_truth(
            &ok_traj,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat
        ),
        Err(GroundTrackerError::Orientation(_))
    ));
}

#[test]
fn tracker_and_measurements_roundtrip_via_serde_json() {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: false,
        include_moon: false,
    };
    let tracker =
        GroundTracker::new(goldstone_station(-FRAC_PI_2), TrackingErrorSpec::IDEAL).unwrap();
    let traj = CircularOrbitTrajectory::new(
        NaifId::EARTH,
        ref_epoch,
        7_000_000.0,
        0.5,
        0.0,
        MU_EARTH,
        100.0,
    );
    let json = serde_json::to_string(&tracker).unwrap();
    let roundtrip: GroundTracker = serde_json::from_str(&json).unwrap();
    assert_eq!(roundtrip, tracker);

    let m = tracker
        .measure_noisy_at_sample(
            5,
            &traj,
            ref_epoch,
            TrackingLinkMode::TwoWayCoherent,
            &cat,
            &cat,
        )
        .unwrap();
    let m_back: TrackingMeasurement =
        serde_json::from_str(&serde_json::to_string(&m).unwrap()).unwrap();
    assert_eq!(m_back, m);
    let truth_back: TrackingTruthMeasurement =
        serde_json::from_str(&serde_json::to_string(&m.truth).unwrap()).unwrap();
    assert_eq!(truth_back, m.truth);
}
