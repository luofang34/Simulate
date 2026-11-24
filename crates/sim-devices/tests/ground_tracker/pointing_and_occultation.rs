use core::f64::consts::{FRAC_PI_2, PI, TAU};

use glam::DVec3;
use sim_devices::{
    GroundTracker, GroundTrackerError, GroundTrackerStationSpec, TrackingErrorSpec,
    TrackingLinkMode,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::{BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, NaifId, TdbEpoch};

use super::fixtures::{AnalyticSolarCatalog, FixedTarget, goldstone_station};

#[cfg(test)]
fn enu_target_builder(
    station: &GroundTrackerStationSpec,
    epoch: TdbEpoch,
    cat: &AnalyticSolarCatalog,
) -> impl Fn(DVec3) -> FixedTarget + use<> {
    let (sta_rel, _, local_enu) = sim_spacetime::surface_station_state_icrf(
        station.body,
        station.ellipsoid,
        &station.geodetic,
        epoch,
        cat,
    )
    .unwrap();
    let rot = cat.body_rotation(station.body, epoch).unwrap();
    let r_fixed_to_icrf = rot.r_icrf_to_fixed.transpose();
    let enu_to_ecef = local_enu.enu_to_ecef();
    let body = station.body;
    move |enu: DVec3| {
        let fixed = sta_rel + enu_to_ecef * enu;
        FixedTarget::at(body, epoch, r_fixed_to_icrf * fixed)
    }
}

#[test]
fn topocentric_enu_zenith_and_cardinal_azimuths_match_convention() {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: false,
        include_moon: false,
    };
    let station = goldstone_station(15.0_f64.to_radians());
    let tracker = GroundTracker::new(station.clone(), TrackingErrorSpec::IDEAL).unwrap();
    let target_from_enu = enu_target_builder(&station, ref_epoch, &cat);

    let zenith = target_from_enu(DVec3::new(0.0, 0.0, 400_000.0));
    let m_zen = tracker
        .measure_truth(
            &zenith,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat,
        )
        .unwrap();
    assert!((m_zen.elevation_rad - FRAC_PI_2).abs() < 1e-9);

    let d = 100_000.0;
    let up = d * 30.0_f64.to_radians().tan();
    for (enu, expected_az) in [
        (DVec3::new(0.0, d, up), 0.0),
        (DVec3::new(d, 0.0, up), FRAC_PI_2),
        (DVec3::new(0.0, -d, up), PI),
        (DVec3::new(-d, 0.0, up), 3.0 * FRAC_PI_2),
    ] {
        let tgt = target_from_enu(enu);
        let m = tracker
            .measure_truth(
                &tgt,
                ref_epoch,
                TrackingLinkMode::OneWayDownlink,
                &cat,
                &cat,
            )
            .unwrap();
        let az_err = (m.azimuth_rad - expected_az + PI).rem_euclid(TAU) - PI;
        assert!(az_err.abs() < 1e-9);
        assert!((m.elevation_rad - 30.0_f64.to_radians()).abs() < 1e-9);
        let dir = DVec3::new(
            m.azimuth_rad.sin() * m.elevation_rad.cos(),
            m.azimuth_rad.cos() * m.elevation_rad.cos(),
            m.elevation_rad.sin(),
        );
        assert!((dir - enu.normalize()).length() < 1e-9);
    }
}

#[test]
fn elevation_mask_rejects_low_targets_and_flags_noisy_crossings() {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: false,
        include_moon: false,
    };
    let station = goldstone_station(15.0_f64.to_radians());
    let target_from_enu = enu_target_builder(&station, ref_epoch, &cat);

    let tracker = GroundTracker::new(station.clone(), TrackingErrorSpec::IDEAL).unwrap();
    let low = target_from_enu(DVec3::new(
        100_000.0,
        0.0,
        100_000.0 * 5.0_f64.to_radians().tan(),
    ));
    let err = tracker
        .measure_truth(
            &low,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat,
        )
        .unwrap_err();
    assert!(matches!(
        err,
        GroundTrackerError::BelowElevationMask {
            elevation_rad,
            min_elevation_rad
        } if elevation_rad < min_elevation_rad
    ));

    let unmasked = tracker
        .measure_noisy_unmasked_at_sample(
            0,
            &low,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat,
        )
        .unwrap();
    assert!(unmasked.below_mask);

    let just_above = target_from_enu(DVec3::new(
        100_000.0,
        0.0,
        100_000.0 * 15.1_f64.to_radians().tan(),
    ));
    let biased_err = TrackingErrorSpec {
        elevation_bias_rad: -0.5_f64.to_radians(),
        ..TrackingErrorSpec::IDEAL
    };
    let biased_tracker = GroundTracker::new(station, biased_err).unwrap();
    let crossed = biased_tracker
        .measure_noisy_at_sample(
            0,
            &just_above,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat,
        )
        .unwrap();
    assert!(crossed.below_mask);
}

#[test]
fn planetary_occultation_blocks_lunar_far_side_and_host_antipode() {
    let ref_epoch = TdbEpoch::JAN_1_2026_00Z_TDB;
    let cat = AnalyticSolarCatalog {
        ref_epoch,
        earth_orbits_sun: true,
        include_moon: true,
    };
    let moon_r = BodyEllipsoid::MOON_IAU.semi_major_m;
    let dir_to_moon = cat
        .geometric_state_icrf(
            NaifId::MOON,
            NaifId::EARTH,
            ref_epoch,
            DerivativeOrder::Position,
        )
        .unwrap()
        .pos_m
        .normalize();

    let station = GroundTrackerStationSpec::new(
        1,
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
        Geodetic::from_degrees(0.0, 0.0, 0.0),
        -FRAC_PI_2,
        2.2e9,
    )
    .with_occulting_body(NaifId::MOON, moon_r)
    .with_host_occultation(BodyEllipsoid::WGS84.semi_major_m);
    let tracker = GroundTracker::new(station, TrackingErrorSpec::IDEAL).unwrap();

    let behind_moon = FixedTarget::at(NaifId::MOON, ref_epoch, dir_to_moon * (moon_r + 500_000.0));
    let err = tracker
        .measure_truth(
            &behind_moon,
            ref_epoch,
            TrackingLinkMode::TwoWayCoherent,
            &cat,
            &cat,
        )
        .unwrap_err();
    assert_eq!(err, GroundTrackerError::BodyOcculted { body: NaifId::MOON });

    let in_front = FixedTarget::at(NaifId::MOON, ref_epoch, -dir_to_moon * (moon_r + 500_000.0));
    assert!(
        tracker
            .measure_truth(
                &in_front,
                ref_epoch,
                TrackingLinkMode::TwoWayCoherent,
                &cat,
                &cat
            )
            .is_ok()
    );

    let antipode = FixedTarget::at(
        NaifId::EARTH,
        ref_epoch,
        DVec3::new(-10_000_000.0, 0.0, 0.0),
    );
    let earth_err = tracker
        .measure_truth(
            &antipode,
            ref_epoch,
            TrackingLinkMode::OneWayDownlink,
            &cat,
            &cat,
        )
        .unwrap_err();
    assert_eq!(
        earth_err,
        GroundTrackerError::BodyOcculted {
            body: NaifId::EARTH
        }
    );
}
