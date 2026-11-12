use sim_geo::geodetic_to_ecef;

use super::*;

const FIX_PERIOD_S: f64 = 0.1;

fn truth() -> VehicleState {
    let at = Geodetic::from_degrees(39.16, -75.52, 75.0);
    VehicleState::level_flight(&at, 0.4, 9.0)
}

/// Position error of `reading` from truth, north-east-down metres.
fn error_ned_m(reading: &GnssReading) -> DVec3 {
    let truth = truth();
    let frame = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef));
    frame.ecef_to_ned_vector(geodetic_to_ecef(&reading.position) - truth.position_ecef)
}

#[test]
fn first_fixes_spread_as_the_receiver_declares() {
    let spec = GnssErrorSpec::OPEN_SKY;
    let (mut position, mut velocity) = (DVec3::ZERO, 0.0);
    let seeds = 600_u64;
    let mut declared = DVec3::ZERO;
    for seed in 0..seeds {
        let reading = GnssReceiver::new(spec, seed)
            .unwrap()
            .measure(&truth(), FIX_PERIOD_S)
            .unwrap();
        let error = error_ned_m(&reading);
        position += error * error;
        let truth_velocity = LocalFrame::at(&ecef_to_geodetic(truth().position_ecef))
            .ecef_to_ned_vector(truth().velocity_ecef);
        velocity += (reading.velocity_ned_m_s - truth_velocity).length_squared() / 3.0;
        declared = reading.position_sigma_ned_m;
    }
    let rms = (position / seeds as f64).map(f64::sqrt);
    assert!((declared.x - 0.6_f64.hypot(1.2)).abs() < 1e-12);
    assert!((declared.z - 1.0_f64.hypot(2.0)).abs() < 1e-12);
    for (measured, expected) in [
        (rms.x, declared.x),
        (rms.y, declared.y),
        (rms.z, declared.z),
    ] {
        assert!(
            (measured / expected - 1.0).abs() < 0.12,
            "{measured} m against {expected} m"
        );
    }
    let velocity_rms = (velocity / seeds as f64).sqrt();
    assert!(
        (velocity_rms / 0.05 - 1.0).abs() < 0.12,
        "{velocity_rms} m/s"
    );
}

#[test]
fn the_correlated_error_persists_from_fix_to_fix() {
    let mut receiver = GnssReceiver::new(GnssErrorSpec::OPEN_SKY, 21).unwrap();
    let errors: Vec<DVec3> = (0..600)
        .map(|_| error_ned_m(&receiver.measure(&truth(), FIX_PERIOD_S).unwrap()))
        .collect();
    let mean = errors.iter().copied().sum::<DVec3>() / errors.len() as f64;
    let about_mean =
        errors.iter().map(|e| (e.x - mean.x).powi(2)).sum::<f64>() / errors.len() as f64;
    // One minute is a fifth of the correlation time: the fixes scatter about
    // a held offset by little more than the white noise.
    assert!(about_mean.sqrt() < 0.9, "scatter {} m", about_mean.sqrt());
}

#[test]
fn a_receiver_without_the_sky_gives_no_fix_and_resumes_the_same_errors() {
    let spec = GnssErrorSpec::OPEN_SKY;
    let mut steady = GnssReceiver::new(spec, 9).unwrap();
    let mut denied = GnssReceiver::new(spec, 9).unwrap();
    for fix in 0..40 {
        denied.set_tracking(!(10..30).contains(&fix));
        let expected = steady.measure(&truth(), FIX_PERIOD_S);
        let got = denied.measure(&truth(), FIX_PERIOD_S);
        if denied.is_tracking() {
            assert_eq!(got, expected, "fix {fix}");
        } else {
            assert_eq!(got, None, "fix {fix}");
        }
    }
}

#[test]
fn unusable_statistics_are_refused() {
    let negative = GnssErrorSpec {
        velocity_sigma_m_s: -0.1,
        ..GnssErrorSpec::OPEN_SKY
    };
    assert!(GnssReceiver::new(negative, 1).is_err());
    let no_memory = GnssErrorSpec {
        correlation_time_s: 0.0,
        ..GnssErrorSpec::OPEN_SKY
    };
    assert!(GnssReceiver::new(no_memory, 1).is_err());
}

#[test]
fn a_displaced_receiver_reports_every_fix_that_far_from_where_it_would() {
    let spec = GnssErrorSpec::OPEN_SKY;
    let mut honest = GnssReceiver::new(spec, 4).unwrap();
    let mut displaced = GnssReceiver::new(spec, 4).unwrap();
    let offset = DVec3::new(-12.0, 30.0, 4.0);
    displaced.displace(offset);
    for _ in 0..5 {
        let a = honest.measure(&truth(), FIX_PERIOD_S).unwrap();
        let b = displaced.measure(&truth(), FIX_PERIOD_S).unwrap();
        assert!((error_ned_m(&b) - error_ned_m(&a) - offset).length() < 1e-6);
        assert_eq!(a.position_sigma_ned_m, b.position_sigma_ned_m);
    }
}

/// What the spoofer adds to the fixes of one receiver: the spoofed
/// receiver's fixes less those of the same receiver left alone.
fn spoof_added(spoofed: &GnssReading, clean: &GnssReading) -> (DVec3, DVec3) {
    (
        error_ned_m(spoofed) - error_ned_m(clean),
        spoofed.velocity_ned_m_s - clean.velocity_ned_m_s,
    )
}

#[test]
fn a_spoofed_receiver_is_dragged_at_the_spoofers_rate_and_released_at_once() {
    let spoof = GnssSpoof::toward(1.0, 90.0_f64.to_radians());
    let mut clean = GnssReceiver::new(GnssErrorSpec::OPEN_SKY, 5).unwrap();
    let mut spoofed = clean;
    spoofed.set_spoof(Some(spoof));
    for fix in 1..=300 {
        // Handing the same spoofer over again each cycle keeps its drag.
        spoofed.set_spoof(Some(spoof));
        let a = spoofed.measure(&truth(), FIX_PERIOD_S).unwrap();
        let b = clean.measure(&truth(), FIX_PERIOD_S).unwrap();
        let (position, velocity) = spoof_added(&a, &b);
        let dragged_m = f64::from(fix) * FIX_PERIOD_S;
        assert!((position - DVec3::new(0.0, dragged_m, 0.0)).length() < 1e-6);
        assert!((velocity - DVec3::new(0.0, 1.0, 0.0)).length() < 1e-9);
        assert_eq!(a.position_sigma_ned_m, b.position_sigma_ned_m);
    }
    spoofed.set_spoof(None);
    let a = spoofed.measure(&truth(), FIX_PERIOD_S).unwrap();
    let b = clean.measure(&truth(), FIX_PERIOD_S).unwrap();
    assert_eq!(a, b, "released, the receiver is the receiver left alone");
}

#[test]
fn a_spoofer_drags_on_while_the_receiver_has_no_sky() {
    let mut receiver = GnssReceiver::new(GnssErrorSpec::OPEN_SKY, 5).unwrap();
    let mut clean = receiver;
    receiver.set_spoof(Some(GnssSpoof::toward(2.0, 0.0)));
    receiver.set_tracking(false);
    for _ in 0..50 {
        assert!(receiver.measure(&truth(), FIX_PERIOD_S).is_none());
        clean.measure(&truth(), FIX_PERIOD_S);
    }
    receiver.set_tracking(true);
    let (position, _) = spoof_added(
        &receiver.measure(&truth(), FIX_PERIOD_S).unwrap(),
        &clean.measure(&truth(), FIX_PERIOD_S).unwrap(),
    );
    assert!((position - DVec3::new(10.2, 0.0, 0.0)).length() < 1e-6);
}
