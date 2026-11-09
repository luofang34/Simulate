use glam::DVec3;

use super::{ClockPotential, ProperTimeClock, orbit_periodic_clock_term_s};

const EARTH: ClockPotential = ClockPotential::EARTH;
/// Earth's rotation rate, rad/s (IERS).
const EARTH_RATE_RAD_S: f64 = 7.292_115e-5;
/// GPS nominal semi-major axis, m.
const GPS_A_M: f64 = 26_561_750.0;
/// IS-GPS-200 `F = -2 √μ / c²`, s/√m.
const GPS_F: f64 = -4.442_807_633e-10;

fn rate(position: DVec3, velocity: DVec3) -> f64 {
    EARTH
        .proper_rate_offset(position, velocity)
        .unwrap_or(f64::NAN)
}

#[test]
fn clocks_on_the_geoid_keep_tt() {
    let equator = DVec3::new(6_378_137.0, 0.0, 0.0);
    let surface_speed = DVec3::new(0.0, EARTH_RATE_RAD_S * equator.x, 0.0);
    let pole = DVec3::new(0.0, 0.0, 6_356_752.314);
    for (name, offset) in [
        ("equator", rate(equator, surface_speed)),
        ("pole", rate(pole, DVec3::ZERO)),
    ] {
        assert!(
            offset.abs() < 1e-12,
            "{name} clock drifts {offset:e} from TT"
        );
    }
}

#[test]
fn a_gps_clock_gains_its_factory_offset() {
    let position = DVec3::new(GPS_A_M, 0.0, 0.0);
    let speed = (EARTH.gm_m3_s2 / GPS_A_M).sqrt();
    let offset = rate(position, DVec3::new(0.0, speed, 0.0));
    // GPS oscillators are built at 10.22999999543 MHz for this offset.
    assert!(
        (offset - 4.4647e-10).abs() < 2e-14,
        "fractional rate {offset:e}"
    );
    let per_day_us = offset * 86_400.0 * 1e6;
    assert!((38.5..38.7).contains(&per_day_us), "{per_day_us} µs/day");
}

#[test]
fn a_low_orbit_clock_loses_time() {
    let a = 6_778_137.0;
    let speed = (EARTH.gm_m3_s2 / a).sqrt();
    assert!(rate(DVec3::new(a, 0.0, 0.0), DVec3::new(0.0, speed, 0.0)) < 0.0);
}

#[test]
fn periodic_term_matches_is_gps_200() {
    let (a, e, eccentric_anomaly) = (GPS_A_M, 0.02_f64, 1.1_f64);
    let mu = EARTH.gm_m3_s2;
    let (sin_e, cos_e) = eccentric_anomaly.sin_cos();
    let r = a * (1.0 - e * cos_e);
    let b = a * (1.0 - e * e).sqrt();
    let n = (mu / (a * a * a)).sqrt();
    let position = DVec3::new(a * (cos_e - e), b * sin_e, 0.0);
    let velocity = DVec3::new(-a * sin_e, b * cos_e, 0.0) * (n * a / r);
    let expected = GPS_F * e * a.sqrt() * sin_e;
    let got = orbit_periodic_clock_term_s(position, velocity);
    assert!(
        (got - expected).abs() < 1e-13,
        "got {got:e}, want {expected:e}"
    );
}

#[test]
fn a_day_on_a_circular_gps_orbit_accumulates_38_6_microseconds() {
    let speed = (EARTH.gm_m3_s2 / GPS_A_M).sqrt();
    let omega = speed / GPS_A_M;
    let state = |t: f64| {
        let (s, c) = (omega * t).sin_cos();
        (
            DVec3::new(c, s, 0.0) * GPS_A_M,
            DVec3::new(-s, c, 0.0) * speed,
        )
    };
    let mut clock = ProperTimeClock::default();
    let dt = 60.0;
    for step in 0..1_440 {
        let t = f64::from(step) * dt;
        let (p0, v0) = state(t);
        let (p1, v1) = state(t + dt);
        clock.advance(dt, rate(p0, v0), rate(p1, v1));
    }
    let us = clock.tau_minus_tt_s() * 1e6;
    assert!((38.5..38.7).contains(&us), "{us} µs after one day");
}

#[test]
fn the_centre_has_no_potential() {
    assert_eq!(EARTH.potential_m2_s2(DVec3::ZERO), None);
    assert_eq!(EARTH.proper_rate_offset(DVec3::ZERO, DVec3::ZERO), None);
}
