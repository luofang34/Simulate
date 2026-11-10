use glam::DVec3;
use sim_core::{Block, BlockError, TickSpan};
use sim_spacetime::ClockPotential;

use super::{OscillatorBlock, OscillatorConfig};

const GPS_A_M: f64 = 26_561_750.0;
/// The offset GPS oscillators are built with: 10.23 MHz set to 10.22999999543 MHz.
const GPS_FACTORY_OFFSET: f64 = -4.4647e-10;

/// `dτ/dTT - 1` on a circular GPS orbit.
fn gps_rate() -> f64 {
    let earth = ClockPotential::EARTH;
    let speed = (earth.gm_m3_s2 / GPS_A_M).sqrt();
    earth
        .proper_rate_offset(DVec3::new(GPS_A_M, 0.0, 0.0), DVec3::new(0.0, speed, 0.0))
        .unwrap()
}

fn run_a_day(fractional_frequency_offset: f64) -> f64 {
    let rate = gps_rate();
    let mut oscillator = OscillatorBlock::new(
        OscillatorConfig {
            fractional_frequency_offset,
            period: TickSpan::from_seconds_and_subsec_ticks(60, 0),
        },
        0.0,
        rate,
    );
    for _ in 0..1_440 {
        oscillator.step(&rate).unwrap();
    }
    oscillator.output()
}

#[test]
fn an_uncorrected_clock_in_gps_orbit_gains_38_microseconds_a_day() {
    let gained_us = run_a_day(0.0) * 1e6;
    assert!((38.5..38.7).contains(&gained_us), "{gained_us} µs");
}

#[test]
fn the_factory_offset_keeps_a_gps_clock_on_system_time() {
    let drift_ns = run_a_day(GPS_FACTORY_OFFSET) * 1e9;
    assert!(drift_ns.abs() < 1.0, "{drift_ns} ns after a day");
}

#[test]
fn a_changing_rate_is_integrated_and_a_snapshot_resumes_the_count() {
    let config = OscillatorConfig {
        fractional_frequency_offset: 0.0,
        period: TickSpan::from_seconds_and_subsec_ticks(10, 0),
    };
    let mut oscillator = OscillatorBlock::new(config, 2.0e-6, 0.0);
    assert_eq!(oscillator.output(), 2.0e-6);
    oscillator.step(&1.0e-9).unwrap();
    // The rate rose linearly from 0 to 1e-9 over 10 s: half of it counts.
    assert!((oscillator.output() - 2.005e-6).abs() < 1e-18);

    let saved = oscillator.snapshot();
    oscillator.step(&1.0e-9).unwrap();
    let onward = oscillator.output();
    assert!((onward - 2.015e-6).abs() < 1e-18);
    oscillator.restore(&saved);
    oscillator.step(&1.0e-9).unwrap();
    assert_eq!(oscillator.output(), onward);

    assert!(matches!(
        oscillator.step(&f64::NAN),
        Err(BlockError::InputOutOfRange { .. })
    ));
}

#[test]
fn an_eccentric_gps_clock_swings_by_the_periodic_term_and_corrects_to_system_time() {
    let earth = ClockPotential {
        j2: 0.0,
        ..ClockPotential::EARTH
    };
    let e = 0.02_f64;
    let b = GPS_A_M * (1.0 - e * e).sqrt();
    let mean_motion = (earth.gm_m3_s2 / (GPS_A_M * GPS_A_M * GPS_A_M)).sqrt();
    let kepler_state = |t_s: f64| -> (DVec3, DVec3) {
        let m = mean_motion * t_s;
        let mut ecc = m;
        for _ in 0..8 {
            ecc -= (ecc - e * ecc.sin() - m) / (1.0 - e * ecc.cos());
        }
        let (sin_e, cos_e) = ecc.sin_cos();
        let r = GPS_A_M * (1.0 - e * cos_e);
        let pos = DVec3::new(GPS_A_M * (cos_e - e), b * sin_e, 0.0);
        let vel = DVec3::new(-GPS_A_M * sin_e, b * cos_e, 0.0) * (mean_motion * GPS_A_M / r);
        (pos, vel)
    };
    let secular_rate = earth
        .proper_rate_offset(
            DVec3::new(GPS_A_M, 0.0, 0.0),
            DVec3::new(0.0, (earth.gm_m3_s2 / GPS_A_M).sqrt(), 0.0),
        )
        .unwrap();
    let dt_s = 10.0;
    let (p0, v0) = kepler_state(0.0);
    let mut oscillator = OscillatorBlock::new(
        OscillatorConfig {
            fractional_frequency_offset: -secular_rate,
            period: TickSpan::from_seconds_and_subsec_ticks(10, 0),
        },
        0.0,
        earth.proper_rate_offset(p0, v0).unwrap(),
    );
    let mut max_raw_ns = 0.0_f64;
    let mut max_corrected_ns = 0.0_f64;
    for step in 1..=4_308 {
        let (pos, vel) = kepler_state(f64::from(step) * dt_s);
        let rate = earth.proper_rate_offset(pos, vel).unwrap();
        oscillator.step(&rate).unwrap();
        max_raw_ns = max_raw_ns.max(oscillator.output().abs() * 1e9);
        max_corrected_ns =
            max_corrected_ns.max(oscillator.system_time_offset_s(pos, vel).abs() * 1e9);
    }
    // At e = 0.02, Δt_r swings by ±45.8 ns, while subtracting orbit_periodic_clock_term_s
    // keeps system time within 0.05 ns.
    assert!(
        (45.0..46.5).contains(&max_raw_ns),
        "raw swing {max_raw_ns} ns"
    );
    assert!(
        max_corrected_ns < 0.05,
        "corrected residual {max_corrected_ns} ns"
    );
}
