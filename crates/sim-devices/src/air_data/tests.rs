use sim_atmosphere::{AtmosphereField, GeoidStandardAtmosphere};
use sim_geo::{Geodetic, UtcTime};

use super::*;

const GROUND_SPEED_M_S: f64 = 40.0;

fn northbound() -> (Geodetic, VehicleState) {
    let at = Geodetic::from_degrees(39.16, -75.52, 600.0);
    (at, VehicleState::level_flight(&at, 0.0, GROUND_SPEED_M_S))
}

/// The ideal probe's sample for a northbound vehicle in a uniform wind.
fn sample_in_wind(wind_ned_m_s: DVec3) -> (AirFlow, AirDataSample) {
    let (at, state) = northbound();
    let air = GeoidStandardAtmosphere::with_wind(wind_ned_m_s)
        .sample(&at, UtcTime::from_unix_seconds(0.0))
        .unwrap();
    let flow = AirFlow::of(&air, &state);
    let ideal = PressureErrorSpec::IDEAL;
    let mut probe = PitotStatic::with_pitot(ideal, ideal, DVec3::X, 1).unwrap();
    (flow, probe.sense(&flow, 0.02))
}

fn airspeed_m_s(sample: &AirDataSample) -> f64 {
    speed_from_impact_pressure_m_s(
        sample.static_pressure_pa,
        sample.static_temperature_k,
        sample.differential_pressure_pa.unwrap(),
    )
}

#[test]
fn the_pitot_reads_the_speed_through_the_air_not_over_the_ground() {
    let (_, still) = sample_in_wind(DVec3::ZERO);
    assert!((airspeed_m_s(&still) - GROUND_SPEED_M_S).abs() < 1e-6);

    // Air moving south at 10 m/s meets a northbound vehicle head on.
    let (_, headwind) = sample_in_wind(DVec3::new(-10.0, 0.0, 0.0));
    assert!((airspeed_m_s(&headwind) - 50.0).abs() < 1e-6);
    let (_, tailwind) = sample_in_wind(DVec3::new(10.0, 0.0, 0.0));
    assert!((airspeed_m_s(&tailwind) - 30.0).abs() < 1e-6);

    // A wind from the side leaves the flow along the probe unchanged and
    // shows only in the relative flow's lateral part.
    let (flow, crosswind) = sample_in_wind(DVec3::new(0.0, 15.0, 0.0));
    assert!((airspeed_m_s(&crosswind) - GROUND_SPEED_M_S).abs() < 1e-6);
    assert!((flow.air_velocity_body_m_s.y + 15.0).abs() < 1e-9);
    assert!((flow.air_velocity_body_m_s.length() - 40.0_f64.hypot(15.0)).abs() < 1e-9);
}

#[test]
fn a_horizontal_wind_leaves_the_static_pressure_alone() {
    let (_, still) = sample_in_wind(DVec3::ZERO);
    let (_, windy) = sample_in_wind(DVec3::new(-10.0, 15.0, 0.0));
    assert_eq!(still.static_pressure_pa, windy.static_pressure_pa);
    assert_eq!(still.static_temperature_k, windy.static_temperature_k);
}

#[test]
fn impact_pressure_is_the_dynamic_pressure_at_low_speed_and_inverts() {
    let (pressure_pa, temperature_k) = (101_325.0, 288.15);
    let density = pressure_pa / (GAS_CONSTANT_J_KG_K * temperature_k);
    let slow = impact_pressure_pa(pressure_pa, temperature_k, 10.0);
    assert!(
        (slow / (0.5 * density * 100.0) - 1.0).abs() < 1e-3,
        "{slow} Pa"
    );
    for speed in [0.0, 12.0, 55.0, 120.0, 250.0] {
        let impact = impact_pressure_pa(pressure_pa, temperature_k, speed);
        let back = speed_from_impact_pressure_m_s(pressure_pa, temperature_k, impact);
        assert!(
            (back - speed).abs() < 1e-6,
            "{speed} m/s came back as {back}"
        );
    }
    assert_eq!(
        speed_from_impact_pressure_m_s(pressure_pa, temperature_k, -5.0),
        0.0
    );
}

#[test]
fn a_static_port_alone_has_no_pitot_and_carries_its_declared_errors() {
    let spec = PressureErrorSpec {
        white_sigma_pa: 3.0,
        bias: GaussMarkov {
            tau_s: 600.0,
            sigma: 20.0,
        },
    };
    let flow = AirFlow {
        static_pressure_pa: 95_000.0,
        static_temperature_k: 280.0,
        air_velocity_body_m_s: DVec3::new(10.0, 0.0, 0.0),
    };
    let errors = |seed: u64| -> Vec<f64> {
        let mut port = PitotStatic::static_only(spec, seed).unwrap();
        (0..500)
            .map(|_| {
                let sample = port.sense(&flow, 0.02);
                assert_eq!(sample.differential_pressure_pa, None);
                sample.static_pressure_pa - flow.static_pressure_pa
            })
            .collect()
    };
    assert_eq!(errors(7), errors(7), "a seed replays");
    assert_ne!(errors(7), errors(8));
    // The offset moves slowly, so the change from one sample to the next is
    // two draws of the white noise.
    let run = errors(7);
    let changes: f64 = run.windows(2).map(|pair| (pair[1] - pair[0]).powi(2)).sum();
    let white = (changes / (2.0 * (run.len() - 1) as f64)).sqrt();
    assert!((white - 3.0).abs() < 0.3, "white noise {white} Pa");
    // Across seeds the first sample carries a draw of the stationary offset.
    let firsts: Vec<f64> = (0..400).map(|seed| errors(seed).remove(0)).collect();
    let rms = (firsts.iter().map(|e| e * e).sum::<f64>() / firsts.len() as f64).sqrt();
    assert!(
        (rms - 20.0_f64.hypot(3.0)).abs() < 3.0,
        "turn-on spread {rms} Pa"
    );
}

#[test]
fn unusable_statistics_and_probe_axes_are_refused() {
    let negative = PressureErrorSpec {
        white_sigma_pa: -1.0,
        bias: GaussMarkov::NONE,
    };
    assert!(PitotStatic::static_only(negative, 1).is_err());
    let ideal = PressureErrorSpec::IDEAL;
    assert!(PitotStatic::with_pitot(ideal, negative, DVec3::X, 1).is_err());
    assert!(PitotStatic::with_pitot(ideal, ideal, DVec3::ZERO, 1).is_err());
}
