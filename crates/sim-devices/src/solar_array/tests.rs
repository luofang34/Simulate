use glam::DVec3;
use sim_core::{Block, BlockError, TICKS_PER_SECOND, TickSpan};
use sim_spacetime::{AtmosphericTransmission, SolarFluxSample};

use super::{SolarArrayBlock, SolarArrayConfig, SolarArrayInput};

fn array() -> SolarArrayBlock {
    SolarArrayBlock::new(SolarArrayConfig {
        area_m2: 2.0,
        efficiency: 0.30,
        temperature_coefficient_per_k: -0.002,
        reference_temperature_k: 301.15,
        period: TickSpan::from_ticks(TICKS_PER_SECOND),
    })
}

fn sunlight(irradiance_w_m2: f64, lit_fraction: f64) -> SolarFluxSample {
    SolarFluxSample {
        to_sun_icrf: DVec3::X,
        sun_distance_m: 1.5e11,
        unobstructed_w_m2: irradiance_w_m2,
        lit_fraction,
    }
}

fn facing_the_sun(sunlight: SolarFluxSample) -> SolarArrayInput {
    SolarArrayInput {
        sunlight,
        normal_icrf: DVec3::X,
        direct_beam_fraction: 1.0,
        cell_temperature_k: 301.15,
    }
}

fn power_w(input: &SolarArrayInput) -> f64 {
    let mut array = array();
    array.step(input).unwrap();
    array.output()
}

#[test]
fn an_array_in_orbit_turns_its_share_of_the_sunlight_into_power() {
    let facing = facing_the_sun(sunlight(1361.0, 1.0));
    assert_eq!(array().output(), 0.0, "nothing before the first step");
    assert!((power_w(&facing) - 816.6).abs() < 1e-9);

    let tilted = SolarArrayInput {
        normal_icrf: DVec3::new(0.5, 0.75_f64.sqrt(), 0.0),
        ..facing
    };
    assert!((power_w(&tilted) - 408.3).abs() < 1e-9);
    let edge_on = SolarArrayInput {
        normal_icrf: DVec3::NEG_X,
        ..facing
    };
    assert_eq!(power_w(&edge_on), 0.0, "the back of the array");
}

#[test]
fn eclipses_air_distance_and_heat_all_take_their_share() {
    let orbit = power_w(&facing_the_sun(sunlight(1361.0, 1.0)));
    assert_eq!(power_w(&facing_the_sun(sunlight(1361.0, 0.0))), 0.0);
    let penumbra = power_w(&facing_the_sun(sunlight(1361.0, 0.25)));
    assert!((penumbra - 0.25 * orbit).abs() < 1e-9);

    let mars_noon = SolarArrayInput {
        direct_beam_fraction: AtmosphericTransmission::MARS_CLEAR_SKY.direct_beam_fraction(1.0),
        ..facing_the_sun(sunlight(586.2, 1.0))
    };
    let on_mars = power_w(&mars_noon);
    assert!(
        (200.0..220.0).contains(&on_mars),
        "2 m² at Mars noon gives {on_mars} W"
    );

    let hot = SolarArrayInput {
        cell_temperature_k: 351.15,
        ..facing_the_sun(sunlight(1361.0, 1.0))
    };
    assert!((power_w(&hot) - 0.9 * orbit).abs() < 1e-9);
}

#[test]
fn inputs_outside_what_an_array_can_see_are_refused() {
    let good = facing_the_sun(sunlight(1361.0, 1.0));
    for bad in [
        SolarArrayInput {
            direct_beam_fraction: 1.2,
            ..good
        },
        SolarArrayInput {
            cell_temperature_k: f64::NAN,
            ..good
        },
    ] {
        assert!(matches!(
            array().step(&bad),
            Err(BlockError::InputOutOfRange { .. })
        ));
    }
    let mut array = array();
    array.step(&good).unwrap();
    let saved = array.snapshot();
    array.step(&facing_the_sun(sunlight(0.0, 1.0))).unwrap();
    array.restore(&saved);
    assert_eq!(array.output(), saved);
}
