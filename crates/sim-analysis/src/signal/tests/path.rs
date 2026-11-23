use sim_analysis_contract::PropagationModelSpec;
use sim_spacetime::{ASTRONOMICAL_UNIT_M, SPEED_OF_LIGHT_M_S};

use super::path;
use crate::signal::{
    PropagationLoss, PropagationLossError, doppler_shift_hz, free_space_path_loss_db,
};

// Reference values: 20*log10(4*pi*d*f/c) evaluated with 40-digit decimal arithmetic.
#[test]
fn free_space_loss_matches_hand_computed_values() {
    let leo = free_space_path_loss_db(2.0e6, 2.2e9);
    assert!((leo - 165.316_836_751_607_1).abs() < 1e-9, "{leo}");
    let deep = free_space_path_loss_db(ASTRONOMICAL_UNIT_M, 8.4e9);
    assert!((deep - 274.431_877_184_034_6).abs() < 1e-9, "{deep}");
    let short = free_space_path_loss_db(1.0e3, 2.2e9);
    assert!((short - 99.296_236_838_327_5).abs() < 1e-9, "{short}");
}

#[test]
fn doubling_range_costs_six_db() {
    let step = free_space_path_loss_db(2.0e6, 2.2e9) - free_space_path_loss_db(1.0e6, 2.2e9);
    assert!((step - 6.020_599_913_279_624).abs() < 1e-9);
}

#[test]
fn doppler_is_blue_when_closing_and_red_when_receding() {
    let f = 2.2e9;
    let closing = doppler_shift_hz(f, -7_000.0);
    assert!(closing > 0.0);
    assert!((closing - f * 7_000.0 / SPEED_OF_LIGHT_M_S).abs() < 1e-6);
    assert!(doppler_shift_hz(f, 7_000.0) < 0.0);
    assert_eq!(doppler_shift_hz(f, 0.0), 0.0);
}

#[test]
fn troposphere_scales_with_cosecant_of_elevation_at_each_surface_end() {
    let model = PropagationModelSpec::TroposphericCosecant {
        zenith_attenuation_db: 0.04,
        min_elevation_rad: 5.0_f64.to_radians(),
    };
    let mut p = path(1.0e6, 8.4e9);
    assert_eq!(model.loss_db(&p).expect("no surface end"), 0.0);
    p.target_surface_elevation_rad = Some(30.0_f64.to_radians());
    let one_end = model.loss_db(&p).expect("one end");
    assert!((one_end - 0.08).abs() < 1e-12);
    p.source_surface_elevation_rad = Some(90.0_f64.to_radians());
    assert!((model.loss_db(&p).expect("two ends") - 0.12).abs() < 1e-12);
}

#[test]
fn troposphere_refuses_paths_below_its_validity_limit() {
    let model = PropagationModelSpec::TroposphericCosecant {
        zenith_attenuation_db: 0.04,
        min_elevation_rad: 5.0_f64.to_radians(),
    };
    let mut p = path(1.0e6, 8.4e9);
    p.target_surface_elevation_rad = Some(2.0_f64.to_radians());
    assert!(matches!(
        model.loss_db(&p),
        Err(PropagationLossError::ElevationBelowModelLimit { .. })
    ));
}
