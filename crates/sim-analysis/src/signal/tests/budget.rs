use core::f64::consts::{FRAC_PI_2, LN_2};

use glam::DVec3;
use sim_analysis_contract::{
    AntennaPointing, AntennaSpec, BudgetUnavailableReason, CosinePowerPatchSpec, LinkBudget,
    LinkEnd, ParabolicDishSpec, PropagationModelSpec, ReceiverNoise, RfLinkSpec,
};
use sim_spacetime::SPEED_OF_LIGHT_M_S;

use super::{dish, isotropic_link, path};
use crate::signal::{LinkBudgetInputs, PathGeometry, PropagationLoss, compute_link_budget};

fn try_budget(
    link: &RfLinkSpec,
    p: &PathGeometry,
    look: Option<DVec3>,
) -> Result<LinkBudget, BudgetUnavailableReason> {
    let extra: Vec<&dyn PropagationLoss> = link
        .propagation
        .iter()
        .map(|m| m as &dyn PropagationLoss)
        .collect();
    compute_link_budget(&LinkBudgetInputs {
        link,
        transmit_antenna: &link.transmitter.antenna,
        receive_antenna: &link.receiver.antenna,
        propagation: &extra,
        path: p,
        transmit_look_in_mount: look,
        receive_look_in_mount: look,
    })
}

fn budget_of(link: &RfLinkSpec, p: &PathGeometry) -> LinkBudget {
    try_budget(link, p, Some(DVec3::Z)).expect("budget")
}

/// Mars Pathfinder X-band downlink, S. V. Hum, ECE422H1S "Applying Friis Formula" (University
/// of Toronto), after http://www.ka9q.net/mpf_budget.html: 10 W, 24.0 dBi, 8420 MHz,
/// 191e6 km, 34 m receive dish. The source rounds to 0.1 dB.
#[test]
fn mars_pathfinder_downlink_reproduces_the_published_table() {
    let mut link = isotropic_link();
    link.carrier_frequency_hz = 8.42e9;
    link.transmitter.antenna = AntennaSpec::CosinePowerPatch(CosinePowerPatchSpec {
        peak_gain_dbi: 24.0,
        cosine_exponent: 0.0,
        back_hemisphere_floor_dbi: -30.0,
    });
    link.receiver.antenna = AntennaSpec::ParabolicDish(dish(34.0, 0.7));
    let b = budget_of(&link, &path(191.0e9, 8.42e9));
    assert!((b.transmit_power_dbw - 10.0).abs() < 1e-12);
    assert!((b.eirp_dbw - 34.0).abs() < 0.05, "EIRP {}", b.eirp_dbw);
    let fspl = b.free_space_path_loss_db;
    assert!((fspl - 276.6).abs() < 0.05, "FSPL {fspl}");
    let rip = b.received_isotropic_power_dbw;
    assert!((rip - (-242.6)).abs() < 0.05, "RIP {rip}");
    let gr = b.receive_antenna_gain_dbi;
    assert!((gr - 68.0).abs() < 0.05, "Gr {gr}");
    let pr = b.received_power_dbw;
    assert!((pr - (-174.6)).abs() < 0.1, "Pr {pr}");
}

#[test]
fn noise_terms_follow_the_definitions_independently_of_the_geometry() {
    let b = budget_of(&isotropic_link(), &path(2.0e6, 2.2e9));
    let k_dbw = 10.0 * 1.380_649e-23_f64.log10();
    assert!((k_dbw - (-228.599_167)).abs() < 1e-5);
    let n0 = k_dbw + 10.0 * 290.0_f64.log10();
    assert!((b.noise_spectral_density_dbw_hz - n0).abs() < 1e-9);
    assert!((b.c_over_n0_dbhz - (b.received_power_dbw - n0)).abs() < 1e-9);
    assert!((b.eb_n0_db - (b.c_over_n0_dbhz - 60.0)).abs() < 1e-12);
    assert!((b.margin_db - (b.eb_n0_db - 10.0)).abs() < 1e-12);
    assert!((b.gain_over_temperature_db_per_k + 10.0 * 290.0_f64.log10()).abs() < 1e-9);
}

#[test]
fn shannon_capacity_matches_b_log2_one_plus_cn() {
    let b = budget_of(&isotropic_link(), &path(2.0e6, 2.2e9));
    let c_over_n = 10.0_f64.powf((b.c_over_n0_dbhz - 60.0) / 10.0);
    let expected = 1.0e6 * (1.0 + c_over_n).ln() / LN_2;
    assert!((b.shannon_capacity_bps - expected).abs() < 1e-6 * expected);
}

#[test]
fn explicit_losses_each_subtract_from_the_budget() {
    let base = budget_of(&isotropic_link(), &path(2.0e6, 2.2e9));
    let mut link = isotropic_link();
    link.polarization_mismatch_loss_db = 0.5;
    link.implementation_loss_db = 1.5;
    link.transmitter.line_loss_db = 0.7;
    link.receiver.line_loss_db = 0.3;
    link.propagation = vec![PropagationModelSpec::TroposphericCosecant {
        zenith_attenuation_db: 0.1,
        min_elevation_rad: 0.1,
    }];
    let mut p = path(2.0e6, 2.2e9);
    p.target_surface_elevation_rad = Some(FRAC_PI_2);
    let b = budget_of(&link, &p);
    let total_loss = 0.5 + 1.5 + 0.7 + 0.3 + 0.1;
    assert!((base.margin_db - b.margin_db - total_loss).abs() < 1e-9);
    assert_eq!(b.propagation_losses.len(), 1);
    assert!((b.eirp_dbw - (base.eirp_dbw - 0.7)).abs() < 1e-12);
}

#[test]
fn margin_falls_monotonically_with_range() {
    let link = isotropic_link();
    let mut previous = f64::INFINITY;
    for k in 1..40 {
        let m = budget_of(&link, &path(f64::from(k) * 5.0e5, 2.2e9)).margin_db;
        assert!(m < previous);
        previous = m;
    }
}

#[test]
fn gain_over_temperature_form_agrees_with_the_temperature_form() {
    let mut by_temp = isotropic_link();
    by_temp.receiver.antenna = AntennaSpec::ParabolicDish(dish(3.0, 0.6));
    let p = path(2.0e6, 2.2e9);
    let reference = budget_of(&by_temp, &p);
    let mut by_gt = by_temp.clone();
    by_gt.receiver.noise = ReceiverNoise::GainOverTemperature {
        g_over_t_db_per_k: reference.gain_over_temperature_db_per_k,
    };
    let b = budget_of(&by_gt, &p);
    assert!((b.system_noise_temperature_k - 290.0).abs() < 1e-6);
    assert!((b.margin_db - reference.margin_db).abs() < 1e-9);
}

#[test]
fn tracking_antenna_loses_gain_by_its_pointing_error_only() {
    let mut link = isotropic_link();
    link.transmitter.antenna = AntennaSpec::ParabolicDish(ParabolicDishSpec {
        sidelobe_floor_below_peak_db: 40.0,
        ..dish(3.0, 0.6)
    });
    link.transmitter.pointing = AntennaPointing::TrackingTarget {
        pointing_error_rad: 0.0,
    };
    let aligned = budget_of(&link, &path(2.0e6, 2.2e9));
    let half_beam = (35.0 * SPEED_OF_LIGHT_M_S / 2.2e9 / 3.0).to_radians();
    link.transmitter.pointing = AntennaPointing::TrackingTarget {
        pointing_error_rad: half_beam,
    };
    let off = budget_of(&link, &path(2.0e6, 2.2e9));
    let lost = aligned.transmit_antenna_gain_dbi - off.transmit_antenna_gain_dbi;
    assert!((lost - 3.0).abs() < 1e-9);
    assert!((off.transmit_off_axis_rad - half_beam).abs() < 1e-12);
}

#[test]
fn fixed_directional_antenna_without_attitude_reports_no_budget() {
    let mut link = isotropic_link();
    link.transmitter.antenna = AntennaSpec::ParabolicDish(dish(3.0, 0.6));
    let outcome = try_budget(&link, &path(2.0e6, 2.2e9), None);
    assert_eq!(
        outcome.expect_err("no orientation"),
        BudgetUnavailableReason::AntennaOrientationUnavailable {
            end: LinkEnd::Transmitter
        }
    );
}

#[test]
fn coincident_endpoints_report_degenerate_geometry() {
    let outcome = try_budget(&isotropic_link(), &path(0.0, 2.2e9), None);
    assert_eq!(
        outcome.expect_err("zero range"),
        BudgetUnavailableReason::DegenerateGeometry
    );
}

#[test]
fn invalid_parameters_are_rejected_before_evaluation() {
    assert!(isotropic_link().validate().is_ok());
    let mut link = isotropic_link();
    link.transmitter.transmit_power_w = 0.0;
    assert!(link.validate().is_err());
    let mut link = isotropic_link();
    link.receiver.antenna = AntennaSpec::ParabolicDish(dish(3.0, 1.5));
    assert!(link.validate().is_err());
    let mut link = isotropic_link();
    link.implementation_loss_db = -1.0;
    assert!(link.validate().is_err());
}
