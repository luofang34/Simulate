use core::f64::consts::PI;

use glam::DVec3;
use sim_analysis_contract::{
    AntennaPointing, AntennaSpec, BudgetUnavailableReason, ReceiverNoise, RfLinkSpec,
    RfReceiverSpec, RfTransmitterSpec,
};
use sim_geo::Geodetic;
use sim_spacetime::{SPEED_OF_LIGHT_M_S, TdbEpoch};
use world_propagation::itm::{Climate, PropagationMode, VariabilityMode};
use world_propagation::p526::{
    DEFAULT_EFFECTIVE_EARTH_RADIUS_M, KnifeEdgeGeometry, KnifeEdgeMethod,
};
use world_propagation::{Ground, Polarisation, ProfilePoint, PropagationError};

use super::super::{
    AntennaPattern, LinkBudgetInputs, PathGeometry, PropagationLoss, compute_link_budget,
};
use super::{
    DeltaBullingtonPropagationLoss, F1245Pattern, ItmPropagationLoss, KnifeEdgePropagationLoss,
    MainBeamPolicy, S465Pattern, S580Pattern, SmoothEarthPropagationLoss,
};
use crate::signal::ExceedancePercentages;

fn off_axis_dir(deg: f64) -> DVec3 {
    let rad = deg.to_radians();
    DVec3::new(rad.sin(), 0.0, rad.cos())
}

fn flat_profile(distance_m: f64, points: usize, ground_m: f64) -> Vec<ProfilePoint> {
    let np = points.saturating_sub(1);
    (0..=np)
        .map(|i| ProfilePoint {
            distance_m: distance_m * (i as f64) / (np as f64),
            height_m: ground_m,
        })
        .collect()
}

fn hill_profile(distance_m: f64, points: usize, peak_height_m: f64) -> Vec<ProfilePoint> {
    let np = points.saturating_sub(1);
    (0..=np)
        .map(|i| {
            let u = (i as f64) / (np as f64);
            let s = (PI * u).sin();
            ProfilePoint {
                distance_m: distance_m * u,
                height_m: peak_height_m * s * s,
            }
        })
        .collect()
}

fn terrestrial_link(freq_hz: f64) -> RfLinkSpec {
    RfLinkSpec {
        carrier_frequency_hz: freq_hz,
        bandwidth_hz: 1.0e6,
        data_rate_bps: 1.0e6,
        required_eb_n0_db: 10.0,
        polarization_mismatch_loss_db: 0.0,
        implementation_loss_db: 0.0,
        transmitter: RfTransmitterSpec {
            transmit_power_w: 10.0,
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::FixedInMount,
            line_loss_db: 0.0,
        },
        receiver: RfReceiverSpec {
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::FixedInMount,
            line_loss_db: 0.0,
            noise: ReceiverNoise::SystemTemperature {
                system_noise_temperature_k: 290.0,
            },
        },
        propagation: Vec::new(),
    }
}

#[test]
fn itu_antenna_patterns_map_vectors_to_off_axis_gain_and_refuse_out_of_domain() {
    let f = 12.0e9;
    let s465 = S465Pattern::new(2.4, f).expect("s465");
    assert!(matches!(
        s465.try_gain_dbi(DVec3::Z, f),
        Err(PropagationError::AngleOutsidePattern { .. })
    ));
    assert!(s465.gain_dbi(DVec3::Z, f).is_nan());
    let g_5deg = s465.gain_dbi(off_axis_dir(5.0), f);
    assert!((g_5deg - (32.0 - 25.0 * 5.0_f64.log10())).abs() < 1e-12);
    assert!((s465.gain_dbi(off_axis_dir(60.0), f) - (-10.0)).abs() < 1e-12);
    assert!(s465.gain_dbi(off_axis_dir(5.0), 500.0e6).is_nan());

    let s465_held = s465.with_main_beam_policy(MainBeamPolicy::HoldMinAngle);
    assert!(s465_held.gain_dbi(DVec3::Z, f).is_finite());

    let s580 = S580Pattern::new(2.4, f).expect("s580");
    assert!(s580.gain_dbi(DVec3::Z, f).is_nan());
    let s580_5deg = s580.gain_dbi(off_axis_dir(5.0), f);
    assert!((s580_5deg - (29.0 - 25.0 * 5.0_f64.log10())).abs() < 1e-12);
    assert!(matches!(
        S580Pattern::new(0.5, f),
        Err(PropagationError::InvalidParameter { .. })
    ));

    let f1245 = F1245Pattern::new(1.8, f).expect("f1245");
    let d_over_lambda = 1.8 / (SPEED_OF_LIGHT_M_S / f);
    let expected_peak = 20.0 * d_over_lambda.log10() + 7.7;
    assert!((f1245.peak_gain_dbi(f) - expected_peak).abs() < 1e-12);
    assert!((f1245.gain_dbi(DVec3::Z, f) - expected_peak).abs() < 1e-12);
    assert!(f1245.gain_dbi(off_axis_dir(10.0), f) < expected_peak);
    let expected_floor = -3.0 - 5.0 * d_over_lambda.log10();
    assert!((f1245.gain_dbi(-DVec3::Z, f) - expected_floor).abs() < 1e-12);
    let f1245_large = F1245Pattern::new(3.0, f).expect("f1245 large");
    assert!((f1245_large.gain_dbi(-DVec3::Z, f) - (-13.0)).abs() < 1e-12);

    // Default S.465 main-beam refusal fails link budget closed rather than returning NaN margin.
    let link = terrestrial_link(f);
    let p = PathGeometry::from_path(
        TdbEpoch::J2000,
        f,
        DVec3::ZERO,
        DVec3::X * 1.0e5,
        1.0e5,
        0.0,
    );
    let refused = compute_link_budget(&LinkBudgetInputs {
        link: &link,
        transmit_antenna: &s465,
        receive_antenna: &f1245,
        propagation: &[],
        path: &p,
        transmit_look_in_mount: Some(DVec3::Z),
        receive_look_in_mount: Some(DVec3::Z),
    });
    assert!(matches!(
        refused,
        Err(BudgetUnavailableReason::PropagationModelInapplicable { .. })
    ));
}

#[test]
fn p526_knife_edge_smooth_earth_and_delta_bullington_losses_match_physics() {
    let f = 900.0e6;
    let ke_model = KnifeEdgePropagationLoss::from_geometry(
        KnifeEdgeMethod::FresnelIntegral,
        KnifeEdgeGeometry {
            height_m: 0.0,
            d1_m: 5_000.0,
            d2_m: 5_000.0,
        },
    );
    let p_flat = PathGeometry::from_path(
        TdbEpoch::J2000,
        f,
        DVec3::ZERO,
        DVec3::X * 10_000.0,
        10_000.0,
        0.0,
    );
    assert!((ke_model.loss_db(&p_flat).expect("grazing") - 6.020_6).abs() < 0.05);

    let clear_profile = hill_profile(20_000.0, 41, 0.0);
    let ridge_profile = hill_profile(20_000.0, 41, 180.0);
    let src = Geodetic::from_degrees(39.0, -75.5, 50.0);
    let dst = Geodetic::from_degrees(39.0, -75.268_8, 50.0);

    let p_clear = p_flat
        .clone()
        .with_polarisation(Polarisation::Vertical)
        .with_geodetic_sites(Some(src), Some(dst))
        .with_terrain_profile(clear_profile);
    let p_ridge = p_flat
        .clone()
        .with_polarisation(Polarisation::Vertical)
        .with_geodetic_sites(Some(src), Some(dst))
        .with_terrain_profile(ridge_profile);

    let db_model =
        DeltaBullingtonPropagationLoss::new(DEFAULT_EFFECTIVE_EARTH_RADIUS_M, Ground::LAND)
            .expect("delta bullington");
    let clear_loss = db_model.loss_db(&p_clear).expect("clear bullington");
    let ridge_loss = db_model.loss_db(&p_ridge).expect("ridge bullington");
    assert!(
        ridge_loss > clear_loss + 10.0,
        "ridge={ridge_loss} clear={clear_loss}"
    );

    let ke_prof = KnifeEdgePropagationLoss::from_terrain_profile(KnifeEdgeMethod::FresnelIntegral);
    assert!(ke_prof.loss_db(&p_ridge).expect("profile ke") > 10.0);

    // Beyond smooth-Earth radio horizon (~48 km at 50 m + 50 m heights on ae = 8500 km):
    let far_src = Geodetic::from_degrees(39.0, -75.5, 50.0);
    let far_dst = Geodetic::from_degrees(39.0, -74.5, 50.0);
    let se_model = SmoothEarthPropagationLoss::new(DEFAULT_EFFECTIVE_EARTH_RADIUS_M, Ground::LAND)
        .expect("smooth earth");
    let se_far = se_model
        .evaluate(
            &p_flat
                .clone()
                .with_geodetic_sites(Some(far_src), Some(far_dst)),
        )
        .expect("beyond horizon");
    assert!(!se_far.line_of_sight);
    assert!(
        se_far.loss_db > 15.0,
        "smooth earth diff={}",
        se_far.loss_db
    );
}

#[test]
fn itm_terrain_loss_distinguishes_line_of_sight_from_obstructed_paths() {
    let f = 915.0e6;
    let itm = ItmPropagationLoss::new(
        Climate::ContinentalTemperate,
        301.0,
        Ground::LAND,
        VariabilityMode::SingleMessage,
    )
    .expect("itm");

    let exceedance = ExceedancePercentages::try_new(50.0, 50.0, 50.0).expect("50%");
    let src = Geodetic::from_degrees(39.1, -75.5, 40.0);
    let dst = Geodetic::from_degrees(39.1, -75.27, 40.0);
    let base = PathGeometry::from_path(
        TdbEpoch::J2000,
        f,
        DVec3::ZERO,
        DVec3::X * 20_000.0,
        20_000.0,
        0.0,
    )
    .with_polarisation(Polarisation::Vertical)
    .with_exceedance(exceedance)
    .with_geodetic_sites(Some(src), Some(dst));

    let los = itm
        .evaluate(
            &base
                .clone()
                .with_terrain_profile(flat_profile(20_000.0, 41, 0.0)),
        )
        .expect("los itm");
    let obstructed = itm
        .evaluate(&base.with_terrain_profile(hill_profile(20_000.0, 41, 220.0)))
        .expect("obstructed itm");

    assert_eq!(los.mode, PropagationMode::LineOfSight);
    assert!(obstructed.basic_transmission_loss_db > los.basic_transmission_loss_db + 12.0);

    let budget_obstructed = compute_link_budget(&LinkBudgetInputs {
        link: &terrestrial_link(f),
        transmit_antenna: &AntennaSpec::Isotropic,
        receive_antenna: &AntennaSpec::Isotropic,
        propagation: &[&itm as &dyn PropagationLoss],
        path: &PathGeometry::from_path(
            TdbEpoch::J2000,
            f,
            DVec3::ZERO,
            DVec3::X * 20_000.0,
            20_000.0,
            0.0,
        )
        .with_polarisation(Polarisation::Vertical)
        .with_exceedance(exceedance)
        .with_geodetic_sites(Some(src), Some(dst))
        .with_terrain_profile(hill_profile(20_000.0, 41, 220.0)),
        transmit_look_in_mount: Some(DVec3::Z),
        receive_look_in_mount: Some(DVec3::Z),
    })
    .expect("itm budget");
    assert_eq!(
        budget_obstructed.propagation_losses[0].model,
        "ntia-itm-longley-rice"
    );
    assert!(budget_obstructed.propagation_losses[0].loss_db > 10.0);
}
