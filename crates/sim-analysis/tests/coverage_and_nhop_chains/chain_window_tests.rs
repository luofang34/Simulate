//! Integration tests for N-hop chain windows, directional FOV constraints, and fail-closed errors.

use glam::{DQuat, DVec3};
use sim_analysis::{
    AccessRestrictionReason, ChainAssetNode, ChainEvaluationError, ChainHopRfSpec, ChainNodeSpec,
    ChainPropagationDirection, ChainQuerySpec, OccultationEnvironmentModel, SensorFovModel,
    SignalTimeModel, VehicleSensorModel, WindowSearchCompleteness, evaluate_chain_at_epoch,
    evaluate_chain_windows,
};
use sim_geo::BodyEllipsoid;
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{NaifId, SensorMount, ValidEpochInterval};

use crate::fixtures::{
    MU_EARTH_M3_S2, circular_equatorial_orbit, mars_opposition_epoch, rf_link, ssb_pos,
    static_ssb_offset_trajectory, sub_solar_or_mars_facing_station,
};

#[test]
fn three_hop_chain_windows_extract_refined_passes() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let start = t0.advanced_seconds(-300.0);
    let span = ValidEpochInterval::new(start, start.advanced_seconds(7_200.0));
    let orb =
        |id, rad, ph| circular_equatorial_orbit(id, NaifId::EARTH, MU_EARTH_M3_S2, rad, ph, span);
    let leo = orb(10, 7_378_137.0, 0.0);
    let geo1 = orb(11, 42_164_000.0, 0.3);
    let geo2 = orb(12, 42_164_000.0, 1.0);
    let gnd = sub_solar_or_mars_facing_station(
        &solar,
        t0,
        DVec3::new(1.0_f64.cos(), 1.0_f64.sin(), 0.0),
        13,
        "Eq-Station",
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
    );
    let omni = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let mk = |id, lbl, traj| {
        ChainNodeSpec::Asset(ChainAssetNode::new(
            id,
            lbl,
            NaifId::EARTH,
            BodyEllipsoid::WGS84,
            traj,
            None,
            &omni,
        ))
    };
    let nodes = [
        mk(10, "LEO", &leo),
        mk(11, "GEO1", &geo1),
        mk(12, "GEO2", &geo2),
        ChainNodeSpec::ground_station(&gnd),
    ];
    let rf = rf_link(2.2e9, 25.0, 1.5, 1.0e6, 3.0);
    let hrf = [
        Some(ChainHopRfSpec::from_link(&rf)),
        Some(ChainHopRfSpec::from_link(&rf)),
        Some(ChainHopRfSpec::from_link(&rf)),
    ];
    let q = ChainQuerySpec::light_time(
        OccultationEnvironmentModel::earth_wgs84(),
        ChainPropagationDirection::BackwardFromRx,
    );
    let win_rep = evaluate_chain_windows(
        &nodes,
        Some(&hrf),
        ValidEpochInterval::new(t0, t0.advanced_seconds(3_600.0)),
        60.0,
        0.5,
        &q,
        None,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("3-hop chain windows");
    assert!(matches!(
        win_rep.completeness,
        WindowSearchCompleteness::BracketRefinedOnGrid { .. }
    ));
    assert!(win_rep.verification_scope.rf_link_budget_verified);
    assert!(!win_rep.windows.is_empty());
    assert!(win_rep.windows.iter().all(|w| w.duration_s > 0.0));
}

#[test]
fn directional_boresight_constraint_blocks_misaligned_hop() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let start = t0.advanced_seconds(-300.0);
    let span = ValidEpochInterval::new(start, start.advanced_seconds(1_200.0));
    let leo = circular_equatorial_orbit(10, NaifId::EARTH, MU_EARTH_M3_S2, 7_378_137.0, 0.0, span);
    let geo1 =
        circular_equatorial_orbit(11, NaifId::EARTH, MU_EARTH_M3_S2, 42_164_000.0, 0.3, span);
    let omni = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let narrow = VehicleSensorModel::directional(
        2,
        "Narrow-Boresight",
        SensorMount {
            offset_body_m: DVec3::ZERO,
            body_to_mount: DQuat::IDENTITY,
        },
        DVec3::ZERO,
        SensorFovModel::ConicalHalfAngle {
            max_half_angle_rad: 0.05,
        },
    );
    let fov_nodes = [
        ChainNodeSpec::Asset(ChainAssetNode::new(
            10,
            "LEO",
            NaifId::EARTH,
            BodyEllipsoid::WGS84,
            &leo,
            Some(&leo),
            &narrow,
        )),
        ChainNodeSpec::Asset(ChainAssetNode::new(
            11,
            "GEO1",
            NaifId::EARTH,
            BodyEllipsoid::WGS84,
            &geo1,
            None,
            &omni,
        )),
    ];
    let geo_q = ChainQuerySpec::geometric(OccultationEnvironmentModel::earth_wgs84());
    let fov_rep = evaluate_chain_at_epoch(
        &fov_nodes,
        None,
        t0,
        &geo_q,
        None,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("fov check");
    assert_eq!(fov_rep.signal_time, SignalTimeModel::Geometric);
    assert!(!fov_rep.end_to_end_accessible);
    assert!(
        fov_rep.hops[0]
            .active_restrictions
            .iter()
            .any(|r| matches!(r, AccessRestrictionReason::OutsideVehicleSensorFov { .. }))
    );
}

#[test]
fn chain_validation_errors_fail_closed() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let gnd = sub_solar_or_mars_facing_station(
        &solar,
        t0,
        DVec3::X,
        13,
        "Eq-Station",
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
    );
    let nodes = [
        ChainNodeSpec::ground_station(&gnd),
        ChainNodeSpec::ground_station(&gnd),
    ];
    let rf = rf_link(2.2e9, 10.0, 1.0, 1.0e6, 3.0);
    let q = ChainQuerySpec::light_time(
        OccultationEnvironmentModel::earth_wgs84(),
        ChainPropagationDirection::BackwardFromRx,
    );
    let eval = |ns, hrf| {
        evaluate_chain_at_epoch(ns, hrf, t0, &q, None, &solar.ephemeris, &solar.orientation)
    };
    assert!(matches!(
        eval(&nodes[..1], None),
        Err(ChainEvaluationError::TooFewNodes { node_count: 1 })
    ));
    let two_rf = [
        Some(ChainHopRfSpec::from_link(&rf)),
        Some(ChainHopRfSpec::from_link(&rf)),
    ];
    assert!(matches!(
        eval(&nodes, Some(&two_rf)),
        Err(ChainEvaluationError::HopRfSpecCountMismatch {
            expected: 1,
            actual: 2
        })
    ));
}

#[test]
fn chain_light_time_non_convergence_fails_closed() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let gnd = sub_solar_or_mars_facing_station(
        &solar,
        t0,
        DVec3::X,
        13,
        "Eq-Station",
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
    );
    let omni = VehicleSensorModel::omnidirectional_at_cg(1, "Omni");
    let earth_pos = ssb_pos(&solar, NaifId::EARTH, t0);
    let fast_probe = static_ssb_offset_trajectory(
        99,
        earth_pos + DVec3::new(1.5e11, 0.0, 0.0),
        DVec3::new(40_000.0, 0.0, 0.0),
        t0.advanced_seconds(-2_000.0),
        4_000.0,
    );
    let nodes = [
        ChainNodeSpec::Asset(ChainAssetNode::new(
            99,
            "FastProbe",
            NaifId::EARTH,
            BodyEllipsoid::WGS84,
            &fast_probe,
            None,
            &omni,
        )),
        ChainNodeSpec::ground_station(&gnd),
    ];
    let mut q = ChainQuerySpec::light_time(
        OccultationEnvironmentModel::earth_wgs84(),
        ChainPropagationDirection::BackwardFromRx,
    );
    q.max_light_time_iterations = 1;
    let res = evaluate_chain_at_epoch(
        &nodes,
        None,
        t0,
        &q,
        None,
        &solar.ephemeris,
        &solar.orientation,
    );
    assert!(matches!(
        res,
        Err(ChainEvaluationError::LightTimeDidNotConverge {
            hop_index: 0,
            max_iterations: 1,
            ..
        })
    ));
}
