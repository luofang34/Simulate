//! Integration tests for 4-hop Mars-to-Earth causal light time, Doppler, margins, and occultation.

use sim_analysis::{
    AccessRestrictionReason, BudgetUnavailableReason, ChainAssetNode, ChainEvaluationReport,
    ChainHopRfSpec, ChainNodeSpec, ChainPropagationDirection, ChainQuerySpec,
    OccultationEnvironmentModel, VehicleSensorModel, evaluate_chain_at_epoch,
};
use sim_geo::BodyEllipsoid;
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{
    NaifId, SPEED_OF_LIGHT_M_S, SampledKinematicTrajectory, TdbEpoch, ValidEpochInterval,
};

use crate::fixtures::{
    MU_EARTH_M3_S2, MU_MARS_M3_S2, circular_equatorial_orbit, circular_orbit_passing_zenith,
    mars_opposition_epoch, rf_link, solar_system_occultation_env, ssb_pos,
    static_ssb_offset_trajectory, sub_solar_or_mars_facing_station, sun_earth_l2_relay_state,
};

struct FourHopFixture {
    solar: BuiltinSolarSystem2026_2028,
    t_rx: TdbEpoch,
    rover: sim_analysis::GroundStationModel,
    mars_orbiter: SampledKinematicTrajectory,
    l2_relay: SampledKinematicTrajectory,
    geo_relay: SampledKinematicTrajectory,
    goldstone: sim_analysis::GroundStationModel,
}

fn build_four_hop_fixture() -> FourHopFixture {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t_rx = mars_opposition_epoch();
    let traj_start = t_rx.advanced_seconds(-1_800.0);
    let span = ValidEpochInterval::new(traj_start, traj_start.advanced_seconds(3_600.0));
    let mars_pos = ssb_pos(&solar, NaifId::MARS, t_rx);
    let earth_pos = ssb_pos(&solar, NaifId::EARTH, t_rx);
    let mars_to_earth = (earth_pos - mars_pos).normalize();
    let earth_to_mars = -mars_to_earth;
    let t_mars = t_rx.advanced_seconds(-350.0);
    let rover = sub_solar_or_mars_facing_station(
        &solar,
        t_mars,
        mars_to_earth,
        101,
        "Jezero-Surface",
        NaifId::MARS,
        BodyEllipsoid::MARS_IAU,
    );
    let mars_orbiter = circular_orbit_passing_zenith(
        201,
        NaifId::MARS,
        MU_MARS_M3_S2,
        4_800_000.0,
        mars_to_earth,
        t_mars,
        span,
    );
    let (l2_pos, l2_vel) = sun_earth_l2_relay_state(&solar, t_rx);
    let l2_relay = static_ssb_offset_trajectory(301, l2_pos, l2_vel, traj_start, 3_600.0);
    let geo_relay = circular_orbit_passing_zenith(
        401,
        NaifId::EARTH,
        MU_EARTH_M3_S2,
        42_164_000.0,
        earth_to_mars,
        t_rx,
        span,
    );
    let goldstone = sub_solar_or_mars_facing_station(
        &solar,
        t_rx,
        earth_to_mars,
        501,
        "DSS-14-Goldstone",
        NaifId::EARTH,
        BodyEllipsoid::WGS84,
    );
    FourHopFixture {
        solar,
        t_rx,
        rover,
        mars_orbiter,
        l2_relay,
        geo_relay,
        goldstone,
    }
}

fn four_hop_nodes<'a>(
    f: &'a FourHopFixture,
    omni: &'a VehicleSensorModel,
) -> [ChainNodeSpec<'a>; 5] {
    [
        ChainNodeSpec::ground_station(&f.rover),
        ChainNodeSpec::Asset(ChainAssetNode::new(
            201,
            "Mars-Orbiter",
            NaifId::MARS,
            BodyEllipsoid::MARS_IAU,
            &f.mars_orbiter,
            None,
            omni,
        )),
        ChainNodeSpec::Asset(ChainAssetNode::new(
            301,
            "Sun-Earth-L2",
            NaifId::EARTH,
            BodyEllipsoid::WGS84,
            &f.l2_relay,
            None,
            omni,
        )),
        ChainNodeSpec::Asset(ChainAssetNode::new(
            401,
            "Earth-GEO",
            NaifId::EARTH,
            BodyEllipsoid::WGS84,
            &f.geo_relay,
            None,
            omni,
        )),
        ChainNodeSpec::ground_station(&f.goldstone),
    ]
}

fn assert_four_hop_invariants(bwd: &ChainEvaluationReport, t_rx: TdbEpoch) {
    assert!(
        bwd.end_to_end_accessible,
        "blocking: {:?}",
        bwd.blocking_hop
    );
    assert_eq!(bwd.hops.len(), 4);
    assert_eq!(bwd.rx_epoch, t_rx);
    let sum_lt: f64 = bwd.hops.iter().map(|h| h.light_time_s).sum();
    assert!((sum_lt - bwd.total_light_time_s).abs() < 1.0e-12);
    assert!((bwd.rx_epoch.signed_offset_seconds_from(bwd.tx_epoch) - sum_lt).abs() < 1.0e-6);
    assert!(bwd.hops.windows(2).all(|p| p[0].rx_epoch == p[1].tx_epoch));
    for h in &bwd.hops {
        let res = (h.target_pos_icrf_m - h.source_pos_icrf_m).length()
            - SPEED_OF_LIGHT_M_S * h.light_time_s;
        assert!(res.abs() < 1.0e-3, "hop {} residual {res}", h.hop_index);
    }
    let prod_doppler = bwd.hops.iter().fold(1.0, |acc, h| acc * h.doppler_factor);
    assert!((bwd.net_doppler_factor - prod_doppler).abs() < 1.0e-15);
    let expected_min = bwd
        .hops
        .iter()
        .filter_map(|h| h.margin_db)
        .fold(f64::INFINITY, f64::min);
    assert_eq!(bwd.min_margin_db, Some(expected_min));
    assert_eq!(bwd.hops[1].margin_db, Some(expected_min));
}

#[test]
fn four_hop_mars_to_earth_causal_light_time_doppler_and_bottleneck_margin() {
    let f = build_four_hop_fixture();
    let omni = VehicleSensorModel::omnidirectional_at_cg(1, "Omni-Transponder");
    let nodes = four_hop_nodes(&f, &omni);
    let rf0 = rf_link(401.5e6, 20.0, 0.5, 64_000.0, 5.0);
    let rf1 = rf_link(32.0e9, 4.0, 0.4, 2_000_000.0, 9.0);
    let rf2 = rf_link(32.0e9, 50.0, 1.8, 2_000_000.0, 4.0);
    let rf3 = rf_link(8.4e9, 20.0, 3.0, 2_000_000.0, 4.0);
    let hop_rf = [
        Some(ChainHopRfSpec::from_link(&rf0)),
        Some(ChainHopRfSpec::from_link(&rf1)),
        Some(ChainHopRfSpec::from_link(&rf2)),
        Some(ChainHopRfSpec::from_link(&rf3)),
    ];
    let occ = solar_system_occultation_env();
    let bwd_q = ChainQuerySpec::light_time(occ.clone(), ChainPropagationDirection::BackwardFromRx);
    let bwd = evaluate_chain_at_epoch(
        &nodes,
        Some(&hop_rf),
        f.t_rx,
        &bwd_q,
        None,
        &f.solar.ephemeris,
        &f.solar.orientation,
    )
    .expect("4-hop backward causal chain");
    assert_four_hop_invariants(&bwd, f.t_rx);

    let fwd_q = ChainQuerySpec::light_time(occ, ChainPropagationDirection::ForwardFromTx);
    let fwd = evaluate_chain_at_epoch(
        &nodes,
        Some(&hop_rf),
        bwd.tx_epoch,
        &fwd_q,
        None,
        &f.solar.ephemeris,
        &f.solar.orientation,
    )
    .expect("4-hop forward roundtrip");
    assert!((fwd.rx_epoch.signed_offset_seconds_from(bwd.rx_epoch)).abs() < 1.0e-6);
    assert!((fwd.total_light_time_s - bwd.total_light_time_s).abs() < 1.0e-6);
}

#[test]
fn three_hop_planetary_occultation_on_hop2_breaks_chain_cleanly() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = mars_opposition_epoch();
    let start = t0.advanced_seconds(-600.0);
    let r = 10_000_000.0;
    let span = ValidEpochInterval::new(start, start.advanced_seconds(1_200.0));
    let orb = |id, deg: f64| {
        circular_equatorial_orbit(id, NaifId::EARTH, MU_EARTH_M3_S2, r, deg.to_radians(), span)
    };
    let (sa, sb, sc, sd) = (orb(1, 30.0), orb(2, 90.0), orb(3, -90.0), orb(4, -30.0));
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
        mk(1, "A", &sa),
        mk(2, "B", &sb),
        mk(3, "C", &sc),
        mk(4, "D", &sd),
    ];
    let rf = rf_link(2.2e9, 10.0, 1.0, 1.0e6, 5.0);
    let hrf = [
        Some(ChainHopRfSpec::from_link(&rf)),
        Some(ChainHopRfSpec::from_link(&rf)),
        Some(ChainHopRfSpec::from_link(&rf)),
    ];
    let q = ChainQuerySpec::light_time(
        OccultationEnvironmentModel::earth_wgs84(),
        ChainPropagationDirection::BackwardFromRx,
    );
    let rep = evaluate_chain_at_epoch(
        &nodes,
        Some(&hrf),
        t0,
        &q,
        None,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("3-hop occulted evaluation");

    assert!(!rep.end_to_end_accessible);
    assert_eq!(rep.blocking_hop, Some(1));
    assert_eq!(rep.min_margin_db, None);
    assert!(rep.hops[0].accessible && rep.hops[0].link_budget.is_some());
    assert!(!rep.hops[1].accessible && rep.hops[1].link_budget.is_none());
    assert_eq!(
        rep.hops[1].budget_unavailable,
        Some(BudgetUnavailableReason::NotAccessible)
    );
    assert!(rep.hops[1].active_restrictions.iter().any(|r| matches!(
        r,
        AccessRestrictionReason::OccultedByBody {
            body: NaifId::EARTH,
            ..
        }
    )));
    assert!(rep.hops[2].accessible && rep.hops[2].link_budget.is_some());
}
