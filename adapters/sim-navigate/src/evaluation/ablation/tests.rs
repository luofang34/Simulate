use navigate_contract::{GeodeticPosition, SourceId, SymmetricCov3};
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};

use super::*;

#[test]
fn a_set_holds_exactly_the_means_put_in_it() {
    let set = AblationSet::only(NavMeans::Visual)
        .with(NavMeans::DeadReckoning)
        .with(NavMeans::Celestial)
        .without(NavMeans::Visual);
    assert!(set.contains(NavMeans::DeadReckoning) && set.contains(NavMeans::Celestial));
    assert!(!set.contains(NavMeans::Visual) && !set.contains(NavMeans::Gnss));
    assert_eq!(
        set.means().collect::<Vec<_>>(),
        vec![NavMeans::DeadReckoning, NavMeans::Celestial]
    );
    assert_eq!(AblationSet::all().means().count(), NavMeans::ALL.len());
    assert_eq!(AblationSet::EMPTY.means().count(), 0);
    assert_ne!(
        AblationSet::all().without(NavMeans::Gnss),
        AblationSet::all()
    );
}

/// A straight eastbound flight at one sample per second.
fn truth(second: u64) -> Truth<SimTruth> {
    let start = Geodetic::from_degrees(39.1, -75.5, 700.0);
    let mut vehicle = VehicleState::level_flight(&start, 90_f64.to_radians(), 50.0);
    vehicle.position_ecef += vehicle.velocity_ecef * second as f64;
    let instant = SimInstant::START.advanced(TickSpan::from_ticks(second * TICKS_PER_SECOND));
    Truth::new(SimTruth::vehicle(instant, vehicle))
}

fn output(
    truth: &Truth<SimTruth>,
    origin: NavigationOrigin,
    north_error_m: f64,
    sigma_m: f64,
) -> Estimate<NavigationOutput> {
    let ecef = truth.get().vehicle.position_ecef;
    let frame = LocalFrame::at(&ecef_to_geodetic(ecef));
    let at = ecef_to_geodetic(ecef + frame.north() * north_error_m);
    let variance = sigma_m * sigma_m;
    Estimate::new(NavigationOutput {
        origin,
        source: SourceId::new(5),
        valid_at: truth.get().instant,
        delivered_at: truth.get().instant,
        position: GeodeticPosition::new(at.lat_rad, at.lon_rad, at.height_m),
        position_cov: SymmetricCov3::from_diagonal(variance, variance, variance),
        velocity: None,
        labels: Vec::new(),
    })
}

/// Dead reckoning drifts a metre a second; vision lands within 2 m but only
/// every other second; their fusion holds 1 m every second. The summary must
/// show each of those, under the right origin.
#[test]
fn each_origin_is_scored_alone_and_the_fusion_beside_it() {
    let dr = NavigationOrigin::Standalone(NavMeans::DeadReckoning);
    let visual = NavigationOrigin::Standalone(NavMeans::Visual);
    let fused =
        NavigationOrigin::Fused(AblationSet::only(NavMeans::DeadReckoning).with(NavMeans::Visual));
    let mut evaluation = AblationEvaluation::default();
    let mut meter = ComputeMeter::default();
    for second in 1..=10_u64 {
        let truth = truth(second);
        for origin in [dr, visual, fused] {
            evaluation.expect(origin);
        }
        let drift = meter.measure(dr, || output(&truth, dr, second as f64, 3.0));
        evaluation.record(&truth, &drift).unwrap();
        if second.is_multiple_of(2) {
            evaluation
                .record(&truth, &output(&truth, visual, 2.0, 2.0))
                .unwrap();
        }
        evaluation
            .record(&truth, &output(&truth, fused, 1.0, 1.0))
            .unwrap();
    }
    let summary = evaluation.summary(&meter);
    let (dr_summary, dr_cost) = summary.standalone[&NavMeans::DeadReckoning];
    let (visual_summary, visual_cost) = summary.standalone[&NavMeans::Visual];
    let (fused_summary, _) =
        summary.fused[&AblationSet::only(NavMeans::DeadReckoning).with(NavMeans::Visual)];
    assert_eq!(dr_summary.availability, Some(1.0));
    assert_eq!(visual_summary.availability, Some(0.5));
    assert_eq!(fused_summary.availability, Some(1.0));
    assert!((dr_summary.horizontal_error_m.unwrap().max - 10.0).abs() < 1e-6);
    assert!((visual_summary.horizontal_error_m.unwrap().p95 - 2.0).abs() < 1e-6);
    assert!((fused_summary.horizontal_error_m.unwrap().max - 1.0).abs() < 1e-6);
    assert_eq!(dr_cost.calls, 10);
    assert_eq!(visual_cost.calls, 0);
    assert!(!summary.standalone.contains_key(&NavMeans::Gnss));
}
