use glam::DVec3;
use navigate_contract::{GeodeticPosition, SourceId, SymmetricCov3};
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};

use super::*;
use crate::{NavMeans, NavigationOrigin};

fn truth_at(instant: SimInstant) -> Truth<SimTruth> {
    let vehicle =
        VehicleState::level_flight(&Geodetic::from_degrees(39.12, -75.48, 800.0), 1.1, 55.0);
    Truth::new(SimTruth::vehicle(instant, vehicle))
}

/// An output `offset_enu_m` from the truth, with diagonal NED variances.
fn output_off(
    truth: &Truth<SimTruth>,
    offset_enu_m: DVec3,
    variances_ned: [f64; 3],
    delay: TickSpan,
) -> Estimate<NavigationOutput> {
    let truth_ecef = truth.get().vehicle.position_ecef;
    let frame = LocalFrame::at(&ecef_to_geodetic(truth_ecef));
    let at = ecef_to_geodetic(truth_ecef + frame.enu_to_ecef() * offset_enu_m);
    let [n, e, d] = variances_ned;
    Estimate::new(NavigationOutput {
        origin: NavigationOrigin::Standalone(NavMeans::Visual),
        source: SourceId::new(3),
        valid_at: truth.get().instant,
        delivered_at: truth.get().instant.advanced(delay),
        position: GeodeticPosition::new(at.lat_rad, at.lon_rad, at.height_m),
        position_cov: SymmetricCov3::from_diagonal(n, e, d),
        velocity: None,
        labels: Vec::new(),
    })
}

#[test]
fn a_known_offset_scores_its_error_nees_and_latency() {
    let truth = truth_at(SimInstant::START);
    let delay = TickSpan::from_ticks(TICKS_PER_SECOND / 10 * 3);
    let output = output_off(&truth, DVec3::new(4.0, 3.0, 2.0), [4.0, 4.0, 1.0], delay);
    let score = score_position(&truth, &output).unwrap();
    // The estimate's tangent plane is not the truth's, which moves each
    // component by micrometres.
    let [east, north, up] = score.error_enu_m;
    assert!((east + 4.0).abs() < 1e-4, "{score:?}");
    assert!((north + 3.0).abs() < 1e-4, "{score:?}");
    assert!((up + 2.0).abs() < 1e-4, "{score:?}");
    assert!((score.horizontal_error_m - 5.0).abs() < 1e-4);
    assert!((score.vertical_error_m - 2.0).abs() < 1e-4);
    // 3²/4 + 4²/4 + 2²/1
    assert!((score.position_nees - 10.25).abs() < 1e-4, "{score:?}");
    assert!((score.latency_s - 0.3).abs() < 1e-12);
}

#[test]
fn truth_from_another_instant_is_refused() {
    let truth = truth_at(SimInstant::START);
    let output = output_off(&truth, DVec3::ZERO, [1.0; 3], TickSpan::ZERO);
    let later = truth_at(SimInstant::START.advanced(TickSpan::from_ticks(1)));
    assert!(matches!(
        score_position(&later, &output),
        Err(NavigateBridgeError::TruthInstantMismatch { .. })
    ));
}

#[test]
fn percentiles_are_nearest_rank() {
    let values: Vec<f64> = (1..=20).rev().map(f64::from).collect();
    let p = Percentiles::of(&values).unwrap();
    assert_eq!((p.p50, p.p95, p.max), (10.0, 19.0, 20.0));
    assert_eq!(Percentiles::of(&[]), None);
    let one = Percentiles::of(&[7.0]).unwrap();
    assert_eq!((one.p50, one.p95, one.max), (7.0, 7.0, 7.0));
}

#[test]
fn availability_counts_delivered_against_due() {
    let mut evaluation = MeansEvaluation::default();
    assert_eq!(evaluation.summary().availability, None);
    let truth = truth_at(SimInstant::START);
    for _ in 0..4 {
        evaluation.expect();
    }
    for offset in [1.0, 3.0, 2.0] {
        let output = output_off(
            &truth,
            DVec3::new(offset, 0.0, 0.0),
            [1.0; 3],
            TickSpan::ZERO,
        );
        evaluation.record(score_position(&truth, &output).unwrap());
    }
    let summary = evaluation.summary();
    assert_eq!((summary.expected, summary.delivered), (4, 3));
    assert_eq!(summary.availability, Some(0.75));
    let horizontal = summary.horizontal_error_m.unwrap();
    assert!((horizontal.p50 - 2.0).abs() < 1e-6 && (horizontal.max - 3.0).abs() < 1e-6);
    assert!((summary.mean_nees.unwrap() - 14.0 / 3.0).abs() < 1e-6);
}
