use std::error::Error;

use sim_scenery::AerodromeId;

use super::{AirportPatternRun, PatternLeg};
use crate::FlightDynamics;

#[test]
fn c172_flies_full_uncontrolled_pattern_at_47n_with_flaps_and_virtual_cameras()
-> Result<(), Box<dyn Error>> {
    let mut run = AirportPatternRun::new(AerodromeId::CentralJersey47N, FlightDynamics::Jsbsim)?;
    assert_eq!(run.diagram().id.code(), "47N");
    assert_eq!(run.virtual_camera_specs().len(), 5);

    let mut tower_min_fx = f64::INFINITY;
    let mut tower_max_fx = 0.0_f64;

    while !run.finished() {
        run.step()?;
        if let Some(tower_cap) = run.latest_virtual_captures().get(3) {
            tower_min_fx = tower_min_fx.min(tower_cap.intrinsics.fx);
            tower_max_fx = tower_max_fx.max(tower_cap.intrinsics.fx);
        }
    }

    let report = run.report();
    assert_eq!(
        report.completed_circuits, 1,
        "47N C172 did not complete circuit within limit: {report:?}"
    );
    for leg in PatternLeg::ALL {
        assert!(
            report.legs_visited.contains(&leg),
            "47N missing leg {leg:?}: {report:?}"
        );
    }
    assert!(
        report.max_flap_norm >= 0.99,
        "flaps: {}",
        report.max_flap_norm
    );
    assert!(
        report.peak_bank_rad < 36.0_f64.to_radians(),
        "bank: {} deg",
        report.peak_bank_rad.to_degrees()
    );
    assert!(
        report.threshold_cross_track_m.abs() < 35.0,
        "47N Rwy 07 threshold cross-track: {} m",
        report.threshold_cross_track_m
    );
    assert!(
        (4.0..=40.0).contains(&report.min_threshold_agl_m),
        "47N Rwy 07 threshold AGL: {} m",
        report.min_threshold_agl_m
    );
    assert!(report.virtual_camera_batches > 3_000);
    assert!(
        tower_max_fx > tower_min_fx * 1.4,
        "47N CTAF PTZ camera did not auto-zoom across pattern range ({tower_min_fx} .. {tower_max_fx})"
    );
    Ok(())
}

#[test]
fn c172_flies_full_towered_class_b_pattern_at_kewr_runway_22l() -> Result<(), Box<dyn Error>> {
    let mut run = AirportPatternRun::new(AerodromeId::NewarkKewr, FlightDynamics::Jsbsim)?;
    assert_eq!(run.diagram().id.code(), "KEWR");
    while !run.finished() {
        run.step()?;
    }
    let report = run.report();
    assert_eq!(
        report.completed_circuits, 1,
        "KEWR C172 did not complete circuit within limit: {report:?}"
    );
    for leg in PatternLeg::ALL {
        assert!(
            report.legs_visited.contains(&leg),
            "KEWR missing leg {leg:?}: {report:?}"
        );
    }
    assert!(
        report.threshold_cross_track_m.abs() < 35.0,
        "KEWR Rwy 22L threshold cross-track: {} m",
        report.threshold_cross_track_m
    );
    assert!(
        (4.0..=40.0).contains(&report.min_threshold_agl_m),
        "KEWR Rwy 22L threshold AGL: {} m",
        report.min_threshold_agl_m
    );
    Ok(())
}
