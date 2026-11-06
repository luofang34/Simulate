//! Corridor flights steered purely on positions recovered from rendered
//! pixels: a nadir camera's CPU-rendered frames go through
//! `ImageCorridorEstimator`, its fixes through the two-tier harness into the
//! run's guidance filter, and the truth is only ever scored.

use std::sync::{Arc, Mutex};

use sim_core::{FusionEligibility, SimInstant};
use sim_navigate::{ImageCorridorEstimator, ImageEstimatorConfig};
use sim_scenario::{
    CorridorRun, CorridorVisualEstimator, FlightDynamics, ImageDrivenConfig, ScenarioProfile,
    SensorObservation, VisualTiers,
};
use sim_scenery::{CatalogScenery, SyntheticCorridorSpec};

/// 99.9 % of chi-square with 3 degrees of freedom.
#[cfg(test)]
const NEES_GATE: f64 = 16.27;

/// The Delaware corridor at zoom 14, about 7 m imagery, covering every
/// corridor profile's path.
#[cfg(test)]
fn world() -> Arc<CatalogScenery> {
    let spec = SyntheticCorridorSpec {
        zoom: 14,
        tiles_east: 7,
        tiles_north: 4,
        ..SyntheticCorridorSpec::delaware()
    };
    Arc::new(CatalogScenery::synthetic_corridor(&spec).unwrap())
}

#[cfg(test)]
fn estimator(world: &Arc<CatalogScenery>, profile: ScenarioProfile) -> ImageCorridorEstimator {
    let config = ImageDrivenConfig::new(Arc::clone(world));
    let mut estimator = ImageEstimatorConfig::new(
        Arc::clone(world),
        CorridorRun::enu_anchor(),
        CorridorRun::planned_start_enu_m(profile),
        config.seed_sigma_m,
    );
    estimator.attitude_sigma_rad = config.attitude_bias_sigma_rad;
    estimator.agl_sigma_m = config.agl_sigma_m;
    ImageCorridorEstimator::new(estimator)
}

/// Passes everything through to the image estimator and keeps a copy of
/// what it was given and what it said.
#[cfg(test)]
#[derive(Debug, Clone)]
struct Recording {
    inner: ImageCorridorEstimator,
    log: Arc<Mutex<Vec<(SensorObservation, VisualTiers)>>>,
}

#[cfg(test)]
impl CorridorVisualEstimator for Recording {
    fn estimate(&mut self, observation: &SensorObservation) -> VisualTiers {
        let tiers = self.inner.estimate(observation);
        self.log.lock().unwrap().push((observation.clone(), tiers));
        tiers
    }

    fn boxed_clone(&self) -> Box<dyn CorridorVisualEstimator> {
        Box::new(self.clone())
    }
}

#[cfg(test)]
fn fly(run: &mut CorridorRun, seconds: f64) {
    while run.now().duration_since(SimInstant::START).as_seconds() < seconds && !run.finished() {
        run.step().unwrap();
    }
}

/// The acquisition profile believes it starts 900 m off the corridor; it
/// is really 200 m further out. Steering on pixel-registered anchors brings
/// it onto the centreline, with honest anchor and filter covariances.
#[test]
fn a_lateral_cold_start_is_recovered_from_rendered_pixels_alone() {
    let world = world();
    let profile = ScenarioProfile::CorridorOffSeedAcquisition;
    let config = ImageDrivenConfig {
        start_lateral_offset_m: 200.0,
        ..ImageDrivenConfig::new(Arc::clone(&world))
    };
    let mut run = CorridorRun::image_driven(
        profile,
        FlightDynamics::Kinematic,
        config,
        Box::new(estimator(&world, profile)),
    )
    .unwrap();
    run.step().unwrap();
    let start_offset = run.report().cross_track_m;
    assert!((start_offset - 1_100.0).abs() < 5.0, "{start_offset}");

    // It is on the centreline within a minute; the margin covers the rest.
    fly(&mut run, 75.0);
    let report = run.report().clone();
    let navigation = run.image_navigation_report().unwrap();
    assert!(
        report.cross_track_m.abs() < 15.0,
        "{report:?} {navigation:?}"
    );
    assert!(navigation.navigation_error_m < 15.0, "{navigation:?}");
    assert!(
        navigation.navigation_error_m < 3.0 * navigation.navigation_sigma_m,
        "{navigation:?}"
    );
    assert!(navigation.fused_anchors >= 4, "{navigation:?}");
    assert!(report.warm_fixes > 1_000, "{report:?}");
    let anchors = run.nees(FusionEligibility::IndependentAnchor);
    let mean = anchors.mean().unwrap();
    assert!(mean < NEES_GATE && mean > 0.03, "{anchors:?}");
    let warm = run.nees(FusionEligibility::CorrelatedTrack);
    assert!(warm.mean().unwrap() < NEES_GATE, "{warm:?}");
}

/// An orbit inside one map cell earns one independent anchor; every later
/// anchor is labelled shared, the filter refuses it, and its covariance
/// grows on dead reckoning rather than collapsing onto repeated evidence.
#[test]
fn a_same_cell_orbit_marks_revisits_shared_and_does_not_collapse() {
    let world = world();
    let profile = ScenarioProfile::CorridorSameCellLoiter;
    let mut run = CorridorRun::image_driven(
        profile,
        FlightDynamics::Kinematic,
        ImageDrivenConfig::new(Arc::clone(&world)),
        Box::new(estimator(&world, profile)),
    )
    .unwrap();
    fly(&mut run, 40.0);
    let report = run.report().clone();
    let navigation = run.image_navigation_report().unwrap();
    assert_eq!(report.independent_anchor_fixes, 1, "{report:?}");
    assert!(report.shared_map_cell_fixes >= 30, "{report:?}");
    assert_eq!(navigation.fused_anchors, 1, "{navigation:?}");
    assert_eq!(navigation.refused_shared_cell, report.shared_map_cell_fixes);
    assert!(
        navigation.navigation_sigma_m > navigation.min_navigation_sigma_m,
        "{navigation:?}"
    );
    assert!(
        navigation.navigation_error_m < 3.0 * navigation.navigation_sigma_m + 5.0,
        "{navigation:?}"
    );
    let shared = run.nees(FusionEligibility::SharedMapCell);
    assert!(shared.mean().unwrap() < NEES_GATE, "{shared:?}");
}

/// The estimator's output is a function of what it was handed and nothing
/// else: replaying the recorded observations into a fresh estimator gives
/// the same estimates bit for bit, so no truth reaches it by another path.
#[test]
fn the_estimator_sees_only_pixels_attitude_and_height() {
    let world = world();
    let profile = ScenarioProfile::CorridorNominalCruise;
    let log = Arc::new(Mutex::new(Vec::new()));
    let recording = Recording {
        inner: estimator(&world, profile),
        log: Arc::clone(&log),
    };
    let config = ImageDrivenConfig {
        start_lateral_offset_m: 120.0,
        ..ImageDrivenConfig::new(Arc::clone(&world))
    };
    let mut run = CorridorRun::image_driven(
        profile,
        FlightDynamics::Kinematic,
        config,
        Box::new(recording),
    )
    .unwrap();
    fly(&mut run, 8.0);

    let recorded = log.lock().unwrap().clone();
    assert!(recorded.len() >= 70, "{}", recorded.len());
    assert!(
        recorded
            .iter()
            .any(|(_, tiers)| tiers.corridor_anchor.is_some())
    );
    let mut replay = estimator(&world, profile);
    for (observation, tiers) in &recorded {
        assert_eq!(&replay.estimate(observation), tiers);
    }
}
