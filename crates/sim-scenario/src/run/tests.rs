use sim_apollo::GuidanceSource;
use sim_core::SourceValidity;
use sim_geo::BodyEllipsoid;

use super::*;
use crate::ScenarioWorld;

/// Runs a profile to its end and returns it.
fn play_out(profile: ScenarioProfile) -> ScenarioRun {
    let mut run = ScenarioRun::new(profile).unwrap();
    while !run.finished() {
        run.step().unwrap();
    }
    run
}

#[test]
fn every_profile_runs_headlessly_to_its_end() {
    for profile in ScenarioProfile::ALL {
        let run = play_out(profile);
        assert!(run.finished(), "{profile:?}");
        assert_eq!(run.profile(), profile);
        assert!(run.now() > SimInstant::START, "{profile:?}");
        assert!(run.vehicle().position_ecef.is_finite(), "{profile:?}");
        assert!(
            run.eye_pose().unwrap().position_ecef.is_finite(),
            "{profile:?}"
        );
        assert!(run.cockpit().attitude.validity.is_usable(), "{profile:?}");
        assert_eq!(run.cockpit().timestamp, run.now(), "{profile:?}");
    }
}

#[test]
fn every_profile_replays_the_same_bits() {
    for profile in ScenarioProfile::ALL {
        let [mut one, mut two] = [(); 2].map(|()| ScenarioRun::new(profile).unwrap());
        for _ in 0..1_500 {
            one.step().unwrap();
            two.step().unwrap();
        }
        assert_eq!(
            one.vehicle().position_ecef.to_array().map(f64::to_bits),
            two.vehicle().position_ecef.to_array().map(f64::to_bits),
            "{profile:?}"
        );
    }
}

#[test]
fn the_digit_keys_reach_every_profile_once() {
    let by_digit: Vec<_> = (1..=8).filter_map(ScenarioProfile::from_digit).collect();
    assert_eq!(by_digit, ScenarioProfile::ALL);
    assert_eq!(ScenarioProfile::from_digit(0), None);
    assert_eq!(ScenarioProfile::from_digit(9), None);
}

#[test]
fn each_profile_flies_in_its_own_world() {
    for profile in ScenarioProfile::ALL {
        let vehicle = ScenarioRun::new(profile).unwrap().vehicle();
        let radius = vehicle.position_ecef.length();
        let world = profile.world();
        let body = world.body();
        assert!(
            (radius - body.semi_major_m).abs() < 200_000.0,
            "{profile:?} at {radius} on {body:?}"
        );
    }
    assert_eq!(
        ScenarioProfile::Apollo11LunarLanding.world(),
        ScenarioWorld::LunarTranquility
    );
    assert_eq!(
        ScenarioProfile::Apollo11LunarLanding.world().body(),
        BodyEllipsoid::MOON_IAU
    );
    assert_eq!(
        ScenarioProfile::CorridorNominalCruise.world(),
        ScenarioWorld::DelawareCorridor
    );
}

#[test]
fn the_landing_ends_with_the_contact_light_on_pgns() {
    let cockpit = play_out(ScenarioProfile::Apollo11LunarLanding).cockpit();
    assert!(cockpit.contact_light);
    assert_eq!(cockpit.guidance_source, Some(GuidanceSource::Pgns));
    assert_eq!(cockpit.attitude.validity, SourceValidity::Valid);
}

/// Steps 60 ticks, snapshots, steps 120 more, rewinds, and checks the replay
/// lands on exactly the same instant and vehicle state.
fn assert_rewind_replays(mut run: ScenarioRun) {
    for _ in 0..60 {
        run.step().unwrap();
    }
    let snap = run.snapshot();
    let snap_now = run.now();
    for _ in 0..120 {
        run.step().unwrap();
    }
    let expected_vehicle = run.vehicle();
    let expected_now = run.now();
    let expected_cockpit = run.cockpit();
    assert!(expected_now > snap_now);

    run.restore(&snap).unwrap();
    assert_eq!(run.now(), snap_now);
    for _ in 0..120 {
        run.step().unwrap();
    }
    assert_eq!(run.vehicle(), expected_vehicle, "{:?}", run.profile());
    assert_eq!(run.now(), expected_now, "{:?}", run.profile());
    assert_eq!(run.cockpit(), expected_cockpit, "{:?}", run.profile());
}

#[test]
fn rewinding_a_jsbsim_airport_pattern_replays_the_same_bits() {
    assert_rewind_replays(
        ScenarioRun::with_dynamics(
            ScenarioProfile::Airport47NUncontrolledPattern,
            FlightDynamics::Jsbsim,
        )
        .unwrap(),
    );
}

#[test]
fn rewinding_corridor_runs_replays_the_same_bits() {
    for dynamics in [FlightDynamics::Kinematic, FlightDynamics::Jsbsim] {
        assert_rewind_replays(
            ScenarioRun::with_dynamics(ScenarioProfile::CorridorNominalCruise, dynamics).unwrap(),
        );
    }
}

#[test]
fn rewinding_apollo_runs_replays_the_same_bits() {
    for profile in [
        ScenarioProfile::Apollo11LunarLanding,
        ScenarioProfile::Apollo11CapsuleReentry,
    ] {
        assert_rewind_replays(ScenarioRun::new(profile).unwrap());
    }
}

#[test]
fn a_snapshot_only_restores_into_its_own_kind_of_run() {
    let pattern = ScenarioRun::with_dynamics(
        ScenarioProfile::Airport47NUncontrolledPattern,
        FlightDynamics::Jsbsim,
    )
    .unwrap();
    let mut kinematic = ScenarioRun::with_dynamics(
        ScenarioProfile::Airport47NUncontrolledPattern,
        FlightDynamics::Kinematic,
    )
    .unwrap();
    let before = kinematic.vehicle();
    assert!(matches!(
        kinematic.restore(&pattern.snapshot()),
        Err(ScenarioError::SnapshotMismatch { .. })
    ));
    assert_eq!(kinematic.vehicle(), before);

    let mut corridor = ScenarioRun::new(ScenarioProfile::CorridorNominalCruise).unwrap();
    assert!(matches!(
        corridor.restore(&pattern.snapshot()),
        Err(ScenarioError::SnapshotMismatch { .. })
    ));
}

/// Taking the stick mid-script hands over the live state and the controls
/// that were holding it, and a manual model seeded with them flies on.
#[test]
fn manual_control_takes_over_at_the_live_state() {
    use sim_core::{Block, KinematicInput, KinematicModel};

    let mut run = ScenarioRun::with_dynamics(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Kinematic,
    )
    .unwrap();
    let (_, before) = run.take_manual_control_state();
    assert_eq!(before, sim_core::ControlInputs::default());
    for _ in 0..240 {
        run.step().unwrap();
    }
    let (state, controls) = run.take_manual_control_state();
    assert_eq!(state, run.vehicle());
    assert!(controls.throttle > 0.0, "{controls:?}");

    let mut manual = KinematicModel::new(state);
    let mut scripted = run;
    let snapshot = scripted.snapshot();
    manual.step(&KinematicInput { controls }).unwrap();
    scripted.restore(&snapshot).unwrap();
    assert_eq!(scripted.take_manual_control_state().1, controls);
    let moved = (manual.output().position_ecef - state.position_ecef).length();
    assert!(moved > 0.1, "{moved}");
}
