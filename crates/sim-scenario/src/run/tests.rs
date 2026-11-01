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
