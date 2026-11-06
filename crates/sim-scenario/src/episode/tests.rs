use super::*;

const CRUISE: ControlInputs = ControlInputs {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
    throttle: 0.3,
};

#[test]
fn an_episode_starts_where_its_scenario_does_and_flies_on_the_stick() {
    let mut env = EpisodeEnv::new(EpisodeConfig::default()).unwrap();
    let start = env.reset(0).unwrap();
    let scripted = ScenarioRun::with_dynamics(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Kinematic,
    )
    .unwrap();
    assert_eq!(start.vehicle, scripted.vehicle());
    assert_eq!(start.step, 0);
    assert!(start.agl_m.unwrap() > 700.0, "{start:?}");

    let banked = ControlInputs {
        roll: 0.5,
        ..CRUISE
    };
    let mut last = None;
    for _ in 0..120 {
        last = Some(env.step(banked).unwrap());
    }
    let last = last.unwrap();
    assert_eq!(last.observation.step, 120);
    assert_eq!(last.observation.controls, banked);
    assert!(!last.terminated && !last.truncated);
    let turned = last
        .observation
        .vehicle
        .attitude
        .angle_between(start.vehicle.attitude);
    assert!(turned > 0.3, "{turned}");
}

#[test]
fn the_atmosphere_is_read_at_the_orthometric_height() {
    let mut env = EpisodeEnv::new(EpisodeConfig::default()).unwrap();
    let observation = env.reset(0).unwrap();
    let undulation = observation.position.height_m - observation.orthometric_msl_m;
    let expected =
        sim_geo::egm96_undulation_m(observation.position.lat_rad, observation.position.lon_rad);
    assert!((undulation - expected).abs() < 1e-6);
    let air = observation.atmosphere.unwrap();
    let isa = sim_atmosphere::isa_at_geopotential(sim_atmosphere::geometric_to_geopotential_m(
        observation.orthometric_msl_m,
    ))
    .unwrap();
    assert_eq!(air.pressure_pa, isa.pressure_pa);
    assert_eq!(air.density_kg_m3, isa.density_kg_m3);
}

#[test]
fn seeds_scatter_starts_reproducibly_and_snapshots_replay_bit_for_bit() {
    let config = EpisodeConfig {
        start_jitter_m: 50.0,
        start_heading_jitter_rad: 0.1,
        max_steps: 40,
        ..EpisodeConfig::default()
    };
    let mut env = EpisodeEnv::new(config).unwrap();
    let a = env.reset(7).unwrap();
    let b = env.reset(8).unwrap();
    assert_ne!(a.vehicle.position_ecef, b.vehicle.position_ecef);
    assert_eq!(env.reset(7).unwrap(), a);

    for _ in 0..10 {
        env.step(CRUISE).unwrap();
    }
    let snapshot = env.snapshot().unwrap();
    let wild = ControlInputs {
        roll: -0.8,
        pitch: 0.4,
        ..CRUISE
    };
    let run = |env: &mut EpisodeEnv| -> Vec<EpisodeStep> {
        (0..30).map(|_| env.step(wild).unwrap()).collect()
    };
    let first = run(&mut env);
    env.restore(&snapshot).unwrap();
    assert_eq!(run(&mut env), first);
    assert!(first.last().unwrap().truncated);
}

#[test]
fn a_batch_steps_every_environment_in_lock_step() {
    let mut batch = BatchScenarioRunner::new(EpisodeConfig::default(), 3).unwrap();
    assert_eq!(batch.reset_all(100).unwrap().len(), 3);
    let steps = batch.step_all(&[CRUISE; 3]).unwrap();
    assert!(steps.iter().all(|step| step.observation.step == 1));
    assert!(batch.step_all(&[CRUISE; 2]).is_err());
    assert_eq!(batch.envs().len(), 3);
}

#[test]
fn the_apollo_profiles_and_unreset_episodes_are_refused() {
    let lunar = EpisodeConfig {
        profile: ScenarioProfile::Apollo11LunarLanding,
        ..EpisodeConfig::default()
    };
    assert!(EpisodeEnv::new(lunar).is_err());
    let mut env = EpisodeEnv::new(EpisodeConfig::default()).unwrap();
    assert!(env.step(CRUISE).is_err());
    assert!(env.snapshot().is_none());
}
