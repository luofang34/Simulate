use super::*;

#[test]
fn cutting_the_fuel_stops_the_piston_engine() {
    let mut plain = c172(0.0);
    let mut cut = c172(0.0);
    cut.set_system_controls(SystemControls {
        mixture_norm: 0.0,
        fuel_cutoff: true,
        ..SystemControls::running_defaults()
    });
    run(&mut plain, 120, cruise());
    run(&mut cut, 120, cruise());
    assert!(plain.engine_status().running);
    assert!(!cut.engine_status().running);
    assert_ne!(bits(&plain.output()), bits(&cut.output()));
}

#[test]
fn the_c172_trims_to_level_flight() {
    let mut block = c172(0.0);
    let trimmed = block.trim_level_flight().unwrap();
    assert!((0.0..=1.0).contains(&trimmed.throttle), "{trimmed:?}");
    assert!(trimmed.pitch.abs() <= 1.0, "{trimmed:?}");
    run(&mut block, 120, trimmed);
    let rate = block.output().angular_rate_body;
    assert!(rate.length() < 0.05, "{rate}");
}

#[test]
fn an_untrimmable_condition_is_reported_and_leaves_a_finite_state() {
    let files = c172_resolver();
    let mut block = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_AIRCRAFT_PATH,
        model: JSBSIM_C172_MODEL,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: start(),
        heading_rad: 0.0,
        speed_m_s: 12.0,
        alpha_rad: CRUISE_ALPHA_RAD,
        engine_start: EngineStartMode::Running,
    })
    .unwrap();
    assert!(matches!(
        block.trim_level_flight(),
        Err(BlockError::Fdm { .. })
    ));
    assert!(block.output().position_ecef.is_finite());
}
