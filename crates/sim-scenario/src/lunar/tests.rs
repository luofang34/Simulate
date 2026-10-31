use std::f64::consts::FRAC_PI_2;

use sim_core::SourceValidity;

use super::*;

fn land(config: &LunarLandingConfig) -> LunarLandingRun {
    let mut run = LunarLandingRun::new(config).unwrap();
    for _ in 0..240 * STEP_HZ {
        if run.finished() {
            return run;
        }
        run.step().unwrap();
    }
    panic!("never reached the surface: {:?}", run.report());
}

#[test]
fn eagle_flies_past_the_boulders_and_lands_on_a_clear_pad() {
    let run = land(&LunarLandingConfig::default());
    let report = run.report();

    assert!(run.cockpit().contact_light, "{report:?}");
    let sink = report.contact_sink_m_s.unwrap();
    assert!((0.5..1.6).contains(&sink), "{report:?}");
    assert!(report.contact_drift_m_s.unwrap() < 0.6, "{report:?}");
    assert!(report.contact_miss_m.unwrap() < 8.0, "{report:?}");

    assert!(report.redesignated, "{report:?}");
    assert!(
        report.first_target_roughness_m > MAX_PAD_ROUGHNESS_M,
        "{report:?}"
    );
    assert!(
        report.target_roughness_m <= MAX_PAD_ROUGHNESS_M,
        "{report:?}"
    );
    let (east, _) = run.target_local_m();
    assert!(east < TARGET_EAST_M, "beyond the boulders: {east}");
    assert!(report.lpd_clear_target_deg.unwrap() < report.lpd_first_target_deg.unwrap());
}

#[test]
fn the_descent_burns_propellant_and_raises_the_cg() {
    let run = land(&LunarLandingConfig::default());
    let report = run.report();
    assert!(report.dps_burned_kg > 400.0, "{report:?}");
    assert!(report.dps_burned_kg < START_DPS_PROPELLANT_KG, "{report:?}");
    assert!(report.cg_rise_m > 0.05, "{report:?}");
}

#[test]
fn the_radar_holds_lock_over_the_tranquility_terrain() {
    let mut run = LunarLandingRun::new(&LunarLandingConfig::default()).unwrap();
    let mut read_valid = false;
    while !run.finished() {
        run.step().unwrap();
        let cockpit = run.cockpit();
        if cockpit.radar_slant_range_m.validity == SourceValidity::Valid {
            read_valid = true;
            let height = cockpit.height_m.value;
            let slant = cockpit.radar_slant_range_m.value;
            // The antenna rides a few metres below the centre of gravity.
            assert!(
                slant > height - 5.0 && slant < 1.6 * height,
                "{slant} at {height}"
            );
        }
    }
    assert!(read_valid);
    let report = run.report();
    assert!(report.radar_valid_samples > 1_000, "{report:?}");
    assert_eq!(report.radar_invalid_samples, 0, "{report:?}");
}

#[test]
fn a_well_aligned_platform_keeps_pgns_in_the_loop_to_contact() {
    let run = land(&LunarLandingConfig::default());
    let cockpit = run.cockpit();
    assert_eq!(cockpit.guidance_source, Some(GuidanceSource::Pgns));
    assert_eq!(cockpit.attitude.validity, SourceValidity::Valid);
    assert!(!cockpit.gimbal_lock_caution && !cockpit.gimbal_lock_captured);
    assert!(run.report().peak_mga_deg < 30.0, "{:?}", run.report());
}

/// A platform turned so pitching towards the vertical drives the middle
/// gimbal up from `start_mga_deg`.
fn poor_alignment(start_mga_deg: f64) -> LunarLandingConfig {
    LunarLandingConfig {
        navbase_to_platform: DQuat::from_rotation_x(-FRAC_PI_2)
            * DQuat::from_rotation_y(-start_mga_deg.to_radians()),
    }
}

#[test]
fn gimbal_lock_during_the_pitch_over_hands_the_landing_to_ags() {
    let mut run = LunarLandingRun::new(&poor_alignment(74.0)).unwrap();
    let mut handover_height = None;
    while !run.finished() {
        let before = run.cockpit();
        run.step().unwrap();
        let after = run.cockpit();
        if after.gimbal_lock_captured && !before.gimbal_lock_captured {
            assert_eq!(after.guidance_source, Some(GuidanceSource::Ags));
            assert_eq!(after.pgns_attitude.validity, SourceValidity::Failed);
            assert_eq!(after.attitude.validity, SourceValidity::Degraded);
            handover_height = Some(after.height_m.value);
        }
        assert!(after.attitude.validity.is_usable(), "{after:?}");
    }
    assert!(handover_height.is_some(), "{:?}", run.report());
    let report = run.report();
    assert!(run.cockpit().contact_light);
    assert!(
        (0.5..1.6).contains(&report.contact_sink_m_s.unwrap()),
        "{report:?}"
    );
    assert!(report.contact_miss_m.unwrap() < 8.0, "{report:?}");
}

#[test]
fn two_landings_fly_the_same_bits() {
    let [one, two] = [(); 2].map(|()| land(&LunarLandingConfig::default()));
    let bits = |run: &LunarLandingRun| run.vehicle().position_ecef.to_array().map(f64::to_bits);
    assert_eq!(bits(&one), bits(&two));
    assert_eq!(one.now(), two.now());
    assert_eq!(one.report(), two.report());
}
