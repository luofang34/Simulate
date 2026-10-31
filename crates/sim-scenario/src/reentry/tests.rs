use super::*;

fn fly() -> CapsuleReentryRun {
    let mut run = CapsuleReentryRun::new().unwrap();
    for _ in 0..1_200 * STEP_HZ {
        if run.finished() {
            return run;
        }
        run.step().unwrap();
    }
    panic!("never slowed down: {:?}", run.report());
}

#[test]
fn the_offset_cg_trims_the_capsule_at_its_analytic_incidence_with_lift() {
    let report = fly().report();
    let trim_error = report.mean_trim_incidence_rad - report.analytic_trim_incidence_rad;
    assert!(trim_error.abs() < 1_f64.to_radians(), "{report:?}");
    assert!(
        report.mean_trim_incidence_rad.abs() > 15_f64.to_radians(),
        "{report:?}"
    );
    assert!((0.25..0.4).contains(&report.lift_to_drag), "{report:?}");
}

#[test]
fn steering_by_bank_alone_holds_the_load_to_a_crewed_level_without_skipping_out() {
    let run = fly();
    let report = run.report();
    assert!((3.5..7.0).contains(&report.peak_g_load), "{report:?}");
    assert!(!report.skipped_out, "{report:?}");
    assert!(report.height_m < 30_000.0, "{report:?}");
    assert!(run.vehicle().velocity_ecef.length() < END_SPEED_M_S + 1.0);
}

#[test]
fn the_bank_reversal_rolls_the_lift_to_the_other_side() {
    let report = fly().report();
    let (low, high) = report.bank_range_rad;
    assert!(report.bank_reversed, "{report:?}");
    assert!(high > 60_f64.to_radians(), "{report:?}");
    assert!(low < -60_f64.to_radians(), "{report:?}");
}

#[test]
fn the_entry_monitor_sees_the_entry_and_the_panel_shows_it() {
    let mut run = CapsuleReentryRun::new().unwrap();
    assert_eq!(run.cockpit().ems_g_load, Some(0.0));
    let mut peak_on_panel: f64 = 0.0;
    while !run.finished() {
        run.step().unwrap();
        peak_on_panel = peak_on_panel.max(run.cockpit().ems_g_load.unwrap());
    }
    let report = run.report();
    let threshold = report.threshold_height_m.unwrap();
    assert!((75_000.0..100_000.0).contains(&threshold), "{report:?}");
    assert_ne!(report.corridor, CorridorIndication::Pending);
    assert_eq!(peak_on_panel, report.peak_g_load);
    assert!(
        report.delta_v_counter_m_s < EMS_INITIAL_DELTA_V_M_S - 10_000.0,
        "{report:?}"
    );

    let cockpit = run.cockpit();
    assert_eq!(cockpit.attitude.validity, sim_core::SourceValidity::Valid);
    assert!(!cockpit.gimbal_lock_captured, "{cockpit:?}");
}

#[test]
fn the_view_at_entry_interface_has_the_earth_in_it() {
    let run = CapsuleReentryRun::new().unwrap();
    let pose = run.eye_pose();
    let down = -run.vehicle().position_ecef.normalize();
    let look = pose.orientation * DVec3::NEG_Z;
    assert!(look.dot(down) > 0.5, "{look} against {down}");
}

#[test]
fn two_entries_fly_the_same_bits() {
    let mut one = CapsuleReentryRun::new().unwrap();
    let mut two = CapsuleReentryRun::new().unwrap();
    for _ in 0..5_000 {
        one.step().unwrap();
        two.step().unwrap();
    }
    let bits = |run: &CapsuleReentryRun| run.vehicle().position_ecef.to_array().map(f64::to_bits);
    assert_eq!(bits(&one), bits(&two));
    assert_eq!(one.report(), two.report());
}
