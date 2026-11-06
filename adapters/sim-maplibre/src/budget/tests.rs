use std::time::Duration;

use super::CockpitDeviceTier;

#[test]
fn desktop_drape_budget_keeps_taxi_frames_short() {
    let budget = CockpitDeviceTier::Desktop.otw_drape_budget();
    assert_eq!(budget.per_frame, 4);
    assert_eq!(budget.per_eye_frame, 3);
    assert_eq!(budget.moving_time, Duration::from_micros(2_000));
}

#[test]
fn conformal_interval_admits_every_second_60hz_frame() {
    let interval = CockpitDeviceTier::Desktop.conformal_min_interval();
    let two_frames = Duration::from_secs_f64(2.0 / 60.0);
    let one_frame = Duration::from_secs_f64(1.0 / 60.0);
    assert!(two_frames >= interval);
    assert!(one_frame < interval);
}
