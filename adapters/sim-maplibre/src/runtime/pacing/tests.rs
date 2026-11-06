use std::time::Duration;

use super::interval_elapsed;

const MIN: Duration = Duration::from_millis(33);

#[test]
fn first_frame_is_always_due() {
    assert!(interval_elapsed(None, Duration::ZERO, MIN));
}

#[test]
fn due_only_after_interval() {
    let prev = Some(Duration::from_millis(100));
    assert!(!interval_elapsed(prev, Duration::from_millis(100), MIN));
    assert!(!interval_elapsed(prev, Duration::from_millis(132), MIN));
    assert!(interval_elapsed(prev, Duration::from_millis(133), MIN));
}

#[test]
fn clock_reset_is_due() {
    assert!(interval_elapsed(
        Some(Duration::from_secs(10)),
        Duration::from_secs(1),
        MIN
    ));
}
