use std::time::{Duration, Instant};

use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan};

use super::*;

fn at_ms(ms: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(ms * (TICKS_PER_SECOND / 1_000)))
}

#[test]
fn paces_a_tenth_of_a_second_to_wall_clock_without_overruns() {
    let started = Instant::now();
    let mut pacer = RealtimePacer::new();
    for k in 1..=20 {
        pacer.pace_until(at_ms(5 * k), Duration::from_millis(20));
    }
    let elapsed = started.elapsed();
    assert!(elapsed >= Duration::from_millis(100), "{elapsed:?}");
    let stats = pacer.stats();
    assert_eq!(stats.steps, 20);
    assert_eq!(stats.overrun_count, 0);
    assert!(stats.total_sleep_us >= 90_000, "{stats:?}");
}

#[test]
fn late_steps_count_overruns_and_long_stalls_reanchor() {
    let mut pacer = RealtimePacer::new();
    pacer.origin_wall = Instant::now()
        .checked_sub(Duration::from_millis(40))
        .unwrap();
    pacer.pace_until(at_ms(10), Duration::from_millis(2));
    let stats = pacer.stats();
    assert_eq!(stats.overrun_count, 1);
    assert!(stats.max_overrun_us >= 30_000, "{stats:?}");

    pacer.origin_wall = Instant::now().checked_sub(Duration::from_secs(2)).unwrap();
    pacer.pace_until(at_ms(20), Duration::from_millis(2));
    assert_eq!(pacer.stats().overrun_count, 2);
    let started = Instant::now();
    pacer.pace_until(at_ms(30), Duration::from_millis(2));
    assert!(started.elapsed() >= Duration::from_millis(9));
    assert_eq!(pacer.stats().overrun_count, 2);
}

#[test]
fn instants_before_the_anchor_never_wait() {
    let mut pacer = RealtimePacer::anchored_at(at_ms(500));
    assert_eq!(pacer.origin(), at_ms(500));
    let started = Instant::now();
    pacer.pace_until(at_ms(100), Duration::from_millis(2));
    assert!(started.elapsed() < Duration::from_millis(50));
    assert_eq!(sim_offset(at_ms(500), at_ms(100)), Duration::ZERO);
    assert_eq!(
        sim_offset(SimInstant::START, at_ms(1_250)),
        Duration::from_millis(1_250)
    );
}
