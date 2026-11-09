use std::hint::black_box;

use super::*;
use crate::NavMeans;

const VISUAL: NavigationOrigin = NavigationOrigin::Standalone(NavMeans::Visual);

#[test]
fn a_metered_call_returns_its_result_and_keeps_one_sample() {
    let mut meter = ComputeMeter::default();
    let sum = meter.measure(VISUAL, || (1..=1_000_u64).sum::<u64>());
    assert_eq!(sum, 500_500);
    assert_eq!(meter.samples(VISUAL).len(), 1);
    assert!(
        meter
            .samples(NavigationOrigin::Standalone(NavMeans::Gnss))
            .is_empty()
    );
}

#[test]
fn the_host_reports_wall_time_and_resident_memory() {
    let mut meter = ComputeMeter::default();
    for size in [1_usize << 20, 1 << 24] {
        meter.measure(VISUAL, || {
            let touched = vec![1_u8; size];
            black_box(touched.iter().map(|v| u64::from(*v)).sum::<u64>())
        });
    }
    let samples = meter.samples(VISUAL);
    for sample in samples {
        assert!(sample.wall.is_some(), "{sample:?}");
        let before = sample.rss_before_bytes.unwrap();
        assert!(before > 0, "{sample:?}");
        assert!(sample.rss_sampled_max_bytes().unwrap() >= before);
        assert!(sample.rss_delta_bytes().is_some());
    }
    let stats = meter.stats(VISUAL);
    assert_eq!(stats.calls, 2);
    let wall = stats.wall_s.unwrap();
    assert!(wall.p50 <= wall.max && wall.max > 0.0, "{stats:?}");
    assert!(stats.rss_sampled_max_bytes.unwrap() > 0);
}

#[test]
fn a_reading_the_platform_lacks_stays_absent() {
    let sample = ComputeSample {
        wall: None,
        rss_before_bytes: None,
        rss_after_bytes: Some(10),
    };
    assert_eq!(sample.rss_delta_bytes(), None);
    assert_eq!(sample.rss_sampled_max_bytes(), Some(10));
    let stats = ComputeMeter::default().stats(VISUAL);
    assert_eq!(stats, ComputeStats::default());
}
