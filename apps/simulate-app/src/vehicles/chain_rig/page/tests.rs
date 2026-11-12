use sim_chain_trial::{ChainTrial, first_link};
use sim_core::{SimInstant, VehicleState};
use sim_navigate::trial::DirectoryStores;

use super::PageSnapshot;
use super::display_clock::DisplayClock;
use super::ring::SnapshotRing;

/// The page's flight at `route_s`: the truth `east_m` east of the origin, the flight
/// controller's estimate stamped `fc_s` and `fc_east_m` east.
fn snapshot(base: &PageSnapshot, route_s: f64, east_m: f64, fc: (f64, f64)) -> PageSnapshot {
    let at = |s: f64| SimInstant::from_nanos((s * 1e9).round() as u64);
    let mut snapshot = base.clone();
    snapshot.truth.at = at(route_s);
    snapshot.truth.state.position_ecef = base.truth.state.position_ecef + glam::DVec3::X * east_m;
    let (fc_s, fc_east_m) = fc;
    snapshot.avionics.at = at(fc_s);
    let estimate = VehicleState {
        position_ecef: base.truth.state.position_ecef + glam::DVec3::X * fc_east_m,
        ..base.truth.state
    };
    snapshot.avionics.estimate = Some(estimate);
    snapshot
}

fn first() -> PageSnapshot {
    let trial = ChainTrial::new(&first_link().unwrap(), &DirectoryStores).unwrap();
    PageSnapshot::of(&trial, None)
}

#[test]
fn the_truth_between_two_instants_is_interpolated_and_the_controller_held() {
    let base = first();
    let mut ring = SnapshotRing::new(snapshot(&base, 1.0, 0.0, (1.0, 0.0)));
    ring.push(snapshot(&base, 1.1, 10.0, (1.1, 9.0)));
    let x0 = base.truth.state.position_ecef.x;
    let mid = ring.at(1.075);
    assert!((mid.truth.state.position_ecef.x - x0 - 7.5).abs() < 1e-6);
    // The displays see the controller's sample of the earlier instant, never the truth
    // and never the later sample.
    let held = mid.avionics.estimate.map(|e| e.position_ecef.x - x0);
    assert_eq!(held, Some(0.0));
    assert!((ring.at(5.0).truth.state.position_ecef.x - x0 - 10.0).abs() < 1e-6);
    assert!((ring.at(0.0).truth.state.position_ecef.x - x0).abs() < 1e-6);
}

/// The display time after each of `frames` of `dt_s` wall seconds, the page publishing
/// `published(frame)` as its newest route time.
fn run(
    clock: &mut DisplayClock,
    frames: std::ops::Range<u32>,
    dt_s: f64,
    published: impl Fn(u32) -> f64,
) -> Vec<f64> {
    frames
        .map(|frame| clock.advance(dt_s, published(frame), 0.0))
        .collect()
}

#[test]
fn the_display_runs_smoothly_through_uneven_slices() {
    let mut clock = DisplayClock::default();
    // The page reaches 4 ms steps at 1×, a whole frame's worth some frames and none or
    // two the next.
    let uneven = |frame: u32| {
        let whole = f64::from(frame) / 60.0;
        let jitter = [0.0, -0.012, 0.0, 0.004][(frame % 4) as usize];
        (whole + jitter).max(0.0)
    };
    let shown = run(&mut clock, 0..240, 1.0 / 60.0, uneven);
    let steps: Vec<f64> = shown.windows(2).map(|w| w[1] - w[0]).skip(120).collect();
    let (least, most) = steps
        .iter()
        .fold((f64::MAX, f64::MIN), |(a, b), s| (a.min(*s), b.max(*s)));
    assert!(
        least > 0.012 && most < 0.021,
        "steps between {least} and {most}"
    );
    let lag = uneven(239) - shown[239];
    assert!(lag > 0.0 && lag < 0.06, "{lag}");
}

#[test]
fn after_a_300_ms_gap_the_display_catches_up_at_a_bounded_rate() {
    let mut clock = DisplayClock::default();
    let pace = |frame: u32| f64::from(frame) / 60.0;
    let before = run(&mut clock, 0..120, 1.0 / 60.0, pace);
    let last = before[119];
    // A 300 ms step: the page's pacer adds at most 100 ms of route time for it.
    let jumped = pace(119) + 0.1;
    let after_gap = clock.advance(0.3, jumped, 0.0);
    let first_step = after_gap - last;
    assert!(
        first_step <= 1.5 / 30.0 + 1e-9,
        "one frame's worth at most 1.5× the pace: {first_step}"
    );
    let mut shown = after_gap;
    let mut frames = 0;
    let mut newest = jumped;
    while newest - shown > 0.05 && frames < 120 {
        newest += 1.0 / 60.0;
        let next = clock.advance(1.0 / 60.0, newest, 0.0);
        assert!(
            next - shown <= 1.5 / 60.0 + 1e-9,
            "catch-up step {}",
            next - shown
        );
        shown = next;
        frames += 1;
    }
    assert!(frames < 120, "the lag closes within two seconds");
}
