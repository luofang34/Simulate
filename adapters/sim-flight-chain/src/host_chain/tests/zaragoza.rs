//! The first shareable flight through Pilotage's own host core: the C172
//! around the closed route near Zaragoza on the flight controller's
//! estimate. The run's numbers go to the log; the bounds say the route was
//! flown, not how well the turns are tuned.

use glam::DVec3;
use sim_geo::{LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::*;

/// What the aeroplane was doing at one instant.
#[derive(Debug, Clone, Copy)]
struct Sample {
    t_s: f64,
    state: Option<MissionState>,
    north_m: f64,
    east_m: f64,
    height_m: f64,
    roll_deg: f64,
    attitude_error_deg: f64,
    position_error_m: f64,
}

fn north_east(position_ecef: DVec3) -> (f64, f64) {
    let offset =
        LocalFrame::at(&start()).ecef_to_ned_vector(position_ecef - geodetic_to_ecef(&start()));
    (offset.x, offset.y)
}

fn fix_north_east(index: usize) -> (f64, f64) {
    let (lat, lon) = FIXES[index];
    north_east(geodetic_to_ecef(&Geodetic::from_degrees(
        lat,
        lon,
        ellipsoidal_m(lat, lon),
    )))
}

fn sample(chain: &HostChain<C172Airframe, FixedWingController, FixedWingMixer>) -> Sample {
    let truth = chain.true_state();
    let position = ecef_to_geodetic(truth.position_ecef);
    let frame = LocalFrame::at(&position);
    let right = frame.ecef_to_ned_vector(truth.attitude * DVec3::Y);
    let down = frame.ecef_to_ned_vector(truth.attitude * DVec3::Z);
    let (north_m, east_m) = north_east(truth.position_ecef);
    let (attitude_error_deg, position_error_m) =
        chain
            .board()
            .estimated_state()
            .map_or((f64::NAN, f64::NAN), |estimate| {
                let angle = (estimate.attitude.inverse() * truth.attitude)
                    .to_axis_angle()
                    .1;
                (
                    angle.min(std::f64::consts::TAU - angle).to_degrees(),
                    (estimate.position_ecef - truth.position_ecef).length(),
                )
            });
    Sample {
        t_s: seconds(chain.now()),
        state: chain.status().mission_state,
        north_m,
        east_m,
        height_m: position.height_m - ellipsoidal_m(START.0, START.1),
        roll_deg: right.z.atan2(down.z).to_degrees(),
        attitude_error_deg,
        position_error_m,
    }
}

/// Signed distance right of the leg from fix `to - 1` to fix `to`, metres.
fn right_of_leg_m(sample: &Sample, to: usize) -> f64 {
    let (a, b) = (fix_north_east(to - 1), fix_north_east(to));
    let (dn, de) = (b.0 - a.0, b.1 - a.1);
    ((sample.east_m - a.1) * dn - (sample.north_m - a.0) * de) / dn.hypot(de)
}

fn span(values: impl Iterator<Item = f64>) -> (f64, f64) {
    values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
        (lo.min(v), hi.max(v))
    })
}

/// Flies the route and `orbit_s` of what follows; a sample every tenth of
/// a second and every mission event with its time.
fn fly_route(
    chain: &mut HostChain<C172Airframe, FixedWingController, FixedWingMixer>,
    orbit_s: f64,
) -> (Vec<Sample>, Vec<(f64, MissionEvent)>) {
    let (mut samples, mut events) = (Vec::new(), Vec::new());
    let mut complete_at = None::<f64>;
    let every = C172_RATE_HZ / 10;
    for cycle in 0..1_500 * C172_RATE_HZ {
        chain.advance_tick().unwrap();
        for (at, event) in chain.drain_events() {
            if let HostEvent::Mission(event) = event {
                if matches!(event, MissionEvent::MissionComplete) {
                    complete_at = Some(seconds(at));
                }
                events.push((seconds(at), event));
            }
        }
        if cycle.is_multiple_of(every) {
            samples.push(sample(chain));
        }
        if complete_at.is_some_and(|at| seconds(chain.now()) - at >= orbit_s)
            || chain.status().mission_state == Some(MissionState::Failed)
        {
            break;
        }
    }
    (samples, events)
}

fn leg_start(events: &[(f64, MissionEvent)], to: usize) -> Option<f64> {
    events.iter().find_map(|(t, event)| {
        matches!(event, MissionEvent::LegAdvanced { to_index, .. } if *to_index == to).then_some(*t)
    })
}

/// Each turn's overshoot past the new leg and the worst cross-track once
/// settled, a minute after the turn began.
fn turns(
    samples: &[Sample],
    events: &[(f64, MissionEvent)],
    done_s: f64,
) -> Vec<(usize, f64, f64)> {
    (2..FIXES.len())
        .filter_map(|to| {
            let start = leg_start(events, to)?;
            let end = leg_start(events, to + 1).unwrap_or(done_s);
            let on_leg = || {
                samples
                    .iter()
                    .filter(move |s| s.t_s >= start && s.t_s < end)
            };
            let (_, overshoot) = span(on_leg().map(|s| right_of_leg_m(s, to)));
            let settled = on_leg()
                .filter(|s| s.t_s >= start + 60.0)
                .map(|s| right_of_leg_m(s, to).abs())
                .fold(0.0, f64::max);
            Some((to, overshoot, settled))
        })
        .collect()
}

/// About eighteen simulated minutes.
#[test]
#[ignore = "the long acceptance flight through the host core; run it by name"]
fn the_host_core_flies_the_c172_around_the_zaragoza_route() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let mut chain = chain(1);
    let (samples, events) = fly_route(&mut chain, 180.0);
    let done_s = events
        .iter()
        .find_map(|(t, e)| matches!(e, MissionEvent::MissionComplete).then_some(*t));
    let enroute = || {
        samples.iter().filter(|s| {
            matches!(
                s.state,
                Some(MissionState::Enroute | MissionState::Complete)
            )
        })
    };
    let heights = span(enroute().map(|s| s.height_m));
    let bank = span(enroute().map(|s| s.roll_deg));
    let attitude = span(
        samples
            .iter()
            .map(|s| s.attitude_error_deg)
            .filter(|v| !v.is_nan()),
    );
    let position = span(
        samples
            .iter()
            .map(|s| s.position_error_m)
            .filter(|v| !v.is_nan()),
    );
    let turns = turns(&samples, &events, done_s.unwrap_or(f64::INFINITY));
    let last = fix_north_east(FIXES.len() - 1);
    let end_s = samples.last().map_or(0.0, |s| s.t_s);
    let orbit = span(
        samples
            .iter()
            .filter(|s| done_s.is_some() && s.t_s >= end_s - 80.0)
            .map(|s| (s.north_m - last.0).hypot(s.east_m - last.1)),
    );
    let legs: Vec<_> = (1..FIXES.len())
        .map(|to| (to, leg_start(&events, to)))
        .collect();
    tracing::info!(
        ?done_s,
        ?legs,
        ?heights,
        ?bank,
        ?attitude,
        ?position,
        ?turns,
        ?orbit,
        state = ?chain.status().mission_state,
        "zaragoza through the host core"
    );
    assert!(done_s.is_some(), "the plan was not completed");
    assert!(
        legs.iter().all(|(_, t)| t.is_some()),
        "a fix was never flown to"
    );
    assert!(heights.0 > -30.0 && heights.1 < 30.0, "height {heights:?}");
    assert!(bank.0 > -35.0 && bank.1 < 35.0, "bank {bank:?}");
    assert!(attitude.1 < 15.0 && position.1 < 15.0);
    assert!(
        turns.iter().all(|(_, _, settled)| *settled < 30.0),
        "{turns:?}"
    );
}
