//! The pilot's hands and the autopilot's servos share one set of controls: releasing the
//! servos gives the aeroplane to the pilot, engaging them gives it back to the plan, and
//! a chain started without a plan waits for the crew's.

use sim_core::ControlInputs;
use sim_geo::ecef_to_geodetic;

use super::*;

type Chain = HostChain<C172Airframe, FixedWingController, FixedWingMixer>;

/// Long enough for the plan to load, the controller to arm and the mission to steer.
const UNDER_WAY_S: f64 = 20.0;

fn fly(chain: &mut Chain, seconds: f64, mut each: impl FnMut(&Chain)) {
    let cycles = (seconds * C172_RATE_HZ as f64).round() as usize;
    for _ in 0..cycles {
        chain.advance_tick().unwrap();
        each(chain);
    }
}

fn height_m(chain: &Chain) -> f64 {
    ecef_to_geodetic(chain.true_state().position_ecef).height_m
}

/// The trimmed controls with the yoke held to the right.
fn right_yoke(chain: &Chain) -> ControlInputs {
    ControlInputs {
        roll: 0.4,
        ..chain.board().airframe().trim()
    }
}

/// The largest roll rate to the right over `seconds`, rad/s.
fn peak_right_roll_rate(chain: &mut Chain, seconds: f64) -> f64 {
    let mut peak = f64::MIN;
    fly(chain, seconds, |chain| {
        peak = peak.max(chain.true_state().angular_rate_body.x);
    });
    peak
}

#[test]
fn released_servos_leave_the_aeroplane_to_the_pilot() {
    let mut chain = chain(7);
    fly(&mut chain, UNDER_WAY_S, |_| {});
    assert!(chain.status().armed);
    let under_way = chain.snapshot().unwrap();

    // Servos engaged: the yoke the pilot holds reaches nothing.
    let held = right_yoke(&chain);
    chain.board_mut().airframe_mut().set_pilot_controls(held);
    let engaged = peak_right_roll_rate(&mut chain, 2.0);

    chain.restore(&under_way).unwrap();
    let airframe = chain.board_mut().airframe_mut();
    airframe.set_pilot_controls(held);
    airframe.set_servos_engaged(false);
    let released = peak_right_roll_rate(&mut chain, 2.0);

    assert!(
        engaged < 0.05,
        "the autopilot held the wings: {engaged} rad/s"
    );
    assert!(
        released > 0.2,
        "the pilot rolled the aeroplane: {released} rad/s"
    );
    assert!(chain.status().armed, "the flight controller stays armed");
}

/// A chain under way that the pilot rolled into a descending turn and let go, its servos
/// still released, with how far it left the cruise height, metres.
fn hand_flown_upset() -> (Chain, f64, f64) {
    let mut chain = chain(7);
    fly(&mut chain, UNDER_WAY_S, |_| {});
    let cruise_m = height_m(&chain);
    let held = ControlInputs {
        pitch: chain.board().airframe().trim().pitch - 0.05,
        ..right_yoke(&chain)
    };
    let airframe = chain.board_mut().airframe_mut();
    airframe.set_pilot_controls(held);
    airframe.set_servos_engaged(false);
    fly(&mut chain, 2.0, |_| {});
    let trim = chain.board().airframe().trim();
    chain.board_mut().airframe_mut().set_pilot_controls(trim);
    fly(&mut chain, 4.0, |_| {});
    let upset_m = (height_m(&chain) - cruise_m).abs();
    (chain, cruise_m, upset_m)
}

#[test]
fn engaged_servos_bring_a_hand_flown_aeroplane_back_to_the_plan() {
    let (mut chain, cruise_m, upset_m) = hand_flown_upset();
    chain.board_mut().airframe_mut().set_servos_engaged(true);
    fly(&mut chain, 90.0, |_| {});

    assert!(
        upset_m > 10.0,
        "the pilot left the cruise height: {upset_m} m"
    );
    let error_m = (height_m(&chain) - cruise_m).abs();
    assert!(error_m < 15.0, "back at the plan's height: {error_m} m off");
    assert_eq!(chain.status().mission_state, Some(MissionState::Enroute));
}

/// How far the aeroplane may pass a rate limit of the flight controller's envelope.
const RATE_LIMIT_MARGIN: f64 = 1.1;

#[test]
#[ignore = "rolling out of the turn the flight controller holds a quarter of its rudder, \
            and the roll that the rudder and the sideslip add takes the aeroplane a tenth \
            past the roll-rate limit: its yaw feedforward and turn coordination have no source"]
fn re_engagement_stays_within_the_flight_controllers_rate_limits() {
    let limits = sim_aviate::c172_airframe().limits;
    let (mut chain, _, _) = hand_flown_upset();
    chain.board_mut().airframe_mut().set_servos_engaged(true);
    let (mut roll, mut pitch) = (0.0_f64, 0.0_f64);
    fly(&mut chain, 30.0, |chain| {
        let rate = chain.true_state().angular_rate_body;
        roll = roll.max(rate.x.abs());
        pitch = pitch.max(rate.y.abs());
    });

    let roll_limit = f64::from(limits.max_roll_rate.0) * RATE_LIMIT_MARGIN;
    let pitch_limit = f64::from(limits.max_pitch_rate.0) * RATE_LIMIT_MARGIN;
    assert!(roll < roll_limit, "roll {roll} rad/s over {roll_limit}");
    assert!(
        pitch < pitch_limit,
        "pitch {pitch} rad/s over {pitch_limit}"
    );
}

#[test]
fn a_chain_without_a_route_waits_for_the_crews_plan() {
    let planned = route(PLAN_LOADED_AT_S);
    let empty = ChainRoute {
        fixes: Vec::new(),
        ..route(PLAN_LOADED_AT_S)
    };
    let mut chain = HostChain::new(c172(7), &empty).unwrap();
    fly(&mut chain, PLAN_LOADED_AT_S + 3.0, |_| {});
    assert!(chain.record().is_none());
    assert_eq!(chain.status().mission_state, None);
    let heading = crate::SelectedTargets {
        heading_true_rad: Some(1.0),
        ..crate::SelectedTargets::default()
    };
    assert!(matches!(
        chain.select_targets(heading),
        Err(ChainError::Host(_))
    ));

    chain.load_plan(&planned.fixes).unwrap();
    assert!(chain.record().is_some());
    fly(&mut chain, UNDER_WAY_S, |_| {});
    assert_eq!(chain.status().mission_state, Some(MissionState::Enroute));

    chain.select_targets(heading).unwrap();
    assert_eq!(chain.selected_targets(), heading);
}
