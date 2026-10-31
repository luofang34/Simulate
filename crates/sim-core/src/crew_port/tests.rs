use super::*;
use crate::TickSpan;

fn at(ticks: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(ticks))
}

fn key(key_code: u8) -> CrewAction {
    CrewAction::KeypadPress {
        unit_id: 1,
        key_code,
    }
}

fn lamp(lit: bool) -> CrewObservation {
    CrewObservation::AnnunciatorLamp { lamp_id: 7, lit }
}

fn sight(visible_count: u8, shaft_rad: f64) -> CrewObservation {
    CrewObservation::OpticalSightView {
        sight_id: 0,
        shaft_rad,
        trunnion_rad: 0.1,
        visible_star_octals: [0o25, 0o31, 0, 0, 0, 0, 0, 0],
        visible_count,
        visible_star_offsets_rad: [[0.0; 2]; 8],
        target_centered: true,
    }
}

#[test]
fn actions_arrive_in_stamp_order_then_submission_order() {
    let mut port = CrewStationPort::new();
    port.submit(at(20), key(3)).unwrap();
    port.submit(at(10), key(1)).unwrap();
    port.submit(at(10), key(2)).unwrap();

    let due = port.take_due(at(20));
    let keys: Vec<_> = due.iter().map(|stamped| stamped.action).collect();
    assert_eq!(keys, [key(1), key(2), key(3)]);
    assert_eq!(due[0].at, at(10));
    assert_eq!(due[2].at, at(20));
    assert_eq!(port.queued_len(), 0);
}

#[test]
fn an_action_is_not_delivered_a_tick_early() {
    let mut port = CrewStationPort::new();
    port.submit(at(100), key(1)).unwrap();
    assert!(port.take_due(at(99)).is_empty());
    assert_eq!(port.queued_len(), 1);
    assert_eq!(port.take_due(at(100)).len(), 1);
}

#[test]
fn an_action_for_a_delivered_instant_is_refused() {
    let mut port = CrewStationPort::new();
    assert!(port.take_due(at(50)).is_empty());
    assert_eq!(
        port.submit(at(50), key(1)),
        Err(CrewPortError::StaleAction {
            at: at(50),
            delivered_through: at(50),
        })
    );
    port.submit(at(51), key(1)).unwrap();
}

#[test]
fn every_action_kind_travels_the_same_queue() {
    let actions = [
        CrewAction::SetSwitch {
            panel_id: 2,
            switch_id: 14,
            state: -1,
        },
        key(0o31),
        CrewAction::OpticalSlew {
            sight_id: 0,
            shaft_rate_rad_s: 0.01,
            trunnion_rate_rad_s: -0.02,
        },
        CrewAction::OpticalMark { sight_id: 0 },
        CrewAction::ControlStick {
            roll: 0.1,
            pitch: -0.2,
            yaw: 0.0,
            throttle: 1.0,
        },
    ];
    let mut port = CrewStationPort::new();
    for action in actions {
        port.submit(at(1), action).unwrap();
    }
    let delivered: Vec<_> = port.take_due(at(1)).iter().map(|s| s.action).collect();
    assert_eq!(delivered, actions);
}

#[test]
fn non_finite_actions_are_refused() {
    let mut port = CrewStationPort::new();
    let stick = CrewAction::ControlStick {
        roll: f64::NAN,
        pitch: 0.0,
        yaw: 0.0,
        throttle: 0.0,
    };
    let slew = CrewAction::OpticalSlew {
        sight_id: 0,
        shaft_rate_rad_s: f64::INFINITY,
        trunnion_rate_rad_s: 0.0,
    };
    assert!(matches!(
        port.submit(at(1), stick),
        Err(CrewPortError::NonFinite { .. })
    ));
    assert!(matches!(
        port.submit(at(1), slew),
        Err(CrewPortError::NonFinite { .. })
    ));
    assert_eq!(port.queued_len(), 0);
}

#[test]
fn the_snapshot_holds_the_latest_state_of_each_indicator() {
    let mut port = CrewStationPort::new();
    port.publish(at(1), lamp(false)).unwrap();
    port.publish(at(2), sight(2, 0.5)).unwrap();
    port.publish(at(3), lamp(true)).unwrap();
    let register = CrewObservation::DisplayRegister {
        unit_id: 1,
        reg_id: 2,
        characters: *b"+12345  ",
    };
    port.publish(at(4), register).unwrap();

    let latest = port.latest_observations();
    assert_eq!(latest.len(), 3);
    assert!(latest.contains(&StampedCrewObservation {
        at: at(3),
        observation: lamp(true),
    }));
    assert!(latest.iter().any(|s| s.observation == register));
}

#[test]
fn an_older_observation_cannot_replace_a_newer_one() {
    let mut port = CrewStationPort::new();
    port.publish(at(5), lamp(true)).unwrap();
    assert_eq!(
        port.publish(at(4), lamp(false)),
        Err(CrewPortError::StaleObservation {
            at: at(4),
            held: at(5),
        })
    );
    port.publish(at(5), lamp(false)).unwrap();
}

#[test]
fn a_sight_report_must_fit_its_slots() {
    let mut port = CrewStationPort::new();
    assert_eq!(
        port.publish(at(1), sight(9, 0.0)),
        Err(CrewPortError::TooManyStars { visible_count: 9 })
    );
    assert!(matches!(
        port.publish(at(1), sight(2, f64::NAN)),
        Err(CrewPortError::NonFinite { .. })
    ));
    assert!(port.latest_observations().is_empty());
    let mut stray_offset = sight(1, 0.1);
    if let CrewObservation::OpticalSightView {
        visible_star_offsets_rad,
        ..
    } = &mut stray_offset
    {
        *visible_star_offsets_rad = [[f64::NAN; 2]; 8];
    }
    assert!(matches!(
        port.publish(SimInstant::START, stray_offset),
        Err(CrewPortError::NonFinite { .. })
    ));
}

#[test]
fn a_cloned_port_replays_the_same_order() {
    let mut port = CrewStationPort::new();
    port.submit(at(10), key(1)).unwrap();
    let mut replay = port.clone();
    port.submit(at(10), key(2)).unwrap();
    replay.submit(at(10), key(2)).unwrap();
    assert_eq!(port.take_due(at(10)), replay.take_due(at(10)));
    assert_eq!(port, replay);
}
