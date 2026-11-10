use glam::{DMat4, DVec3};
use sim_core::{CockpitControls, FlapSelector};

use super::{ControlChannel, ControlJoint, control_matrix};

fn joint(channel: ControlChannel, origin: DVec3, axis: DVec3, per_unit: f64) -> ControlJoint {
    ControlJoint {
        channel,
        origin,
        axis,
        per_unit,
        rest: 0.0,
        slides: false,
    }
}

#[test]
fn every_authored_token_names_a_channel() {
    for (token, channel) in [
        ("pitch", ControlChannel::Pitch),
        ("roll", ControlChannel::Roll),
        ("yaw", ControlChannel::Yaw),
        ("throttle_right", ControlChannel::Throttle(1)),
        ("propeller_left", ControlChannel::Propeller(0)),
        ("mixture_right", ControlChannel::Mixture(1)),
        ("trim_elevator", ControlChannel::Trim(1)),
        ("trim_rudder", ControlChannel::Trim(2)),
        ("flap_selector", ControlChannel::FlapSelector),
        ("gear_lever", ControlChannel::GearLever),
    ] {
        assert_eq!(ControlChannel::from_token(token), Some(channel), "{token}");
    }
    for unknown in ["", "throttle_centre", "trim_flap", "gear", "pitch_left"] {
        assert_eq!(ControlChannel::from_token(unknown), None, "{unknown}");
    }
}

#[test]
fn a_joint_at_rest_leaves_the_part_where_it_was_modelled() {
    let joints = [joint(
        ControlChannel::Throttle(0),
        DVec3::new(0.3, 0.7, 0.2),
        DVec3::X,
        1.0,
    )];
    let m = control_matrix(&joints, &CockpitControls::default());
    assert!(m.abs_diff_eq(DMat4::IDENTITY, 1e-12));
}

/// The wheel rolls about its own hub, and that hub rides the column as it pitches
/// about the pivot: the hub moves only with pitch, a rim point with both.
#[test]
fn the_wheel_rolls_about_a_hub_carried_by_the_pitching_column() {
    let pivot = DVec3::new(0.36, 0.86, -0.06);
    let hub = DVec3::new(0.0, 0.92, 0.42);
    let joints = [
        joint(ControlChannel::Pitch, pivot, DVec3::X, 0.2),
        joint(ControlChannel::Roll, hub, DVec3::Y, 1.0),
    ];
    let rolled = CockpitControls {
        roll_norm: 1.0,
        ..CockpitControls::default()
    };
    let m = control_matrix(&joints, &rolled);
    assert!(m.transform_point3(hub).abs_diff_eq(hub, 1e-12));
    let rim = hub + DVec3::new(0.19, 0.0, 0.0);
    let turned = m.transform_point3(rim);
    assert!((turned.distance(hub) - 0.19).abs() < 1e-12);
    assert!(
        (turned.y - hub.y).abs() < 1e-12,
        "roll stays in the wheel's plane"
    );

    let back = CockpitControls {
        pitch_norm: 1.0,
        roll_norm: 1.0,
        ..CockpitControls::default()
    };
    let m = control_matrix(&joints, &back);
    let hub_back = m.transform_point3(hub);
    assert!(
        hub_back.y < hub.y - 0.05,
        "pulling the column brings the hub aft"
    );
    assert!((hub_back.distance(pivot) - hub.distance(pivot)).abs() < 1e-12);
    let rim_back = m.transform_point3(rim);
    assert!((rim_back.distance(hub_back) - 0.19).abs() < 1e-12);
}

#[test]
fn sliding_pedals_and_detented_levers_follow_their_channels() {
    let pedal = ControlJoint {
        slides: true,
        ..joint(ControlChannel::Yaw, DVec3::ZERO, DVec3::Y, -0.08)
    };
    let right = CockpitControls {
        yaw_norm: 1.0,
        ..CockpitControls::default()
    };
    let moved = control_matrix(&[pedal], &right).transform_point3(DVec3::ZERO);
    assert!(moved.abs_diff_eq(DVec3::new(0.0, -0.08, 0.0), 1e-12));

    let lever = ControlJoint {
        rest: 1.0,
        ..joint(ControlChannel::FlapSelector, DVec3::ZERO, DVec3::X, -0.25)
    };
    let tip = DVec3::new(0.0, 0.0, 0.1);
    let at = |flap_selector| {
        let cockpit = CockpitControls {
            flap_selector,
            ..CockpitControls::default()
        };
        control_matrix(&[lever], &cockpit).transform_point3(tip)
    };
    assert!(at(FlapSelector::Closed).abs_diff_eq(tip, 1e-12));
    let (up, maneuver, down) = (
        at(FlapSelector::Up),
        at(FlapSelector::Maneuver),
        at(FlapSelector::Down),
    );
    assert!(up.y < 0.0 && maneuver.y > 0.0 && down.y > maneuver.y);
}
