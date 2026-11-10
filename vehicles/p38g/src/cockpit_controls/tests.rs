use std::collections::BTreeSet;

use glam::{Mat4, Vec3};

use super::cockpit_control_prims;
use crate::mesh::P38PartRole;

/// Every control the flight model commands has a part following it, and only the
/// moving parts carry joints.
#[test]
fn every_commanded_control_moves() {
    let prims = cockpit_control_prims();
    let channels: BTreeSet<&str> = prims
        .iter()
        .flat_map(|p| p.control_joints.iter().map(|j| j.channel))
        .collect();
    let expected = [
        "flap_selector",
        "gear_lever",
        "mixture_left",
        "mixture_right",
        "pitch",
        "propeller_left",
        "propeller_right",
        "roll",
        "throttle_left",
        "throttle_right",
        "trim_aileron",
        "trim_elevator",
        "trim_rudder",
        "yaw",
    ];
    assert_eq!(channels, expected.into_iter().collect());
    for prim in &prims {
        assert_eq!(
            prim.role == P38PartRole::CockpitControl,
            !prim.control_joints.is_empty(),
            "{}",
            prim.prim_path
        );
    }
}

/// The wheel pitches with the column, and rolls about its own hub inside it.
#[test]
fn the_wheel_rides_the_column_and_rolls_about_its_hub() {
    let prims = cockpit_control_prims();
    let wheel = prims
        .iter()
        .find(|p| p.prim_path == "/P38g/CockpitInterior/ControlWheel")
        .expect("control wheel");
    let channels: Vec<&str> = wheel.control_joints.iter().map(|j| j.channel).collect();
    assert_eq!(channels, ["pitch", "roll"]);
    let hub = Vec3::from_array(wheel.control_joints[1].origin_m);
    let rim_extent = wheel
        .vertices
        .iter()
        .map(|v| Vec3::from_array(v.position).distance(hub))
        .fold(0.0_f32, f32::max);
    assert!(
        (0.17..0.20).contains(&rim_extent),
        "rim reaches {rim_extent} m"
    );
}

/// The front of the pilot's chest ahead of the design eye. Assumed: the contract
/// seats the eye 7 in ahead of the seat's back line, and a chest about 10 in deep
/// with a parachute harness then stands some 3 in ahead of the eye.
const CHEST_AHEAD_OF_EYE_M: f32 = 0.075;

/// At full forward column the rim stays clear of the panel board, and at full back
/// it stays ahead of the pilot's chest.
#[test]
fn the_wheel_clears_the_panel_and_the_pilot_at_full_travel() {
    let prims = cockpit_control_prims();
    let contract = crate::cockpit_reservation::p38g_cockpit_contract();
    let panel_y = contract.main_instrument_panel.center_m[1];
    let eye_y = contract.design_eyepoint_m[1];
    for prim in prims.iter().filter(|p| {
        p.prim_path
            .starts_with("/P38g/CockpitInterior/ControlWheel")
    }) {
        let pitch = prim.control_joints[0];
        let origin = Vec3::from_array(pitch.origin_m);
        for value in [-1.0_f32, 1.0] {
            let m = Mat4::from_translation(origin)
                * Mat4::from_axis_angle(Vec3::from_array(pitch.axis), value * pitch.per_unit)
                * Mat4::from_translation(-origin);
            for v in &prim.vertices {
                let p = m.transform_point3(Vec3::from_array(v.position));
                assert!(
                    p.y < panel_y - 0.15,
                    "{} reaches y {} at {value}",
                    prim.prim_path,
                    p.y
                );
                assert!(
                    p.y > eye_y + CHEST_AHEAD_OF_EYE_M,
                    "{} comes back to y {} at {value}",
                    prim.prim_path,
                    p.y
                );
            }
        }
    }
}
