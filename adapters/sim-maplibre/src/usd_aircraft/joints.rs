//! Articulation joint matrices for OpenUSD aircraft parts (`evaluate_draws_exposed`).

use glam::{DMat4, DQuat, DVec3};
use sim_core::AirframeArticulation;

use super::{PartRole, UsdMeshPart, controls, dials};

pub(super) fn mount_vibration_matrix([rx, ry, rz]: [f64; 3]) -> DMat4 {
    DMat4::from_quat(
        DQuat::from_rotation_y(rx) * DQuat::from_rotation_x(ry) * DQuat::from_rotation_z(-rz),
    )
}

pub(super) fn joint_delta_matrix(part: &UsdMeshPart, art: &AirframeArticulation) -> DMat4 {
    let [comp_nose, comp_left, comp_right] = art.gear_compression_m;
    let [spin_nose, spin_left, spin_right] = art.wheel_phase_rad;
    let retract = (1.0 - art.gear_pos_norm.clamp(0.0, 1.0)).max(0.0);
    match part.role {
        PartRole::PropellerSpinner | PartRole::PropellerBlades => propeller_joint_matrix(part, art),
        PartRole::FlapLeft | PartRole::FlapRight => {
            let rot = DMat4::from_rotation_x(art.flap_deg.to_radians());
            let f = (art.flap_deg / 40.0).clamp(0.0, 1.0);
            if part.prim_path.contains("P38") {
                DMat4::from_translation(DVec3::new(0.0, -0.22 * f, -0.045 * f)) * rot
            } else {
                rot
            }
        }
        PartRole::AileronLeft => DMat4::from_rotation_x(art.left_aileron_rad),
        PartRole::AileronRight => DMat4::from_rotation_x(art.right_aileron_rad),
        PartRole::Elevator => DMat4::from_rotation_x(art.elevator_rad),
        PartRole::RuddervatorLeft => DMat4::from_rotation_x(art.elevator_rad - art.rudder_rad),
        PartRole::RuddervatorRight => DMat4::from_rotation_x(art.elevator_rad + art.rudder_rad),
        PartRole::Rudder => {
            let default_axis = DVec3::new(0.0, -0.261_309, 0.965_255);
            let axis = part
                .custom_hinge_axis
                .unwrap_or(default_axis)
                .normalize_or_zero();
            DMat4::from_axis_angle(axis, -art.rudder_rad)
        }
        PartRole::GearStruts if retract > 1e-4 => {
            DMat4::from_translation(DVec3::new(0.0, -retract * 0.45, retract * 1.15))
        }
        PartRole::LandingLight if retract > 1e-4 => {
            DMat4::from_translation(DVec3::new(0.0, 0.0, retract * 0.08))
                * DMat4::from_rotation_x(-retract * 1.35)
        }
        PartRole::NoseWheel => {
            DMat4::from_translation(DVec3::new(0.0, -retract * 0.72, comp_nose + retract * 1.16))
                * DMat4::from_rotation_x(-retract * 1.35)
                * DMat4::from_rotation_z(-art.nosewheel_steer_rad)
                * DMat4::from_rotation_x(-spin_nose)
        }
        PartRole::WheelLeft | PartRole::WheelRight => {
            let (comp, spin) = if part.role == PartRole::WheelLeft {
                (comp_left, spin_left)
            } else {
                (comp_right, spin_right)
            };
            let t = DVec3::new(0.0, -retract * 0.78, comp + retract * 1.22);
            DMat4::from_translation(t) * DMat4::from_rotation_x(-retract * 1.35 - spin)
        }
        PartRole::InstrumentNeedle | PartRole::InstrumentDrum | PartRole::InstrumentCard => {
            dials::moving_part_matrix(part, &art.instrument_needles)
        }
        PartRole::CockpitControl => controls::control_matrix(&part.control_joints, &art.cockpit),
        // A canopy pane that opens, and the frame it carries: the hatch on its
        // hinge, a window winding down.
        PartRole::CabinGlass | PartRole::CabinExterior if !part.control_joints.is_empty() => {
            controls::control_matrix(&part.control_joints, &art.cockpit)
        }
        _ => articulate_auxiliary_part(part, art),
    }
}

fn propeller_joint_matrix(part: &UsdMeshPart, art: &AirframeArticulation) -> DMat4 {
    let r = part.prim_path.contains("PropRight");
    let s = if r {
        art.propeller_right.sense
    } else {
        art.propeller_sense
    };
    let ph = if r {
        art.propeller_right.phase_rad
    } else {
        art.propeller_phase_rad
    };
    let deg = if r {
        art.propeller_right.blade_angle_deg
    } else {
        art.propeller_blade_angle_deg
    };
    let spin = DMat4::from_rotation_y(-s * ph);
    if part.role == PartRole::PropellerBlades
        && part.custom_hinge_axis.is_some_and(|a| a.x.abs() > 0.9)
    {
        spin * DMat4::from_rotation_x((deg - 24.0).to_radians())
    } else {
        spin
    }
}

fn articulate_auxiliary_part(part: &UsdMeshPart, art: &AirframeArticulation) -> DMat4 {
    let [cl, cr] = art.coolant_flap_norm;
    let [ol, or] = art.oil_shutter_norm;
    let [tr, tp, ty] = art.trim_tab_rad;
    let path = part.prim_path.as_str();
    let sign = part.custom_hinge_axis.map_or(1.0, |a| a.z.signum());
    if path.contains("CoolantFlap") {
        let v = if path.contains("Left") { cl } else { cr };
        DMat4::from_rotation_z(sign * v.clamp(0.0, 1.0) * 0.48)
    } else if path.contains("OilShutter") {
        let v = if path.contains("Left") { ol } else { or };
        DMat4::from_rotation_x(-v.clamp(0.0, 1.0) * 0.45)
    } else if path.contains("ElevatorTrimTab") {
        DMat4::from_rotation_x(art.elevator_rad - tp)
    } else if path.contains("AileronTrimTab") {
        DMat4::from_rotation_x(art.left_aileron_rad - tr)
    } else if path.contains("GearDoor") {
        let open = part.custom_hinge_axis.map_or(1.45, |a| a.y);
        DMat4::from_rotation_y(open * art.gear_pos_norm.clamp(0.0, 1.0))
    } else if path.contains("RudderTrimTab") {
        DMat4::from_rotation_z(-(art.rudder_rad - ty))
    } else {
        DMat4::IDENTITY
    }
}
