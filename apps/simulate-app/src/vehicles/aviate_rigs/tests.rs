use glam::DVec3;
use sim_core::{Block, ControlInputs, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

use super::{build_aviate_x500, step_aviate_x500};
use crate::vehicles::{FreeFlightPlant, FreeFlightVehicle};
use crate::world_surface::WorldSurfaceHandle;

fn signed_angle_diff_deg(a_deg: f64, b_deg: f64) -> f64 {
    (a_deg - b_deg + 180.0).rem_euclid(360.0) - 180.0
}

fn true_heading_deg(frame: &LocalFrame, state: &VehicleState) -> f64 {
    let nose_ned = frame.ecef_to_ned_vector(state.attitude * DVec3::X);
    nose_ned.y.atan2(nose_ned.x).to_degrees().rem_euclid(360.0)
}

#[test]
fn x500_spawns_and_holds_newark_rwy_22l_true_heading() {
    let origin = Geodetic::from_degrees(40.69, -74.17, 5.0);
    let rwy_22l_true_deg = 206.0_f64;
    let heading_rad = rwy_22l_true_deg.to_radians();
    let frame = LocalFrame::at(&origin);
    let surface = WorldSurfaceHandle::new();

    let mut airborne = FreeFlightPlant::new_at(
        FreeFlightVehicle::AviateX500Direct,
        origin,
        heading_rad,
        5.0,
    )
    .expect("airborne x500");
    let ground = FreeFlightPlant::new_on_ground(
        FreeFlightVehicle::AviateX500Direct,
        origin,
        heading_rad,
        false,
    )
    .expect("ground-hover x500");

    assert!(
        signed_angle_diff_deg(
            true_heading_deg(&frame, &airborne.state()),
            rwy_22l_true_deg
        )
        .abs()
            < 1.0
    );
    assert!(
        signed_angle_diff_deg(true_heading_deg(&frame, &ground.state()), rwy_22l_true_deg).abs()
            < 1.0
    );

    let hover = ControlInputs {
        throttle: 0.5,
        ..ControlInputs::default()
    };
    for _ in 0..200 {
        airborne.step(hover, &surface).expect("hover step");
    }
    let settled_deg = true_heading_deg(&frame, &airborne.state());
    assert!(
        signed_angle_diff_deg(settled_deg, rwy_22l_true_deg).abs() < 1.0,
        "hover heading drifted to {settled_deg:.2}°T instead of {rwy_22l_true_deg:.1}°T"
    );
}

#[test]
fn x500_forward_stick_accelerates_on_nose_and_stops_without_windup() {
    let origin = Geodetic::from_degrees(40.69, -74.17, 115.0);
    let rwy_22l_true_deg = 206.0_f64;
    let heading_rad = rwy_22l_true_deg.to_radians();
    let origin_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&origin);
    let frame = LocalFrame::at(&origin);
    let mut rig = build_aviate_x500(origin, heading_rad).expect("x500 rig");

    let forward = ControlInputs {
        pitch: -1.0,
        roll: 0.0,
        yaw: 0.0,
        throttle: 0.5,
    };
    for _ in 0..1_200 {
        step_aviate_x500(&mut rig, forward).expect("forward step");
    }

    let mid = rig.body.output();
    let vel_ned = frame.ecef_to_ned_vector(mid.velocity_ecef);
    let horiz_speed_m_s = vel_ned.x.hypot(vel_ned.y);
    let track_deg = vel_ned.y.atan2(vel_ned.x).to_degrees().rem_euclid(360.0);
    assert!(
        horiz_speed_m_s > 7.5,
        "expected >14.5 kt (7.5 m/s) after 3 s full forward stick, got {horiz_speed_m_s:.2} m/s"
    );
    assert!(
        signed_angle_diff_deg(track_deg, rwy_22l_true_deg).abs() < 2.0,
        "ground track {track_deg:.2}°T skewed off nose {rwy_22l_true_deg:.1}°T"
    );

    let release_pos_ned = frame.ecef_to_ned_vector(mid.position_ecef - origin_ecef);
    let (fwd_sin, fwd_cos) = heading_rad.sin_cos();
    let neutral = ControlInputs {
        throttle: 0.5,
        ..ControlInputs::default()
    };
    let mut speed_at_0p35s = 0.0_f64;
    let mut min_along_vel_m_s = f64::INFINITY;
    for step in 0..1_600_u32 {
        step_aviate_x500(&mut rig, neutral).expect("braking step");
        let v = frame.ecef_to_ned_vector(rig.body.output().velocity_ecef);
        if step == 140 {
            speed_at_0p35s = v.x.hypot(v.y);
        }
        min_along_vel_m_s = min_along_vel_m_s.min(v.x * fwd_cos + v.y * fwd_sin);
    }

    let stopped = rig.body.output();
    let end_vel_ned = frame.ecef_to_ned_vector(stopped.velocity_ecef);
    let end_pos_ned = frame.ecef_to_ned_vector(stopped.position_ecef - origin_ecef);
    let braking_dist_m =
        (end_pos_ned.x - release_pos_ned.x).hypot(end_pos_ned.y - release_pos_ned.y);
    assert!(
        speed_at_0p35s < horiz_speed_m_s,
        "stick release did not start decelerating promptly ({speed_at_0p35s:.2} vs {horiz_speed_m_s:.2} m/s)"
    );
    assert!(
        end_vel_ned.length() < 0.25 && rig.holding_position,
        "expected latched hover <0.25 m/s, got {:.3} m/s (holding={})",
        end_vel_ned.length(),
        rig.holding_position
    );
    assert!(
        braking_dist_m < 20.0 && min_along_vel_m_s > -0.5,
        "unexpected braking rollout {braking_dist_m:.2} m or reverse snap {min_along_vel_m_s:.2} m/s"
    );
}
