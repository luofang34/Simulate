//! The Command Module alone in the upper atmosphere: it trims on its offset
//! centre of gravity, steers by banking, and the EMS sees entry begin.

use glam::{DMat3, DQuat, DVec3};
use sim_apollo::{
    CM_BMAG, CM_EMS, CM_STRUCTURE, CSM_SERVICE_MODULE, CorridorIndication, EntryMonitorBlock,
    EntryMonitorConfig, InertialSiteSample, apollo_cm_aero, build_apollo_csm_tree,
    cm_analytic_trim_incidence_rad,
};
use sim_atmosphere::ReentryAtmosphere;
use sim_core::{
    Block, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, UtcTime};

const HZ: u64 = 100;
const EPOCH_UNIX_S: f64 = -14_000_000.0;

/// The entry state at `height_m`: `speed_m_s` eastwards along a flight path
/// `flight_path_rad` below the horizon, the capsule at its trim incidence and
/// banked by `bank_rad` about the velocity, lift up at zero.
#[cfg(test)]
fn entry_state(height_m: f64, speed_m_s: f64, flight_path_rad: f64, bank_rad: f64) -> VehicleState {
    let body = BodyEllipsoid::WGS84;
    let position = body.geodetic_to_fixed(&Geodetic::from_degrees(0.0, -170.0, height_m));
    let up = position.normalize();
    let east = DVec3::Z.cross(up).normalize();
    let velocity = (east * flight_path_rad.cos() - up * flight_path_rad.sin()) * speed_m_s;
    let along = velocity.normalize();

    // Lift points to the side of the velocity the centre of gravity sits on,
    // body -z, so body +z starts opposite the local vertical.
    let lift_up = (up - along * up.dot(along)).normalize();
    let trim = cm_analytic_trim_incidence_rad().unwrap();
    let body_x = (along * trim.cos() + lift_up * trim.sin()).normalize();
    let body_z_unbanked = (along * trim.sin() - lift_up * trim.cos()).normalize();
    let body_y_unbanked = body_z_unbanked.cross(body_x);
    let unbanked = DQuat::from_mat3(&DMat3::from_cols(body_x, body_y_unbanked, body_z_unbanked));
    VehicleState {
        position_ecef: position,
        velocity_ecef: velocity,
        attitude: (DQuat::from_axis_angle(along, bank_rad) * unbanked).normalize(),
        angular_rate_body: DVec3::ZERO,
    }
}

/// The CSM set up at `initial`, with the Service Module then jettisoned.
#[cfg(test)]
fn command_module(initial: VehicleState) -> RigidBody6DofBlock {
    let mut vehicle = RigidBody6DofBlock::new(
        RigidBodyConfig {
            body: BodyEllipsoid::WGS84,
            period: TickSpan::period_of_hz(HZ).unwrap(),
            epoch: UtcTime::from_unix_seconds(EPOCH_UNIX_S),
            initial,
        },
        build_apollo_csm_tree().unwrap(),
        Box::new(apollo_cm_aero().unwrap()),
        Box::new(ReentryAtmosphere),
    )
    .unwrap();
    vehicle.separate_stage(CSM_SERVICE_MODULE).unwrap();
    assert!(vehicle.tree().is_attached(CM_STRUCTURE).unwrap());
    vehicle
}

#[cfg(test)]
fn incidence_rad(vehicle: &RigidBody6DofBlock) -> f64 {
    let state = vehicle.output();
    let air = state.attitude.inverse() * state.velocity_ecef;
    air.z.atan2(air.x)
}

#[cfg(test)]
fn height_m(vehicle: &RigidBody6DofBlock) -> f64 {
    BodyEllipsoid::WGS84
        .fixed_to_geodetic(vehicle.output().position_ecef)
        .height_m
}

#[test]
fn the_capsule_pitches_back_to_the_trim_set_by_its_offset_cg() {
    let upset = entry_state(75_000.0, 7_500.0, 2f64.to_radians(), 0.0);
    let pitched = DQuat::from_axis_angle(upset.attitude * DVec3::Y, 0.15) * upset.attitude;
    let mut vehicle = command_module(VehicleState {
        attitude: pitched,
        ..upset
    });

    let trim = cm_analytic_trim_incidence_rad().unwrap();
    assert!(
        (trim.abs() - 20f64.to_radians()).abs() < 2f64.to_radians(),
        "{trim}"
    );
    let upset_error = (incidence_rad(&vehicle) - trim).abs();
    assert!(upset_error > 0.1, "{upset_error}");
    // The aerodynamic damping is weak, as it was on the real capsule, so the
    // test reads the mean of the last ten seconds, not one instant.
    let none = RigidBodyInput::default();
    let mut sign_changes = 0;
    let mut last_error = upset_error;
    let mut settled_sum = 0.0;
    let mut settled_peak: f64 = 0.0;
    let settled_steps = 10 * HZ;
    for step in 0..40 * HZ {
        vehicle.step(&none).unwrap();
        let error = incidence_rad(&vehicle) - trim;
        if error.signum() != last_error.signum() {
            sign_changes += 1;
        }
        last_error = error;
        if step >= 30 * HZ {
            settled_sum += error;
            settled_peak = settled_peak.max(error.abs());
        }
    }
    assert!(sign_changes >= 2, "oscillates about trim: {sign_changes}");
    let settled_mean = settled_sum / settled_steps as f64;
    assert!(settled_mean.abs() < 0.015, "{settled_mean}");
    assert!(
        settled_peak < upset_error,
        "{settled_peak} vs {upset_error}"
    );

    let force = vehicle.aero_wrench().force_body_n;
    let state = vehicle.output();
    let along = state.velocity_ecef.normalize();
    let force_world = state.attitude * force;
    let drag = -force_world.dot(along);
    let lift = (force_world + along * drag).length();
    let lift_to_drag = lift / drag;
    assert!((lift_to_drag - 0.3).abs() < 0.05, "{lift_to_drag}");
}

#[test]
fn banking_about_the_velocity_rotates_the_lift_vector_and_steers() {
    let run = |bank_rad: f64| {
        let mut vehicle =
            command_module(entry_state(75_000.0, 7_500.0, 2f64.to_radians(), bank_rad));
        for _ in 0..30 * HZ {
            vehicle.step(&RigidBodyInput::default()).unwrap();
        }
        vehicle
    };
    let lift_up = run(0.0);
    let lift_down = run(180f64.to_radians());
    let lift_right = run(90f64.to_radians());
    let lift_left = run(-90f64.to_radians());

    assert!(
        height_m(&lift_up) > height_m(&lift_down) + 1_000.0,
        "{} vs {}",
        height_m(&lift_up),
        height_m(&lift_down)
    );
    let north = |vehicle: &RigidBody6DofBlock| vehicle.output().position_ecef.z;
    assert!(
        (north(&lift_right) - north(&lift_left)).abs() > 1_000.0,
        "{} vs {}",
        north(&lift_right),
        north(&lift_left)
    );
    assert!(
        north(&lift_right) * north(&lift_left) < 0.0,
        "opposite sides of the ground track"
    );
}

#[test]
fn the_ems_latches_at_a_twentieth_of_a_g_on_the_way_through_75_km() {
    let initial = entry_state(100_000.0, 7_620.0, 3f64.to_radians(), 0.0);
    let mut vehicle = command_module(initial);
    let mut ems = EntryMonitorBlock::new(EntryMonitorConfig {
        period: vehicle.period(),
        sensing_axis_site: DVec3::X,
        initial_delta_v_m_s: 7_620.0,
    })
    .unwrap();
    ems.step(&sample(&vehicle, CM_EMS)).unwrap();
    assert!(!ems.output().threshold_reached, "{}", ems.output().g_load);

    let mut latched_at_m = None;
    let mut speed_at_75_km = None;
    let none = RigidBodyInput::default();
    for _ in 0..120 * HZ {
        vehicle.step(&none).unwrap();
        ems.step(&sample(&vehicle, CM_EMS)).unwrap();
        let height = height_m(&vehicle);
        if ems.output().threshold_reached && latched_at_m.is_none() {
            latched_at_m = Some(height);
            assert!(ems.output().g_load >= 0.05);
            assert!(ems.output().g_load < 0.06, "{}", ems.output().g_load);
        }
        if height <= 75_000.0 && speed_at_75_km.is_none() {
            speed_at_75_km = Some(vehicle.output().velocity_ecef.length());
        }
    }
    let latched_at_m = latched_at_m.expect("deceleration reached 0.05 g");
    assert!(
        (75_000.0..100_000.0).contains(&latched_at_m),
        "{latched_at_m}"
    );
    let speed = speed_at_75_km.expect("descended through 75 km");
    assert!((7_000.0..7_700.0).contains(&speed), "{speed}");

    let output = ems.output();
    assert_ne!(output.corridor, CorridorIndication::Pending);
    assert!(output.delta_v_counter_m_s < 7_620.0 - 500.0, "{output:?}");
    assert!(output.peak_g_load > 1.0, "{output:?}");

    let rates = sample(&vehicle, CM_BMAG).angular_rate_site_rad_s;
    assert!(rates.length() < 0.2, "the capsule stays trimmed: {rates}");
}

#[cfg(test)]
fn sample(vehicle: &RigidBody6DofBlock, site: mbd_core::ComponentId) -> InertialSiteSample {
    let utc = UtcTime::from_unix_seconds(EPOCH_UNIX_S).advanced_by(
        vehicle
            .now()
            .duration_since(sim_core::SimInstant::START)
            .as_seconds(),
    );
    InertialSiteSample::of(vehicle, site, &InertialTransform::earth_at(utc)).unwrap()
}
