use sim_core::ControlInputs;
use sim_geo::{Geodetic, ecef_to_geodetic};

use super::*;

/// A parked aircraft settles onto its gear and stays there, finite; with the engine idling
/// and brakes off it may only creep.
#[test]
fn parked_jsbsim_aircraft_rest_on_their_gear() {
    let ground = Geodetic::from_degrees(40.524, -74.598, 25.0);
    let world = crate::world_surface::WorldSurfaceHandle::new();
    for (vehicle, cold) in [
        (FreeFlightVehicle::JsbsimC172, false),
        (FreeFlightVehicle::JsbsimC172, true),
        (FreeFlightVehicle::JsbsimP38g, false),
        (FreeFlightVehicle::JsbsimP38g, true),
    ] {
        let mut plant = FreeFlightPlant::new_on_ground(vehicle, ground, 1.2, cold).expect("parked");
        let mut plant_ground = ground;
        if let Some(height_m) = world.ellipsoidal_ground_m(&ground) {
            plant_ground.height_m = height_m;
            plant =
                FreeFlightPlant::new_on_ground(vehicle, plant_ground, 1.2, cold).expect("parked");
        }
        let start = plant.state();
        let periods = (3.0 / plant.period().as_seconds()) as usize;
        for _ in 0..periods {
            plant.step(ControlInputs::default(), &world).expect("step");
        }
        let end = plant.state();
        assert!(end.position_ecef.is_finite(), "{vehicle:?}");
        let moved = (end.position_ecef - start.position_ecef).length();
        let limit = if cold { 1.0 } else { 15.0 };
        assert!(moved < limit, "{vehicle:?} cold={cold} drifted {moved} m");
        let agl = ecef_to_geodetic(end.position_ecef).height_m - plant_ground.height_m;
        assert!(
            (-0.5..3.0).contains(&agl),
            "{vehicle:?} rests {agl} m above the ground"
        );
        let speed_limit = if cold { 0.5 } else { 5.0 };
        assert!(
            end.velocity_ecef.length() < speed_limit,
            "{vehicle:?} cold={cold} moving"
        );
        let running = plant.engine_status().is_some_and(|e| e.running);
        if cold || vehicle == FreeFlightVehicle::JsbsimC172 {
            assert_eq!(running, !cold, "{vehicle:?}");
        }
    }
}
mod gear_datum;

/// An airborne P-38G spawns with both engines turning: its level-flight trim does not
/// converge (the constant-speed governors), and the failed trim must leave the running
/// engines as they were instead of stopping them.
#[test]
fn an_airborne_p38g_spawns_with_both_engines_turning() {
    let spawn = Geodetic::from_degrees(40.524, -74.598, 300.0);
    let plant = FreeFlightPlant::new_at(FreeFlightVehicle::JsbsimP38g, spawn, 0.0, 0.0).unwrap();
    let art = plant.articulation(0.0);
    assert!(art.engine_running, "{art:?}");
    assert!(art.gear_pos_norm < 0.01, "gear at {}", art.gear_pos_norm);
    assert!(
        art.propeller_rpm > 500.0 && art.propeller_right.rpm > 500.0,
        "propellers at {} and {} rpm",
        art.propeller_rpm,
        art.propeller_right.rpm
    );
}

/// Steps the mission plant for `seconds` under `stick`, which nothing in its chain reads.
#[cfg(not(target_arch = "wasm32"))]
fn fly_mission(seconds: f64, stick: ControlInputs) -> FreeFlightPlant {
    let pad = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let world = crate::world_surface::WorldSurfaceHandle::new();
    let vehicle = FreeFlightVehicle::X500Mission;
    let mut plant = FreeFlightPlant::new_on_ground(vehicle, pad, 0.6, false).expect("on its pad");
    assert_eq!(plant.aviate_armed(), Some(false), "starts disarmed");
    let periods = (seconds / plant.period().as_seconds()) as usize;
    for _ in 0..periods {
        plant.step(stick, &world).expect("step");
    }
    plant
}

/// The mission plant arms, leaves its pad and sets out on its route with nobody at the
/// controls, and a pilot's stick changes nothing: Pilotage commands this vehicle.
#[cfg(not(target_arch = "wasm32"))]
#[test]
fn the_mission_plant_flies_itself_and_ignores_the_stick() {
    let pad = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let hands_off = fly_mission(55.0, ControlInputs::default());
    assert_eq!(hands_off.aviate_armed(), Some(true));
    let state = hands_off.state();
    let climbed_m = ecef_to_geodetic(state.position_ecef).height_m - pad.height_m;
    assert!((50.0..70.0).contains(&climbed_m), "climbed {climbed_m} m");
    let away_m = (state.position_ecef - sim_geo::geodetic_to_ecef(&pad)).length();
    assert!(
        away_m > 100.0,
        "{away_m} m from the pad: the route is under way"
    );

    let full_stick = ControlInputs {
        roll: 1.0,
        pitch: -1.0,
        yaw: 1.0,
        throttle: 1.0,
    };
    let mut stirred = fly_mission(55.0, full_stick);
    assert_eq!(stirred.state(), state);
    let snapshot = stirred.snapshot();
    assert!(
        stirred.restore(&snapshot).is_err(),
        "a mission flight has no state to rewind to"
    );
}
