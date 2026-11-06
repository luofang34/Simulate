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
        (FreeFlightVehicle::JsbsimLightTrainer, false),
    ] {
        let mut plant = FreeFlightPlant::new_on_ground(vehicle, ground, 1.2, cold).expect("parked");
        let mut plant_ground = ground;
        if let Ok(surface) = sim_scenery::AuthoritativeWorldSurface::regional_earth_shared()
            && let Ok(Some(sample)) =
                sim_scenery::WorldSurfaceQuery::sample_surface(surface, &ground)
        {
            plant_ground.height_m = sample.ellipsoidal_height_m.0;
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
        // The light trainer's engine model reports no power at zero throttle, so only the
        // C172's idle is checked.
        let running = plant.engine_status().is_some_and(|e| e.running);
        if cold || vehicle == FreeFlightVehicle::JsbsimC172 {
            assert_eq!(running, !cold, "{vehicle:?}");
        }
    }
}
