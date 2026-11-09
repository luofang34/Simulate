//! Aerodrome spawn modes put the vehicle where and how each mode promises.

use sim_core::ControlInputs;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, egm96_undulation_m, geodetic_to_ecef};
use sim_scenario::SimTimeScale;
use sim_scenery::{AerodromeId, OsmAerodromeDiagram};

use super::manual::heading_rad;
use super::*;
use crate::app::trim_throttle_for_tests as trim_throttle;
use crate::vehicles::FreeFlightVehicle;
use crate::world_surface::WorldSurfaceHandle;

fn angle_between(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(std::f64::consts::TAU);
    d.min(std::f64::consts::TAU - d)
}

#[test]
fn threshold_short_final_and_ramp_spawns_match_their_modes() {
    let diagram = OsmAerodromeDiagram::builtin(AerodromeId::NewarkKewr).expect("KEWR");
    let end = diagram
        .surface_graph
        .runway_ends
        .first()
        .copied()
        .expect("runway end");
    let threshold = Geodetic::from_degrees(
        end.threshold.latitude_deg,
        end.threshold.longitude_deg,
        diagram.field_elevation_m,
    );
    let mut flight = Flight::new().expect("flight");

    flight
        .spawn_at_aerodrome(&diagram, 0, SpawnMode::Threshold)
        .expect("threshold");
    assert!(flight.spawned_on_ground());
    assert_eq!(
        trim_throttle(FreeFlightVehicle::JsbsimC172, flight.spawned_on_ground()),
        0.0
    );
    let parked = flight.vehicle_state();
    assert!(parked.velocity_ecef.length() < 0.5);
    assert!(angle_between(heading_rad(&parked), end.true_bearing_rad) < 0.02);
    let local = LocalFrame::at(&threshold);
    let offset = parked.position_ecef - geodetic_to_ecef(&threshold);
    let horizontal = (offset - local.up() * offset.dot(local.up())).length();
    assert!(
        horizontal < 60.0,
        "lined up {horizontal} m from the threshold"
    );
    flight
        .advance_by(2.0, ControlInputs::default(), SimTimeScale::RealTime)
        .expect("idle on the runway");
    assert!(flight.vehicle_state().velocity_ecef.length() < 2.0);

    flight
        .spawn_at_aerodrome(&diagram, 0, SpawnMode::ShortFinal)
        .expect("short final");
    assert!(!flight.spawned_on_ground());
    assert_eq!(
        trim_throttle(FreeFlightVehicle::JsbsimC172, flight.spawned_on_ground()),
        0.65
    );
    let fin = flight.vehicle_state();
    let fin_geo = ecef_to_geodetic(fin.position_ecef);
    let offset = fin.position_ecef - geodetic_to_ecef(&threshold);
    let horizontal = (offset - local.up() * offset.dot(local.up())).length();
    assert!((horizontal - 5_556.0).abs() < 150.0, "{horizontal} m out");
    let agl = fin_geo.height_m - diagram.field_elevation_m;
    assert!((250.0..400.0).contains(&agl), "{agl} m on the glide path");
    let toward = -offset.normalize();
    let nose = fin.attitude * glam::DVec3::X;
    assert!(nose.dot(toward) > 0.95, "pointed at the runway");

    if diagram.surface_graph.parking_spots.is_empty() {
        return;
    }
    flight
        .spawn_at_aerodrome(&diagram, 0, SpawnMode::Ramp)
        .expect("ramp");
    assert!(flight.spawned_on_ground());
    assert_eq!(
        flight.engine_status().map(|e| e.running),
        Some(false),
        "cold and parked"
    );
    flight
        .select_vehicle(FreeFlightVehicle::AviateX500Direct)
        .expect("x500 at the stand");
    assert_eq!(
        trim_throttle(FreeFlightVehicle::AviateX500Direct, false),
        0.5
    );
    flight.restart().expect("restart returns to the stand");
    assert!(flight.spawned_on_ground());
}

fn real_world_surface() -> WorldSurfaceHandle {
    let surface = WorldSurfaceHandle::new();
    #[cfg(not(target_arch = "wasm32"))]
    let source = crate::map_rig::ScenerySource::RealWorld { cache_dir: None };
    #[cfg(target_arch = "wasm32")]
    let source = crate::map_rig::ScenerySource::RealWorld {};
    surface.configure(&source).expect("real-world surface");
    surface
}

/// Reference-point height over the ground the plant stands on and the downward speed. Off
/// the runway strips a real-world surface without streamed tiles has no ground, and the
/// plant keeps the spawn's.
fn agl_and_sink(flight: &Flight) -> (f64, f64) {
    let state = flight.vehicle_state();
    let geo = ecef_to_geodetic(state.position_ecef);
    let FreeSpawn::Ground { spawn, .. } = flight.free_spawn else {
        panic!("not parked");
    };
    let ground = flight
        .surface
        .ellipsoidal_ground_m(&geo)
        .unwrap_or(spawn.height_m);
    let down = -LocalFrame::at(&geo).up().dot(state.velocity_ecef);
    (geo.height_m - ground, down)
}

/// Parked on the runway or the ramp, each aircraft starts at rest on its gear and stays
/// there at idle: no sinking through a field below the ellipsoid, no catapult off it.
fn assert_rests_on_gear(flight: &mut Flight, label: &str, gear_m: f64) {
    assert!(flight.spawned_on_ground(), "{label}");
    let (start_agl, start_sink) = agl_and_sink(flight);
    assert!(
        (start_agl - gear_m).abs() < 0.05 && start_sink.abs() < 0.1,
        "{label}: starts {start_agl:.3} m up sinking {start_sink:.3} m/s"
    );
    for _ in 0..6 {
        flight
            .advance_by(0.5, ControlInputs::default(), SimTimeScale::RealTime)
            .expect("idle on the ground");
    }
    let (agl, sink) = agl_and_sink(flight);
    assert!(
        (agl - gear_m).abs() < 0.05 && sink.abs() < 0.1,
        "{label}: after 3 s {agl:.3} m up sinking {sink:.3} m/s"
    );
}

#[test]
fn parked_aircraft_rest_on_their_gear_at_airports_worldwide() {
    use crate::vehicles::{C172_GEAR_HEIGHT_M, P38G_GEAR_HEIGHT_M};
    let db = sim_scenery::AirportDatabase::builtin();
    let mut flight = Flight::with_surface(real_world_surface()).expect("flight");
    assert_eq!(
        flight.surface.vertical_datum(),
        sim_scenery::CatalogVerticalDatum::OrthometricMslEgm96
    );
    for (vehicle, gear_m) in [
        (FreeFlightVehicle::JsbsimC172, C172_GEAR_HEIGHT_M),
        (FreeFlightVehicle::JsbsimP38g, P38G_GEAR_HEIGHT_M),
    ] {
        flight.select_vehicle(vehicle).expect("vehicle");
        for code in ["KEWR", "47N", "EGLL", "RJTT"] {
            let (_, record) = db.by_code(code).expect("airport");
            for mode in [SpawnMode::Threshold, SpawnMode::Ramp] {
                flight
                    .spawn_at_airport_record(record, 0, mode)
                    .expect("spawn");
                let label = format!("{} {code} {}", vehicle.title(), mode.title());
                assert_rests_on_gear(&mut flight, &label, gear_m);
            }
        }
    }
}

#[test]
fn kewr_spawns_sit_on_the_charted_field_in_either_datum() {
    let diagram = OsmAerodromeDiagram::builtin(AerodromeId::NewarkKewr).expect("KEWR");
    for surface in [WorldSurfaceHandle::new(), real_world_surface()] {
        let mut flight = Flight::with_surface(surface).expect("flight");
        flight
            .spawn_at_aerodrome(&diagram, 0, SpawnMode::Threshold)
            .expect("threshold");
        let geo = ecef_to_geodetic(flight.vehicle_state().position_ecef);
        let charted = flight.surface.msl_to_ellipsoidal_m(
            geo.lat_rad,
            geo.lon_rad,
            diagram.field_elevation_m,
        );
        let ground = flight.surface.ellipsoidal_ground_m(&geo).expect("runway");
        assert!((ground - charted).abs() < 3.0, "{ground} vs {charted}");
        let msl = flight.surface.orthometric_ground_m(&geo).expect("runway");
        let undulation = egm96_undulation_m(geo.lat_rad, geo.lon_rad);
        assert!((msl - (ground - undulation)).abs() < 1e-6, "{msl}");

        flight
            .spawn_at_aerodrome(&diagram, 0, SpawnMode::Overhead)
            .expect("overhead");
        let over = ecef_to_geodetic(flight.vehicle_state().position_ecef).height_m - ground;
        assert!((250.0..350.0).contains(&over), "overhead {over} m up");
    }
}

#[test]
fn global_airport_spawns_register_its_runways_and_line_up() {
    let db = sim_scenery::AirportDatabase::builtin();
    let (_, egll) = db.by_code("EGLL").expect("EGLL");
    let mut flight = Flight::with_surface(real_world_surface()).expect("flight");
    flight
        .spawn_at_airport_record(egll, 0, SpawnMode::ShortFinal)
        .expect("short final");
    assert!(!flight.spawned_on_ground());
    let datum = flight.surface.vertical_datum();
    let end = egll.to_flightbag_graph(datum).runway_ends[0];
    let threshold = Geodetic::from_degrees(
        end.threshold.latitude_deg,
        end.threshold.longitude_deg,
        end.threshold.ellipsoidal_height_m,
    );
    assert!(flight.surface.ellipsoidal_ground_m(&threshold).is_some());
    let fin = flight.vehicle_state();
    let local = LocalFrame::at(&threshold);
    let offset = fin.position_ecef - geodetic_to_ecef(&threshold);
    let up = offset.dot(local.up());
    let horizontal = (offset - local.up() * up).length();
    assert!((horizontal - 5_556.0).abs() < 150.0, "{horizontal} m out");
    assert!((250.0..400.0).contains(&up), "{up} m on the glide path");
    assert!(angle_between(heading_rad(&fin), end.true_bearing_rad) < 0.02);

    flight.respawn_at_airport_record(egll).expect("overhead");
    let over = ecef_to_geodetic(flight.vehicle_state().position_ecef).height_m
        - egll.arp_geodetic(datum).height_m;
    assert!((over - 300.0).abs() < 1.0, "{over} m over the field");
    flight.restart().expect("restart keeps the airport");
    assert!(flight.surface.ellipsoidal_ground_m(&threshold).is_some());
}
