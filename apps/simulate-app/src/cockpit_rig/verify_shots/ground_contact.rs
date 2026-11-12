//! Parked aircraft stand on the terrain the physical scene draws: the airframe's lowest
//! point meets the drawn ground, and keeps meeting it after the simulation's live ground
//! evicts tiles and fetches them again.

use std::path::PathBuf;

use sim_core::ControlInputs;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_maplibre::UsdAircraftAsset;
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::{AirportDatabase, RunwayDatum, project_to_tile};

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::error::AppSimError;
use crate::esc_menu::SpawnMode;
use crate::sky_rig::SkyFrame;
use crate::vehicles::FreeFlightVehicle;

/// How far the wheels may sit off the drawn terrain.
const CONTACT_TOLERANCE_M: f64 = 0.05;

fn fail(detail: impl ToString) -> AppSimError {
    AppSimError::Screenshot {
        detail: detail.to_string(),
    }
}

/// Steps the flight and draws `frames` chase frames, so tiles stream in and drain.
fn run_frames(h: &mut VerifyHarness, frames: usize) -> Result<(), AppSimError> {
    let sky = SkyFrame::fixed_sun(
        h.flight.world().body(),
        h.flight.world().sun_direction_ecef(),
    );
    for _ in 0..frames {
        h.flight
            .advance_by(1.0 / 30.0, ControlInputs::default(), SimTimeScale::RealTime)?;
        let cam = h.flight.display_camera(OperatorViewportMode::Chase)?;
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(15));
    }
    Ok(())
}

/// The ownship, the drawn terrain under it and the simulation's ground there, MSL.
fn contact(h: &VerifyHarness) -> Result<(Geodetic, f64, f64), AppSimError> {
    let own = ecef_to_geodetic(h.flight.vehicle_state().position_ecef);
    let surface = h.flight.world_surface();
    let undulation = surface.geoid_undulation_m(own.lat_rad, own.lon_rad);
    let drawn = h
        .rig
        .maps()
        .and_then(|maps| maps.truth().drawn_ground_msl(own.lat_deg(), own.lon_deg()))
        .ok_or_else(|| fail("no drawn terrain under the ownship"))?;
    let ground = surface
        .orthometric_ground_m(&own)
        .ok_or_else(|| fail("no ground under the ownship"))?;
    Ok((
        Geodetic {
            height_m: own.height_m - undulation,
            ..own
        },
        drawn,
        ground,
    ))
}

/// Parks `vehicle` at KEWR and KDEN and requires the drawn model's lowest point, its
/// wheels at their static compression, within 5 cm of the drawn terrain, and the ground
/// within 5 cm of it too.
fn assert_parked_wheels_touch(
    vehicle: FreeFlightVehicle,
    asset: &UsdAircraftAsset,
) -> Result<(), AppSimError> {
    let Some(mut h) = chase_harness()? else {
        return Ok(());
    };
    let db = AirportDatabase::builtin();
    h.flight.select_vehicle(vehicle)?;
    let lowest_m = f64::from(asset.lowest_point_z_m());
    for code in ["KEWR", "KDEN"] {
        let (idx, record) = db.by_code(code).ok_or_else(|| fail(code))?;
        h.rig.supervisor_mut().set_free_airport(idx, 0);
        h.flight
            .spawn_at_airport_record(record, 0, SpawnMode::Threshold)?;
        run_frames(&mut h, 120)?;
        let (own, drawn, ground) = contact(&h)?;
        let state = h.flight.vehicle_state();
        let origin_up_m =
            (state.attitude * h.flight.model_origin_from_cg_frd_m()).dot(LocalFrame::at(&own).up());
        let compression_m = h
            .flight
            .articulation()
            .gear_compression_m
            .into_iter()
            .fold(f64::INFINITY, f64::min);
        let wheels = own.height_m + origin_up_m + lowest_m + compression_m;
        let label = format!("{} {code}", vehicle.title());
        assert!(
            (wheels - drawn).abs() < CONTACT_TOLERANCE_M,
            "{label}: wheels {wheels:.3} m vs drawn {drawn:.3} m MSL"
        );
        assert!(
            (ground - drawn).abs() < CONTACT_TOLERANCE_M,
            "{label}: ground {ground:.3} m vs drawn {drawn:.3} m MSL"
        );
    }
    h.gpu.check_device()?;
    Ok(())
}

/// Within tolerance once drawn from its structural datum; the static per-leg check
/// (`c172_wheels_meet_the_fdm_gear_red_while_the_fit_leaves_the_gear_1_in_aft`) still sees
/// the mesh's wheels off the flight model's gear fore and aft.
#[test]
#[ignore = "checks parked wheels against drawn terrain on the GPU; needs $SIMULATE_VERIFY_OUT"]
fn c172_parked_wheels_touch_the_drawn_terrain() -> Result<(), AppSimError> {
    assert_parked_wheels_touch(
        FreeFlightVehicle::JsbsimC172,
        &UsdAircraftAsset::embedded_n642sp()?,
    )
}

/// Red while the P-38G flight model's gear is 8.7 in short of the Lockheed geometry: the
/// The P-38G's drawn wheels stand on the drawn terrain where its flight model parks it.
#[test]
#[ignore = "checks parked wheels against drawn terrain on the GPU; needs $SIMULATE_VERIFY_OUT"]
fn p38g_parked_wheels_touch_the_drawn_terrain() -> Result<(), AppSimError> {
    assert_parked_wheels_touch(FreeFlightVehicle::JsbsimP38g, &UsdAircraftAsset::p38g()?)
}

/// With room for only a few live tiles, flying to another airport and back evicts the
/// ground under the first; it is fetched again and matches what is drawn.
#[test]
#[ignore = "checks live ground against drawn terrain on the GPU; needs $SIMULATE_VERIFY_OUT"]
fn live_ground_matches_the_drawn_terrain_through_eviction() -> Result<(), AppSimError> {
    let Some(mut h) = chase_harness()? else {
        return Ok(());
    };
    let h = &mut h;
    h.flight.world_surface().set_live_dem_capacity(12);
    let db = AirportDatabase::builtin();
    for code in ["KEWR", "KDEN", "KEWR"] {
        let (idx, record) = db.by_code(code).ok_or_else(|| fail(code))?;
        h.rig.supervisor_mut().set_free_airport(idx, 0);
        h.flight
            .spawn_at_airport_record(record, 0, SpawnMode::Threshold)?;
        run_frames(h, 150)?;
        // Off the runway, where only resident terrain says where the ground is.
        let own = ecef_to_geodetic(h.flight.vehicle_state().position_ecef);
        let local = LocalFrame::at(&own);
        let mut compared = 0_u32;
        let datum = RunwayDatum::builtin();
        for step in 0..8 {
            let bearing = f64::from(step) * std::f64::consts::FRAC_PI_4;
            let offset = (local.north() * bearing.cos() + local.east() * bearing.sin()) * 250.0;
            let at = ecef_to_geodetic(geodetic_to_ecef(&own) + offset);
            // A runway's skirt can be steep where the terrain sits metres off the
            // charted runway; compare natural terrain only.
            if datum.height_and_weight(&at).is_some() {
                continue;
            }
            let surface = h.flight.world_surface();
            let drawn = h
                .rig
                .maps()
                .and_then(|maps| maps.truth().drawn_ground_msl(at.lat_deg(), at.lon_deg()));
            let (Some(drawn), Some(ground)) = (drawn, surface.orthometric_ground_m(&at)) else {
                continue;
            };
            compared = compared.wrapping_add(1);
            let drawn_zoom = h
                .rig
                .maps()
                .map(|maps| maps.truth().drawn_dem_tiles())
                .unwrap_or_default()
                .into_iter()
                .filter(|tile| project_to_tile(&at, tile.zoom).is_some_and(|c| c.tile == *tile))
                .map(|tile| tile.zoom)
                .max();
            let ground_zoom = surface.finest_live_zoom_at(&at);
            let label = format!(
                "{code} after eviction, {:.0}° 250 m out",
                bearing.to_degrees()
            );
            assert!(
                ground_zoom >= drawn_zoom,
                "{label}: ground from z{ground_zoom:?}, drawn from z{drawn_zoom:?}"
            );
            // maplibre meshes every other texel and places texels half a pixel off, so
            // natural terrain agrees to decimetres, not centimetres.
            assert!(
                (ground - drawn).abs() < 0.75,
                "{label}: ground {ground:.3} m vs drawn {drawn:.3} m"
            );
        }
        assert!(
            compared >= 2,
            "{code}: only {compared} points drawn around the ownship"
        );
    }
    h.gpu.check_device()?;
    Ok(())
}

/// A verify harness in the chase view, which draws the ground under the ownship; `None`
/// without `$SIMULATE_VERIFY_OUT` or a GPU.
fn chase_harness() -> Result<Option<VerifyHarness>, AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(None);
    };
    std::fs::create_dir_all(&dir).map_err(fail)?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(None);
    };
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    Ok(Some(h))
}
