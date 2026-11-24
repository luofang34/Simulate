//! Hermetic external hangar plugin tests, scripted/ground NPC fleet verification, and
//! high-resolution offscreen `HangarPreviewPass` screenshot rendering.

use std::f64::consts::TAU;
use std::path::{Path, PathBuf};

use sim_core::{ControlInputs, SimInstant};
use sim_geo::Geodetic;
use sim_maplibre::{HANGAR_DEFAULT_SIZE, HangarOrbit, HangarPreviewPass};
use sim_render::HeadlessGpu;
use sim_scenario::SimTimeScale;

use crate::error::AppSimError;
use crate::esc_menu::HubCatalog;
use crate::flight::Flight;
use crate::gallery::{readback_target_rgba, save_png};
use crate::livery_session::LiverySession;
use crate::map_rig::aircraft::traffic_aircraft_filtered;
use crate::supervisor_rig::SupervisorRig;
use crate::vehicles::FreeFlightVehicle;
use crate::world_surface::WorldSurfaceHandle;

const PREVIEW_SIZE: [u32; 2] = [960, 600];

struct TempHangar(PathBuf);

impl TempHangar {
    fn new(tag: &str) -> Result<Self, AppSimError> {
        let dir = std::env::temp_dir().join(format!(
            "simulate_hangar_plugin_test_{tag}_{}",
            std::process::id()
        ));
        drop(std::fs::remove_dir_all(&dir));
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../vehicles/c172p");
        let plugin_dir = dir.join("c172p_plugin");
        copy_dir_recursive(&src_dir, &plugin_dir)
            .map_err(|detail| AppSimError::NpcScript { detail })?;
        let manifest_path = plugin_dir.join("syspack.json");
        let manifest_txt =
            std::fs::read_to_string(&manifest_path).map_err(|e| AppSimError::NpcScript {
                detail: format!("read {}: {e}", manifest_path.display()),
            })?;
        let patched = manifest_txt.replacen(r#""id": "c172p""#, r#""id": "c172p_plugin""#, 1);
        std::fs::write(&manifest_path, patched.as_bytes()).map_err(|e| AppSimError::NpcScript {
            detail: format!("write {}: {e}", manifest_path.display()),
        })?;
        std::fs::write(dir.join("npc_fleet.json"), HERMETIC_NPC_SCRIPT).map_err(|e| {
            AppSimError::NpcScript {
                detail: format!("write npc_fleet.json: {e}"),
            }
        })?;
        Ok(Self(dir))
    }
}

impl Drop for TempHangar {
    fn drop(&mut self) {
        drop(std::fs::remove_dir_all(&self.0));
    }
}

fn copy_dir_recursive(src: &Path, dst: &Path) -> Result<(), String> {
    std::fs::create_dir_all(dst).map_err(|e| format!("mkdir {}: {e}", dst.display()))?;
    let entries = std::fs::read_dir(src).map_err(|e| format!("readdir {}: {e}", src.display()))?;
    for entry in entries.flatten() {
        let path = entry.path();
        let out = dst.join(entry.file_name());
        if path.is_dir() {
            copy_dir_recursive(&path, &out)?;
        } else {
            std::fs::copy(&path, &out).map_err(|e| format!("copy {}: {e}", path.display()))?;
        }
    }
    Ok(())
}

#[test]
fn external_hangar_plugins_load_fit_and_step_jsbsim() -> Result<(), AppSimError> {
    let hangar = TempHangar::new("plugin")?;
    let mut session = LiverySession::new()?;
    let registered = session.import_hangar_dir(&hangar.0);
    assert!(
        registered.iter().any(|id| id == "c172p_plugin"),
        "expected c172p_plugin, got {registered:?}"
    );
    let slot = session
        .external_slot_by_id("c172p_plugin")
        .expect("external slot");
    let meta = session.external_meta(slot).expect("external meta");
    assert!(
        meta.parked_gear_height_m > 0.0,
        "expected positive parked gear height, got {}",
        meta.parked_gear_height_m
    );
    assert!(
        !meta.camera_suite.cameras.is_empty(),
        "expected camera views from external plugin"
    );
    let asset = session
        .usd_asset_for("c172p_plugin", None)?
        .expect("default external OpenUSD asset");
    assert!(asset.triangle_count() > 0, "expected non-empty mesh");
    assert!(
        session.liveries_for("c172p_plugin").len() >= 2,
        "c172p_plugin should expose >=2 liveries"
    );
    let _ = session.select_livery_index("c172p_plugin", 1)?;
    assert!(
        session.active_usd_override("c172p_plugin").is_some(),
        "c172p_plugin secondary livery"
    );

    verify_external_jsbsim_flight(&hangar.0)
}

fn verify_external_jsbsim_flight(hangar_dir: &Path) -> Result<(), AppSimError> {
    let mut catalog = HubCatalog::new(std::iter::empty::<&sim_scenery::OsmAerodromeDiagram>());
    let mut flight = Flight::new()?;
    let imported = flight.liveries_mut().import_hangar_dir(hangar_dir);
    assert!(!imported.is_empty(), "got {imported:?}");
    catalog.sync_external_vehicles(flight.liveries().externals());
    assert!(
        catalog.vehicles.len() >= 8,
        "7 built-ins + >=1 hangar plugin"
    );

    let slot = flight
        .liveries()
        .external_slot_by_id("c172p_plugin")
        .expect("external flight slot");
    flight.select_vehicle(FreeFlightVehicle::External(slot))?;
    assert!(
        flight.engine_status().is_some_and(|e| e.running),
        "c172p_plugin engine running"
    );
    let start = flight.vehicle_state();
    flight.advance_by(0.4, ControlInputs::default(), SimTimeScale::RealTime)?;
    let after = flight.vehicle_state();
    assert!(after.position_ecef.is_finite(), "position finite");
    assert!(
        (after.position_ecef - start.position_ecef).length() > 5.0,
        "c172p_plugin advancing in flight"
    );
    Ok(())
}

#[test]
fn npc_fleet_spawns_apron_flies_routes_and_feeds_adsb() -> Result<(), AppSimError> {
    let hangar = TempHangar::new("npc")?;
    let mut flight = Flight::new()?;
    let spawned = flight.spawn_ground_npc("p38g", None)?;
    assert!(spawned, "built-in p38g spawns on apron");
    assert_eq!(flight.npc_count(), 1);
    let anchor = Geodetic::from_degrees(39.1220, -75.4680, 8.5);
    let parked = flight.evaluate_npcs(anchor);
    assert_eq!(parked.len(), 1);
    assert!(
        parked
            .first()
            .is_some_and(|n| n.ground_height_msl_m.is_some())
    );

    let imported = flight.liveries_mut().import_hangar_dir(&hangar.0);
    assert!(!imported.is_empty(), "got {imported:?}");
    let script_count = flight.load_npc_script_path(&hangar.0.join("npc_fleet.json"))?;
    assert_eq!(script_count, 2, "hermetic npc_fleet.json loads 2 spawns");
    let script_parked = flight.evaluate_npcs(anchor);
    assert_eq!(script_parked.len(), 2);
    assert!(
        script_parked
            .iter()
            .all(|n| n.ground_height_msl_m.is_some())
    );

    let loaded = flight.load_npc_script_bytes(ORBIT_AND_ROUTE_SCRIPT)?;
    assert_eq!(loaded, 2);
    let surface = WorldSurfaceHandle::new();
    let t0 = flight.evaluate_npcs(anchor);
    flight.advance_by(2.0, ControlInputs::default(), SimTimeScale::RealTime)?;
    let t1 = flight.evaluate_npcs(anchor);
    assert_eq!(t0.len(), 2);
    assert_eq!(t1.len(), 2);
    assert_ne!(
        t0.first().map(|n| n.placement.position),
        t1.first().map(|n| n.placement.position),
        "orbit NPC must advance across time"
    );

    let mut sup = SupervisorRig::new()?;
    sup.ingest_npc_transponders(&t1, SimInstant::START);
    let truth_entities = sup.truth_entities(sim_maplibre::ViewSlotMask::ALL);
    assert!(
        !truth_entities.is_empty(),
        "transponder NPC added to truth surveillance"
    );
    let filtered = traffic_aircraft_filtered(&truth_entities, &sup, 1.0);
    assert_eq!(
        filtered.len(),
        truth_entities
            .iter()
            .filter(
                |e| !sup.is_npc_track(e.id) && e.kind == sim_maplibre::DynamicEntityKind::FixedWing
            )
            .count()
    );
    drop(surface);
    Ok(())
}

const HERMETIC_NPC_SCRIPT: &[u8] = br#"{
    "schema_version": 1,
    "spawns": [
        {
            "id": "static-plugin-ramp",
            "aircraft_id": "c172p_plugin",
            "mode": {
                "kind": "parked_apron",
                "offset_east_m": 40.0,
                "offset_north_m": 20.0,
                "heading_deg": 45.0,
                "props_idle": false
            },
            "transponder": false
        },
        {
            "id": "static-p38g-ramp",
            "aircraft_id": "p38g",
            "mode": {
                "kind": "parked_geodetic",
                "lat_deg": 39.1225,
                "lon_deg": -75.4675,
                "heading_deg": 180.0,
                "props_idle": false
            },
            "transponder": false
        }
    ]
}"#;

const ORBIT_AND_ROUTE_SCRIPT: &[u8] = br#"{
    "schema_version": 1,
    "spawns": [
        {
            "id": "orbit-p38",
            "aircraft_id": "p38g",
            "callsign": "ORBIT38",
            "transponder": true,
            "mode": {
                "kind": "orbit",
                "center_lat_deg": 39.125,
                "center_lon_deg": -75.460,
                "altitude_msl_m": 600.0,
                "radius_m": 1200.0,
                "speed_kt": 180.0,
                "clockwise": true,
                "phase_deg": 0.0
            }
        },
        {
            "id": "route-c172",
            "aircraft_id": "c172s",
            "transponder": false,
            "mode": {
                "kind": "waypoints",
                "loop_route": true,
                "speed_kt": 110.0,
                "points": [
                    { "lat_deg": 39.120, "lon_deg": -75.480, "altitude_msl_m": 450.0 },
                    { "lat_deg": 39.130, "lon_deg": -75.450, "altitude_msl_m": 550.0 }
                ]
            }
        }
    ]
}"#;

#[test]
fn render_hangar_previews_and_optional_png_dumps() -> Result<(), AppSimError> {
    let hangar = TempHangar::new("render")?;
    let mut session = LiverySession::new()?;
    session.import_hangar_dir(&hangar.0);
    let gpu = pollster::block_on(HeadlessGpu::new())?;
    let mut pass = HangarPreviewPass::new(&gpu.device, HANGAR_DEFAULT_SIZE)?;
    pass.ensure_size(&gpu.device, PREVIEW_SIZE)?;

    // Render empty studio backdrop once for exact silhouette coverage verification.
    let backdrop = render_preview_rgba(&gpu, &mut pass, &HangarOrbit::default())?;
    let dump_dir = std::env::var_os("HANGAR_PREVIEW_DUMP_DIR").map(PathBuf::from);
    if let Some(dir) = &dump_dir {
        std::fs::create_dir_all(dir).map_err(|err| AppSimError::Screenshot {
            detail: format!("creating {}: {err}", dir.display()),
        })?;
    }

    let shots = [
        ("c172p_plugin", "plugin_three_quarter.png", TAU - 0.78, 0.28),
        ("c172p_plugin", "plugin_front_quarter.png", 0.58, 0.14),
    ];
    for (id, filename, azimuth_rad, elevation_rad) in shots {
        let asset = session
            .usd_asset_for(id, None)?
            .expect("external OpenUSD stage");
        pass.set_asset(&gpu.device, &gpu.queue, &asset)?;
        let orbit = HangarOrbit {
            azimuth_rad,
            elevation_rad,
            zoom: 1.0,
            auto_rotate: false,
        };
        let rgba = render_preview_rgba(&gpu, &mut pass, &orbit)?;
        let changed = backdrop
            .as_chunks::<4>()
            .0
            .iter()
            .zip(rgba.as_chunks::<4>().0.iter())
            .filter(|(a, b)| a.iter().zip(b.iter()).any(|(x, y)| x.abs_diff(*y) > 4))
            .count();
        let total = usize::try_from(PREVIEW_SIZE[0] * PREVIEW_SIZE[1]).unwrap_or(1);
        let pct = changed * 100 / total;
        assert!(
            pct >= 4,
            "{filename} silhouette covered only {pct}% of frame"
        );
        if let Some(dir) = &dump_dir {
            save_png(&dir.join(filename), PREVIEW_SIZE, rgba)?;
        }
    }
    Ok(())
}

fn render_preview_rgba(
    gpu: &HeadlessGpu,
    pass: &mut HangarPreviewPass,
    orbit: &HangarOrbit,
) -> Result<Vec<u8>, AppSimError> {
    let mut enc = gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("hangar preview test"),
        });
    let mut art = sim_core::AirframeArticulation {
        engine_running: true,
        engine_rpm: 1_100.0,
        propeller_rpm: 1_100.0,
        propeller_phase_rad: 0.4,
        gear_pos_norm: 1.0,
        ..sim_core::AirframeArticulation::default()
    };
    art.propeller_right.rpm = 1_100.0;
    art.propeller_right.sense = -1.0;
    art.propeller_right.phase_rad = 2.1;
    pass.render(&gpu.device, &gpu.queue, &mut enc, &art, orbit)?;
    gpu.queue.submit([enc.finish()]);
    readback_target_rgba(&gpu.device, &gpu.queue, pass.texture())
}
