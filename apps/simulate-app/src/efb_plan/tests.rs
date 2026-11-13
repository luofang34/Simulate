//! Deterministic unit and closed-loop JSBSim bridge-landing tests for `efb_plan`.

use glam::DVec3;
use sim_geo::{LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_render::{Camera, CameraPose};
use sim_scenario::SimTimeScale;

use super::navlog::{export_simulation_link_fragment, import_simulation_link_fragment};
use super::tokens::HoldTurn;
use super::{
    AdHocPickMode, EfbAutopilotPhase, EfbHostEffect, EfbPlanAction, EfbPresetId, EfbWorkspace,
    autocomplete_candidates,
};
use crate::esc_menu::HubCatalog;
use crate::flight::Flight;
use crate::sensor_bus::SensorRealism;
use crate::world_surface::WorldSurfaceHandle;

fn seeded_catalog_and_surface() -> (HubCatalog, WorldSurfaceHandle) {
    let surface = WorldSurfaceHandle::new();
    let mut catalog = HubCatalog::with_global_airports();
    catalog.seed_ad_hoc_presets(&surface);
    (catalog, surface)
}

fn workspace_with_omni(
    line: &str,
) -> (EfbWorkspace, HubCatalog, WorldSurfaceHandle, EfbHostEffect) {
    let (mut catalog, surface) = seeded_catalog_and_surface();
    let mut ws = EfbWorkspace::default();
    let _ = ws.apply_action(
        EfbPlanAction::FocusOmni(true),
        [60.25, 25.04],
        &mut catalog,
        &surface,
    );
    for ch in line.chars() {
        ws.push_omni_char(ch);
    }
    let eff = ws.apply_action(
        EfbPlanAction::SubmitOmni,
        [60.25, 25.04],
        &mut catalog,
        &surface,
    );
    (ws, catalog, surface, eff)
}

#[test]
fn omni_tokens_and_proximity_autocomplete() {
    let (ws, catalog, _surface, eff) = workspace_with_omni("efhf/18 kohtl kreml zmrs/33");
    assert_eq!(eff, EfbHostEffect::RouteUpdated);
    assert_eq!(ws.tokens.len(), 4);
    assert_eq!(
        ws.tokens.first().map(|t| t.chip_label()),
        Some("EFHF/18".to_owned())
    );
    assert_eq!(
        ws.tokens.last().map(|t| t.chip_label()),
        Some("ZMRS/33".to_owned())
    );
    let near_moscow = autocomplete_candidates("ZM", Some([55.75, 37.62]), &catalog, 4);
    assert!(
        near_moscow
            .iter()
            .any(|c| c.insert_text.starts_with("ZMRS"))
    );
}

#[test]
fn holds_direct_to_undo_and_link_v2_round_trip() {
    let (mut ws, mut catalog, surface, _) = workspace_with_omni("efhf/18 kohtl kreml zmrs/33");
    let _ = ws.apply_action(
        EfbPlanAction::SelectChip(2),
        [55.75, 37.62],
        &mut catalog,
        &surface,
    );
    let _ = ws.apply_action(
        EfbPlanAction::CycleHoldSelected,
        [55.75, 37.62],
        &mut catalog,
        &surface,
    );
    assert_eq!(
        ws.tokens.get(2).and_then(|t| t.hold).map(|h| h.turn),
        Some(HoldTurn::Right)
    );
    let frag = export_simulation_link_fragment(&ws.tokens, 750).unwrap_or_default();
    assert!(frag.starts_with("v=2&"));
    let mut next_id = 100;
    let decoded =
        import_simulation_link_fragment(&frag, &mut next_id, &catalog).unwrap_or_default();
    assert_eq!(decoded.len(), 4);
    assert_eq!(
        decoded.get(2).and_then(|t| t.hold).map(|h| h.turn),
        Some(HoldTurn::Right)
    );
    let _ = ws.apply_action(
        EfbPlanAction::DirectToSelected,
        [56.20, 36.80],
        &mut catalog,
        &surface,
    );
    assert_eq!(ws.tokens.len(), 3);
    assert_eq!(
        ws.tokens.first().map(|t| t.ident()),
        Some("PPOS".to_owned())
    );
    let _ = ws.apply_action(
        EfbPlanAction::UndoRoute,
        [56.20, 36.80],
        &mut catalog,
        &surface,
    );
    assert_eq!(ws.tokens.len(), 4);
}

#[test]
fn mathias_rust_preset_navlog_and_rnp_phase_scaling() {
    let (mut catalog, surface) = seeded_catalog_and_surface();
    let mut ws = EfbWorkspace::default();
    let _ = ws.apply_action(
        EfbPlanAction::LoadPreset(EfbPresetId::MathiasRust1987Corridor),
        [60.254, 25.042],
        &mut catalog,
        &surface,
    );
    assert!(ws.tokens.len() >= 7);
    let rows = ws.navlog_rows(105.0);
    assert_eq!(rows.len(), ws.tokens.len().saturating_sub(1));
    let total_nm = rows.last().map_or(0.0, |r| r.cumulative_nm);
    assert!(
        (450.0..=560.0).contains(&total_nm),
        "EFHF -> ZMRS corridor distance expected ~495 NM, got {total_nm:.1} NM"
    );

    // Enroute leg reports RNP 2.0 ENR; final approach leg to ZMRS/15 reports RNP 0.3 APR.
    ws.progress.to_fix = Some(2);
    let enr = ws.active_summary([59.8, 27.5], 105.0);
    assert_eq!(enr.as_ref().map(|s| s.phase_badge), Some("ENR"));

    ws.progress.to_fix = Some(ws.tokens.len().saturating_sub(1));
    let apr = ws.active_summary([55.760, 37.615], 70.0);
    assert_eq!(apr.as_ref().map(|s| s.phase_badge), Some("APR"));
    assert_eq!(apr.as_ref().map(|s| s.rnp_nm), Some(0.3));
}

#[test]
fn two_point_adhoc_runway_delineation_and_3d_raycast() {
    let (mut catalog, surface) = seeded_catalog_and_surface();
    let mut ws = EfbWorkspace::default();

    let eff = ws.apply_action(
        EfbPlanAction::AdHocPickWorld3d,
        [55.75, 37.62],
        &mut catalog,
        &surface,
    );
    assert_eq!(eff, EfbHostEffect::EnterWorld3dPicker);
    assert_eq!(ws.ad_hoc.pick_mode, AdHocPickMode::World3dThresholdA);

    assert!(!ws.ad_hoc.record_point(55.760_0, 37.610_0));
    assert!(ws.ad_hoc.record_point(55.754_5, 37.616_0));
    assert_eq!(ws.ad_hoc.pick_mode, AdHocPickMode::None);

    let created = ws.apply_action(
        EfbPlanAction::AdHocCreateAndAppend,
        [55.75, 37.62],
        &mut catalog,
        &surface,
    );
    assert!(matches!(created, EfbHostEffect::CreatedAdHoc { ref code, .. } if code == "ZZ01"));
    assert!(catalog.ad_hoc_by_code("ZZ01").is_some());

    // Verify 3D cursor raycast directly hits both ZMRS (128 m bridge) and ZMRQ (144 m Red Square plaza).
    assert!(catalog.ad_hoc_by_code("ZMRS").is_some());
    assert!(catalog.ad_hoc_by_code("ZMRQ").is_some());
    let mid = sim_geo::Geodetic::from_degrees(55.750_18, 37.623_63, 428.0);
    let frame = LocalFrame::at(&mid);
    let cam = Camera {
        pose: CameraPose::looking(geodetic_to_ecef(&mid), -frame.up(), frame.north()),
        fov_y_rad: 60.0_f64.to_radians(),
        near_m: 0.5,
    };
    let hit = surface.raycast_cursor_geodetic(&cam, [640.0, 360.0], [1280, 720]);
    assert!(hit.is_some());
    if let Some((g, _msl)) = hit {
        assert!((g.lat_deg() - 55.750_18).abs() < 0.002);
        assert!((g.lon_deg() - 37.623_63).abs() < 0.002);
        assert!(
            (g.height_m - 128.0).abs() < 0.25,
            "offline EllipsoidWgs84 bridge deck expected 128.0 m, got {:.2} m",
            g.height_m
        );
    }

    // Switching to RealWorld (OrthometricMslEgm96) preserves ZMRS at 128.0 m MSL and ZMRQ at 144.0 m MSL.
    #[cfg(not(target_arch = "wasm32"))]
    let real_src = crate::map_rig::ScenerySource::RealWorld { cache_dir: None };
    #[cfg(target_arch = "wasm32")]
    let real_src = crate::map_rig::ScenerySource::RealWorld {};
    assert!(surface.configure(&real_src).is_ok());
    let hit_real = surface.raycast_cursor_geodetic(&cam, [640.0, 360.0], [1280, 720]);
    assert!(hit_real.is_some());
    if let Some((_, msl)) = hit_real {
        assert!(
            (msl - 128.0).abs() < 0.25,
            "RealWorld EGM96 bridge deck expected 128.0 m MSL, got {msl:.2} m"
        );
    }
    let plaza_mid = sim_geo::Geodetic::from_degrees(55.753_88, 37.620_32, 0.0);
    assert!((surface.orthometric_ground_m(&plaza_mid).unwrap_or(0.0) - 144.0).abs() < 0.25);
}

#[test]
fn closed_loop_c172_lands_and_stops_on_moskvoretsky_bridge_and_red_square_plaza() {
    for (preset, exp_code, exp_rwy, station_m, max_along_m, half_w_m) in [
        (
            EfbPresetId::RedSquareBridgeFinal,
            "ZMRS",
            "33",
            -1_650.0,
            445.0,
            14.0,
        ),
        (
            EfbPresetId::RedSquarePlazaFinal,
            "ZMRQ",
            "30",
            -1_450.0,
            288.0,
            18.0,
        ),
    ] {
        let (mut catalog, surface) = seeded_catalog_and_surface();
        let mut flight = Flight::with_surface(surface.clone())
            .unwrap_or_else(|e| unreachable_in_test(&e.to_string()));
        flight
            .select_vehicle(crate::vehicles::FreeFlightVehicle::JsbsimC172p)
            .unwrap_or_else(|e| unreachable_in_test(&e.to_string()));
        assert_eq!(
            flight
                .liveries()
                .active_livery("c172p")
                .map(|(_, l)| l.tail_code.as_str()),
            Some("D-ECJB")
        );
        flight.set_sensor_realism(SensorRealism::Ideal);

        let mut ws = EfbWorkspace::default();
        let effect = ws.apply_action(
            EfbPlanAction::LoadPreset(preset),
            [55.735, 37.630],
            &mut catalog,
            &surface,
        );
        assert!(matches!(
            effect,
            EfbHostEffect::SpawnAtRunway {
                ref code,
                ref designator,
                engage_autoland: true,
                ..
            } if code == exp_code && designator == exp_rwy
        ));

        let (record, rwy) = ws
            .destination_landing_runway(&catalog, &surface)
            .unwrap_or_else(|| unreachable_in_test("missing Moscow landing strip"));
        flight
            .spawn_at_landing_station(&record, exp_rwy, station_m)
            .unwrap_or_else(|e| unreachable_in_test(&e.to_string()));
        ws.autopilot.engage();
        flight.update_system_controls(|sys| {
            sys.flaps_norm = 0.67;
            sys.wheel_brake_norm = 0.0;
        });

        run_bridge_autoland_loop(
            &mut flight,
            &mut ws,
            &rwy,
            &surface,
            (max_along_m, half_w_m),
        );
    }
}

fn run_bridge_autoland_loop(
    flight: &mut Flight,
    ws: &mut EfbWorkspace,
    rwy: &sim_scenery::LandingRunwayGeometry,
    surface: &WorldSurfaceHandle,
    (max_along_m, half_w_m): (f64, f64),
) {
    let dt_s = 1.0 / 60.0;
    let thr_ecef = geodetic_to_ecef(&rwy.threshold);
    let thr_frame = LocalFrame::at(&rwy.threshold);
    let (sin_b, cos_b) = rwy.true_bearing_rad.sin_cos();
    let mut saw_approach = false;
    let mut saw_rollout = false;
    let mut min_deck_clearance_m = f64::INFINITY;

    for _ in 0..(95 * 60) {
        let state = flight.estimated_vehicle_state();
        let art = flight.articulation();
        let geo = ecef_to_geodetic(state.position_ecef);
        ws.update_sequencing([geo.lat_deg(), geo.lon_deg()]);
        let undul = surface.geoid_undulation_m(geo.lat_rad, geo.lon_rad);
        let Some(out) = ws.autopilot.step(
            &state,
            &art,
            &ws.tokens,
            ws.progress,
            Some(rwy),
            undul,
            dt_s,
        ) else {
            break;
        };
        saw_approach |= out.phase == EfbAutopilotPhase::Approach;
        saw_rollout |= out.phase == EfbAutopilotPhase::Rollout;
        flight.update_system_controls(|sys| {
            sys.flaps_norm = out.flaps_norm;
            sys.wheel_brake_norm = out.wheel_brake_norm;
        });
        assert!(
            flight
                .advance_by(dt_s, out.controls, SimTimeScale::RealTime)
                .is_ok()
        );

        let truth = flight.vehicle_state();
        let tg = ecef_to_geodetic(truth.position_ecef);
        min_deck_clearance_m = min_deck_clearance_m.min(tg.height_m - rwy.threshold.height_m);
        if saw_rollout && truth.velocity_ecef.length() < 0.35 {
            break;
        }
    }

    let end_state = flight.vehicle_state();
    let enu: DVec3 = thr_frame.ecef_to_enu_vector(end_state.position_ecef - thr_ecef);
    let along_m = enu.y * cos_b + enu.x * sin_b;
    let right_m = enu.x * cos_b - enu.y * sin_b;
    let end_speed = end_state.velocity_ecef.length();

    assert!(saw_approach, "autopilot should capture Approach phase");
    assert!(saw_rollout, "autopilot should touch down and enter Rollout");
    assert!(
        min_deck_clearance_m > 0.4,
        "CG should remain cleanly above 128 m bridge deck (min clearance {min_deck_clearance_m:.2} m)"
    );
    assert!(
        (10.0..=max_along_m).contains(&along_m),
        "aircraft must stop within {max_along_m:.0} m strip, stopped at along_m = {along_m:.1} m"
    );
    assert!(
        right_m.abs() < half_w_m,
        "aircraft must stay within strip half-width {half_w_m:.1} m, right_m = {right_m:.2} m"
    );
    assert!(
        end_speed < 0.5,
        "aircraft must brake to a full stop on the bridge, speed = {end_speed:.2} m/s"
    );
}

fn unreachable_in_test(msg: &str) -> ! {
    assert_eq!(msg, "");
    loop {
        std::hint::spin_loop();
    }
}
