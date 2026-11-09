//! End-to-end polar `NZSP RWY 02T` validation: `World` tab selection & `fly_to` globe
//! picker, `ShortFinal` spawn on the opposite side of `90° S`, polar `TRU` heading
//! reversion, conformal HUD + HSI alignment across the South Pole, Globe EFB minimap +
//! 2D chart rendering, and closed-loop ILS approach to touchdown and rollout on `RWY 02T`.

use glam::DVec3;
use sim_core::{ControlInputs, VehicleState};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_indicate::HeadingReference;
use sim_maplibre::HostGpu;
use sim_render::{HeadlessGpu, VectorScenePass};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::{AirportDatabase, AirportIdx, AirportRecord};

use crate::cockpit_rig::CockpitRig;
use crate::error::AppSimError;
use crate::esc_menu::{
    EscMenu, HubCatalog, HubTab, HubView, MenuInput, MenuStatus, SpawnMode, WidgetId,
};
use crate::flight::Flight;
use crate::map_rig::{AvionicsPreset, GlobeSelection, ScenerySource};
use crate::sky_rig::SkyFrame;
use crate::vehicles::{C172_GEAR_HEIGHT_M, FreeFlightVehicle};
use crate::world_surface::WorldSurfaceHandle;

const VIEWPORT_SIZE: [u32; 2] = [1280, 720];

struct RunwayFrame {
    thr_ecef: DVec3,
    along_ecef: DVec3,
    across_ecef: DVec3,
    thr_ellip_m: f64,
}

fn angle_diff_rad(a: f32, b: f32) -> f32 {
    let d = (a - b).rem_euclid(std::f32::consts::TAU);
    d.min(std::f32::consts::TAU - d)
}

fn autoland_c172(state: &VehicleState, rwy: &RunwayFrame, ground_ellip_m: f64) -> ControlInputs {
    let geo = ecef_to_geodetic(state.position_ecef);
    let local = LocalFrame::at(&geo);
    let rel = state.position_ecef - rwy.thr_ecef;
    let s_m = rel.dot(rwy.along_ecef);
    let xtk_m = rel.dot(rwy.across_ecef);
    let v_along = state.velocity_ecef.dot(rwy.along_ecef);
    let v_across = state.velocity_ecef.dot(rwy.across_ecef);
    let v_up = state.velocity_ecef.dot(local.up());
    let wheel_agl_m = (geo.height_m - ground_ellip_m - C172_GEAR_HEIGHT_M).max(0.0);

    let nose = state.attitude * DVec3::X;
    let right_wing = state.attitude * DVec3::Y;
    let pitch_rad = nose.dot(local.up()).clamp(-1.0, 1.0).asin();
    let bank_rad = (-right_wing.dot(local.up())).clamp(-1.0, 1.0).asin();

    let on_ground = wheel_agl_m < 0.20 && s_m > -50.0;
    let target_v_across = (-0.08 * xtk_m).clamp(-10.0, 10.0);
    let target_bank = if on_ground {
        0.0
    } else {
        (0.045 * (target_v_across - v_across)).clamp(-0.35, 0.35)
    };
    let roll = (2.4 * (target_bank - bank_rad) - 0.55 * state.angular_rate_body.x).clamp(-1.0, 1.0);
    let hdg_err = nose.dot(rwy.across_ecef);
    let yaw = if on_ground {
        (-1.8 * hdg_err - 0.015 * xtk_m - 0.5 * state.angular_rate_body.z).clamp(-1.0, 1.0)
    } else {
        (-0.35 * state.angular_rate_body.z).clamp(-1.0, 1.0)
    };
    if on_ground {
        return ControlInputs {
            roll,
            pitch: (-1.5 * pitch_rad - 0.4 * state.angular_rate_body.y).clamp(-0.4, 0.4),
            yaw,
            throttle: 0.0,
        };
    }

    let (target_v_up, throttle) = if wheel_agl_m < 12.0 && s_m > -150.0 {
        ((-0.14 * wheel_agl_m - 0.35).clamp(-1.8, -0.2), 0.0)
    } else {
        let gs_tan = 3.0_f64.to_radians().tan();
        let target_h = rwy.thr_ellip_m + C172_GEAR_HEIGHT_M + (260.0 - s_m).max(20.0) * gs_tan;
        let vz_cmd =
            -v_along.max(25.0) * gs_tan - (0.16 * (geo.height_m - target_h)).clamp(-3.5, 3.5);
        (vz_cmd, (0.48 + 0.035 * (42.0 - v_along)).clamp(0.05, 0.85))
    };
    let target_pitch = (0.045 + 0.032 * (target_v_up - v_up)).clamp(-0.18, 0.22);
    let pitch =
        (2.8 * (target_pitch - pitch_rad) - 0.65 * state.angular_rate_body.y).clamp(-1.0, 1.0);
    ControlInputs {
        roll,
        pitch,
        yaw,
        throttle,
    }
}

fn verify_world_hub_nzsp() {
    let catalog = HubCatalog::with_global_airports();
    let mut menu = EscMenu::default();
    let status = MenuStatus::default();
    let v0 = HubView::build(&menu, &catalog, &status, VIEWPORT_SIZE);
    let _ = menu.handle(MenuInput::Toggle, &v0, &catalog);
    let _ = menu.handle(MenuInput::Tab(HubTab::World), &v0, &catalog);
    for ch in "NZSP".chars() {
        let v = HubView::build(&menu, &catalog, &status, VIEWPORT_SIZE);
        let _ = menu.handle(MenuInput::Char(ch), &v, &catalog);
    }
    let view = HubView::build(&menu, &catalog, &status, VIEWPORT_SIZE);
    let selected = catalog
        .aerodromes
        .get(menu.aerodrome())
        .expect("NZSP selected");
    assert_eq!(selected.code, "NZSP");
    let rwy_chip = view.find(WidgetId::RunwayEnd(0)).expect("RWY 02T chip");
    assert_eq!(rwy_chip.label, "RWY 02T 020T");
}

#[cfg(not(target_arch = "wasm32"))]
fn build_gpu_harness(
    nzsp_idx: AirportIdx,
    nzsp_rec: &AirportRecord,
    flight: &mut Flight,
    surface: &WorldSurfaceHandle,
) -> Result<Option<(HeadlessGpu, CockpitRig)>, AppSimError> {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return Ok(None);
    };
    if gpu.backend == wgpu::Backend::Gl {
        return Ok(None);
    }
    let host_gpu = HostGpu {
        instance: gpu.instance.clone(),
        adapter: gpu.adapter.clone(),
        device: gpu.device.clone(),
        queue: gpu.queue.clone(),
    };
    let hud_pass = pollster::block_on(VectorScenePass::new(&gpu.device))?;
    let mut rig = CockpitRig::new_with_gpu(
        host_gpu,
        VIEWPORT_SIZE,
        Some(ScenerySource::Catalog),
        hud_pass,
        surface.clone(),
    )?;
    rig.set_avionics_preset(AvionicsPreset::GlassCockpit)?;
    rig.supervisor_mut().set_free_airport(nzsp_idx, 0);
    flight.spawn_at_airport_record(nzsp_rec, 0, SpawnMode::ShortFinal)?;
    if let Some(maps) = rig.maps_mut() {
        let (kewr_idx, kewr_rec) = AirportDatabase::builtin().by_code("KEWR").expect("KEWR");
        maps.render_globe_picker(
            [560, 350],
            Some(GlobeSelection {
                lat_deg: kewr_rec.lat_deg,
                lon_deg: kewr_rec.lon_deg,
                airport: Some(kewr_idx),
                runway: Some("22L".to_owned()),
            }),
        )?;
        maps.render_globe_picker(
            [560, 350],
            Some(GlobeSelection {
                lat_deg: nzsp_rec.lat_deg,
                lon_deg: nzsp_rec.lon_deg,
                airport: Some(nzsp_idx),
                runway: Some("02T".to_owned()),
            }),
        )?;
        assert!(
            maps.globe_needs_redraw(),
            "switching KEWR -> NZSP starts smooth fly_to animation"
        );
        assert!(maps.globe_view().is_some());
    }
    Ok(Some((gpu, rig)))
}

#[cfg(not(target_arch = "wasm32"))]
fn verify_gpu_polar_cockpit(
    gpu: &HeadlessGpu,
    rig: &mut CockpitRig,
    flight: &mut Flight,
    check_chart_map: bool,
) -> Result<(), AppSimError> {
    rig.supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    let cam = flight.display_camera(OperatorViewportMode::Cockpit)?;
    let sky = SkyFrame::of_world(flight.world());
    let exposures = flight.drain_sensor_exposures();
    let _ = rig.prepare_frame(&gpu.queue, VIEWPORT_SIZE, flight, &cam, &sky, &exposures)?;
    assert_eq!(
        rig.avionics().last_heading_reference(),
        Some(HeadingReference::True),
        "polar approach must revert PFD/HSI to TRU"
    );
    if let Some((hdg, Some(crs))) = rig.avionics().last_heading_and_course_rad() {
        assert!(
            angle_diff_rad(hdg, crs).to_degrees() < 6.0,
            "HSI course ({:.1} deg) must align with TRU heading ({:.1} deg)",
            crs.to_degrees(),
            hdg.to_degrees()
        );
    }
    assert!(
        rig.avionics().hud_view().is_some(),
        "conformal HUD rendered"
    );
    if let Some(maps) = rig.maps() {
        assert!(maps.efb_view().is_some(), "Globe EFB minimap rendered");
        assert_eq!(
            maps.efb_status().map(|s| s.airport),
            Some("NZSP".to_owned())
        );
    }
    if check_chart_map {
        rig.supervisor_mut()
            .set_viewport_mode(OperatorViewportMode::ChartMap);
        rig.invalidate_chart();
        let chart_cam = flight.display_camera(OperatorViewportMode::ChartMap)?;
        let _ = rig.prepare_frame(&gpu.queue, VIEWPORT_SIZE, flight, &chart_cam, &sky, &[])?;
        rig.supervisor_mut()
            .set_viewport_mode(OperatorViewportMode::Cockpit);
    }
    Ok(())
}

fn build_nzsp_runway_frame(nzsp_rec: &AirportRecord, surface: &WorldSurfaceHandle) -> RunwayFrame {
    let datum = surface.vertical_datum();
    let rwy_spec = *nzsp_rec
        .to_flightbag_graph(datum)
        .runway_ends
        .first()
        .expect("NZSP RWY 02T");
    assert_eq!(rwy_spec.designator.as_str(), "02T");
    let thr_geo = Geodetic::from_degrees(
        rwy_spec.threshold.latitude_deg,
        rwy_spec.threshold.longitude_deg,
        rwy_spec.threshold.ellipsoidal_height_m,
    );
    let thr_ellip_m = surface
        .ellipsoidal_ground_m(&thr_geo)
        .expect("NZSP RWY 02T strip registered");
    assert!((thr_ellip_m - thr_geo.height_m).abs() < 1.0);
    let thr_local = LocalFrame::at(&thr_geo);
    let (sin_c, cos_c) = rwy_spec.true_bearing_rad.sin_cos();
    RunwayFrame {
        thr_ecef: geodetic_to_ecef(&Geodetic {
            height_m: thr_ellip_m,
            ..thr_geo
        }),
        along_ecef: thr_local.north() * cos_c + thr_local.east() * sin_c,
        across_ecef: thr_local.east() * cos_c - thr_local.north() * sin_c,
        thr_ellip_m,
    }
}

fn advance_autoland_step(
    flight: &mut Flight,
    rwy: &RunwayFrame,
    surface: &WorldSurfaceHandle,
) -> Result<(f64, f64, f64, f64), AppSimError> {
    let state = flight.vehicle_state();
    let geo = ecef_to_geodetic(state.position_ecef);
    let ground_m = surface
        .ellipsoidal_ground_m(&geo)
        .unwrap_or(rwy.thr_ellip_m);
    let cmd = autoland_c172(&state, rwy, ground_m);
    flight.advance_by(0.10, cmd, SimTimeScale::RealTime)?;
    let post = flight.vehicle_state();
    let post_geo = ecef_to_geodetic(post.position_ecef);
    let rel = post.position_ecef - rwy.thr_ecef;
    let s_m = rel.dot(rwy.along_ecef);
    let xtk_m = rel.dot(rwy.across_ecef);
    let wheel_agl_m = post_geo.height_m - ground_m - C172_GEAR_HEIGHT_M;
    Ok((s_m, xtk_m, wheel_agl_m, post_geo.lon_rad.to_degrees()))
}

#[test]
fn nzsp_rwy_02t_world_hub_and_closed_loop_crossing_south_pole_to_landing() -> Result<(), AppSimError>
{
    verify_world_hub_nzsp();
    let (nzsp_idx, nzsp_rec) = AirportDatabase::builtin().by_code("NZSP").expect("NZSP");
    let surface = WorldSurfaceHandle::new();
    let mut flight = Flight::with_surface(surface.clone())?;
    flight.select_vehicle(FreeFlightVehicle::JsbsimC172)?;
    flight.spawn_at_airport_record(nzsp_rec, 0, SpawnMode::ShortFinal)?;
    #[cfg(not(target_arch = "wasm32"))]
    let mut gpu_harness = build_gpu_harness(nzsp_idx, nzsp_rec, &mut flight, &surface)?;

    let rwy = build_nzsp_runway_frame(nzsp_rec, &surface);
    let start = flight.vehicle_state();
    assert!(ecef_to_geodetic(start.position_ecef).lon_rad.to_degrees() < 0.0);
    assert!((start.attitude * DVec3::X).dot(rwy.along_ecef) > 0.995);

    let (mut crossed_pole, mut touchdown_s_m, mut checked_rollout_gpu) = (false, None, false);
    for step in 0..1_800_u32 {
        let (s_m, xtk_m, wheel_agl_m, lon_deg) =
            advance_autoland_step(&mut flight, &rwy, &surface)?;
        crossed_pole |= lon_deg > 90.0;
        if s_m > 0.0 && wheel_agl_m < 0.15 && touchdown_s_m.is_none() {
            touchdown_s_m = Some(s_m);
        }
        let rollout_ready =
            touchdown_s_m.is_some_and(|td| s_m > td + 150.0) && !checked_rollout_gpu;
        #[cfg(not(target_arch = "wasm32"))]
        if (step == 0 || step == 1_050 || rollout_ready)
            && let Some((gpu, rig)) = gpu_harness.as_mut()
        {
            verify_gpu_polar_cockpit(gpu, rig, &mut flight, step == 0)?;
            checked_rollout_gpu |= rollout_ready;
        }
        #[cfg(target_arch = "wasm32")]
        if rollout_ready {
            checked_rollout_gpu = true;
        }
        assert!(xtk_m.abs() < 22.0, "s={s_m:.1}m xtk={xtk_m:.2}m");
        if touchdown_s_m.is_some_and(|td| s_m > td + 220.0) {
            break;
        }
    }
    assert!(crossed_pole && touchdown_s_m.is_some());
    let end_geo = ecef_to_geodetic(flight.vehicle_state().position_ecef);
    let end_ground = surface.ellipsoidal_ground_m(&end_geo).expect("on RWY 02T");
    assert!((end_geo.height_m - end_ground - C172_GEAR_HEIGHT_M).abs() < 0.25);
    #[cfg(not(target_arch = "wasm32"))]
    if let Some((gpu, _)) = gpu_harness {
        gpu.check_device()?;
    }
    let _ = (nzsp_idx, checked_rollout_gpu);
    Ok(())
}
