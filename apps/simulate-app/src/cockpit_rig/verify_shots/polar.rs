//! Polar `NZSP RWY 02T` (short final, landed threshold, World-tab globe) and Retina
//! `KDOV Z10.6` expanded-EFB verification shots.

use std::path::{Path, PathBuf};

use sim_core::ControlInputs;
use sim_maplibre::EfbDisplayMode;
use sim_render::{OverlayQuad, ViewportRect};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::{AerodromeId, AirportDatabase};

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::camera_view::LookOffset;
use crate::error::AppSimError;
use crate::esc_menu::{HubTab, HubView, MenuInput, OverlayContent, SpawnMode};
use crate::gallery::readback_target_rgba;
use crate::map_rig::{AvionicsPreset, EfbCommand, GlobeSelection};
use crate::sky_rig::SkyFrame;
use crate::vehicles::FreeFlightVehicle;

#[test]
#[ignore = "writes 1280x720 polar NZSP and Retina KDOV Z10.6 EFB PNGs into $SIMULATE_VERIFY_OUT"]
fn render_polar_nzsp_and_world_tab_verification_shots() -> Result<(), AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    std::fs::create_dir_all(&dir).map_err(|e| AppSimError::Screenshot {
        detail: e.to_string(),
    })?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.rig.set_avionics_preset(AvionicsPreset::GlassCockpit)?;
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    h.rig.set_show_conformal_hud(true);

    render_nzsp_shots(&mut h, &dir)?;
    render_nzsp_polar_efb_zoomout(&mut h, &dir)?;
    render_kdov_z10_6_expanded_efb(&mut h, &dir)?;
    render_nzsp_world_tab(&mut h, &dir)?;
    h.gpu.check_device()?;
    Ok(())
}

fn render_nzsp_polar_efb_zoomout(h: &mut VerifyHarness, dir: &Path) -> Result<(), AppSimError> {
    let (nzsp_idx, nzsp_rec) =
        AirportDatabase::builtin()
            .by_code("NZSP")
            .ok_or_else(|| AppSimError::Screenshot {
                detail: "missing NZSP in AirportDatabase".to_owned(),
            })?;
    h.rig.supervisor_mut().set_free_airport(nzsp_idx, 0);
    h.flight
        .spawn_at_airport_record(nzsp_rec, 0, SpawnMode::Threshold)?;
    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb))?;
    // Zoom from default 11.6 down to 4.4 so the 85.05S Ross Sea coast and 90S pole both
    // sit inside the compass rose, matching the user's polar minimap screenshot.
    h.rig.efb_command(EfbCommand::ZoomBy(-7.2))?;
    h.rig.efb_command(EfbCommand::PanPixels([80.0, -20.0]))?;
    h.settle_and_draw_flight(&dir.join("16_nzsp_expanded_efb_polar_ross_sea.png"))?;
    h.rig.efb_command(EfbCommand::Recenter)?;
    h.rig.efb_command(EfbCommand::ZoomBy(7.2))?;
    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner))?;
    Ok(())
}

fn render_nzsp_shots(h: &mut VerifyHarness, dir: &Path) -> Result<(), AppSimError> {
    let (nzsp_idx, nzsp_rec) =
        AirportDatabase::builtin()
            .by_code("NZSP")
            .ok_or_else(|| AppSimError::Screenshot {
                detail: "missing NZSP in AirportDatabase".to_owned(),
            })?;
    h.rig.supervisor_mut().set_free_airport(nzsp_idx, 0);
    h.flight
        .spawn_at_airport_record(nzsp_rec, 0, SpawnMode::ShortFinal)?;
    let austral_summer_sun = glam::DVec3::new(0.45, 0.35, -0.55).normalize();
    h.settle_and_draw_flight_with_sun(
        austral_summer_sun,
        &dir.join("13a_nzsp_rwy02t_short_final_level_spawn.png"),
    )?;
    assert_nzsp_short_final_gap_free(h, "level spawn")?;

    for _ in 0..30 {
        h.flight.advance_by(
            0.10,
            ControlInputs {
                throttle: 0.62,
                ..ControlInputs::default()
            },
            SimTimeScale::RealTime,
        )?;
    }
    h.settle_and_draw_flight_with_sun(
        austral_summer_sun,
        &dir.join("13_nzsp_rwy02t_short_final_hud_globe_efb.png"),
    )?;
    assert_nzsp_short_final_gap_free(h, "3s approach")?;

    h.flight
        .spawn_at_airport_record(nzsp_rec, 0, SpawnMode::Threshold)?;
    for _ in 0..25 {
        h.flight
            .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
    }
    h.settle_and_draw_flight_with_sun(
        austral_summer_sun,
        &dir.join("14_nzsp_rwy02t_threshold_cockpit_hud.png"),
    )
}

fn assert_nzsp_short_final_gap_free(h: &VerifyHarness, stage: &str) -> Result<(), AppSimError> {
    let rgba = readback_target_rgba(&h.gpu.device, &h.gpu.queue, &h.target)?;
    let w = SHOT_SIZE[0] as usize;
    // Unobstructed OTW snowfield band below the horizon and above both PFD and HSI overlays.
    for (y, row) in rgba
        .as_chunks::<4>()
        .0
        .chunks_exact(w)
        .enumerate()
        .skip(395)
        .take(70)
    {
        for (x, &[r, g, b, _]) in row.iter().enumerate() {
            assert!(
                !(r < 60 && g < 60 && b < 80),
                "NZSP ({stage}): dark #0b1626 withheld-drape wedge pixel [{r},{g},{b}] at ({x},{y})"
            );
            assert!(
                r <= b.saturating_add(12),
                "NZSP ({stage}): atmospheric limb leaked through culled polar wedge [{r},{g},{b}] at ({x},{y})"
            );
        }
    }
    Ok(())
}

fn render_kdov_z10_6_expanded_efb(h: &mut VerifyHarness, dir: &Path) -> Result<(), AppSimError> {
    h.rig.set_pixel_ratio(2.0);
    h.rig
        .supervisor_mut()
        .set_free_aerodrome(AerodromeId::DoverKdov);
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::DoverKdov).cloned() {
        h.flight
            .spawn_at_aerodrome(&diag, 0, SpawnMode::ShortFinal)?;
    }
    for _ in 0..20 {
        h.flight.advance_by(
            0.10,
            ControlInputs {
                throttle: 0.62,
                ..ControlInputs::default()
            },
            SimTimeScale::RealTime,
        )?;
    }
    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb))?;
    if h.rig
        .maps()
        .and_then(|m| m.efb_status())
        .is_some_and(|s| s.north_up)
    {
        h.rig.efb_command(EfbCommand::ToggleNorthUp)?;
    }
    h.rig.efb_command(EfbCommand::ZoomBy(-1.0))?;
    // First verify Retina 2x pixel_ratio stability across consecutive frames.
    h.settle_and_draw_flight(&dir.join("15_kdov_expanded_efb_z10_6_plate_diag.png"))?;
    assert_zero_efb_flicker(h, "Retina 2x")?;

    // Then capture the full-frame 1280x720 logical-viewport Z10.6 EFB matching the user's framing,
    // and assert consecutive-frame stability there too.
    h.rig.set_pixel_ratio(1.0);
    h.settle_and_draw_flight(&dir.join("15_kdov_expanded_efb_z10_6_plate_diag.png"))?;
    assert_zero_efb_flicker(h, "1x logical")?;
    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner))?;
    Ok(())
}

fn assert_zero_efb_flicker(h: &mut VerifyHarness, label: &str) -> Result<(), AppSimError> {
    let frame_a = step_and_readback_composite(h)?;
    let frame_b = step_and_readback_composite(h)?;
    let diff = frame_a
        .as_chunks::<4>()
        .0
        .iter()
        .zip(frame_b.as_chunks::<4>().0)
        .filter(|(p, q)| p != q)
        .count();
    assert_eq!(
        diff, 0,
        "Expanded EFB at Z10.6 ({label}) must be 100% stable across consecutive frames, got {diff} differing pixels"
    );
    Ok(())
}

fn step_and_readback_composite(h: &mut VerifyHarness) -> Result<Vec<u8>, AppSimError> {
    let sky = SkyFrame::fixed_sun(h.flight.world().sun_direction_ecef());
    let mode = h.rig.supervisor().viewport_mode();
    let cam = h.flight.display_camera(mode)?;
    h.rig.efb_command(EfbCommand::ZoomBy(0.0))?;
    let _globe = h
        .rig
        .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
    poll_gpu(&h.gpu)?;
    let chrome = h.rig.efb_chrome(h.flight.world());
    h.overlay.render(
        &h.gpu.device,
        &h.gpu.queue,
        OverlayContent::FlightChrome {
            banner: None,
            efb: chrome.as_ref(),
        },
        SHOT_SIZE,
    )?;
    let mut quads = h.rig.overlay_quads(h.flight.world());
    if let Some(menu) = h.overlay.view() {
        quads.push(OverlayQuad::vector_overlay(ViewportRect::FULL, menu));
    }
    let mut encoder = h
        .gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("verify efb stability"),
        });
    h.compositor.encode(
        &h.gpu.device,
        &h.gpu.queue,
        &mut encoder,
        sim_render::CompositorTarget {
            view: &h.target_view,
            clear: Some(wgpu::Color::BLACK),
            size_px: SHOT_SIZE,
        },
        &quads,
    );
    h.gpu.queue.submit([encoder.finish()]);
    readback_target_rgba(&h.gpu.device, &h.gpu.queue, &h.target)
}

fn render_nzsp_world_tab(h: &mut VerifyHarness, dir: &Path) -> Result<(), AppSimError> {
    let (nzsp_idx, nzsp_rec) =
        AirportDatabase::builtin()
            .by_code("NZSP")
            .ok_or_else(|| AppSimError::Screenshot {
                detail: "missing NZSP in AirportDatabase".to_owned(),
            })?;
    h.rig.supervisor_mut().set_free_airport(nzsp_idx, 0);
    let status = h.menu_status();
    let v0 = HubView::build(&h.menu, &h.catalog, &status, SHOT_SIZE);
    let _ = h.menu.handle(MenuInput::Toggle, &v0, &h.catalog);
    let _ = h
        .menu
        .handle(MenuInput::Tab(HubTab::World), &v0, &h.catalog);
    for ch in "NZSP".chars() {
        let v = HubView::build(&h.menu, &h.catalog, &status, SHOT_SIZE);
        let _ = h.menu.handle(MenuInput::Char(ch), &v, &h.catalog);
    }
    let globe_sel = Some(GlobeSelection {
        lat_deg: nzsp_rec.lat_deg,
        lon_deg: nzsp_rec.lon_deg,
        airport: Some(nzsp_idx),
        runway: Some("02T".to_owned()),
    });
    h.draw_hub_tab(
        HubTab::World,
        globe_sel,
        &dir.join("12_hub_world_tab_overhaul_nzsp.png"),
    )
}

#[test]
#[ignore = "writes 1280x720 P-38G third-person verification PNGs into $SIMULATE_VERIFY_OUT"]
fn render_p38_third_person_verification_shots() -> Result<(), AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    std::fs::create_dir_all(&dir).map_err(|e| AppSimError::Screenshot {
        detail: e.to_string(),
    })?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.flight.select_vehicle(FreeFlightVehicle::JsbsimP38g)?;
    h.menu
        .sync_vehicle(&h.catalog, FreeFlightVehicle::JsbsimP38g);
    h.draw_hub_tab(HubTab::Hangar, None, &dir.join("p38_01_hangar.png"))?;
    if h.menu.is_open() {
        let status = h.menu_status();
        let v = HubView::build(&h.menu, &h.catalog, &status, SHOT_SIZE);
        let _ = h.menu.handle(MenuInput::Toggle, &v, &h.catalog);
    }

    h.rig.set_show_instruments(false);
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    h.rig
        .supervisor_mut()
        .set_free_aerodrome(AerodromeId::NewarkKewr);
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::NewarkKewr).cloned() {
        h.flight
            .spawn_at_aerodrome(&diag, 0, SpawnMode::Threshold)?;
    }
    h.flight.update_system_controls(|sys| {
        sys.landing_light = true;
        sys.coolant_flap_norm = [0.85, 0.85];
        sys.oil_shutter_norm = [0.80, 0.80];
    });
    for _ in 0..35 {
        h.flight
            .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
    }
    draw_p38_chase_orbit(
        &mut h,
        0.0,
        -0.05,
        0.68,
        &dir.join("p38_02_chase_rear_runway.png"),
    )?;
    draw_p38_chase_orbit(
        &mut h,
        2.35,
        -0.08,
        0.58,
        &dir.join("p38_03_chase_front_quarter.png"),
    )?;
    draw_p38_chase_orbit(
        &mut h,
        1.50,
        -0.08,
        0.62,
        &dir.join("p38_04_chase_side.png"),
    )?;
    render_p38_airborne_and_feathered_shots(&mut h, &dir)?;
    h.gpu.check_device()?;
    Ok(())
}

fn render_p38_airborne_and_feathered_shots(
    h: &mut VerifyHarness,
    dir: &Path,
) -> Result<(), AppSimError> {
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::NewarkKewr).cloned() {
        h.flight
            .spawn_at_aerodrome(&diag, 0, SpawnMode::ShortFinal)?;
    }
    h.flight.update_system_controls(|sys| {
        sys.gear_down = false;
        sys.flaps_norm = 0.20;
        sys.landing_light = false;
        sys.coolant_flap_norm = [0.35, 0.35];
        sys.oil_shutter_norm = [0.35, 0.35];
    });
    for _ in 0..25 {
        h.flight.advance_by(
            0.05,
            ControlInputs {
                roll: 0.12,
                pitch: 0.04,
                yaw: -0.02,
                throttle: 0.82,
            },
            SimTimeScale::RealTime,
        )?;
    }
    draw_p38_chase_orbit(h, 0.55, -0.08, 0.62, &dir.join("p38_05_chase_airborne.png"))?;

    h.flight.update_system_controls(|sys| {
        sys.prop_feather = [true, false];
        sys.throttle_split = Some([0.0, 0.92]);
        sys.trim_norm = [0.15, 0.05, 0.45];
    });
    for _ in 0..25 {
        h.flight.advance_by(
            0.05,
            ControlInputs {
                roll: -0.10,
                pitch: 0.03,
                yaw: 0.28,
                throttle: 0.88,
            },
            SimTimeScale::RealTime,
        )?;
    }
    draw_p38_chase_orbit(
        h,
        2.50,
        -0.06,
        0.58,
        &dir.join("p38_06_single_engine_feathered.png"),
    )
}

fn draw_p38_chase_orbit(
    h: &mut VerifyHarness,
    yaw_rad: f64,
    pitch_rad: f64,
    dist_scale: f64,
    out: &Path,
) -> Result<(), AppSimError> {
    let sky = SkyFrame::fixed_sun(h.flight.world().sun_direction_ecef());
    let look = LookOffset { yaw_rad, pitch_rad };
    for _ in 0..35 {
        let state = h.flight.display_vehicle_state();
        let base_cam = h.flight.display_camera(OperatorViewportMode::Chase)?;
        let mut cam = look.apply(OperatorViewportMode::Chase, base_cam, &state);
        cam.pose.position_ecef =
            state.position_ecef + (cam.pose.position_ecef - state.position_ecef) * dist_scale;
        h.rig.invalidate_chart();
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    let chrome = h.rig.efb_chrome(h.flight.world());
    h.overlay.render(
        &h.gpu.device,
        &h.gpu.queue,
        OverlayContent::FlightChrome {
            banner: None,
            efb: chrome.as_ref(),
        },
        SHOT_SIZE,
    )?;
    let mut quads = h.rig.overlay_quads(h.flight.world());
    if let Some(menu) = h.overlay.view() {
        quads.push(OverlayQuad::vector_overlay(ViewportRect::FULL, menu));
    }
    h.composite_to_png(&quads, out)
}
