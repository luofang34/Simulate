//! Polar `NZSP RWY 02T` (short final, landed threshold, World-tab globe) and Retina
//! `KDOV Z10.6` expanded-EFB verification shots.

use std::path::{Path, PathBuf};

use sim_core::ControlInputs;
use sim_maplibre::EfbDisplayMode;
use sim_render::{OverlayQuad, ViewportRect};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::{AerodromeId, AirportDatabase};

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::error::AppSimError;
use crate::esc_menu::{HubTab, HubView, MenuInput, OverlayContent, SpawnMode};
use crate::gallery::readback_target_rgba;
use crate::map_rig::{AvionicsPreset, EfbCommand, GlobeSelection};
use crate::sky_rig::SkyFrame;

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
    let austral_summer_sun = glam::DVec3::new(0.45, 0.35, -0.55).normalize();
    h.settle_and_draw_flight_with_sun(
        austral_summer_sun,
        &dir.join("13_nzsp_rwy02t_short_final_hud_globe_efb.png"),
    )?;

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
