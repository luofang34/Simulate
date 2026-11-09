//! Lockheed P-38G verification shots in every simulator view: the hangar
//! studio, chase orbits on the runway and airborne, a feathered engine, the
//! cockpit out-the-window view, the mounted sensor aperture and the chart map.

use std::path::{Path, PathBuf};

use sim_core::ControlInputs;
use sim_render::{OverlayQuad, ViewportRect};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::AerodromeId;

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::camera_view::LookOffset;
use crate::error::AppSimError;
use crate::esc_menu::{HubTab, HubView, MenuInput, OverlayContent, SpawnMode};
use crate::sky_rig::SkyFrame;
use crate::vehicles::FreeFlightVehicle;

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
    render_p38_other_views(&mut h, &dir)?;
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

/// The same airborne aircraft from the cockpit, through its mounted sensor and on
/// the chart, so every view a pilot can switch to is covered.
fn render_p38_other_views(h: &mut VerifyHarness, dir: &Path) -> Result<(), AppSimError> {
    for (mode, instruments, name) in [
        (OperatorViewportMode::Cockpit, true, "p38_07_cockpit.png"),
        (
            OperatorViewportMode::Cockpit,
            false,
            "p38_08_cockpit_out_the_window.png",
        ),
        (
            OperatorViewportMode::SensorAperture,
            false,
            "p38_09_sensor_aperture.png",
        ),
        (
            OperatorViewportMode::ChartMap,
            false,
            "p38_10_chart_map.png",
        ),
    ] {
        h.rig.set_show_instruments(instruments);
        h.rig.set_show_conformal_hud(instruments);
        h.rig.supervisor_mut().set_viewport_mode(mode);
        h.settle_and_draw_flight(&dir.join(name))?;
    }
    h.rig.set_show_instruments(false);
    h.rig.set_show_conformal_hud(true);
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    Ok(())
}
