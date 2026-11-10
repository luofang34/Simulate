//! Lockheed P-38G verification shots in every simulator view: the hangar
//! studio, chase orbits on the runway and airborne, a feathered engine, the
//! cockpit out-the-window view, the mounted sensor aperture and the chart map.

use std::path::{Path, PathBuf};

use glam::DVec3;
use sim_core::ControlInputs;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_render::{OverlayQuad, ViewportRect};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::AerodromeId;

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::camera_view::LookOffset;
use crate::error::AppSimError;
use crate::esc_menu::{HubTab, HubView, MenuInput, OverlayContent, SpawnMode};
use crate::sky_rig::SkyFrame;
use crate::vehicles::FreeFlightVehicle;

mod photo_views;

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
    let low_sun = sun_relative_to_nose(&h, 1.3, 10.0_f64.to_radians());
    draw_chase_orbit_lit(
        &mut h,
        [2.0, -0.10, 0.62],
        low_sun,
        &dir.join("p38_14_low_sun_runway.png"),
    )?;
    render_p38_airborne_and_feathered_shots(&mut h, &dir)?;
    render_p38_other_views(&mut h, &dir)?;
    render_c172_reference(&mut h, &dir)?;
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
    let sun = h.flight.world().sun_direction_ecef();
    draw_chase_orbit_lit(h, [yaw_rad, pitch_rad, dist_scale], sun, out)
}

/// A low Sun `azimuth_rad` clockwise from the nose and `elevation_rad` above the local
/// horizon, so shadows and occlusion are judged in raking light.
fn sun_relative_to_nose(h: &VerifyHarness, azimuth_rad: f64, elevation_rad: f64) -> DVec3 {
    let state = h.flight.display_vehicle_state();
    let up = LocalFrame::at(&ecef_to_geodetic(state.position_ecef)).up();
    let nose = state.attitude * DVec3::X;
    let forward = (nose - up * nose.dot(up)).normalize_or(nose);
    let right = forward.cross(up).normalize_or(DVec3::Y);
    let horizontal = forward * azimuth_rad.cos() + right * azimuth_rad.sin();
    (horizontal * elevation_rad.cos() + up * elevation_rad.sin()).normalize_or(up)
}

/// Chase orbit `[yaw_rad, pitch_rad, dist_scale]` about the aircraft, lit by `sun_ecef`.
fn draw_chase_orbit_lit(
    h: &mut VerifyHarness,
    [yaw_rad, pitch_rad, dist_scale]: [f64; 3],
    sun_ecef: DVec3,
    out: &Path,
) -> Result<(), AppSimError> {
    let sky = SkyFrame::fixed_sun(h.flight.world().body(), sun_ecef);
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
            space: None,
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
    h.rig.set_show_conformal_hud(false);
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    for (yaw_deg, pitch_deg, name) in [
        (-80.0, -8.0, "p38_11_cockpit_look_left.png"),
        (80.0, -8.0, "p38_12_cockpit_look_right.png"),
        (0.0, -30.0, "p38_13_cockpit_panel.png"),
    ] {
        draw_p38_cockpit_look(h, yaw_deg, pitch_deg, &dir.join(name))?;
    }
    render_p38_panel_closeups(h, dir)?;
    render_p38_cabin_looks(h, dir)?;
    h.rig.set_show_conformal_hud(true);
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    Ok(())
}

/// The flight instruments from close in and off to one side, so the needles
/// stand visibly off the recessed faces under the cover glass; then the
/// panel at dusk and at night, when the luminous paint glows.
fn render_p38_panel_closeups(h: &mut VerifyHarness, dir: &Path) -> Result<(), AppSimError> {
    // A low Sun from behind, under the glare shield onto the faces.
    let sun = sun_relative_to_nose(h, 2.9, 14.0_f64.to_radians());
    let close = CockpitLook {
        eye_m: [0.02, -0.14, 0.52],
        yaw_deg: -20.0,
        pitch_deg: -22.0,
    };
    draw_p38_cockpit_view(h, close, sun, &dir.join("p38_16_panel_closeup_oblique.png"))?;
    let dusk = sun_relative_to_nose(h, 2.6, 2.0_f64.to_radians());
    let panel = CockpitLook {
        eye_m: [0.0, 0.0, 0.0],
        yaw_deg: 0.0,
        pitch_deg: -30.0,
    };
    draw_p38_cockpit_view(h, panel, dusk, &dir.join("p38_17_panel_dusk.png"))?;
    let night = sun_relative_to_nose(h, 2.6, -12.0_f64.to_radians());
    draw_p38_cockpit_view(h, close, night, &dir.join("p38_18_panel_night_closeup.png"))
}

/// The pilot turned to look back over the right shoulder at the armour, headrest
/// and canopy; then the controls from above the pilot's right shoulder, centred
/// and then held deflected, so the wheel, column, quadrant and pedals are seen to
/// move with the flight model's commands.
fn render_p38_cabin_looks(h: &mut VerifyHarness, dir: &Path) -> Result<(), AppSimError> {
    let sun = h.flight.world().sun_direction_ecef();
    let back = CockpitLook {
        eye_m: [0.06, 0.02, 0.03],
        yaw_deg: 150.0,
        pitch_deg: -10.0,
    };
    draw_p38_cockpit_view(h, back, sun, &dir.join("p38_19_cockpit_look_back.png"))?;
    photo_views::render_p38_looks_in_from_outside(h, dir, sun)?;
    let controls = CockpitLook {
        eye_m: [0.16, 0.10, -0.02],
        yaw_deg: -18.0,
        pitch_deg: -48.0,
    };
    draw_p38_cockpit_view(h, controls, sun, &dir.join("p38_20_controls_centred.png"))?;
    h.flight.update_system_controls(|sys| {
        sys.flaps_norm = 1.0;
        sys.gear_down = false;
        sys.throttle_split = Some([0.25, 1.0]);
        sys.trim_norm = [0.0, -0.6, 0.0];
    });
    for _ in 0..4 {
        h.flight.advance_by(
            0.05,
            ControlInputs {
                roll: 0.9,
                pitch: 0.8,
                yaw: -0.8,
                throttle: 1.0,
            },
            SimTimeScale::RealTime,
        )?;
    }
    draw_p38_cockpit_view(h, controls, sun, &dir.join("p38_21_controls_deflected.png"))?;
    // The gyro horizon, directional gyro, turn and bank, compass and clock,
    // seen past the wheel's right horn while the deflected controls roll and
    // yaw the aircraft.
    let gyros = CockpitLook {
        eye_m: [0.17, -0.06, 0.40],
        yaw_deg: -8.0,
        pitch_deg: -20.0,
    };
    draw_p38_cockpit_view(h, gyros, sun, &dir.join("p38_22_gyro_group.png"))
}

/// A seated view moved `eye_m` `[right, up, forward]` from the design eye and
/// turned `yaw_deg` right and `pitch_deg` up.
#[derive(Clone, Copy)]
struct CockpitLook {
    eye_m: [f64; 3],
    yaw_deg: f64,
    pitch_deg: f64,
}

/// The seated pilot's view turned `yaw_deg` right and `pitch_deg` up.
fn draw_p38_cockpit_look(
    h: &mut VerifyHarness,
    yaw_deg: f64,
    pitch_deg: f64,
    out: &Path,
) -> Result<(), AppSimError> {
    let sun = h.flight.world().sun_direction_ecef();
    let look = CockpitLook {
        eye_m: [0.0; 3],
        yaw_deg,
        pitch_deg,
    };
    draw_p38_cockpit_view(h, look, sun, out)
}

fn draw_p38_cockpit_view(
    h: &mut VerifyHarness,
    view: CockpitLook,
    sun_ecef: DVec3,
    out: &Path,
) -> Result<(), AppSimError> {
    draw_p38_view_with_fov(h, view, None, sun_ecef, out)
}

/// [`draw_p38_cockpit_view`] with the vertical field of view `fov_y_deg`
/// in place of the cockpit camera's own, to match a photograph's.
fn draw_p38_view_with_fov(
    h: &mut VerifyHarness,
    view: CockpitLook,
    fov_y_deg: Option<f64>,
    sun_ecef: DVec3,
    out: &Path,
) -> Result<(), AppSimError> {
    let sky = SkyFrame::fixed_sun(h.flight.world().body(), sun_ecef);
    let look = LookOffset {
        yaw_rad: view.yaw_deg.to_radians(),
        pitch_rad: view.pitch_deg.to_radians(),
    };
    let [right, up, forward] = view.eye_m;
    for _ in 0..35 {
        let state = h.flight.display_vehicle_state();
        let mut base_cam = h.flight.display_camera(OperatorViewportMode::Cockpit)?;
        base_cam.pose.position_ecef += base_cam.pose.orientation * DVec3::new(right, up, -forward);
        let mut cam = look.apply(OperatorViewportMode::Cockpit, base_cam, &state);
        if let Some(fov) = fov_y_deg {
            cam.fov_y_rad = fov.to_radians();
        }
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    let quads = h.rig.overlay_quads(h.flight.world());
    h.composite_to_png(&quads, out)
}

/// The C172 shares the aircraft pass; one parked chase shot in the same low Sun checks
/// that shading tuned on the P-38G still suits it.
fn render_c172_reference(h: &mut VerifyHarness, dir: &Path) -> Result<(), AppSimError> {
    h.flight.select_vehicle(FreeFlightVehicle::JsbsimC172)?;
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::NewarkKewr).cloned() {
        h.flight
            .spawn_at_aerodrome(&diag, 0, SpawnMode::Threshold)?;
    }
    for _ in 0..20 {
        h.flight
            .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
    }
    let low_sun = sun_relative_to_nose(h, 1.3, 10.0_f64.to_radians());
    draw_chase_orbit_lit(
        h,
        [2.0, -0.10, 0.62],
        low_sun,
        &dir.join("p38_15_c172_low_sun_reference.png"),
    )
}
