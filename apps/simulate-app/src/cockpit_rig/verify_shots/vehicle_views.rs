//! Every view a vehicle package declares, parked and on short final: one shot per
//! view, so a changed camera set or airframe is judged in the views the operator
//! cycles through.

use std::path::{Path, PathBuf};

use glam::DVec3;
use sim_core::ControlInputs;
use sim_render::Camera;
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::AerodromeId;

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::error::AppSimError;
use crate::esc_menu::SpawnMode;
use crate::gallery::save_png;
use crate::sky_rig::SkyFrame;
use crate::vehicles::FreeFlightVehicle;

/// Cameras before the vehicle's own in the operator's cycle.
const SUITE_CAMERAS: usize = 5;
/// Frames drawn before a shot is taken, so streamed tiles settle.
const SETTLE_FRAMES: usize = 35;

#[test]
#[ignore = "writes one 1280x720 PNG per P-38G and C172 package view into $SIMULATE_VERIFY_OUT"]
fn render_vehicle_view_sets() -> Result<(), AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    std::fs::create_dir_all(&dir).map_err(|e| AppSimError::Screenshot {
        detail: e.to_string(),
    })?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.rig.set_show_instruments(false);
    h.rig.set_show_conformal_hud(false);
    h.rig
        .supervisor_mut()
        .set_free_aerodrome(AerodromeId::NewarkKewr);
    for (vehicle, tag) in [
        (FreeFlightVehicle::JsbsimP38g, "p38g"),
        (FreeFlightVehicle::JsbsimC172, "c172"),
    ] {
        h.flight.select_vehicle(vehicle)?;
        for (spawn, phase) in [
            (SpawnMode::Threshold, "parked"),
            (SpawnMode::ShortFinal, "final"),
        ] {
            spawn_and_settle(&mut h, spawn)?;
            h.rig
                .supervisor_mut()
                .set_viewport_mode(OperatorViewportMode::Chase);
            let chase = h.flight.display_camera(OperatorViewportMode::Chase)?;
            draw_camera(
                &mut h,
                |_| Some(chase),
                &dir.join(format!("views_{tag}_{phase}_00_chase_mode.png")),
            )?;
            render_declared_views(&mut h, &dir, &format!("views_{tag}_{phase}"))?;
        }
    }
    h.gpu.check_device()?;
    Ok(())
}

fn spawn_and_settle(h: &mut VerifyHarness, spawn: SpawnMode) -> Result<(), AppSimError> {
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::NewarkKewr).cloned() {
        h.flight.spawn_at_aerodrome(&diag, 0, spawn)?;
    }
    let airborne = spawn != SpawnMode::Threshold;
    h.flight
        .update_system_controls(|sys| sys.gear_down = !airborne);
    let throttle = if airborne { 0.8 } else { 0.0 };
    for _ in 0..30 {
        h.flight.advance_by(
            0.05,
            ControlInputs {
                throttle,
                ..ControlInputs::default()
            },
            SimTimeScale::RealTime,
        )?;
    }
    Ok(())
}

/// Each of the flying vehicle's own views in cycling order, as the aperture view shows them.
fn render_declared_views(
    h: &mut VerifyHarness,
    dir: &Path,
    prefix: &str,
) -> Result<(), AppSimError> {
    let cameras = h.flight.vehicle_cameras();
    h.rig.supervisor_mut().set_vehicle_cameras(cameras);
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::SensorAperture);
    for (i, spec) in cameras.iter().enumerate() {
        h.rig
            .supervisor_mut()
            .set_virtual_camera_index(SUITE_CAMERAS.saturating_add(i));
        let shown = h.rig.supervisor().active_virtual_camera(None);
        assert_eq!(shown.id, spec.id, "the cycle selects the declared view");
        let out = dir.join(format!(
            "{prefix}_{:02}_{}.png",
            i.saturating_add(1),
            spec.id
        ));
        draw_camera(
            h,
            |h| {
                let (body, spec) = h.rig.supervisor().active_virtual_camera_on_body(None);
                h.flight.virtual_camera(body, &spec)
            },
            &out,
        )?;
    }
    Ok(())
}

/// Draws `camera` (re-placed every frame) until tiles settle, then writes `out`.
pub(super) fn draw_camera(
    h: &mut VerifyHarness,
    camera: impl Fn(&VerifyHarness) -> Option<Camera>,
    out: &Path,
) -> Result<(), AppSimError> {
    let sun = h.flight.world().sun_direction_ecef();
    draw_camera_lit(h, camera, sun, out)
}

/// [`draw_camera`] lit by a Sun along `sun_ecef`.
pub(super) fn draw_camera_lit(
    h: &mut VerifyHarness,
    camera: impl Fn(&VerifyHarness) -> Option<Camera>,
    sun_ecef: DVec3,
    out: &Path,
) -> Result<(), AppSimError> {
    let rgba = capture_camera_lit(h, camera, sun_ecef)?;
    save_png(out, SHOT_SIZE, rgba)
}

/// The frame `camera` sees lit by a Sun along `sun_ecef`, as [`SHOT_SIZE`] RGBA rows,
/// once streamed tiles have settled.
pub(super) fn capture_camera_lit(
    h: &mut VerifyHarness,
    camera: impl Fn(&VerifyHarness) -> Option<Camera>,
    sun_ecef: DVec3,
) -> Result<Vec<u8>, AppSimError> {
    let sky = SkyFrame::fixed_sun(h.flight.world().body(), sun_ecef);
    for _ in 0..SETTLE_FRAMES {
        let Some(cam) = camera(h) else {
            return Err(AppSimError::Screenshot {
                detail: "no camera for the shot".to_owned(),
            });
        };
        h.rig.invalidate_chart();
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    let quads = h.rig.overlay_quads(h.flight.world());
    h.composite_rgba(&quads)
}
