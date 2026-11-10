//! The P-38G's cockpit looked into from outside, the way Lockheed's
//! photographs look in.

use std::path::Path;

use glam::{DMat3, DQuat, DVec3};
use sim_core::ControlInputs;
use sim_scenario::{OperatorViewportMode, SimTimeScale};

use super::{CockpitLook, draw_p38_view_with_fov};
use crate::cockpit_rig::verify_shots::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::error::AppSimError;
use crate::sky_rig::SkyFrame;

/// A camera placed in the aircraft's own axes (`x` right, `y` forward, `z` up
/// from the model's origin), looking `yaw_deg` right of the nose and
/// `pitch_deg` up, with a vertical field `fov_y_deg`.
struct AirframeCamera {
    eye_m: [f64; 3],
    yaw_deg: f64,
    pitch_deg: f64,
    fov_y_deg: f64,
}

/// Where the camera of Lockheed's frontal photograph Z-4986 ("P-38 #3474
/// INSTRUMENT PANEL 12-23-42") solves to on the station 126.2 frame and the
/// instrument board: over the seat, looking forward and down at the board.
/// Good to about 0.15 m fore and aft and 0.05 m in height.
const Z4986_CAMERA: AirframeCamera = AirframeCamera {
    eye_m: [0.033, -0.456, 1.146],
    yaw_deg: -3.5,
    pitch_deg: -35.9,
    fov_y_deg: 36.7,
};

/// Draws the cockpit from `camera`.
fn draw_from_airframe(
    h: &mut VerifyHarness,
    camera: &AirframeCamera,
    sun_ecef: DVec3,
    out: &Path,
) -> Result<(), AppSimError> {
    let sky = SkyFrame::fixed_sun(h.flight.world().body(), sun_ecef);
    let (yaw, pitch) = (camera.yaw_deg.to_radians(), camera.pitch_deg.to_radians());
    let forward = DVec3::new(
        yaw.sin() * pitch.cos(),
        yaw.cos() * pitch.cos(),
        pitch.sin(),
    );
    let right = forward.cross(DVec3::Z).normalize_or(DVec3::X);
    let up = right.cross(forward);
    for _ in 0..35 {
        let state = h.flight.display_vehicle_state();
        // Model axes (x right, y forward, z up) to body FRD, then ECEF.
        let to_ecef = |v: DVec3| state.attitude * DVec3::new(v.y, v.x, -v.z);
        let mut cam = h.flight.display_camera(OperatorViewportMode::Cockpit)?;
        let origin = state.position_ecef + state.attitude * h.flight.model_origin_from_cg_frd_m();
        cam.pose.position_ecef = origin + to_ecef(DVec3::from_array(camera.eye_m));
        // Camera-local axes: x right, y up, z back.
        let axes = DMat3::from_cols(to_ecef(right), to_ecef(up), to_ecef(-forward));
        cam.pose.orientation = DQuat::from_mat3(&axes).normalize();
        cam.fov_y_rad = camera.fov_y_deg.to_radians();
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    let quads = h.rig.overlay_quads(h.flight.world());
    h.composite_to_png(&quads, out)
}

/// The cockpit looked into from outside, as Lockheed's photographs look in:
/// over the left sill with the canopy shut and open, then forward over the
/// seat at the board.
pub(super) fn render_p38_looks_in_from_outside(
    h: &mut VerifyHarness,
    dir: &Path,
    sun: DVec3,
) -> Result<(), AppSimError> {
    // From over the left sill looking down and right across the cockpit, the
    // way Lockheed's negative Z-4972 looks in
    // (<https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_left_wing_061019-F-1234P-004.jpg>).
    // The viewpoint and field are chosen by eye; the photograph's camera is
    // not solved.
    let z4972 = CockpitLook {
        eye_m: [-0.36, 0.20, -0.30],
        yaw_deg: 30.0,
        pitch_deg: -36.0,
    };
    draw_p38_view_with_fov(
        h,
        z4972,
        Some(70.0),
        sun,
        &dir.join("p38_23_over_left_sill.png"),
    )?;
    // The same look with the canopy as the photograph has it: hatch raised,
    // both side windows wound down.
    let open = sim_core::CanopyOpening {
        hatch: 1.0,
        side_windows: [1.0; 2],
    };
    h.flight.update_system_controls(|sys| sys.canopy = open);
    h.flight
        .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
    draw_p38_view_with_fov(
        h,
        z4972,
        Some(70.0),
        sun,
        &dir.join("p38_24_over_left_sill_canopy_open.png"),
    )?;
    draw_from_airframe(
        h,
        &Z4986_CAMERA,
        sun,
        &dir.join("p38_25_over_the_seat_canopy_open.png"),
    )?;
    h.flight
        .update_system_controls(|sys| sys.canopy = sim_core::CanopyOpening::default());
    Ok(())
}
