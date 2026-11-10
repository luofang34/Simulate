//! Renders the P-38G from each photograph's resected camera and writes the comparison
//! images: one ignored GPU test, repeatable on any later model.

use std::path::{Path, PathBuf};

use glam::{DMat3, DQuat, DVec3};
use sim_core::ControlInputs;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_render::{Camera, CameraPose};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::AerodromeId;

use super::super::vehicle_views::capture_camera_lit;
use super::super::{SHOT_SIZE, VerifyHarness};
use super::compose::{Framing, write_comparisons};
use super::landmarks::p38g_landmarks;
use super::resection::PinholePose;
use super::{Match, Photo, match_photo, photos};
use crate::error::AppSimError;
use crate::esc_menu::SpawnMode;
use crate::vehicles::FreeFlightVehicle;

/// Sun for a photo whose light gives no direction: high and over the camera's left
/// shoulder, which shows shape without hard shadows.
const DEFAULT_SUN_DEG: (f64, f64) = (-35.0, 55.0);
/// `50 ms` steps flown before a shot.
const GEAR_SETTLE_STEPS: usize = 200;

#[test]
#[ignore = "renders the P-38G from each resected photo camera; needs $SIMULATE_PHOTOMATCH_DIR and $SIMULATE_VERIFY_OUT"]
fn render_photo_matches() -> Result<(), AppSimError> {
    let (Some(out), Some(images)) = (
        std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from),
        std::env::var_os("SIMULATE_PHOTOMATCH_DIR").map(PathBuf::from),
    ) else {
        return Ok(());
    };
    let out = out.join("photomatch");
    std::fs::create_dir_all(&out).map_err(|e| shot_error(&e))?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.rig.set_show_instruments(false);
    h.rig.set_show_conformal_hud(false);
    h.rig
        .supervisor_mut()
        .set_free_aerodrome(AerodromeId::NewarkKewr);
    h.flight.select_vehicle(FreeFlightVehicle::JsbsimP38g)?;
    let mut summary = Vec::new();
    for photo in photos() {
        let Some(matched) = match_photo(&photo) else {
            continue;
        };
        let Ok(image) = image::open(images.join(&photo.file)) else {
            tracing::warn!(file = %photo.file, "photo not found; skipped");
            continue;
        };
        let parked = pose_aircraft(&mut h, &matched)?;
        let framing = Framing::new(&matched.solved.pose, photo.size_px, SHOT_SIZE);
        let shown = capture(&mut h, &matched, &framing, OperatorViewportMode::Chase)?;
        let hidden = capture(
            &mut h,
            &matched,
            &framing,
            OperatorViewportMode::SensorAperture,
        )?;
        let tone = write_comparisons(
            &out,
            &matched,
            &framing,
            &image.to_rgb8(),
            [&shown, &hidden],
        )?;
        let mut solution = solution_json(&matched);
        if let Some(fields) = solution.as_object_mut() {
            fields.insert("tone_p05_p25_p50_p75_p95".to_owned(), tone);
            fields.insert("parked".to_owned(), parked.into());
            let above = eye_above_wheels_m(&matched.solved.pose);
            fields.insert("eye_above_wheels_m".to_owned(), above.into());
        }
        summary.push(solution);
    }
    let doc = serde_json::to_string_pretty(&summary).map_err(|e| shot_error(&e))?;
    std::fs::write(out.join("solutions.json"), doc).map_err(|e| shot_error(&e))?;
    h.gpu.check_device()?;
    Ok(())
}

pub(super) fn shot_error(e: &dyn std::fmt::Display) -> AppSimError {
    AppSimError::Screenshot {
        detail: e.to_string(),
    }
}

/// Lowest the camera of a ground photo may sit above the model's wheel plane and still
/// be rendered with the aircraft parked: lower, and drawn terrain would cover the lens.
const MIN_EYE_ABOVE_WHEELS_M: f64 = 0.25;

/// Poses the aircraft as photographed and says whether it stands on the ground.
///
/// A ground photo is rendered standing on a runway, where the drawn ground and the
/// ground its shadow falls on agree, so the shadow lies under it and the ground lights
/// its underside, unless the resected camera sits at or below the
/// model's wheel plane (the model's stance differs from the photographed aircraft's);
/// then, as for a photo taken in flight, the aircraft is flown clear of the ground with
/// its gear as photographed. A ground photo's propellers are feathered so its blades
/// stand still.
fn pose_aircraft(h: &mut VerifyHarness, matched: &Match) -> Result<bool, AppSimError> {
    let photo = &matched.photo;
    let parked = !photo.airborne
        && eye_above_wheels_m(&matched.solved.pose).is_some_and(|m| m >= MIN_EYE_ABOVE_WHEELS_M);
    let mode = if parked {
        SpawnMode::Threshold
    } else {
        SpawnMode::ShortFinal
    };
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::NewarkKewr).cloned() {
        h.flight.spawn_at_aerodrome(&diag, 0, mode)?;
    }
    let (gear_down, stopped) = (photo.gear_down, !photo.airborne);
    h.flight.update_system_controls(|sys| {
        sys.gear_down = gear_down;
        sys.prop_feather = [stopped; 2];
    });
    let throttle = if stopped { 0.0 } else { 0.8 };
    // Long enough for the gear to finish travelling, or the parked airframe to settle.
    for _ in 0..GEAR_SETTLE_STEPS {
        h.flight.advance_by(
            0.05,
            ControlInputs {
                throttle,
                ..ControlInputs::default()
            },
            SimTimeScale::RealTime,
        )?;
    }
    Ok(parked)
}

/// Height of `pose`'s eye above the plane through the model's three wheel bottoms.
fn eye_above_wheels_m(pose: &PinholePose) -> Option<f64> {
    let marks = p38g_landmarks();
    let wheel = |id: &str| marks.iter().find(|(m, _)| *m == id).map(|(_, p)| *p);
    let (left, right, nose) = (wheel("wheel_l")?, wheel("wheel_r")?, wheel("nose_wheel")?);
    let up = (right - left).cross(nose - left).try_normalize()?;
    let up = if up.z < 0.0 { -up } else { up };
    Some((pose.eye - left).dot(up))
}

/// The frame from the photo's camera, with the airframe drawn in `Chase` and hidden in
/// the aperture view, whose first camera looks past it.
fn capture(
    h: &mut VerifyHarness,
    matched: &Match,
    framing: &Framing,
    mode: OperatorViewportMode,
) -> Result<Vec<u8>, AppSimError> {
    h.rig.supervisor_mut().set_viewport_mode(mode);
    h.rig.supervisor_mut().set_virtual_camera_index(0);
    let pose = matched.solved.pose;
    let fov_y_rad = framing.render_fov_y_rad;
    let camera = |h: &VerifyHarness| Some(photo_camera(h, &pose, fov_y_rad));
    let sun = photo_sun(h, &pose, matched.photo.sun_deg.unwrap_or(DEFAULT_SUN_DEG));
    capture_camera_lit(h, camera, sun)
}

/// Mesh axes (`x` right, `y` forward, `z` up) to FRD body axes.
const MESH_TO_FRD: DMat3 = DMat3::from_cols(
    DVec3::new(0.0, 1.0, 0.0),
    DVec3::new(1.0, 0.0, 0.0),
    DVec3::new(0.0, 0.0, -1.0),
);
/// Optical axes (`y` down, `z` forward) to render axes (`y` up, looking along `-z`).
const OPTICAL_TO_RENDER: DMat3 = DMat3::from_diagonal(DVec3::new(1.0, -1.0, -1.0));

/// `pose`, solved in mesh axes, on the displayed aircraft.
fn photo_camera(h: &VerifyHarness, pose: &PinholePose, fov_y_rad: f64) -> Camera {
    let state = h.flight.display_vehicle_state();
    let attitude = DMat3::from_quat(state.attitude);
    let origin = state.position_ecef + attitude * h.flight.model_origin_from_cg_frd_m();
    let mesh_to_ecef = attitude * MESH_TO_FRD;
    let optical_to_mesh = DMat3::from_quat(pose.mesh_to_optical).transpose();
    let render_to_ecef = mesh_to_ecef * optical_to_mesh * OPTICAL_TO_RENDER;
    Camera {
        pose: CameraPose {
            position_ecef: origin + mesh_to_ecef * pose.eye,
            orientation: DQuat::from_mat3(&render_to_ecef).normalize(),
        },
        fov_y_rad,
        near_m: 0.3,
    }
}

/// The Sun `(azimuth from the view, elevation)` degrees for the photo's camera.
fn photo_sun(h: &VerifyHarness, pose: &PinholePose, (az_deg, el_deg): (f64, f64)) -> DVec3 {
    let state = h.flight.display_vehicle_state();
    let up = LocalFrame::at(&ecef_to_geodetic(state.position_ecef)).up();
    let view_mesh = pose.mesh_to_optical.inverse() * DVec3::Z;
    let view = DMat3::from_quat(state.attitude) * MESH_TO_FRD * view_mesh;
    let forward = (view - up * view.dot(up)).normalize_or(up.any_orthonormal_vector());
    let right = forward.cross(up);
    let (az, el) = (az_deg.to_radians(), el_deg.to_radians());
    let horizontal = -forward * az.cos() + right * az.sin();
    (horizontal * el.cos() + up * el.sin()).normalize_or(up)
}

fn solution_json(m: &Match) -> serde_json::Value {
    let pose = m.solved.pose;
    let frd = MESH_TO_FRD * pose.eye;
    let round = |v: f64, places: f64| (v * places).round() / places;
    let residuals: serde_json::Map<String, serde_json::Value> = m
        .used
        .iter()
        .zip(&m.solved.residual_px)
        .map(|((id, _, _), r)| (id.clone(), serde_json::json!(round(*r, 10.0))))
        .collect();
    let eye_frd_m = [frd.x, frd.y, frd.z].map(|v| round(v, 100.0));
    serde_json::json!({
        "id": m.photo.id,
        "eye_frd_m": eye_frd_m,
        "range_m": round(pose.eye.length(), 10.0),
        "focal_px": pose.focal_px.round(),
        "fov_y_deg": round(pose.fov_y_rad(m.photo.size_px[1]).to_degrees(), 10.0),
        "rms_px": round(m.solved.rms_px, 10.0),
        "residual_px": residuals,
    })
}

/// Where a photo's comparison image of `kind` lands.
pub(super) fn out_path(dir: &Path, photo: &Photo, kind: &str) -> PathBuf {
    dir.join(format!("{}_{kind}.png", photo.id))
}
