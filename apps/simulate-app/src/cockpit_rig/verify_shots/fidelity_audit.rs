//! Visual-fidelity audit shots for the P-38G and the C172: orthographic silhouettes
//! registered to published general-arrangement drawings, long-focal body-axis views,
//! photo-matched three-quarter views and close-ups, all in the flight renderer.
//!
//! Every camera is placed in the airframe's own model frame (`+X` starboard, `+Y`
//! forward, `+Z` up, the frame the OpenUSD parts are authored in), so a shot frames the
//! same feature whatever the aircraft's heading or attitude.

use std::path::{Path, PathBuf};

use glam::DVec3;
use sim_core::ControlInputs;
use sim_maplibre::{PartRole, UsdAircraftAsset};
use sim_render::{Camera, CameraPose};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::AerodromeId;

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::camera_view::LookOffset;
use crate::error::AppSimError;
use crate::esc_menu::SpawnMode;
use crate::esc_menu::catalog::HangarModel;
use crate::hangar::load_model;
use crate::sky_rig::SkyFrame;
use crate::vehicles::FreeFlightVehicle;

mod ortho;
mod shots;

use shots::{C172_GROUND, P38G_GROUND};

/// One camera placement: look at `target` (model frame, metres) from `target + eye`.
struct Shot {
    name: &'static str,
    target: [f64; 3],
    eye: [f64; 3],
    fov_y_deg: f64,
}

const fn shot(name: &'static str, target: [f64; 3], eye: [f64; 3], fov_y_deg: f64) -> Shot {
    Shot {
        name,
        target,
        eye,
        fov_y_deg,
    }
}

/// Distance of the long-focal body-axis views; at 500 m the depth of a 16 m airframe
/// changes image scale by under 2 %, close enough to orthographic to read proportions.
const FAR_M: f64 = 500.0;

/// The view from below stays closer: the aircraft flies only a few hundred metres up.
const BELOW_M: f64 = 150.0;

/// Vertical field of view that spans `extent_m` at `dist_m`.
fn far_fov(extent_m: f64, dist_m: f64) -> f64 {
    (2.0 * (0.5 * extent_m / dist_m).atan()).to_degrees()
}

fn far_views(names: [&'static str; 5], centre: [f64; 3], length_m: f64, span_m: f64) -> Vec<Shot> {
    let aspect = f64::from(SHOT_SIZE[0]) / f64::from(SHOT_SIZE[1]);
    let plan_extent = (length_m * 1.1).max(span_m * 1.1 / aspect);
    let plan = far_fov(plan_extent, FAR_M);
    let side = far_fov(length_m * 1.1 / aspect, FAR_M);
    let front = far_fov(span_m * 1.08 / aspect, FAR_M);
    let [top, bottom, left, front_name, rear] = names;
    vec![
        shot(top, centre, [0.0, 0.0, FAR_M], plan),
        shot(
            bottom,
            centre,
            [0.0, 0.0, -BELOW_M],
            far_fov(plan_extent, BELOW_M),
        ),
        shot(left, centre, [-FAR_M, 0.0, 0.0], side),
        shot(front_name, centre, [0.0, FAR_M, 0.0], front),
        shot(rear, centre, [0.0, -FAR_M, 0.0], front),
    ]
}

/// Model-frame vector to the body frame (`x` forward, `y` right, `z` down).
fn model_to_body(v: DVec3) -> DVec3 {
    DVec3::new(v.y, v.x, -v.z)
}

/// Sun from the front-left quarter, 40° up, in the model frame: raking enough to show
/// panel relief and shadow, and the same for every shot so materials compare fairly.
fn audit_sun(h: &VerifyHarness) -> DVec3 {
    let el = 40.0_f64.to_radians();
    let az = 135.0_f64.to_radians();
    let model = DVec3::new(-az.sin() * el.cos(), -az.cos() * el.cos(), el.sin());
    let state = h.flight.display_vehicle_state();
    (state.attitude * model_to_body(model)).normalize()
}

fn draw_shot(h: &mut VerifyHarness, s: &Shot, dir: &Path) -> Result<(), AppSimError> {
    let sky = SkyFrame::fixed_sun(h.flight.world().body(), audit_sun(h));
    let far = DVec3::from(s.eye).length() > 100.0;
    for _ in 0..30 {
        let state = h.flight.display_vehicle_state();
        let to_ecef = |v: DVec3| state.attitude * model_to_body(v);
        let target = state.position_ecef + to_ecef(DVec3::from(s.target));
        let eye = target + to_ecef(DVec3::from(s.eye));
        // Looking straight down or up, the nose is "up" in the image.
        let up_model = if DVec3::from(s.eye).normalize().z.abs() > 0.95 {
            DVec3::Y
        } else {
            DVec3::Z
        };
        let cam = Camera {
            pose: CameraPose::looking(eye, target - eye, to_ecef(up_model)),
            fov_y_rad: s.fov_y_deg.to_radians(),
            near_m: if far { 50.0 } else { 0.05 },
        };
        h.rig.invalidate_chart();
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(15));
    }
    let quads = h.rig.overlay_quads(h.flight.world());
    h.composite_to_png(&quads, &dir.join(format!("{}.png", s.name)))
}

fn settle(h: &mut VerifyHarness, steps: u32) -> Result<(), AppSimError> {
    for _ in 0..steps {
        h.flight
            .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
    }
    Ok(())
}

fn spawn(h: &mut VerifyHarness, mode: SpawnMode) -> Result<(), AppSimError> {
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::NewarkKewr).cloned() {
        h.flight.spawn_at_aerodrome(&diag, 0, mode)?;
    }
    Ok(())
}

/// Renders one airframe's ground set, its cockpit looks, and its airborne far views.
fn render_airframe(
    h: &mut VerifyHarness,
    vehicle: FreeFlightVehicle,
    ground: &[Shot],
    far: &[Shot],
    dir: &Path,
) -> Result<(), AppSimError> {
    h.flight.select_vehicle(vehicle)?;
    h.rig.set_show_instruments(false);
    h.rig.set_show_conformal_hud(false);
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    spawn(h, SpawnMode::Threshold)?;
    settle(h, 30)?;
    for s in ground {
        draw_shot(h, s, dir)?;
    }
    let prefix = if vehicle == FreeFlightVehicle::JsbsimP38g {
        "p38g"
    } else {
        "c172"
    };
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    for (yaw, pitch, what) in [
        (0.0, -5.0, "look_forward"),
        (-75.0, -10.0, "look_left"),
        (75.0, -10.0, "look_right"),
        (0.0, 35.0, "look_up_canopy"),
        (-150.0, 0.0, "look_back_left"),
        (0.0, -55.0, "look_down_floor_controls"),
    ] {
        draw_cockpit_look(
            h,
            yaw,
            pitch,
            &dir.join(format!("{prefix}_cockpit_{what}.png")),
        )?;
    }
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    spawn(h, SpawnMode::Overhead)?;
    h.flight.update_system_controls(|sys| sys.gear_down = true);
    settle(h, 10)?;
    for s in far {
        draw_shot(h, s, dir)?;
    }
    Ok(())
}

/// The seated pilot's view turned `yaw_deg` right and `pitch_deg` up.
fn draw_cockpit_look(
    h: &mut VerifyHarness,
    yaw_deg: f64,
    pitch_deg: f64,
    out: &Path,
) -> Result<(), AppSimError> {
    let sky = SkyFrame::fixed_sun(h.flight.world().body(), audit_sun(h));
    let look = LookOffset {
        yaw_rad: yaw_deg.to_radians(),
        pitch_rad: pitch_deg.to_radians(),
    };
    for _ in 0..30 {
        let state = h.flight.display_vehicle_state();
        let base = h.flight.display_camera(OperatorViewportMode::Cockpit)?;
        let cam = look.apply(OperatorViewportMode::Cockpit, base, &state);
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(15));
    }
    let quads = h.rig.overlay_quads(h.flight.world());
    h.composite_to_png(&quads, out)
}

fn out_dir() -> Result<Option<PathBuf>, AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(None);
    };
    std::fs::create_dir_all(&dir).map_err(|e| AppSimError::Screenshot {
        detail: format!("{}: {e}", dir.display()),
    })?;
    Ok(Some(dir))
}

#[test]
#[ignore = "writes P-38G and C172 fidelity-audit PNGs into $SIMULATE_VERIFY_OUT"]
fn render_fidelity_audit_shots() -> Result<(), AppSimError> {
    let Some(dir) = out_dir()? else {
        return Ok(());
    };
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    let p38_far = far_views(
        [
            "p38g_far_top",
            "p38g_far_bottom",
            "p38g_far_left_side",
            "p38g_far_front",
            "p38g_far_rear",
        ],
        [0.0, -2.2, 0.3],
        11.53,
        15.85,
    );
    render_airframe(
        &mut h,
        FreeFlightVehicle::JsbsimP38g,
        P38G_GROUND,
        &p38_far,
        &dir,
    )?;
    let c172_far = far_views(
        [
            "c172_far_top",
            "c172_far_bottom",
            "c172_far_left_side",
            "c172_far_front",
            "c172_far_rear",
        ],
        [0.0, -1.8, 0.2],
        8.28,
        11.0,
    );
    render_airframe(
        &mut h,
        FreeFlightVehicle::JsbsimC172,
        C172_GROUND,
        &c172_far,
        &dir,
    )?;
    h.gpu.check_device()?;
    Ok(())
}

/// Model-frame extremes of the drawn (non-disc) geometry: `(min, max)`.
fn drawn_bounds(asset: &UsdAircraftAsset) -> (DVec3, DVec3) {
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for part in asset.parts() {
        if part.role == PartRole::PropellerDisc {
            continue;
        }
        for v in &part.vertices {
            let p = part
                .rest_to_aircraft
                .transform_point3(glam::Vec3::from(v.position).as_dvec3());
            lo = lo.min(p);
            hi = hi.max(p);
        }
    }
    (lo, hi)
}

#[test]
#[ignore = "writes drawing-registered orthographic overlays into $SIMULATE_VERIFY_OUT"]
fn render_fidelity_audit_ortho_overlays() -> Result<(), AppSimError> {
    let Some(dir) = out_dir()? else {
        return Ok(());
    };
    let p38 = load_model(HangarModel::P38g)?;
    for view in ortho::p38g_sheet_views()? {
        ortho::write_sheet_overlay(&p38, &view, &dir)?;
    }
    let c172 = load_model(HangarModel::C172)?;
    let (lo, hi) = drawn_bounds(&c172);
    for view in ortho::c172_sheet_views(hi.y, lo.z) {
        ortho::write_sheet_overlay(&c172, &view, &dir)?;
    }
    let mut report = String::new();
    for (name, asset) in [("p38g", &p38), ("c172", &c172)] {
        let (lo, hi) = drawn_bounds(asset);
        report.push_str(&format!(
            "{name}: parts {} triangles {} min {lo:.3} max {hi:.3} size {:.3}\n",
            asset.parts().len(),
            asset.triangle_count(),
            hi - lo
        ));
        for part in asset.parts() {
            report.push_str(&format!(
                "  {:<70} {:>6} tris  rough {:.2} metal {:.2} rgb {:?}\n",
                part.prim_path,
                part.indices.len() / 3,
                part.material.roughness,
                part.material.metallic,
                part.material.diffuse_rgb
            ));
        }
    }
    std::fs::write(dir.join("fidelity_metrics.txt"), report).map_err(|e| AppSimError::Screenshot {
        detail: e.to_string(),
    })
}
