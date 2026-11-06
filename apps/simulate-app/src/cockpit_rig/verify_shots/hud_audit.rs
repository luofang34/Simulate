//! Multi-airport HUD conformal runway vs. live ortho/DEM raster audit, post-landing
//! rollout verification, and polar mini-map `Z3.5 -> Z5.1` + trackpad twist verification.

use std::path::{Path, PathBuf};

use glam::DVec3;
use sim_core::ControlInputs;
use sim_geo::{BodyEllipsoid, LocalFrame, ecef_to_geodetic, egm96_undulation_m};
use sim_maplibre::EfbDisplayMode;
use sim_render::{OverlayQuad, ViewportRect};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::{AirportDatabase, AirportIdx, AirportRecord, LandingRunwayGeometry};

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::avionics::{ConformalSurface, hud_runway};
use crate::error::AppSimError;
use crate::esc_menu::{OverlayContent, SpawnMode};
use crate::map_rig::{AvionicsPreset, EfbCommand};
use crate::sensor_bus::SensorRealism;
use crate::sky_rig::SkyFrame;

const AIRPORTS: [(&str, &str); 6] = [
    ("KDOV", "01"),
    ("KEWR", "22L"),
    ("47N", "07"),
    ("KSFO", "28R"),
    ("KDEN", "16R"),
    ("NZSP", "02T"),
];

#[test]
#[ignore = "renders polar zoom/rotate and multi-airport HUD-vs-raster audit into $SIMULATE_VERIFY_OUT"]
fn render_hud_vs_raster_and_polar_zoom_audit() -> Result<(), AppSimError> {
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

    render_polar_ross_sea_zoom_and_twist(&mut h, &dir)?;

    let mut report = String::from(
        "# Multi-Airport HUD vs. Map Raster & Post-Landing Audit\n\n\
         | Airport | RWY | Auth / Eff | Disp Thr (m) | Thr->Far MSL (m) | Far Drop (m) | GP (deg) | DEM-FAA Thr (m) | Geoid N (m) | Realistic Pos NED (m) | Rollout Eye Down (m) |\n\
         |---|---|---|---:|---|---:|---:|---:|---:|---|---:|\n",
    );
    for (code, designator) in AIRPORTS {
        let row = audit_airport(&mut h, &dir, code, designator)?;
        report.push_str(&row);
        report.push('\n');
    }
    std::fs::write(dir.join("hud_vs_raster_audit.md"), report).map_err(|e| {
        AppSimError::Screenshot {
            detail: e.to_string(),
        }
    })?;
    h.gpu.check_device()?;
    Ok(())
}

fn render_polar_ross_sea_zoom_and_twist(
    h: &mut VerifyHarness,
    dir: &Path,
) -> Result<(), AppSimError> {
    let (nzsp_idx, nzsp_rec) = lookup_airport("NZSP")?;
    h.rig.supervisor_mut().set_free_airport(nzsp_idx, 0);
    h.flight
        .spawn_at_airport_record(nzsp_rec, 0, SpawnMode::Threshold)?;
    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb))?;
    // Sync EFB camera to NZSP (90S) first before panning off ownship toward Ross Sea.
    settle_and_save(h, 3, &dir.join("17_polar_ross_sea_efb_z3_5.png"))?;
    // Default ExpandedEfb zoom is 11.6; -8.1 lands at Z3.5, matching screenshot 1.
    h.rig.efb_command(EfbCommand::ZoomBy(-8.1))?;
    h.rig.efb_command(EfbCommand::PanPixels([90.0, -30.0]))?;
    settle_and_save(h, 45, &dir.join("17_polar_ross_sea_efb_z3_5.png"))?;

    // Pinch-open zoom +1.6 levels from Z3.5 -> Z5.1, matching screenshot 2.
    h.rig.efb_command(EfbCommand::ZoomBy(1.6))?;
    settle_and_save(h, 45, &dir.join("18_polar_ross_sea_efb_z5_1.png"))?;

    // Two-finger trackpad twist +40 deg counter-clockwise -> BRG UP.
    h.rig.efb_command(EfbCommand::RotateBy(40.0))?;
    let status =
        h.rig
            .maps()
            .and_then(|m| m.efb_status())
            .ok_or_else(|| AppSimError::Screenshot {
                detail: "missing EFB status".to_owned(),
            })?;
    assert!(
        status.custom_bearing,
        "RotateBy must engage custom_bearing (BRG UP)"
    );
    settle_and_save(h, 25, &dir.join("19_polar_ross_sea_efb_rotated_brg_up.png"))?;

    h.rig.efb_command(EfbCommand::Recenter)?;
    h.rig.efb_command(EfbCommand::ToggleNorthUp)?;
    h.rig.efb_command(EfbCommand::ToggleNorthUp)?;
    h.rig.efb_command(EfbCommand::ZoomBy(6.5))?;
    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner))?;
    Ok(())
}

fn audit_airport(
    h: &mut VerifyHarness,
    dir: &Path,
    code: &str,
    designator: &str,
) -> Result<String, AppSimError> {
    let (idx, rec) = lookup_airport(code)?;
    let datum = h.rig.supervisor().vertical_datum();
    let rwy_idx = rec
        .to_flightbag_graph(datum)
        .runway_ends
        .iter()
        .position(|e| e.designator.as_str().eq_ignore_ascii_case(designator))
        .ok_or_else(|| AppSimError::Screenshot {
            detail: format!("missing runway {designator} in {code} graph"),
        })?;
    h.rig.supervisor_mut().set_free_airport(idx, rwy_idx);
    let landing = h
        .rig
        .supervisor()
        .active_landing_runway(None)
        .ok_or_else(|| AppSimError::Screenshot {
            detail: format!("missing active_landing_runway for {code} {designator}"),
        })?;

    let sun = sun_for_airport(rec, h);
    let slug = format!(
        "{}_{}",
        code.to_ascii_lowercase(),
        designator.to_ascii_lowercase()
    );

    // 1. Short final (~0.51 NM / 950 m out on published glidepath) in Ideal nav to isolate
    // surveyed navdata vs. Esri/Terrarium raster registration, warming live tiles first.
    h.flight.set_sensor_realism(SensorRealism::Ideal);
    h.flight.spawn_at_landing_station(rec, designator, -950.0)?;
    settle_and_save_sun(
        h,
        50,
        sun,
        &dir.join(format!("20_{slug}_short_final_ideal.png")),
    )?;

    // 2. Post-landing rollout (+180 m past landing threshold on resident DEM) in Realistic
    // WAAS/AHRS mode, verifying centreline dashes, GP suppression, and rollout height guard.
    h.flight.set_sensor_realism(SensorRealism::Realistic);
    h.flight.spawn_at_landing_station(rec, designator, 180.0)?;
    for _ in 0..20 {
        h.flight
            .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
    }
    settle_and_save_sun(
        h,
        35,
        sun,
        &dir.join(format!("21_{slug}_rollout_realistic.png")),
    )?;

    format_audit_row(h, code, &landing)
}

fn format_audit_row(
    h: &VerifyHarness,
    code: &str,
    landing: &LandingRunwayGeometry,
) -> Result<String, AppSimError> {
    let surface = h.flight.world_surface();
    let dem_msl = surface.orthometric_ground_m(&landing.threshold);
    let dem_minus_faa = dem_msl.map(|d| d - landing.threshold_elevation_msl_m);
    let geoid_n = egm96_undulation_m(landing.threshold.lat_rad, landing.threshold.lon_rad);

    let truth = h.flight.display_vehicle_state();
    let est = h.flight.display_estimated_vehicle_state();
    let own_local = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef));
    let pos_err_ned = own_local.ecef_to_ned_vector(est.position_ecef - truth.position_ecef);

    let datum = Some(h.rig.supervisor().vertical_datum());
    let hud_surface = ConformalSurface::earth_globe();
    let cam = h.flight.display_camera(OperatorViewportMode::Cockpit)?;
    let lever = truth.attitude.inverse() * (cam.pose.position_ecef - truth.position_ecef);
    let est_eye = est.position_ecef + est.attitude * lever;
    let hud_rwy = hud_runway(est_eye, hud_surface, datum, landing);
    let (sin_c, cos_c) = hud_rwy.course_rad.sin_cos();
    let station_m = -(hud_rwy.threshold_m[0] * cos_c + hud_rwy.threshold_m[1] * sin_c);
    assert!(
        hud_rwy.length_m > 400.0 && station_m > 100.0,
        "{code} rollout station must be > 100 m past threshold: station={station_m:.1} m, thr={:?}",
        hud_rwy.threshold_m
    );
    let eff = format!(
        "{:?} {:04}-{:02}-{:02}",
        landing.authority,
        landing.effective_ymd[0],
        landing.effective_ymd[1],
        landing.effective_ymd[2]
    );
    let dem_str = dem_minus_faa.map_or_else(|| "n/a".to_owned(), |d| format!("{d:+.2}"));
    let thr_eye_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&landing.threshold)
        + LocalFrame::at(&landing.threshold).up() * 1.4;
    let thr_hud = hud_runway(thr_eye_ecef, hud_surface, datum, landing);
    Ok(format!(
        "| `{code}` | `{}` | {eff} | {:.1} | {:.1} -> {:.1} | {:+.2} | {:.2} | {dem_str} | {:+.2} | `[{:+.2}, {:+.2}, {:+.2}]` | {:+.2} |",
        landing.designator,
        landing.displaced_threshold_m,
        landing.threshold_elevation_msl_m,
        landing.far_elevation_msl_m,
        thr_hud.far_down_offset_m,
        landing.glidepath_rad.to_degrees(),
        geoid_n,
        pos_err_ned.x,
        pos_err_ned.y,
        pos_err_ned.z,
        hud_rwy.threshold_m[2],
    ))
}

fn sun_for_airport(rec: &AirportRecord, h: &VerifyHarness) -> DVec3 {
    if rec.lat_deg < -80.0 {
        DVec3::new(0.45, 0.35, -0.55).normalize()
    } else {
        // High-elevation afternoon sun over North America so live orthoimagery is bright.
        let local = LocalFrame::at(&sim_geo::Geodetic::from_degrees(
            rec.lat_deg,
            rec.lon_deg,
            0.0,
        ));
        (local.up() * 0.78 - local.north() * 0.35 - local.east() * 0.52).normalize()
    }
    .normalize_or(h.flight.world().sun_direction_ecef())
}

fn lookup_airport(code: &str) -> Result<(AirportIdx, &'static AirportRecord), AppSimError> {
    AirportDatabase::builtin()
        .by_code(code)
        .ok_or_else(|| AppSimError::Screenshot {
            detail: format!("missing {code} in AirportDatabase"),
        })
}

fn settle_and_save(h: &mut VerifyHarness, frames: usize, out: &Path) -> Result<(), AppSimError> {
    let sun = h.flight.world().sun_direction_ecef();
    settle_and_save_sun(h, frames, sun, out)
}

fn settle_and_save_sun(
    h: &mut VerifyHarness,
    frames: usize,
    sun_ecef: DVec3,
    out: &Path,
) -> Result<(), AppSimError> {
    let sky = SkyFrame::fixed_sun(sun_ecef);
    for _ in 0..frames {
        let mode = h.rig.supervisor().viewport_mode();
        let cam = h.flight.display_camera(mode)?;
        h.rig.invalidate_chart();
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(30));
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
