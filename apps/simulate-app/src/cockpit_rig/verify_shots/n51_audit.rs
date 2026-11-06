//! Visual and quantitative data-source audit for `N51` (Solberg-Hunterdon) and nearby NJ
//! `RWY 07` airfields (`47N` Central Jersey Regional, `N07` Lincoln Park, etc.), comparing
//! surveyed NASR runway records, live AWS Terrarium DEM (`z = 14` SVS vs `z = 15` OTW),
//! Esri World Imagery raster, conformal HUD symbology, PFD SVS 3D overlay, and EFB charts.

use std::path::{Path, PathBuf};

use glam::DVec3;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_maplibre::EfbDisplayMode;
use sim_render::{OverlayQuad, ViewportRect};
use sim_scenario::OperatorViewportMode;
use sim_scenery::{AirportDatabase, AirportIdx, AirportRecord, CatalogVerticalDatum};

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::error::AppSimError;
use crate::esc_menu::OverlayContent;
use crate::map_rig::{AvionicsPreset, EfbCommand, PFD_SVS_MAP};
use crate::sensor_bus::SensorRealism;
use crate::sky_rig::SkyFrame;

#[test]
#[ignore = "renders N51 + NJ RWY 07 SVS, HUD, OTW, and EFB diagnostic shots into $SIMULATE_VERIFY_OUT"]
fn render_n51_and_rwy07_svs_data_audit() -> Result<(), AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    std::fs::create_dir_all(&dir).map_err(|e| AppSimError::Screenshot {
        detail: e.to_string(),
    })?;
    dump_nj_rwy07_candidates();

    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.rig.set_avionics_preset(AvionicsPreset::GlassCockpit)?;
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    h.rig.set_show_conformal_hud(true);
    h.flight.set_sensor_realism(SensorRealism::Ideal);

    for (code, desig, slug) in [
        ("N51", "04", "n51_04"),
        ("N51", "22", "n51_22"),
        ("N51", "10", "n51_10"),
        ("47N", "07", "47n_07"),
    ] {
        audit_airport_runway(&mut h, &dir, code, desig, slug)?;
    }
    render_47n_short_final_realism_pair(&mut h, &dir)?;

    // Verify CompactCorner mini-map collapse (`HIDE` -> `MAP` button, map quad hidden).
    if let Some((idx, rec)) = AirportDatabase::builtin().by_code("N51") {
        select_airport_runway(&mut h, idx, rec, "04")?;
        let local = LocalFrame::at(&Geodetic::from_degrees(rec.lat_deg, rec.lon_deg, 0.0));
        let sun = (local.up() * 0.78 - local.north() * 0.35 - local.east() * 0.52).normalize();
        h.rig
            .efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner))?;
        h.rig.efb_command(EfbCommand::SetCollapsed(true))?;
        h.flight.spawn_at_landing_station(rec, "04", -550.0)?;
        settle_cockpit(
            &mut h,
            25,
            sun,
            &dir.join("25_n51_04_minimap_collapsed.png"),
        )?;
        h.rig.efb_command(EfbCommand::SetCollapsed(false))?;
    }

    h.gpu.check_device()?;
    Ok(())
}

fn render_47n_short_final_realism_pair(
    h: &mut VerifyHarness,
    dir: &Path,
) -> Result<(), AppSimError> {
    let Some((idx, rec)) = AirportDatabase::builtin().by_code("47N") else {
        return Ok(());
    };
    select_airport_runway(h, idx, rec, "07")?;
    let local = LocalFrame::at(&Geodetic::from_degrees(rec.lat_deg, rec.lon_deg, 0.0));
    let sun = (local.up() * 0.78 - local.north() * 0.35 - local.east() * 0.52).normalize();
    h.flight.set_sensor_realism(SensorRealism::Ideal);
    h.flight.spawn_at_landing_station(rec, "07", -280.0)?;
    settle_cockpit(h, 45, sun, &dir.join("26_47n_07_280m_ideal.png"))?;
    h.flight.set_sensor_realism(SensorRealism::Realistic);
    h.flight.spawn_at_landing_station(rec, "07", -280.0)?;
    settle_cockpit(h, 25, sun, &dir.join("27_47n_07_280m_realistic.png"))?;
    h.flight.set_sensor_realism(SensorRealism::Ideal);
    Ok(())
}

fn dump_nj_rwy07_candidates() {
    let db = AirportDatabase::builtin();
    tracing::info!("=== Airports near N51 (within ~0.6 deg) with RWY 07 or N* code ===");
    for rec in db.records() {
        let near = (rec.lat_deg - 40.58).abs() < 0.65 && (rec.lon_deg - (-74.74)).abs() < 0.85;
        let has_07 = rec.runways.iter().any(|r| {
            r.ends
                .iter()
                .any(|e| e.designator.as_str().eq_ignore_ascii_case("07"))
        });
        let code = rec.ids.primary.as_str();
        if near && (has_07 || code.starts_with('N') || code.contains("51") || code.contains("47")) {
            let rwys: Vec<String> = rec
                .runways
                .iter()
                .map(|r| {
                    let [e0, e1] = &r.ends;
                    format!(
                        "{}({:.0}x{:.0}m {:?} disp={:.0}/{:.0}m)",
                        r.id,
                        r.length_m,
                        r.width_m,
                        r.surface,
                        e0.displaced_threshold_m,
                        e1.displaced_threshold_m,
                    )
                })
                .collect();
            tracing::info!(
                "  code={} name={:?} city={:?} lat={:.4} lon={:.4} rwys={}",
                code,
                rec.name,
                rec.municipality,
                rec.lat_deg,
                rec.lon_deg,
                rwys.join(", ")
            );
        }
    }
}

fn audit_airport_runway(
    h: &mut VerifyHarness,
    dir: &Path,
    code: &str,
    desig: &str,
    slug: &str,
) -> Result<(), AppSimError> {
    let (idx, rec) =
        AirportDatabase::builtin()
            .by_code(code)
            .ok_or_else(|| AppSimError::Screenshot {
                detail: format!("missing {code} in AirportDatabase"),
            })?;
    tracing::info!("==================== {code} RWY {desig} ====================");
    dump_airport_record(code, rec);
    select_airport_runway(h, idx, rec, desig)?;

    let local = LocalFrame::at(&Geodetic::from_degrees(rec.lat_deg, rec.lon_deg, 0.0));
    let sun = (local.up() * 0.78 - local.north() * 0.35 - local.east() * 0.52).normalize();

    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner))?;
    h.flight.spawn_at_landing_station(rec, desig, -550.0)?;
    settle_and_save_all(
        h,
        65,
        sun,
        &dir.join(format!("22_{slug}_cockpit_svs_hud.png")),
        &dir.join(format!("23_{slug}_pfd_svs_raw.png")),
    )?;
    dump_dem_profile(h, rec, desig);

    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb))?;
    h.rig.efb_command(EfbCommand::ZoomBy(3.0))?;
    settle_cockpit(h, 30, sun, &dir.join(format!("24_{slug}_expanded_efb.png")))?;
    h.rig.efb_command(EfbCommand::ZoomBy(-3.0))?;
    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner))?;
    Ok(())
}

fn select_airport_runway(
    h: &mut VerifyHarness,
    idx: AirportIdx,
    rec: &AirportRecord,
    desig: &str,
) -> Result<(), AppSimError> {
    let datum = h.rig.supervisor().vertical_datum();
    let graph = rec.to_flightbag_graph(datum);
    let rwy_idx = graph
        .runway_ends
        .iter()
        .position(|e| e.designator.as_str().eq_ignore_ascii_case(desig))
        .ok_or_else(|| AppSimError::Screenshot {
            detail: format!("missing RWY {desig} in {} flightbag graph", rec.ids.primary),
        })?;
    h.rig.supervisor_mut().set_free_airport(idx, rwy_idx);
    Ok(())
}

fn dump_airport_record(code: &str, rec: &AirportRecord) {
    tracing::info!(
        "{code} record: name={:?} city={:?} lat={:.7} lon={:.7} elev_msl={:.2}m mag_var={:.2}deg auth={:?} eff={:?}",
        rec.name,
        rec.municipality,
        rec.lat_deg,
        rec.lon_deg,
        rec.elevation_msl_m,
        rec.mag_var_deg,
        rec.authority,
        rec.effective_ymd
    );
    for rwy in &rec.runways {
        tracing::info!(
            "  rwy id={} len={:.2}m wid={:.2}m surf={:?} lighted={}",
            rwy.id,
            rwy.length_m,
            rwy.width_m,
            rwy.surface,
            rwy.lighted
        );
        for end in &rwy.ends {
            tracing::info!(
                "    end={} lat={:.7} lon={:.7} elev_msl={:.2}m disp={:.2}m true_hdg={:.2} mag_hdg={:.2} gp={:.2}",
                end.designator,
                end.lat_deg,
                end.lon_deg,
                end.elevation_msl_m,
                end.displaced_threshold_m,
                end.true_heading_deg,
                end.magnetic_heading_deg,
                end.glidepath_deg
            );
        }
    }
}

fn dump_dem_profile(h: &VerifyHarness, rec: &AirportRecord, designator: &str) {
    let Some(landing) = rec.landing_runway(designator, CatalogVerticalDatum::OrthometricMslEgm96)
    else {
        return;
    };
    tracing::info!(
        "LandingRunwayGeometry {}: full_len={:.2}m disp={:.2}m land_len={:.2}m wid={:.2}m start_msl={:.2}m thr_msl={:.2}m far_msl={:.2}m brg={:.3}deg",
        landing.designator,
        landing.full_length_m,
        landing.displaced_threshold_m,
        landing.landing_length_m,
        landing.width_m,
        landing.start_elevation_msl_m,
        landing.threshold_elevation_msl_m,
        landing.far_elevation_msl_m,
        landing.true_bearing_rad.to_degrees(),
    );
    let a = geodetic_to_ecef(&Geodetic::from_degrees(
        landing.pavement_start.lat_deg(),
        landing.pavement_start.lon_deg(),
        0.0,
    ));
    let b = geodetic_to_ecef(&Geodetic::from_degrees(
        landing.far_end.lat_deg(),
        landing.far_end.lon_deg(),
        0.0,
    ));
    let surface = h.flight.world_surface();
    for step in 0..=10 {
        let frac = f64::from(step) / 10.0;
        let pt = ecef_to_geodetic(a + (b - a) * frac);
        let nav_msl = landing.start_elevation_msl_m
            + (landing.far_elevation_msl_m - landing.start_elevation_msl_m) * frac;
        let flat_thr_msl = landing.start_elevation_msl_m;
        let dem_msl = surface.orthometric_ground_m(&pt);
        tracing::info!(
            "  frac={frac:.1} dist={:.1}m lat={:.6} lon={:.6} nav_chord={nav_msl:.2}m flat_start={flat_thr_msl:.2}m dem_msl={dem_msl:?} dem_minus_chord={:?} dem_minus_flat={:?}",
            landing.full_length_m * frac,
            pt.lat_deg(),
            pt.lon_deg(),
            dem_msl.map(|d| d - nav_msl),
            dem_msl.map(|d| d - flat_thr_msl),
        );
    }
}

fn settle_and_save_all(
    h: &mut VerifyHarness,
    frames: usize,
    sun_ecef: DVec3,
    cockpit_out: &Path,
    svs_out: &Path,
) -> Result<(), AppSimError> {
    settle_cockpit(h, frames, sun_ecef, cockpit_out)?;
    if let Some(svs) = h
        .rig
        .maps()
        .and_then(|maps| maps.instrument_view(PFD_SVS_MAP))
    {
        let quads = [OverlayQuad::inset_map(ViewportRect::FULL, svs, false)];
        h.composite_to_png(&quads, svs_out)?;
    }
    Ok(())
}

fn settle_cockpit(
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
        std::thread::sleep(std::time::Duration::from_millis(35));
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
