//! Runway height audit: the charted runway, the world-surface ground, the terrain the
//! physical scene draws, the SVS DEM, the JSBSim ground and the parked wheels, measured in
//! one frame at several airports while terrain tiles stream in.

use std::fmt::Write as _;
use std::path::PathBuf;

use sim_core::ControlInputs;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::{AirportDatabase, LandingRunwayGeometry};

use super::{SHOT_SIZE, VerifyHarness, poll_gpu};
use crate::error::AppSimError;
use crate::esc_menu::SpawnMode;
use crate::map_rig::AvionicsPreset;
use crate::sky_rig::SkyFrame;
use crate::vehicles::C172_GEAR_HEIGHT_M;

const AIRPORTS: [(&str, &str); 8] = [
    ("N51", "04"),
    ("KEWR", "22L"),
    ("KEWR", "04R"),
    ("47N", "07"),
    ("KDOV", "01"),
    ("KSFO", "28R"),
    ("KDEN", "16R"),
    ("VCBI", "04"),
];
const CHECKPOINTS: [usize; 4] = [1, 15, 90, 240];

fn shot_error(detail: impl ToString) -> AppSimError {
    AppSimError::Screenshot {
        detail: detail.to_string(),
    }
}

#[test]
#[ignore = "measures runway heights against drawn terrain into $SIMULATE_VERIFY_OUT"]
fn measure_runway_datum_at_airports() -> Result<(), AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    std::fs::create_dir_all(&dir).map_err(shot_error)?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.rig.set_avionics_preset(AvionicsPreset::GlassCockpit)?;
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    h.rig.set_show_conformal_hud(true);
    let mut report = String::from(
        "| apt rwy | frame | point | N | chart MSL | surface MSL | drawn MSL | loaded MSL | SVS MSL | HUD MSL | FDM MSL | wheels MSL |\n\
         |---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n",
    );
    let only = std::env::var("SIMULATE_DATUM_AIRPORTS").ok();
    for (code, designator) in AIRPORTS {
        if only
            .as_ref()
            .is_some_and(|only| !only.split(',').any(|c| c == code))
        {
            continue;
        }
        audit_airport(&mut h, code, designator, &mut report)?;
        let slug = format!(
            "{}_{}",
            code.to_ascii_lowercase(),
            designator.to_ascii_lowercase()
        );
        let quads = h.rig.overlay_quads(h.flight.world());
        h.composite_to_png(&quads, &dir.join(format!("datum_{slug}.png")))?;
    }
    tracing::info!("runway datum audit\n{report}");
    std::fs::write(dir.join("runway_datum.md"), report).map_err(shot_error)?;
    Ok(())
}

fn audit_airport(
    h: &mut VerifyHarness,
    code: &str,
    designator: &str,
    report: &mut String,
) -> Result<(), AppSimError> {
    let (idx, rec) = AirportDatabase::builtin()
        .by_code(code)
        .ok_or_else(|| shot_error(format!("no {code}")))?;
    let datum = h.rig.supervisor().vertical_datum();
    let rwy = rec
        .to_flightbag_graph(datum)
        .runway_ends
        .iter()
        .position(|e| e.designator.as_str().eq_ignore_ascii_case(designator))
        .ok_or_else(|| shot_error(format!("no {code} {designator}")))?;
    h.rig.supervisor_mut().set_free_airport(idx, rwy);
    h.flight
        .spawn_at_airport_record(rec, rwy, SpawnMode::Threshold)?;
    let landing = rec
        .landing_runway(designator, datum)
        .ok_or_else(|| shot_error(format!("no landing {code} {designator}")))?;
    let sky = SkyFrame::fixed_sun(
        h.flight.world().body(),
        h.flight.world().sun_direction_ecef(),
    );
    let last = CHECKPOINTS.iter().copied().max().unwrap_or(1);
    for frame in 1..=last {
        h.flight
            .advance_by(1.0 / 30.0, ControlInputs::default(), SimTimeScale::RealTime)?;
        let cam = h.flight.display_camera(OperatorViewportMode::Cockpit)?;
        let _globe = h
            .rig
            .prepare_frame(&h.gpu.queue, SHOT_SIZE, &h.flight, &cam, &sky, &[])?;
        poll_gpu(&h.gpu)?;
        std::thread::sleep(std::time::Duration::from_millis(20));
        if CHECKPOINTS.contains(&frame) {
            record(h, &landing, &format!("{code} {designator}"), frame, report)?;
        }
    }
    Ok(())
}

/// The point `along_m` down the runway from its threshold, with its charted MSL height.
fn along_runway(landing: &LandingRunwayGeometry, along_m: f64) -> (Geodetic, f64) {
    let frame = LocalFrame::at(&landing.threshold);
    let (sin_b, cos_b) = landing.true_bearing_rad.sin_cos();
    let ahead = frame.north() * cos_b + frame.east() * sin_b;
    let at = ecef_to_geodetic(geodetic_to_ecef(&landing.threshold) + ahead * along_m);
    let frac = (along_m / landing.landing_length_m.max(1.0)).clamp(0.0, 1.0);
    let msl = landing.threshold_elevation_msl_m
        + (landing.far_elevation_msl_m - landing.threshold_elevation_msl_m) * frac;
    (at, msl)
}

/// How far down the runway the ownship is, and a label with its cross-track offset,
/// heading error against the runway and the live tiles resident.
fn ownship_on_runway(
    landing: &LandingRunwayGeometry,
    state: &sim_core::VehicleState,
    live_tiles: usize,
) -> (f64, String) {
    let own = ecef_to_geodetic(state.position_ecef);
    let frame = LocalFrame::at(&landing.threshold);
    let d = state.position_ecef - geodetic_to_ecef(&landing.threshold);
    let (sin_b, cos_b) = landing.true_bearing_rad.sin_cos();
    let along = d.dot(frame.north() * cos_b + frame.east() * sin_b);
    let across = d.dot(frame.east() * cos_b - frame.north() * sin_b);
    let local = LocalFrame::at(&own);
    let nose = state.attitude * glam::DVec3::X;
    let hdg = nose.dot(local.east()).atan2(nose.dot(local.north()));
    let heading_err_deg = (hdg - landing.true_bearing_rad + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        .to_degrees()
        - 180.0;
    (
        along,
        format!("ownship xt {across:+.2} m hdg {heading_err_deg:+.3}° live {live_tiles}"),
    )
}

fn record(
    h: &VerifyHarness,
    landing: &LandingRunwayGeometry,
    label: &str,
    frame: usize,
    report: &mut String,
) -> Result<(), AppSimError> {
    let maps = h.rig.maps().ok_or_else(|| shot_error("no maps"))?;
    let truth = maps.truth();
    let surface = h.flight.world_surface();
    let state = h.flight.vehicle_state();
    let own = ecef_to_geodetic(state.position_ecef);
    let (along_own, own_label) = ownship_on_runway(landing, &state, surface.live_dem_len());
    let points = [
        ("threshold", 0.0),
        (own_label.as_str(), along_own),
        ("mid", landing.landing_length_m * 0.5),
        ("far -100 m", landing.landing_length_m - 100.0),
    ];
    let hud = h
        .rig
        .supervisor()
        .active_landing_runway(None)
        .map(|rwy| {
            surface
                .runway_datum()
                .touchdown_plane(&rwy, surface.vertical_datum())
        })
        .ok_or_else(|| shot_error("no active runway"))?;
    let fmt = |v: Option<f64>| v.map_or_else(|| "-".to_owned(), |v| format!("{v:.2}"));
    for (name, along) in points {
        let (mut at, chart) = along_runway(landing, along);
        let (_, hud_msl) = along_runway(&hud, along);
        if name.starts_with("ownship") {
            at = Geodetic {
                height_m: 0.0,
                ..own
            };
        }
        let n = surface.geoid_undulation_m(at.lat_rad, at.lon_rad);
        let (lat, lon) = (at.lat_deg(), at.lon_deg());
        let (fdm, wheels) = if name.starts_with("ownship") {
            (
                h.flight.fdm_ground_m().map(|g| g - n),
                Some(own.height_m - C172_GEAR_HEIGHT_M - n),
            )
        } else {
            (None, None)
        };
        writeln!(
            report,
            "| {label} | {frame} | {name} | {n:.2} | {chart:.2} | {} | {} | {} | {} | {hud_msl:.2} | {} | {} |",
            fmt(surface.orthometric_ground_m(&at)),
            fmt(truth.drawn_ground_msl(lat, lon)),
            fmt(truth.loaded_ground_msl(lat, lon)),
            fmt(maps.sample_svs_dem_msl(lat, lon)),
            fmt(fdm),
            fmt(wheels),
        )
        .map_err(shot_error)?;
    }
    Ok(())
}
