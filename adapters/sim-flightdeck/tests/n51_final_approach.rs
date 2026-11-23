//! Verification of Solberg-Hunterdon (`N51`) Runway 22 destination airport runway resolution,
//! FlightAware Mode-C QNH pressure-altitude calibration, conformal velocity-vector alignment
//! with the real touchdown zone, and dynamic `DesignFrame` HUD emission.

use std::error::Error;

use sim_flightdeck::terrain::runways::{RunwayEyeAnchor, resolve_approach_and_overlays};
use sim_flightdeck::{CameraViewPose, FlightDeckSession, PlanetaryReference};

/// Real departure + pattern + short-final samples from `FlightAware_N9758H_N51_N51_20260830.kml`.
const N9758H_N51_FINAL_KML: &[u8] = br#"<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
<Document>
    <name>FlightAware N9758H 30-Aug-2026 (N51-N51)</name>
    <Placemark>
        <name>N51 Airport</name>
        <Point><coordinates>-74.7367067,40.5827872,0</coordinates></Point>
    </Placemark>
    <Placemark>
        <name>N51 Airport</name>
        <Point><coordinates>-74.7367067,40.5827872,0</coordinates></Point>
    </Placemark>
    <Placemark>
        <name>N9758H</name>
        <description>N51 - N51</description>
        <gx:Track>
            <when>2026-08-30T04:21:36Z</when>
            <when>2026-08-30T04:21:52Z</when>
            <when>2026-08-30T04:22:08Z</when>
            <when>2026-08-30T05:10:09Z</when>
            <when>2026-08-30T05:10:25Z</when>
            <when>2026-08-30T05:10:41Z</when>
            <when>2026-08-30T05:10:57Z</when>
            <when>2026-08-30T05:11:13Z</when>
            <when>2026-08-30T05:11:29Z</when>
            <when>2026-08-30T05:11:45Z</when>
            <when>2026-08-30T05:12:01Z</when>
            <when>2026-08-30T05:12:17Z</when>
            <when>2026-08-30T05:12:33Z</when>
            <gx:coord>-74.73871 40.57530 46</gx:coord>
            <gx:coord>-74.74048 40.57072 122</gx:coord>
            <gx:coord>-74.74152 40.56610 190</gx:coord>
            <gx:coord>-74.69536 40.60877 274</gx:coord>
            <gx:coord>-74.70041 40.61133 267</gx:coord>
            <gx:coord>-74.70599 40.61188 251</gx:coord>
            <gx:coord>-74.71046 40.61068 206</gx:coord>
            <gx:coord>-74.71484 40.60753 145</gx:coord>
            <gx:coord>-74.71796 40.60405 99</gx:coord>
            <gx:coord>-74.72107 40.60025 69</gx:coord>
            <gx:coord>-74.72394 40.59650 46</gx:coord>
            <gx:coord>-74.72717 40.59229 23</gx:coord>
        </gx:Track>
    </Placemark>
</Document>
</kml>"#;

#[test]
fn n51_final_resolves_rwy22_and_aligns_speed_vector_to_touchdown_zone() -> Result<(), Box<dyn Error>>
{
    let earth = PlanetaryReference::earth_wgs84_egm96();
    let mut session = FlightDeckSession::new_on(earth)?;
    session.load_kml_bytes(N9758H_N51_FINAL_KML, 1_000)?;

    let status = session.kml_status();
    let frac = ((status.duration_s - 7.0) / status.duration_s).clamp(0.0, 1.0);
    session.seek_kml_fraction(frac, 2_000);

    let snap = session.render_frame(2_000)?;
    let lat = snap.ownship.latitude_deg.ok_or("missing lat")?;
    let lon = snap.ownship.longitude_deg.ok_or("missing lon")?;
    let alt_m = snap.ownship.gnss_altitude_m.ok_or("missing alt")?;
    let gs_kt = snap.ownship.ground_speed_kt.ok_or("missing gs")?;
    let vsi_fpm = snap.ownship.vertical_speed_fpm.ok_or("missing vsi")?;
    let trk_deg = snap.ownship.ground_track_true_deg.ok_or("missing trk")?;

    assert!(
        (88.0..150.0).contains(&alt_m),
        "unexpected MSL {alt_m:.1} m"
    );
    assert!((55.0..75.0).contains(&gs_kt), "unexpected GS {gs_kt:.1} kt");
    assert!(
        (-420.0..-180.0).contains(&vsi_fpm),
        "unexpected VSI {vsi_fpm:.0} fpm"
    );
    assert!(
        (202.0..218.0).contains(&trk_deg),
        "unexpected TRK {trk_deg:.1}°T"
    );

    let own_flat = earth.geodetic_from_datum(lat, lon, 0.0);
    let eye = RunwayEyeAnchor {
        latitude_deg: lat,
        longitude_deg: lon,
        altitude_datum_m: alt_m,
        onboard_dem_datum_m: None,
    };
    let (overlays, approach) = resolve_approach_and_overlays(
        &snap.ownship,
        &earth,
        &own_flat,
        alt_m,
        eye,
        Some("N51 → N51"),
    );
    assert!(overlays.iter().any(|r| r.label == "N51 RWY 22"));
    let sel = approach.ok_or("expected active ApproachRunwaySelection")?;
    assert_eq!(sel.airport_code, "N51");
    assert_eq!(sel.designator, "22");

    let (sin_c, cos_c) = sel.hud_runway.course_rad.sin_cos();
    let aim_north = sel.hud_runway.threshold_m[0] + 180.0 * cos_c;
    let aim_east = sel.hud_runway.threshold_m[1] + 180.0 * sin_c;
    let aim_horiz = libm::hypot(f64::from(aim_north), f64::from(aim_east));
    let aim_gamma_deg =
        libm::atan2(f64::from(-sel.hud_runway.threshold_m[2]), aim_horiz).to_degrees();
    let fpv_gamma_deg = libm::atan2(vsi_fpm * 0.005_08, gs_kt * 0.514_444).to_degrees();
    assert!(
        (fpv_gamma_deg - aim_gamma_deg).abs() < 0.75,
        "speed vector gamma ({fpv_gamma_deg:.2}°) must align with RWY 22 aiming point ({aim_gamma_deg:.2}°)"
    );
    Ok(())
}

#[test]
fn n51_final_hud_clears_loc_gs_missing_and_exports_airport_runways_geojson()
-> Result<(), Box<dyn Error>> {
    let earth = PlanetaryReference::earth_wgs84_egm96();
    let mut session = FlightDeckSession::new_on(earth)?;
    session.load_kml_bytes(N9758H_N51_FINAL_KML, 1_000)?;
    session.seek_kml_fraction(0.997, 2_000);

    let snap = session.render_frame(2_000)?;
    let hud_ascii = String::from_utf8_lossy(&snap.instruments.hud_scene);
    assert!(
        !hud_ascii.contains("LOC GS MISSING"),
        "HUD must show LNAV/VNAV approach guidance instead of LOC GS MISSING"
    );

    let lat = snap.ownship.latitude_deg.ok_or("missing lat")?;
    let lon = snap.ownship.longitude_deg.ok_or("missing lon")?;
    let alt_m = snap.ownship.gnss_altitude_m.ok_or("missing alt")?;
    let trk_deg = snap.ownship.ground_track_true_deg.ok_or("missing trk")?;
    session.set_camera_view_pose(
        Some(CameraViewPose {
            latitude_deg: lat,
            longitude_deg: lon,
            altitude_datum_m: alt_m,
            true_heading_deg: trk_deg,
            pitch_deg: snap.ownship.pitch_deg.unwrap_or(0.0),
            roll_deg: snap.ownship.roll_deg.unwrap_or(0.0),
        }),
        Some(58.0),
    );
    let _ = session.render_frame(2_020)?;
    let custom_hud = session.emit_hud_scene(2_180.0, 1_520.0)?;
    assert!(!custom_hud.is_empty());

    let geojson = session.airport_runways_geojson(lat, lon, 30.0);
    assert!(geojson.contains("\"N51 RWY 22\""));
    assert!(geojson.contains("\"dest\":1"));
    assert!(geojson.contains("\"kind\":\"outline\""));
    assert!(geojson.contains("\"kind\":\"threshold\""));
    Ok(())
}
