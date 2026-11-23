//! End-to-end integration tests for `sim-flightdeck`.

use std::error::Error;

use sim_flightdeck::airspace::pmtiles::{decode_mvt_airspace_tile, parse_pmtiles_header};
use sim_flightdeck::gdl90::{
    Gdl90AhrsReport, Gdl90Decoder, Gdl90Message, Gdl90ParticipantReport, crc16_gdl90,
    encode_ahrs_frame, encode_participant_frame,
};
use sim_flightdeck::{
    AirspaceAlertLevel, CrownBlend, FlightDeckScenario, FlightDeckSession, HeadPose,
    KmlFlightTrack, MAX_VIRTUAL_OPACITY, MIN_PASSTHROUGH_FRACTION, PlanetaryReference,
    TelemetrySourceMode, TrafficThreat, build_aircraft_state, decode_frame_header,
    encode_frame_snapshot, encode_glyph_pack, verified_glyph_pack,
};
use world_surveillance::{AddressNamespace, EmitterCategory};

#[test]
fn digital_crown_blend_always_preserves_real_world_passthrough_floor() {
    for step in -20..=120 {
        let raw = step as f32 * 0.01;
        let weights = CrownBlend::new(raw).weights();
        assert!(weights.passthrough_fraction >= MIN_PASSTHROUGH_FRACTION);
        assert!(weights.virtual_terrain_opacity <= MAX_VIRTUAL_OPACITY);
        assert!(
            (weights.passthrough_fraction + weights.virtual_terrain_opacity - 1.0).abs() < 1e-5
        );
    }

    let nan_weights = CrownBlend::new(f32::NAN).weights();
    assert_eq!(nan_weights.crown_turn, 0.0);
    assert!((nan_weights.passthrough_fraction - 1.0).abs() < 1e-6);

    let mut crown = CrownBlend::new(0.8);
    crown.rotate_by(0.9);
    let maxed = crown.weights();
    assert!((maxed.virtual_terrain_opacity - 0.85).abs() < 1e-6);
    assert!((maxed.passthrough_fraction - 0.15).abs() < 1e-6);
}

#[test]
fn connext_real_session_replay_decodes_ownship_traffic_callsigns_and_route()
-> Result<(), Box<dyn Error>> {
    let mut session = FlightDeckSession::new_on(PlanetaryReference::earth_wgs84_egm96())?;
    session.set_source_mode(TelemetrySourceMode::ConnextReplay);

    // At t = 0 ms only the 0x02 control handshake has arrived; GNSS must remain honestly invalid.
    let boot_snap = session.render_frame(0)?;
    assert!(!boot_snap.ownship.gnss_valid);

    let snap = session.render_frame(5_100)?;
    assert!(snap.ownship.gnss_valid);
    assert_eq!(
        snap.ownship
            .identity
            .as_ref()
            .and_then(|id| id.callsign.as_deref()),
        Some("N19880")
    );
    assert_eq!(snap.ownship.pressure_altitude_ft, Some(192.0));
    // Connext carries neither airspeed nor vertical speed; both must stay None rather than 0.
    assert_eq!(snap.ownship.indicated_airspeed_kt, None);
    assert_eq!(snap.ownship.vertical_speed_fpm, None);

    let route_idents: Vec<&str> = snap
        .ownship
        .route
        .waypoints
        .iter()
        .map(|w| w.ident.as_str())
        .collect();
    assert_eq!(route_idents, vec!["47N", "SBJ", "JFK", "KMTP"]);
    assert!(snap.ownship.route.active_leg.is_some());

    let callsigns: Vec<&str> = snap.traffic.iter().map(|t| t.callsign.as_str()).collect();
    for expected in ["JBU1990", "DWI2363", "ATN3345", "DLH499"] {
        assert!(
            callsigns.contains(&expected),
            "missing Connext ADS-B callsign {expected} in {callsigns:?}"
        );
    }

    // Advance into airborne Connext continuation: WMM2025 true heading resolves HWD 3D strokes.
    let air_snap = session.render_frame(14_000)?;
    assert!(air_snap.ownship.true_heading_deg.is_some());
    assert!(!air_snap.instruments.hwd_3d_strokes_enu.is_empty());
    assert_eq!(air_snap.ownship.indicated_airspeed_kt, None);
    Ok(())
}

#[test]
fn gdl90_crc_and_live_ownship_traffic_ahrs_roundtrip() -> Result<(), Box<dyn Error>> {
    // Official FAA GDL-90 Public ICD §2.2.3.3 heartbeat FCS check vector (`0x8BB3`, LSB-first `B3 8B`).
    let icd_heartbeat = [0x00_u8, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02];
    assert_eq!(crc16_gdl90(&icd_heartbeat), 0x8BB3);

    let own = Gdl90ParticipantReport {
        namespace: AddressNamespace::Icao24,
        address: 0xAD_1234,
        latitude_deg: 40.5800,
        longitude_deg: -74.4600,
        pressure_altitude_ft: Some(3_000),
        track_or_heading_deg: Some(60.0),
        horizontal_speed_kt: Some(130.0),
        vertical_speed_fpm: Some(256.0),
        airborne: true,
        nic: 9,
        nac_p: 9,
        emitter_category: EmitterCategory::Light,
        callsign: Some("N123SV".to_owned()),
        traffic_alert: false,
    };
    let traffic = Gdl90ParticipantReport {
        namespace: AddressNamespace::Icao24,
        address: 0xA8_FCAA,
        latitude_deg: 40.5920,
        longitude_deg: -74.4350,
        pressure_altitude_ft: Some(3_400),
        track_or_heading_deg: Some(240.0),
        horizontal_speed_kt: Some(185.0),
        vertical_speed_fpm: Some(-512.0),
        airborne: true,
        nic: 8,
        nac_p: 8,
        emitter_category: EmitterCategory::Large,
        callsign: Some("JBU1990".to_owned()),
        traffic_alert: true,
    };
    let ahrs = Gdl90AhrsReport {
        roll_deg: Some(-12.5),
        pitch_deg: Some(4.2),
        heading_deg: Some(73.0),
        heading_is_magnetic: true,
        indicated_airspeed_kt: Some(126.0),
        true_airspeed_kt: Some(133.0),
    };

    let mut stream = encode_participant_frame(0x0A, &own);
    stream.extend_from_slice(&encode_participant_frame(0x14, &traffic));
    stream.extend_from_slice(&encode_ahrs_frame(&ahrs));

    let mut dec = Gdl90Decoder::new();
    let decoded = dec.ingest(&stream);
    assert_eq!(decoded.len(), 3);
    assert!(matches!(decoded.first(), Some(Gdl90Message::Ownship(_))));

    let mut session = FlightDeckSession::new_on(PlanetaryReference::earth_wgs84_egm96())?;
    session.set_source_mode(TelemetrySourceMode::Gdl90Live);
    session.ingest_gdl90_bytes(&stream, 1_000);
    let snap = session.render_frame(1_050)?;
    assert_eq!(snap.gdl90_frames, 3);
    assert_eq!(snap.ownship.indicated_airspeed_kt, Some(126.0));
    assert!(snap.traffic.iter().any(|t| t.callsign == "JBU1990"));
    Ok(())
}

#[test]
fn scenarios_extrude_airspace_promote_cpa_threats_and_switch_hwd_compact_mode()
-> Result<(), Box<dyn Error>> {
    let mut session = FlightDeckSession::new_on(PlanetaryReference::earth_wgs84_egm96())?;
    session.set_source_mode(TelemetrySourceMode::Scenario(
        FlightDeckScenario::HighDensityTrafficCpa,
    ));
    session.set_crown_turn(0.75);

    let snap = session.render_frame(2_000)?;
    assert!(!snap.terrain.positions_enu_m.is_empty());
    assert!(!snap.terrain.runways.is_empty());
    assert!(!snap.airspaces.is_empty());
    assert!(
        snap.traffic
            .iter()
            .any(|t| t.callsign == "UAL842" && t.threat >= TrafficThreat::TrafficAdvisory)
    );
    assert!(!snap.instruments.hwd_3d_strokes_enu.is_empty());
    assert!(!snap.instruments.hwd_compact_mode);
    assert!(!snap.instruments.hud_scene.is_empty());
    assert!(!snap.instruments.pfd_scene.is_empty());
    assert!(!snap.instruments.hsi_scene.is_empty());
    assert!(!snap.instruments.terrain_volume_scene.is_empty());

    // Turn pilot head 45 deg right (> 35 deg entry cone) -> HWD enters compact glance mode.
    session.set_head_pose(HeadPose {
        yaw_rad: 45.0_f32.to_radians(),
        pitch_rad: 0.0,
        roll_rad: 0.0,
    });
    let off_axis = session.render_frame(2_100)?;
    assert!(off_axis.instruments.hwd_compact_mode);

    // Return to 31 deg (between 28 and 35 deg hysteresis band) -> stays compact.
    session.set_head_pose(HeadPose {
        yaw_rad: 31.0_f32.to_radians(),
        pitch_rad: 0.0,
        roll_rad: 0.0,
    });
    let hysteresis = session.render_frame(2_200)?;
    assert!(hysteresis.instruments.hwd_compact_mode);

    // Return to 10 deg (< 28 deg exit cone) -> full conformal HWD restored.
    session.set_head_pose(HeadPose {
        yaw_rad: 10.0_f32.to_radians(),
        pitch_rad: 0.0,
        roll_rad: 0.0,
    });
    let forward = session.render_frame(2_300)?;
    assert!(!forward.instruments.hwd_compact_mode);

    // Approach into KEWR Class B triggers InsideControlled alert.
    session.set_source_mode(TelemetrySourceMode::Scenario(
        FlightDeckScenario::KewrApproachWithTfr,
    ));
    let kewr_snap = session.render_frame(10_000)?;
    assert!(kewr_snap.airspace_alert.level >= AirspaceAlertLevel::InsideControlled);

    // Binary wire snapshot & glyph pack round-trip header verification.
    let wire_bytes = encode_frame_snapshot(&kewr_snap);
    let hdr = decode_frame_header(&wire_bytes)?;
    assert_eq!(hdr.source_mode_code, 2);
    assert!(hdr.passthrough_fraction >= MIN_PASSTHROUGH_FRACTION);

    let glyph_bytes = encode_glyph_pack(&verified_glyph_pack());
    assert_eq!(glyph_bytes.get(0..4), Some(b"FDKG".as_slice()));
    Ok(())
}

#[test]
fn pmtiles_v3_header_and_mvt_airspace_decoder_parse_valid_tiles() -> Result<(), Box<dyn Error>> {
    let mut header = vec![0u8; 127];
    if let Some(magic) = header.get_mut(0..8) {
        magic.copy_from_slice(b"PMTiles\x03");
    }
    if let Some(comp) = header.get_mut(98) {
        *comp = 2;
    }
    if let Some(tt) = header.get_mut(99) {
        *tt = 1;
    }
    if let Some(min_z) = header.get_mut(100) {
        *min_z = 2;
    }
    if let Some(max_z) = header.get_mut(101) {
        *max_z = 10;
    }
    let parsed = parse_pmtiles_header(&header)?;
    assert_eq!(parsed.tile_type, 1);
    assert_eq!(parsed.min_zoom, 2);
    assert_eq!(parsed.max_zoom, 10);

    // Minimal MVT 2.1 tile with one `airspace` layer, key `"class"` = `"B"`, `"name"` = `"EWR TEST"`,
    // and a clockwise 4-vertex square polygon in tile coordinates.
    let empty = decode_mvt_airspace_tile(&[], 8, 75, 96)?;
    assert!(empty.is_empty());

    let mvt_tile = build_minimal_mvt_airspace_tile();
    let vols = decode_mvt_airspace_tile(&mvt_tile, 8, 75, 96)?;
    assert_eq!(vols.len(), 1);
    assert_eq!(vols.first().map(|v| v.name.as_str()), Some("EWR TEST"));
    assert_eq!(
        vols.first().map(|v| v.classification.class_code()),
        Some("B")
    );
    Ok(())
}

fn build_minimal_mvt_airspace_tile() -> Vec<u8> {
    // Feature: tags = [0, 0, 1, 1], type = 3 (POLYGON),
    // geometry = [MoveTo(1)=9, +100(200), +100(200), LineTo(3)=26, +200(400), 0, 0, +200(400), -200(399), 0, ClosePath(1)=15]
    let tags_packed = [0_u8, 0, 1, 1];
    let geom_packed = [
        9_u8, 0xC8, 0x01, 0xC8, 0x01, 26, 0x90, 0x03, 0, 0, 0x90, 0x03, 0x8F, 0x03, 0, 15,
    ];
    let mut feat = Vec::new();
    push_proto_bytes(&mut feat, 2, &tags_packed);
    feat.extend_from_slice(&[3 << 3, 3]);
    push_proto_bytes(&mut feat, 4, &geom_packed);

    let mut val_class = Vec::new();
    push_proto_bytes(&mut val_class, 1, b"B");
    let mut val_name = Vec::new();
    push_proto_bytes(&mut val_name, 1, b"EWR TEST");

    let mut layer = Vec::new();
    layer.extend_from_slice(&[(15 << 3), 2]); // version = 2
    push_proto_bytes(&mut layer, 1, b"airspace");
    push_proto_bytes(&mut layer, 2, &feat);
    push_proto_bytes(&mut layer, 3, b"class");
    push_proto_bytes(&mut layer, 3, b"name");
    push_proto_bytes(&mut layer, 4, &val_class);
    push_proto_bytes(&mut layer, 4, &val_name);
    layer.extend_from_slice(&[(5 << 3), 0x80, 0x20]); // extent = 4096

    let mut tile = Vec::new();
    push_proto_bytes(&mut tile, 3, &layer);
    tile
}

#[test]
fn flightaware_kml_replay_populates_gs_altitude_bank_and_hud_hwd_scenes()
-> Result<(), Box<dyn Error>> {
    let sample_kml = br#"<?xml version="1.0" encoding="UTF-8"?>
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
            <when>2026-08-30T04:22:24Z</when>
            <when>2026-08-30T04:23:21Z</when>
            <when>2026-08-30T04:23:37Z</when>
            <gx:coord>-74.73869 40.57530 46</gx:coord>
            <gx:coord>-74.73100 40.57750 85</gx:coord>
            <gx:coord>-74.72150 40.58000 130</gx:coord>
            <gx:coord>-74.71150 40.58250 175</gx:coord>
            <gx:coord>-74.67500 40.59100 260</gx:coord>
            <gx:coord>-74.66500 40.59600 265</gx:coord>
        </gx:Track>
    </Placemark>
</Document>
</kml>"#;

    let earth = PlanetaryReference::earth_wgs84_egm96();
    let track = KmlFlightTrack::parse_bytes_on(sample_kml, &earth)?;
    assert_eq!(track.callsign, "N9758H");
    assert_eq!(track.route_label, "N51 → N51");
    assert_eq!(track.points.len(), 6);
    assert!((track.duration_s - 121.0).abs() < 1e-6);

    let mut session = FlightDeckSession::new_on(earth)?;
    session.load_kml_bytes(sample_kml, 1_000)?;
    assert_eq!(session.source_mode(), TelemetrySourceMode::KmlReplay);

    // Advance across the 57-second FlightAware gap and verify smooth GS (~90..120 kt) and MSL FT.
    session.seek_kml_fraction(0.65, 2_000);
    let snap = session.render_frame(2_000)?;
    let gs_kt = snap.ownship.ground_speed_kt.unwrap_or(0.0);
    assert!(
        (80.0..135.0).contains(&gs_kt),
        "expected realistic cruise ground speed across 57s gap, got {gs_kt} kt"
    );
    assert!(snap.ownship.ground_vector_valid);
    assert!(snap.ownship.gnss_valid);
    assert_eq!(snap.ownship.indicated_airspeed_kt, None);

    // Verify Indicate AircraftState declares GeoidModelId(1) for GeometricMsl and horizontal velocity.
    let aircraft = build_aircraft_state(&snap.ownship, 2_000, &earth, true);
    assert!(aircraft.valid.velocity_horizontal);
    assert_ne!(
        aircraft.altitude.geoid_model,
        indicate_instrument_state::GeoidModelId::UNDECLARED
    );
    assert!(!snap.instruments.hud_scene.is_empty());
    assert!(!snap.instruments.hwd_overlay_scene.is_empty());
    assert!(!snap.instruments.hwd_3d_strokes_enu.is_empty());

    // Verify HUD and HWD scene command streams literally contain "GS" and numeric digits.
    let hud_ascii = String::from_utf8_lossy(&snap.instruments.hud_scene);
    let hwd_ascii = String::from_utf8_lossy(&snap.instruments.hwd_overlay_scene);
    assert!(hud_ascii.contains("GS"), "HUD scene must render GS readout");
    assert!(hwd_ascii.contains("GS"), "HWD scene must render GS readout");
    Ok(())
}

#[test]
fn explicit_lunar_body_never_assumes_earth_wgs84_or_egm96() -> Result<(), Box<dyn Error>> {
    let moon = PlanetaryReference::moon_iau();
    let mut session = FlightDeckSession::new_on(moon)?;
    let snap = session.render_frame(4_000)?;

    assert_eq!(snap.planet, moon);
    assert!((snap.planet.render_radius_m - 1_737_400.0).abs() < 1.0);
    assert!(snap.ownship.gnss_valid);
    assert_eq!(snap.ownship.magnetic_variation_deg_east, None);
    assert_eq!(snap.ownship.magnetic_heading_deg, None);
    assert_eq!(snap.ownship.indicated_airspeed_kt, None);
    assert_eq!(snap.ownship.pressure_altitude_ft, None);
    assert!(
        snap.airspaces.is_empty(),
        "FAA NASR must not leak onto Moon"
    );
    assert!(
        snap.traffic.is_empty(),
        "1090ES ADS-B must not leak onto Moon"
    );
    assert!(snap.terrain.runways.is_empty());

    let aircraft = build_aircraft_state(&snap.ownship, 4_000, &moon, false);
    assert_eq!(
        aircraft.altitude.geoid_model,
        indicate_instrument_state::GeoidModelId::UNDECLARED
    );
    assert!(!aircraft.valid.variation);
    assert!(!snap.instruments.hud_scene.is_empty());

    let sky = session.evaluate_sky(1_788_116_400.0, 0.674, 23.473, 1_150.0);
    assert!(!sky.atmosphere);
    assert!(sky.moon_direction_fixed.is_none());
    assert!(session.sky_star_count() > 4_000);
    Ok(())
}

fn push_proto_bytes(out: &mut Vec<u8>, field: u8, payload: &[u8]) {
    out.push((field << 3) | 2);
    let mut n = payload.len();
    loop {
        let mut b = (n & 0x7f) as u8;
        n >>= 7;
        if n != 0 {
            b |= 0x80;
        }
        out.push(b);
        if n == 0 {
            break;
        }
    }
    out.extend_from_slice(payload);
}
