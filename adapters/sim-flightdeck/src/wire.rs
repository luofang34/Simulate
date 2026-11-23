//! Deterministic little-endian binary wire serializer for crossing the Rust-to-Swift (`visionOS`) bridge.
//!
//! Encodes [`FlightDeckFrameSnapshot`] (`FDKF` v1) and [`VerifiedGlyphPack`] (`FDKG` v1) into single
//! contiguous `Vec<u8>` buffers so Swift decodes 3D meshes, 3D airspaces, ADS-B traffic callsigns,
//! HWD unit-sphere strokes, and `Indicate` scene command buffers in one pass.

use crate::airspace::AirspaceRenderVolume;
use crate::error::FlightDeckError;
use crate::instruments::VerifiedGlyphPack;
use crate::session::FlightDeckFrameSnapshot;
use crate::terrain::{RunwayOverlay3D, TerrainMesh};
use crate::traffic::TrafficDisplayTarget;

/// Magic header bytes for an encoded [`FlightDeckFrameSnapshot`].
pub const FRAME_WIRE_MAGIC: &[u8; 4] = b"FDKF";
/// Wire schema version for [`encode_frame_snapshot`].
pub const FRAME_WIRE_VERSION: u32 = 1;
/// Magic header bytes for an encoded [`VerifiedGlyphPack`].
pub const GLYPH_WIRE_MAGIC: &[u8; 4] = b"FDKG";
/// Wire schema version for [`encode_glyph_pack`].
pub const GLYPH_WIRE_VERSION: u32 = 1;

/// Summary header decoded from the front of an `FDKF` v1 frame buffer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DecodedFrameHeader {
    /// Wire version (`1`).
    pub version: u32,
    /// Source mode code (`0..=5`).
    pub source_mode_code: u8,
    /// Digital Crown virtual terrain opacity (`<= 0.85`).
    pub virtual_terrain_opacity: f32,
    /// Real-world passthrough fraction (`>= 0.15`).
    pub passthrough_fraction: f32,
}

/// Serializes a complete [`FlightDeckFrameSnapshot`] into the `FDKF` v1 binary format.
#[must_use]
pub fn encode_frame_snapshot(snap: &FlightDeckFrameSnapshot) -> Vec<u8> {
    let mut out = Vec::with_capacity(65_536);
    out.extend_from_slice(FRAME_WIRE_MAGIC);
    push_u32(&mut out, FRAME_WIRE_VERSION);
    out.push(snap.source_mode.code());
    out.push(u8::from(snap.ownship.gnss_valid));
    out.push(u8::from(snap.ownship.ground_vector_valid));
    out.push(u8::from(snap.instruments.hwd_compact_mode));

    // Blend weights (6 x f32)
    push_f32(&mut out, snap.blend.crown_turn);
    push_f32(&mut out, snap.blend.virtual_terrain_opacity);
    push_f32(&mut out, snap.blend.passthrough_fraction);
    push_f32(&mut out, snap.blend.airspace_fill_opacity);
    push_f32(&mut out, snap.blend.airspace_rim_opacity);
    push_f32(&mut out, snap.blend.symbology_opacity);

    encode_ownship_section(&mut out, snap);
    encode_terrain_section(&mut out, &snap.terrain);
    encode_airspace_section(
        &mut out,
        &snap.airspaces,
        &snap.airspace_alert.message,
        snap.airspace_alert.level as u8,
    );
    encode_traffic_section(&mut out, &snap.traffic);
    encode_instruments_section(&mut out, snap);
    out
}

/// Validates and reads the leading header of an `FDKF` v1 binary buffer.
///
/// # Errors
///
/// Returns [`FlightDeckError::Wire`] if magic bytes, version, or header length do not match.
pub fn decode_frame_header(bytes: &[u8]) -> Result<DecodedFrameHeader, FlightDeckError> {
    if bytes.len() < 36 || bytes.get(0..4) != Some(FRAME_WIRE_MAGIC) {
        return Err(FlightDeckError::Wire {
            reason: "invalid FDKF frame magic or truncated header".to_owned(),
        });
    }
    let version = read_u32(bytes, 4)?;
    if version != FRAME_WIRE_VERSION {
        return Err(FlightDeckError::Wire {
            reason: format!("unsupported FDKF version {version}"),
        });
    }
    let source_mode_code = *bytes.get(8).unwrap_or(&0);
    let virtual_terrain_opacity = read_f32(bytes, 16)?;
    let passthrough_fraction = read_f32(bytes, 20)?;
    Ok(DecodedFrameHeader {
        version,
        source_mode_code,
        virtual_terrain_opacity,
        passthrough_fraction,
    })
}

/// Serializes [`VerifiedGlyphPack`] into the `FDKG` v1 binary format for `IndicateAppleDisplay`.
#[must_use]
pub fn encode_glyph_pack(pack: &VerifiedGlyphPack) -> Vec<u8> {
    let mut out = Vec::with_capacity(4_096);
    out.extend_from_slice(GLYPH_WIRE_MAGIC);
    push_u32(&mut out, GLYPH_WIRE_VERSION);
    out.push(pack.cell_width);
    out.push(pack.cell_height);
    out.extend_from_slice(&[0u8, 0u8]);
    out.extend_from_slice(&pack.content_hash);
    push_u32(&mut out, pack.glyphs.len() as u32);
    for g in &pack.glyphs {
        push_u32(&mut out, g.scalar);
        out.push(g.advance);
        push_bytes(&mut out, &g.rows);
    }
    out
}

fn encode_ownship_section(out: &mut Vec<u8>, snap: &FlightDeckFrameSnapshot) {
    let own = &snap.ownship;
    push_opt_f64(out, own.latitude_deg);
    push_opt_f64(out, own.longitude_deg);
    push_opt_f64(out, own.best_altitude_m());
    push_opt_f64(out, own.pressure_altitude_ft);
    push_opt_f64(out, own.ground_speed_kt);
    push_opt_f64(out, own.indicated_airspeed_kt);
    push_opt_f64(out, own.vertical_speed_fpm);
    push_opt_f64(out, own.magnetic_heading_deg);
    push_opt_f64(out, own.true_heading_deg);
    push_opt_f64(out, own.ground_track_true_deg);
    push_opt_f64(out, own.pitch_deg);
    push_opt_f64(out, own.roll_deg);
    push_opt_f64(out, snap.terrain.agl_ft);

    let own_cs = own
        .identity
        .as_ref()
        .and_then(|id| id.callsign.as_deref())
        .unwrap_or("");
    push_str(out, own_cs);
    let active_leg_str = own
        .route
        .active_leg
        .as_ref()
        .map_or_else(String::new, |(a, b)| format!("{} → {}", a.ident, b.ident));
    push_str(out, &active_leg_str);
    push_u64(out, snap.connext_records);
    push_u64(out, snap.gdl90_frames);
}

fn encode_terrain_section(out: &mut Vec<u8>, terrain: &TerrainMesh) {
    let v_count = terrain.positions_enu_m.len();
    push_u32(out, v_count as u32);
    for &[e, n, u] in &terrain.positions_enu_m {
        push_f32(out, e);
        push_f32(out, n);
        push_f32(out, u);
    }
    for &[nx, ny, nz] in &terrain.normals_enu {
        push_f32(out, nx);
        push_f32(out, ny);
        push_f32(out, nz);
    }
    for rgba in &terrain.colors_rgba {
        out.extend_from_slice(rgba);
    }
    push_u32(out, terrain.indices.len() as u32);
    for &idx in &terrain.indices {
        push_u32(out, idx);
    }
    push_u32(out, terrain.runways.len() as u32);
    for rwy in &terrain.runways {
        encode_runway(out, rwy);
    }
}

fn encode_runway(out: &mut Vec<u8>, rwy: &RunwayOverlay3D) {
    push_str(out, &rwy.label);
    for &[e, n, u] in &rwy.corners_enu_m {
        push_f32(out, e);
        push_f32(out, n);
        push_f32(out, u);
    }
    for &[e, n, u] in &rwy.centerline_enu_m {
        push_f32(out, e);
        push_f32(out, n);
        push_f32(out, u);
    }
    push_u32(out, rwy.glideslope_gates_enu_m.len() as u32);
    for gate in &rwy.glideslope_gates_enu_m {
        for &[e, n, u] in gate {
            push_f32(out, e);
            push_f32(out, n);
            push_f32(out, u);
        }
    }
}

fn encode_airspace_section(
    out: &mut Vec<u8>,
    volumes: &[AirspaceRenderVolume],
    alert_msg: &str,
    alert_level: u8,
) {
    out.push(alert_level);
    push_str(out, alert_msg);
    push_u32(out, volumes.len() as u32);
    for vol in volumes {
        push_str(out, &vol.airspace_key);
        push_str(out, &vol.label);
        out.push(vol.style as u8);
        out.push(u8::from(vol.active));
        out.push(u8::from(vol.ownship_inside));
        let [r, g, b] = vol.rgb;
        push_f32(out, r);
        push_f32(out, g);
        push_f32(out, b);
        let [le, ln, lu] = vol.label_anchor_enu_m;
        push_f32(out, le);
        push_f32(out, ln);
        push_f32(out, lu);
        push_u32(out, vol.wall_positions_enu_m.len() as u32);
        for &[e, n, u] in &vol.wall_positions_enu_m {
            push_f32(out, e);
            push_f32(out, n);
            push_f32(out, u);
        }
        push_u32(out, vol.wall_indices.len() as u32);
        for &idx in &vol.wall_indices {
            push_u32(out, idx);
        }
        push_u32(out, vol.rim_segments_enu_m.len() as u32);
        for &[[e0, n0, u0], [e1, n1, u1]] in &vol.rim_segments_enu_m {
            push_f32(out, e0);
            push_f32(out, n0);
            push_f32(out, u0);
            push_f32(out, e1);
            push_f32(out, n1);
            push_f32(out, u1);
        }
    }
}

fn encode_traffic_section(out: &mut Vec<u8>, traffic: &[TrafficDisplayTarget]) {
    push_u32(out, traffic.len() as u32);
    for t in traffic {
        push_u64(out, t.track_id);
        push_str(out, &t.address_hex);
        push_str(out, &t.callsign);
        push_str(out, &t.placard);
        let [pe, pn, pu] = t.offset_enu_m;
        push_f32(out, pe);
        push_f32(out, pn);
        push_f32(out, pu);
        let [ve, vn, vu] = t.velocity_enu_mps;
        push_f32(out, ve);
        push_f32(out, vn);
        push_f32(out, vu);
        push_f32(out, t.track_true_deg.unwrap_or(f32::NAN));
        push_f32(out, t.slant_range_nm);
        push_i32(out, t.rel_altitude_hundreds_ft);
        out.push(t.vertical_trend as u8);
        out.push(t.threat as u8);
        out.push(u8::from(t.on_ground));
        out.push(u8::from(t.active));
        push_f32(out, t.time_to_cpa_s);
        push_f32(out, t.cpa_horizontal_m);
    }
}

fn encode_instruments_section(out: &mut Vec<u8>, snap: &FlightDeckFrameSnapshot) {
    let inst = &snap.instruments;
    push_u32(out, inst.hwd_3d_strokes_enu.len() as u32);
    for &[[ax, ay, az], [bx, by, bz]] in &inst.hwd_3d_strokes_enu {
        push_f32(out, ax);
        push_f32(out, ay);
        push_f32(out, az);
        push_f32(out, bx);
        push_f32(out, by);
        push_f32(out, bz);
    }
    push_bytes(out, &inst.hwd_overlay_scene);
    push_bytes(out, &inst.hud_scene);
    push_bytes(out, &inst.pfd_scene);
    push_bytes(out, &inst.hsi_scene);
    push_bytes(out, &inst.terrain_volume_scene);
}

fn push_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn push_i32(out: &mut Vec<u8>, v: i32) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn push_u64(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn push_f32(out: &mut Vec<u8>, v: f32) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn push_opt_f64(out: &mut Vec<u8>, v: Option<f64>) {
    out.extend_from_slice(&v.unwrap_or(f64::NAN).to_le_bytes());
}

fn push_str(out: &mut Vec<u8>, s: &str) {
    push_bytes(out, s.as_bytes());
}

fn push_bytes(out: &mut Vec<u8>, b: &[u8]) {
    push_u32(out, b.len() as u32);
    out.extend_from_slice(b);
}

fn read_u32(bytes: &[u8], offset: usize) -> Result<u32, FlightDeckError> {
    let slice = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| FlightDeckError::Wire {
            reason: "u32 out of bounds".to_owned(),
        })?;
    let arr = <[u8; 4]>::try_from(slice).map_err(|_| FlightDeckError::Wire {
        reason: "u32 slice error".to_owned(),
    })?;
    Ok(u32::from_le_bytes(arr))
}

fn read_f32(bytes: &[u8], offset: usize) -> Result<f32, FlightDeckError> {
    let slice = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| FlightDeckError::Wire {
            reason: "f32 out of bounds".to_owned(),
        })?;
    let arr = <[u8; 4]>::try_from(slice).map_err(|_| FlightDeckError::Wire {
        reason: "f32 slice error".to_owned(),
    })?;
    Ok(f32::from_le_bytes(arr))
}
