//! Pure-Rust `PMTiles` v3 127-byte header reader and Mapbox Vector Tile 2.1 `airspace` decoder.
//!
//! Reads compiled `Flightbag` `airspace.pmtiles` headers and uncompressed `.mvt` tile payloads
//! into [`flightbag_core::AirspaceVolume`] values without external protobuf/archive dependencies.

use core::f64::consts::PI;
use flightbag_core::{
    AirspaceClassification, AirspacePolygon, AirspaceVolume, AltitudeDatum,
    ControlledAirspaceClass, ControlledLocalType, SuaKind, VerticalLimit,
};
use world_core::GeodeticPosition;

use crate::error::FlightDeckError;

const PMTILES_V3_MAGIC: &[u8; 8] = b"PMTiles\x03";
const PMTILES_HEADER_BYTES: usize = 127;

/// Parsed 127-byte `PMTiles` v3 archive header (`Flightbag` `airspace.pmtiles`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmTilesHeader {
    /// Byte offset of the root directory.
    pub root_dir_offset: u64,
    /// Byte length of the root directory.
    pub root_dir_bytes: u64,
    /// Byte offset of the JSON metadata block.
    pub metadata_offset: u64,
    /// Byte length of the JSON metadata block.
    pub metadata_bytes: u64,
    /// Byte offset of tile data payloads.
    pub tile_data_offset: u64,
    /// Byte length of tile data payloads.
    pub tile_data_bytes: u64,
    /// Number of addressed tiles in the archive.
    pub addressed_tiles_count: u64,
    /// Tile payload type (`1` = MVT).
    pub tile_type: u8,
    /// Tile compression (`1` = None, `2` = Gzip).
    pub tile_compression: u8,
    /// Minimum zoom level.
    pub min_zoom: u8,
    /// Maximum zoom level.
    pub max_zoom: u8,
    /// Geographic bounds `[min_lon, min_lat, max_lon, max_lat]` in degrees.
    pub bounds_deg: [f64; 4],
}

/// Parses and validates a 127-byte `PMTiles` v3 header prefix.
///
/// # Errors
///
/// Returns [`FlightDeckError::PmTiles`] if `bytes` is shorter than 127 bytes or lacks `PMTiles\x03`.
pub fn parse_pmtiles_header(bytes: &[u8]) -> Result<PmTilesHeader, FlightDeckError> {
    if bytes.len() < PMTILES_HEADER_BYTES {
        return Err(pm_err(format!(
            "header shorter than 127 bytes ({})",
            bytes.len()
        )));
    }
    if bytes.get(0..8) != Some(PMTILES_V3_MAGIC) {
        return Err(pm_err("invalid PMTiles v3 magic"));
    }
    let min_lon = f64::from(i32_le(bytes, 102)?) * 1e-7;
    let min_lat = f64::from(i32_le(bytes, 106)?) * 1e-7;
    let max_lon = f64::from(i32_le(bytes, 110)?) * 1e-7;
    let max_lat = f64::from(i32_le(bytes, 114)?) * 1e-7;
    Ok(PmTilesHeader {
        root_dir_offset: u64_le(bytes, 8)?,
        root_dir_bytes: u64_le(bytes, 16)?,
        metadata_offset: u64_le(bytes, 24)?,
        metadata_bytes: u64_le(bytes, 32)?,
        tile_data_offset: u64_le(bytes, 56)?,
        tile_data_bytes: u64_le(bytes, 64)?,
        addressed_tiles_count: u64_le(bytes, 72)?,
        tile_compression: *bytes.get(98).unwrap_or(&0),
        tile_type: *bytes.get(99).unwrap_or(&0),
        min_zoom: *bytes.get(100).unwrap_or(&0),
        max_zoom: *bytes.get(101).unwrap_or(&0),
        bounds_deg: [min_lon, min_lat, max_lon, max_lat],
    })
}

#[derive(Clone, Debug)]
enum MvtVal {
    Str(String),
    Num(f64),
    Bool,
}

#[derive(Default)]
struct RawLayer<'a> {
    name: &'a str,
    extent: u32,
    keys: Vec<&'a str>,
    values: Vec<MvtVal>,
    features: Vec<&'a [u8]>,
}

/// Decodes `airspace` polygon features from an uncompressed Mapbox Vector Tile 2.1 buffer at `(zoom, tile_x, tile_y)`.
///
/// # Errors
///
/// Returns [`FlightDeckError::PmTiles`] on malformed protobuf varints or wire tags.
pub fn decode_mvt_airspace_tile(
    mvt_bytes: &[u8],
    zoom: u8,
    tile_x: u32,
    tile_y: u32,
) -> Result<Vec<AirspaceVolume>, FlightDeckError> {
    let mut cursor = 0usize;
    let mut out = Vec::new();
    while cursor < mvt_bytes.len() {
        let tag = read_varint(mvt_bytes, &mut cursor)?;
        let field = tag >> 3;
        let wire = tag & 0x07;
        if field == 3 && wire == 2 {
            let slice = read_len_bytes(mvt_bytes, &mut cursor)?;
            let layer = decode_layer(slice)?;
            if layer.name == "airspace" {
                decode_layer_volumes(&layer, zoom, tile_x, tile_y, &mut out)?;
            }
        } else {
            skip_field(mvt_bytes, &mut cursor, wire)?;
        }
    }
    Ok(out)
}

type PmResult<T> = Result<T, FlightDeckError>;

fn decode_layer(bytes: &[u8]) -> PmResult<RawLayer<'_>> {
    let mut layer = RawLayer {
        extent: 4096,
        ..RawLayer::default()
    };
    let mut cursor = 0usize;
    while cursor < bytes.len() {
        let tag = read_varint(bytes, &mut cursor)?;
        match (tag >> 3, tag & 0x07) {
            (1, 2) => {
                layer.name = core::str::from_utf8(read_len_bytes(bytes, &mut cursor)?).unwrap_or("")
            }
            (2, 2) => layer.features.push(read_len_bytes(bytes, &mut cursor)?),
            (3, 2) => layer
                .keys
                .push(core::str::from_utf8(read_len_bytes(bytes, &mut cursor)?).unwrap_or("")),
            (4, 2) => layer
                .values
                .push(decode_mvt_value(read_len_bytes(bytes, &mut cursor)?)?),
            (5, 0) => layer.extent = read_varint(bytes, &mut cursor)? as u32,
            (_, wire) => skip_field(bytes, &mut cursor, wire)?,
        }
    }
    Ok(layer)
}

fn decode_mvt_value(bytes: &[u8]) -> Result<MvtVal, FlightDeckError> {
    let mut cursor = 0usize;
    let mut result = MvtVal::Str(String::new());
    while cursor < bytes.len() {
        let tag = read_varint(bytes, &mut cursor)?;
        match (tag >> 3, tag & 0x07) {
            (1, 2) => {
                let s = read_len_bytes(bytes, &mut cursor)?;
                result = MvtVal::Str(core::str::from_utf8(s).unwrap_or("").to_owned());
            }
            (2, 5) => {
                let b = read_fixed(bytes, &mut cursor, 4)?;
                let arr = <[u8; 4]>::try_from(b).unwrap_or([0; 4]);
                result = MvtVal::Num(f64::from(f32::from_le_bytes(arr)));
            }
            (3, 1) => {
                let b = read_fixed(bytes, &mut cursor, 8)?;
                let arr = <[u8; 8]>::try_from(b).unwrap_or([0; 8]);
                result = MvtVal::Num(f64::from_le_bytes(arr));
            }
            (4 | 5, 0) => result = MvtVal::Num(read_varint(bytes, &mut cursor)? as f64),
            (6, 0) => {
                let raw = read_varint(bytes, &mut cursor)?;
                let signed = ((raw >> 1) as i64) ^ -((raw & 1) as i64);
                result = MvtVal::Num(signed as f64);
            }
            (7, 0) => {
                let _ = read_varint(bytes, &mut cursor)?;
                result = MvtVal::Bool;
            }
            (_, wire) => skip_field(bytes, &mut cursor, wire)?,
        }
    }
    Ok(result)
}

fn decode_layer_volumes(
    layer: &RawLayer<'_>,
    zoom: u8,
    tile_x: u32,
    tile_y: u32,
    out: &mut Vec<AirspaceVolume>,
) -> Result<(), FlightDeckError> {
    for &feat_bytes in &layer.features {
        let mut cursor = 0usize;
        let mut geom_type = 0u64;
        let mut tags = Vec::new();
        let mut geom_cmds = Vec::new();
        while cursor < feat_bytes.len() {
            let tag = read_varint(feat_bytes, &mut cursor)?;
            match (tag >> 3, tag & 0x07) {
                (2, 2) => tags = read_packed_varints(read_len_bytes(feat_bytes, &mut cursor)?)?,
                (3, 0) => geom_type = read_varint(feat_bytes, &mut cursor)?,
                (4, 2) => {
                    geom_cmds = read_packed_varints(read_len_bytes(feat_bytes, &mut cursor)?)?;
                }
                (_, wire) => skip_field(feat_bytes, &mut cursor, wire)?,
            }
        }
        if geom_type != 3 {
            continue;
        }
        let polygons = decode_mvt_polygons(&geom_cmds, layer.extent.max(1), zoom, tile_x, tile_y);
        if polygons.is_empty() {
            continue;
        }
        if let Some(vol) = build_volume_from_tags(layer, &tags, (zoom, tile_x, tile_y), polygons) {
            out.push(vol);
        }
    }
    Ok(())
}

fn build_volume_from_tags(
    layer: &RawLayer<'_>,
    tags: &[u64],
    (zoom, tile_x, tile_y): (u8, u32, u32),
    polygons: Vec<AirspacePolygon>,
) -> Option<AirspaceVolume> {
    let mut airspace_key = String::from("MVT-AIRSPACE");
    let mut ident = String::from("AIRSPACE");
    let mut name = String::from("AIRSPACE");
    let mut class_code = String::from("E");
    let mut local_type = String::from("CLASS_E5");
    let mut lower_ft = 0i32;
    let mut upper_ft = 5_000i32;
    let mut display_upper_ft: Option<i32> = None;
    let mut lower_datum = AltitudeDatum::Msl;
    let mut upper_datum = AltitudeDatum::Msl;
    let mut part = String::from("full");

    let mut idx = 0usize;
    while let (Some(&k_idx), Some(&v_idx)) = (tags.get(idx), tags.get(idx.saturating_add(1))) {
        idx = idx.saturating_add(2);
        let Some(&key) = layer.keys.get(k_idx as usize) else {
            continue;
        };
        let Some(val) = layer.values.get(v_idx as usize) else {
            continue;
        };
        match (key, val) {
            ("airspace_key", MvtVal::Str(s)) if !s.is_empty() => airspace_key.clone_from(s),
            ("ident", MvtVal::Str(s)) if !s.is_empty() => ident.clone_from(s),
            ("name", MvtVal::Str(s)) if !s.is_empty() => name.clone_from(s),
            ("class", MvtVal::Str(s)) => class_code.clone_from(s),
            ("local_type", MvtVal::Str(s)) => local_type.clone_from(s),
            ("part", MvtVal::Str(s)) if !s.is_empty() => part.clone_from(s),
            ("lower_ft", MvtVal::Num(n)) => lower_ft = n.round() as i32,
            ("upper_ft", MvtVal::Num(n)) => upper_ft = n.round() as i32,
            ("display_upper_m", MvtVal::Num(m)) if *m > 0.0 => {
                display_upper_ft = Some((m / 0.3048).round() as i32);
            }
            ("lower_datum", MvtVal::Str(s)) => lower_datum = parse_datum(s),
            ("upper_datum" | "display_upper_datum", MvtVal::Str(s)) => upper_datum = parse_datum(s),
            _ => {}
        }
    }
    let eff_upper_ft = display_upper_ft.unwrap_or(upper_ft).max(lower_ft);
    let unique_key =
        format!("{airspace_key}#{part}@{lower_ft}..{eff_upper_ft}@{zoom}/{tile_x}/{tile_y}");
    Some(AirspaceVolume {
        airspace_key: unique_key,
        ident,
        name,
        sector: Some(format!("mvt:{part}:{zoom}:{tile_x}:{tile_y}")),
        classification: parse_classification(&class_code, &local_type),
        lower: VerticalLimit {
            datum: lower_datum,
            value_ft: lower_ft,
        },
        upper: VerticalLimit {
            datum: upper_datum,
            value_ft: eff_upper_ft,
        },
        hours_of_use: None,
        controlling_agency: None,
        polygons,
    })
}

fn decode_mvt_polygons(
    cmds: &[u64],
    extent: u32,
    zoom: u8,
    tile_x: u32,
    tile_y: u32,
) -> Vec<AirspacePolygon> {
    let mut cx = 0i32;
    let mut cy = 0i32;
    let mut idx = 0usize;
    let mut current: Vec<(i32, i32)> = Vec::new();
    let mut polys: Vec<AirspacePolygon> = Vec::new();

    while let Some(&cmd_int) = cmds.get(idx) {
        idx = idx.saturating_add(1);
        let cmd_id = cmd_int & 0x07;
        let count = (cmd_int >> 3) as usize;
        match cmd_id {
            1 | 2 => {
                if cmd_id == 1 && current.len() >= 3 {
                    push_ring(&mut polys, &current, extent, zoom, tile_x, tile_y);
                    current.clear();
                }
                for _ in 0..count {
                    let Some(&dx_raw) = cmds.get(idx) else {
                        break;
                    };
                    let Some(&dy_raw) = cmds.get(idx.saturating_add(1)) else {
                        break;
                    };
                    idx = idx.saturating_add(2);
                    cx = cx.saturating_add(unzigzag32(dx_raw));
                    cy = cy.saturating_add(unzigzag32(dy_raw));
                    current.push((cx, cy));
                }
            }
            7 => {
                if current.len() >= 3 {
                    push_ring(&mut polys, &current, extent, zoom, tile_x, tile_y);
                }
                current.clear();
            }
            _ => break,
        }
    }
    polys
}

fn push_ring(
    polys: &mut Vec<AirspacePolygon>,
    ring: &[(i32, i32)],
    extent: u32,
    zoom: u8,
    tile_x: u32,
    tile_y: u32,
) {
    let mut area2 = 0i64;
    for (i, &(x0, y0)) in ring.iter().enumerate() {
        let next = if i + 1 == ring.len() { 0 } else { i + 1 };
        if let Some(&(x1, y1)) = ring.get(next) {
            area2 += i64::from(x0) * i64::from(y1) - i64::from(x1) * i64::from(y0);
        }
    }
    let geo: Vec<GeodeticPosition> = ring
        .iter()
        .map(|&(px, py)| tile_px_to_geodetic(zoom, tile_x, tile_y, px, py, extent))
        .collect();
    if geo.len() < 3 {
        return;
    }
    if area2 >= 0 || polys.is_empty() {
        polys.push(AirspacePolygon {
            exterior: geo,
            interiors: Vec::new(),
        });
    } else if let Some(last) = polys.last_mut() {
        last.interiors.push(geo);
    }
}

fn tile_px_to_geodetic(
    zoom: u8,
    tile_x: u32,
    tile_y: u32,
    px: i32,
    py: i32,
    extent: u32,
) -> GeodeticPosition {
    let n = f64::from(1u32 << zoom.min(24));
    let u = (f64::from(tile_x) + f64::from(px) / f64::from(extent)) / n;
    let v = (f64::from(tile_y) + f64::from(py) / f64::from(extent)) / n;
    let lon_deg = (u * 360.0 - 180.0).clamp(-180.0, 180.0);
    let lat_rad = libm::atan(libm::sinh(PI * (1.0 - 2.0 * v)));
    let lat_deg = lat_rad.to_degrees().clamp(-85.051_129, 85.051_129);
    GeodeticPosition::wgs84(lat_deg, lon_deg, 0.0)
}

fn parse_datum(s: &str) -> AltitudeDatum {
    match s {
        "sfc" => AltitudeDatum::Surface,
        "agl" => AltitudeDatum::Agl,
        "fl" => AltitudeDatum::FlightLevel,
        "unl" => AltitudeDatum::Unlimited,
        _ => AltitudeDatum::Msl,
    }
}

fn parse_classification(class_code: &str, local_type: &str) -> AirspaceClassification {
    let controlled = |class, local_type| AirspaceClassification::Controlled { class, local_type };
    match class_code {
        "B" => controlled(ControlledAirspaceClass::ClassB, ControlledLocalType::ClassB),
        "C" => controlled(ControlledAirspaceClass::ClassC, ControlledLocalType::ClassC),
        "D" => controlled(ControlledAirspaceClass::ClassD, ControlledLocalType::ClassD),
        "P" => AirspaceClassification::SpecialUse(SuaKind::Prohibited),
        "R" => AirspaceClassification::SpecialUse(SuaKind::Restricted),
        "W" => AirspaceClassification::SpecialUse(SuaKind::Warning),
        "MOA" => AirspaceClassification::SpecialUse(SuaKind::Moa),
        "A" => AirspaceClassification::SpecialUse(SuaKind::Alert),
        _ => controlled(
            ControlledAirspaceClass::ClassE,
            if local_type == "CLASS_E2" {
                ControlledLocalType::ClassE2Surface
            } else {
                ControlledLocalType::ClassE5Transition
            },
        ),
    }
}

fn unzigzag32(v: u64) -> i32 {
    let v32 = v as u32;
    ((v32 >> 1) as i32) ^ -((v32 & 1) as i32)
}

fn pm_err(reason: impl Into<String>) -> FlightDeckError {
    FlightDeckError::PmTiles {
        reason: reason.into(),
    }
}

fn read_varint(bytes: &[u8], cursor: &mut usize) -> Result<u64, FlightDeckError> {
    let mut val = 0u64;
    let mut shift = 0u32;
    loop {
        let b = *bytes
            .get(*cursor)
            .ok_or_else(|| pm_err("unexpected EOF in varint"))?;
        *cursor = cursor.saturating_add(1);
        val |= u64::from(b & 0x7f) << shift;
        if (b & 0x80) == 0 {
            return Ok(val);
        }
        shift = shift.saturating_add(7);
        if shift >= 64 {
            return Err(pm_err("varint overflow"));
        }
    }
}

fn read_len_bytes<'a>(bytes: &'a [u8], cursor: &mut usize) -> PmResult<&'a [u8]> {
    let len = read_varint(bytes, cursor)? as usize;
    read_fixed(bytes, cursor, len)
}

fn read_fixed<'a>(bytes: &'a [u8], cursor: &mut usize, len: usize) -> PmResult<&'a [u8]> {
    let end = cursor.saturating_add(len);
    let slice = bytes
        .get(*cursor..end)
        .ok_or_else(|| pm_err("field exceeds buffer"))?;
    *cursor = end;
    Ok(slice)
}

fn read_packed_varints(bytes: &[u8]) -> Result<Vec<u64>, FlightDeckError> {
    let mut cursor = 0usize;
    let mut out = Vec::new();
    while cursor < bytes.len() {
        out.push(read_varint(bytes, &mut cursor)?);
    }
    Ok(out)
}

fn skip_field(bytes: &[u8], cursor: &mut usize, wire: u64) -> Result<(), FlightDeckError> {
    match wire {
        0 => read_varint(bytes, cursor).map(|_| ()),
        1 => read_fixed(bytes, cursor, 8).map(|_| ()),
        2 => read_len_bytes(bytes, cursor).map(|_| ()),
        5 => read_fixed(bytes, cursor, 4).map(|_| ()),
        other => Err(pm_err(format!("unsupported wire type {other}"))),
    }
}

fn u64_le(bytes: &[u8], offset: usize) -> Result<u64, FlightDeckError> {
    let mut c = offset;
    let arr = <[u8; 8]>::try_from(read_fixed(bytes, &mut c, 8)?).unwrap_or([0; 8]);
    Ok(u64::from_le_bytes(arr))
}

fn i32_le(bytes: &[u8], offset: usize) -> Result<i32, FlightDeckError> {
    let mut c = offset;
    let arr = <[u8; 4]>::try_from(read_fixed(bytes, &mut c, 4)?).unwrap_or([0; 4]);
    Ok(i32::from_le_bytes(arr))
}
