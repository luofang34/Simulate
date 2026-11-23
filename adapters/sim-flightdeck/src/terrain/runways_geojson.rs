//! Surveyed airport runway GeoJSON `FeatureCollection` builder backed by
//! `sim_scenery::AirportDatabase::builtin()` on `WGS-84` Earth.

use crate::planet::PlanetaryReference;
use sim_scenery::{AirportDatabase, AirportRecord, CatalogVerticalDatum};

/// Generates a GeoJSON `FeatureCollection` of surveyed airport runways (surface polygons, perimeter outlines,
/// centerlines, threshold bars, approach lead-in lines, and designator labels) from
/// `AirportDatabase::builtin()`. Returns an empty `FeatureCollection` on non-Earth bodies.
#[must_use]
pub fn build_airport_runways_geojson(
    planet: &PlanetaryReference,
    center_lat_deg: f64,
    center_lon_deg: f64,
    radius_km: f64,
    route_codes: &[String],
) -> String {
    let Some(datum) = planet.catalog_vertical_datum() else {
        return "{\"type\":\"FeatureCollection\",\"features\":[]}".to_owned();
    };
    let db = AirportDatabase::builtin();
    let mut airports: Vec<(bool, &AirportRecord)> = Vec::new();
    let mut add = |is_dest: bool, rec: &'static AirportRecord| {
        if !airports
            .iter()
            .any(|(_, r)| r.ids.primary == rec.ids.primary)
        {
            airports.push((is_dest, rec));
        }
    };
    for code in route_codes {
        if let Some((_, rec)) = db.by_code(code) {
            add(true, rec);
        }
    }
    for hub in [
        "N51", "47N", "KEWR", "KJFK", "KLGA", "KTEB", "KORF", "KSNA", "KDCA", "KSEA", "KCOS",
    ] {
        if let Some((_, rec)) = db.by_code(hub) {
            let is_dest = route_codes.iter().any(|c| c == hub);
            add(is_dest, rec);
        }
    }
    let zoom = if radius_km > 120.0 { 7.8 } else { 9.4 };
    for idx in db.query_globe_visible(center_lat_deg, center_lon_deg, zoom, 48) {
        if let Some(rec) = db.get(idx) {
            add(false, rec);
        }
    }

    let mut out = String::with_capacity(16_384);
    out.push_str("{\"type\":\"FeatureCollection\",\"features\":[");
    let mut first = true;
    for (is_dest, airport) in airports {
        append_airport_runway_features(&mut out, &mut first, airport, is_dest, datum);
    }
    out.push_str("]}");
    out
}

fn append_airport_runway_features(
    out: &mut String,
    first: &mut bool,
    airport: &AirportRecord,
    is_dest: bool,
    datum: CatalogVerticalDatum,
) {
    let code = airport.ids.primary.as_str();
    let airport_has_paved = airport.runways.iter().any(|r| !r.surface.is_unpaved());
    for rwy in &airport.runways {
        if rwy.surface.is_unpaved() && (airport_has_paved || !is_dest) {
            continue;
        }
        let [end_a, end_b] = &rwy.ends;
        let Some(geom_a) = airport.landing_runway(end_a.designator.as_str(), datum) else {
            continue;
        };
        let (a_lat, a_lon) = (
            geom_a.pavement_start.lat_deg(),
            geom_a.pavement_start.lon_deg(),
        );
        let (b_lat, b_lon) = (geom_a.far_end.lat_deg(), geom_a.far_end.lon_deg());
        let cos_lat = a_lat.to_radians().cos().max(0.15);
        let half_w = (geom_a.width_m * 0.5).max(10.0);
        let dlon = (geom_a.true_bearing_rad.cos() * half_w) / (111_320.0 * cos_lat);
        let dlat = (-geom_a.true_bearing_rad.sin() * half_w) / 110_540.0;
        let unpaved_flag = usize::from(rwy.surface.is_unpaved());
        let dest_flag = usize::from(is_dest);
        let ring = format!(
            "[{:.6},{:.6}],[{:.6},{:.6}],[{:.6},{:.6}],[{:.6},{:.6}],[{:.6},{:.6}]",
            a_lon - dlon,
            a_lat - dlat,
            a_lon + dlon,
            a_lat + dlat,
            b_lon + dlon,
            b_lat + dlat,
            b_lon - dlon,
            b_lat - dlat,
            a_lon - dlon,
            a_lat - dlat,
        );
        push_sep(out, first);
        out.push_str(&format!(
            "{{\"type\":\"Feature\",\"properties\":{{\"kind\":\"strip\",\"dest\":{dest_flag},\"unpaved\":{unpaved_flag}}},\"geometry\":{{\"type\":\"Polygon\",\"coordinates\":[[{ring}]]}}}}"
        ));
        push_sep(out, first);
        out.push_str(&format!(
            "{{\"type\":\"Feature\",\"properties\":{{\"kind\":\"outline\",\"dest\":{dest_flag},\"unpaved\":{unpaved_flag}}},\"geometry\":{{\"type\":\"LineString\",\"coordinates\":[{ring}]}}}}"
        ));
        push_sep(out, first);
        out.push_str(&format!(
            "{{\"type\":\"Feature\",\"properties\":{{\"kind\":\"centerline\",\"dest\":{dest_flag},\"unpaved\":{unpaved_flag}}},\"geometry\":{{\"type\":\"LineString\",\"coordinates\":[[{a_lon:.6},{a_lat:.6}],[{b_lon:.6},{b_lat:.6}]]}}}}"
        ));
        for end in [end_a, end_b] {
            append_end_features(
                out,
                first,
                airport,
                end.designator.as_str(),
                code,
                dest_flag,
                cos_lat,
                dlon,
                dlat,
                datum,
            );
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn append_end_features(
    out: &mut String,
    first: &mut bool,
    airport: &AirportRecord,
    desig: &str,
    code: &str,
    dest_flag: usize,
    cos_lat: f64,
    dlon: f64,
    dlat: f64,
    datum: CatalogVerticalDatum,
) {
    let Some(g) = airport.landing_runway(desig, datum) else {
        return;
    };
    let (tlat, tlon) = (g.threshold.lat_deg(), g.threshold.lon_deg());
    push_sep(out, first);
    out.push_str(&format!(
        "{{\"type\":\"Feature\",\"properties\":{{\"kind\":\"threshold\",\"dest\":{dest_flag}}},\"geometry\":{{\"type\":\"LineString\",\"coordinates\":[[{:.6},{:.6}],[{:.6},{:.6}]]}}}}",
        tlon - dlon,
        tlat - dlat,
        tlon + dlon,
        tlat + dlat
    ));
    if !g.surface.is_unpaved() {
        let lead_m = if dest_flag == 1 { 3_600.0 } else { 1_800.0 };
        let llat = tlat - (g.true_bearing_rad.cos() * lead_m) / 110_540.0;
        let llon = tlon - (g.true_bearing_rad.sin() * lead_m) / (111_320.0 * cos_lat);
        push_sep(out, first);
        out.push_str(&format!(
            "{{\"type\":\"Feature\",\"properties\":{{\"kind\":\"lead\",\"dest\":{dest_flag}}},\"geometry\":{{\"type\":\"LineString\",\"coordinates\":[[{tlon:.6},{tlat:.6}],[{llon:.6},{llat:.6}]]}}}}"
        ));
    }
    let label = if dest_flag == 1 {
        format!("{code} RWY {desig}")
    } else {
        format!("{code} {desig}")
    };
    push_sep(out, first);
    out.push_str(&format!(
        "{{\"type\":\"Feature\",\"properties\":{{\"kind\":\"label\",\"dest\":{dest_flag},\"name\":\"{label}\"}},\"geometry\":{{\"type\":\"Point\",\"coordinates\":[{tlon:.6},{tlat:.6}]}}}}"
    ));
}

fn push_sep(out: &mut String, first: &mut bool) {
    if *first {
        *first = false;
    } else {
        out.push(',');
    }
}

/// Resolves an airport identifier (`"N51"`, `"KEWR"`, `"KHSV"`, `"EWR"`, etc.) from
/// `AirportDatabase::builtin()` on `WGS-84` Earth, returning UTF-8 JSON metadata with surveyed
/// coordinates, field elevation (`m` above declared datum), towered flag, and runway designators
/// (`"{}"` when unknown or on a non-Earth body).
#[must_use]
pub fn lookup_airport_json(planet: &PlanetaryReference, query_code: &str) -> String {
    if !planet.is_earth_wgs84() {
        return "{}".to_owned();
    }
    let clean: String = query_code
        .trim()
        .chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .map(|c| c.to_ascii_uppercase())
        .collect();
    if clean.len() < 2 || clean.len() > 6 {
        return "{}".to_owned();
    }
    let db = AirportDatabase::builtin();
    let found = db.by_code(&clean).or_else(|| {
        (clean.len() == 3)
            .then(|| format!("K{clean}"))
            .and_then(|k| db.by_code(&k))
    });
    let Some((_, rec)) = found else {
        return "{}".to_owned();
    };
    let msl_geo = PlanetaryReference::earth_wgs84_egm96().geodetic_from_datum(
        rec.lat_deg,
        rec.lon_deg,
        f64::from(rec.elevation_msl_m),
    );
    let elev_datum_m = planet.height_above_datum_m(&msl_geo);
    let mut rwy_list = String::new();
    let mut rwy_first = true;
    for rwy in &rec.runways {
        for end in &rwy.ends {
            let d = escape_json_ascii(end.designator.as_str());
            if !d.is_empty() {
                push_sep(&mut rwy_list, &mut rwy_first);
                rwy_list.push('"');
                rwy_list.push_str(&d);
                rwy_list.push('"');
            }
        }
    }
    let code = escape_json_ascii(rec.ids.primary.as_str());
    let name = escape_json_ascii(&rec.name);
    let city = escape_json_ascii(&rec.municipality);
    format!(
        "{{\"code\":\"{code}\",\"name\":\"{name}\",\"municipality\":\"{city}\",\
         \"latitude\":{:.6},\"longitude\":{:.6},\"elevation_m\":{:.2},\
         \"towered\":{},\"longest_runway_m\":{:.1},\"runways\":[{rwy_list}]}}",
        rec.lat_deg, rec.lon_deg, elev_datum_m, rec.towered, rec.longest_runway_m,
    )
}

fn escape_json_ascii(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if c.is_control() => out.push(' '),
            c => out.push(c),
        }
    }
    out
}
