//! Zero-dependency XML extractor for FlightAware / Google Earth `<gx:Track>`, `<LineString>`,
//! and GPX `<trkpt>` flight track logs.

use crate::error::FlightDeckError;
use crate::telemetry::RouteWaypoint;

/// Raw decoded sample before kinematic smoothing.
#[derive(Clone, Copy, Debug)]
pub(super) struct RawSample {
    pub timestamp_s: Option<f64>,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_m: f64,
}

/// Raw document metadata and samples extracted from KML or GPX text.
#[derive(Clone, Debug)]
pub(super) struct ParsedTrackDoc {
    pub document_title: String,
    pub callsign: String,
    pub route_label: String,
    pub waypoints: Vec<RouteWaypoint>,
    pub samples: Vec<RawSample>,
}

pub(super) fn parse_track_xml(xml: &str) -> Result<ParsedTrackDoc, FlightDeckError> {
    let trimmed = xml.trim();
    if trimmed.is_empty() {
        return Err(FlightDeckError::KmlTrack {
            reason: "empty flight track document".to_owned(),
        });
    }
    if trimmed.contains("<trkpt") {
        return parse_gpx_doc(trimmed);
    }
    parse_kml_doc(trimmed)
}

fn parse_kml_doc(xml: &str) -> Result<ParsedTrackDoc, FlightDeckError> {
    let doc_title = first_document_name(xml);
    let mut waypoints = Vec::new();
    let mut track_callsign = String::new();
    let mut track_desc = String::new();
    let mut gx_samples = Vec::new();
    let mut line_samples = Vec::new();

    for placemark in extract_tag_bodies(xml, "Placemark") {
        let pm_name = clean_text(first_tag_body(placemark, "name").unwrap_or_default());
        let pm_desc = clean_text(first_tag_body(placemark, "description").unwrap_or_default());
        if let Some(point_body) = first_tag_body(placemark, "Point")
            && let Some(wp) = parse_point_waypoint(&pm_name, point_body)
        {
            waypoints.push(wp);
        }
        let pm_gx = parse_gx_tracks_in(placemark);
        if !pm_gx.is_empty() {
            if track_callsign.is_empty() && !pm_name.is_empty() {
                track_callsign = pm_name.clone();
            }
            if track_desc.is_empty() && !pm_desc.is_empty() {
                track_desc = pm_desc.clone();
            }
            gx_samples.extend(pm_gx);
        }
        if let Some(ls_body) = first_tag_body(placemark, "LineString") {
            let ls = parse_linestring_coords(ls_body);
            if ls.len() > line_samples.len() {
                if track_callsign.is_empty() && !pm_name.is_empty() {
                    track_callsign = pm_name;
                }
                line_samples = ls;
            }
        }
    }

    if gx_samples.is_empty() {
        gx_samples = parse_gx_tracks_in(xml);
    }
    let samples = if gx_samples.len() >= 2 {
        gx_samples
    } else if line_samples.len() >= 2 {
        line_samples
    } else {
        return Err(FlightDeckError::KmlTrack {
            reason: "no <gx:Track> or <LineString> with at least 2 valid coordinates".to_owned(),
        });
    };

    let callsign = resolve_callsign(&track_callsign, &doc_title);
    let route_label = resolve_route_label(&track_desc, &waypoints, &doc_title);
    Ok(ParsedTrackDoc {
        document_title: doc_title,
        callsign,
        route_label,
        waypoints,
        samples,
    })
}

fn parse_gpx_doc(xml: &str) -> Result<ParsedTrackDoc, FlightDeckError> {
    let title = clean_text(first_tag_body(xml, "name").unwrap_or_default());
    let mut samples = Vec::new();
    let mut rest = xml;
    while let Some((_, after_open)) = rest.split_once("<trkpt") {
        let Some((attrs, after_attrs)) = after_open.split_once('>') else {
            break;
        };
        let (body, tail) = match after_attrs.split_once("</trkpt>") {
            Some((b, t)) => (b, t),
            None => ("", after_attrs),
        };
        rest = tail;
        let lat = extract_attr(attrs, "lat").and_then(|s| s.parse::<f64>().ok());
        let lon = extract_attr(attrs, "lon").and_then(|s| s.parse::<f64>().ok());
        if let (Some(latitude_deg), Some(longitude_deg)) = (lat, lon)
            && valid_lat_lon(latitude_deg, longitude_deg)
        {
            let altitude_msl_m = first_tag_body(body, "ele")
                .and_then(|s| s.trim().parse::<f64>().ok())
                .filter(|v| v.is_finite())
                .unwrap_or(0.0);
            let timestamp_s = first_tag_body(body, "time").and_then(parse_iso8601_utc_seconds);
            samples.push(RawSample {
                timestamp_s,
                latitude_deg,
                longitude_deg,
                altitude_msl_m,
            });
        }
    }
    if samples.len() < 2 {
        return Err(FlightDeckError::KmlTrack {
            reason: "GPX track contains fewer than 2 valid <trkpt> fixes".to_owned(),
        });
    }
    let callsign = if title.is_empty() {
        "GPX".to_owned()
    } else {
        resolve_callsign(&title, &title)
    };
    Ok(ParsedTrackDoc {
        document_title: title,
        callsign,
        route_label: "GPX REPLAY".to_owned(),
        waypoints: Vec::new(),
        samples,
    })
}

fn parse_gx_tracks_in(scope: &str) -> Vec<RawSample> {
    let mut out = Vec::new();
    for track_body in extract_tag_bodies_ns(scope, "Track") {
        let mut whens = Vec::new();
        for w in extract_tag_bodies(track_body, "when") {
            whens.push(parse_iso8601_utc_seconds(w));
        }
        let mut coords = Vec::new();
        for c in extract_tag_bodies_ns(track_body, "coord") {
            if let Some(xyz) = parse_space_coord(c) {
                coords.push(xyz);
            }
        }
        for (idx, &(lon, lat, alt)) in coords.iter().enumerate() {
            let timestamp_s = whens.get(idx).copied().flatten();
            out.push(RawSample {
                timestamp_s,
                latitude_deg: lat,
                longitude_deg: lon,
                altitude_msl_m: alt,
            });
        }
    }
    out
}

fn parse_linestring_coords(scope: &str) -> Vec<RawSample> {
    let Some(coords_body) = first_tag_body(scope, "coordinates") else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for token in coords_body.split_whitespace() {
        if let Some((lon, lat, alt)) = parse_comma_coord(token) {
            out.push(RawSample {
                timestamp_s: None,
                latitude_deg: lat,
                longitude_deg: lon,
                altitude_msl_m: alt,
            });
        }
    }
    out
}

fn parse_point_waypoint(pm_name: &str, point_body: &str) -> Option<RouteWaypoint> {
    let coords = first_tag_body(point_body, "coordinates")?;
    let first_tok = coords.split_whitespace().next()?;
    let (lon, lat, _) = parse_comma_coord(first_tok)?;
    let ident_raw = pm_name
        .split_whitespace()
        .next()
        .unwrap_or(pm_name)
        .trim_matches(|c: char| !c.is_ascii_alphanumeric());
    if ident_raw.is_empty() {
        return None;
    }
    Some(RouteWaypoint {
        ident: ident_raw.to_ascii_uppercase(),
        latitude_deg: lat,
        longitude_deg: lon,
    })
}

fn parse_space_coord(raw: &str) -> Option<(f64, f64, f64)> {
    let mut parts = raw.split_whitespace();
    let lon = parts.next()?.parse::<f64>().ok()?;
    let lat = parts.next()?.parse::<f64>().ok()?;
    let alt = parts
        .next()
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|v| v.is_finite())
        .unwrap_or(0.0);
    valid_lat_lon(lat, lon).then_some((lon, lat, alt))
}

fn parse_comma_coord(raw: &str) -> Option<(f64, f64, f64)> {
    let mut parts = raw.split(',');
    let lon = parts.next()?.trim().parse::<f64>().ok()?;
    let lat = parts.next()?.trim().parse::<f64>().ok()?;
    let alt = parts
        .next()
        .and_then(|s| s.trim().parse::<f64>().ok())
        .filter(|v| v.is_finite())
        .unwrap_or(0.0);
    valid_lat_lon(lat, lon).then_some((lon, lat, alt))
}

fn valid_lat_lon(lat: f64, lon: f64) -> bool {
    lat.is_finite()
        && lon.is_finite()
        && (-90.0..=90.0).contains(&lat)
        && (-180.0..=180.0).contains(&lon)
}

fn first_document_name(xml: &str) -> String {
    let before_pm = xml.split_once("<Placemark").map_or(xml, |(head, _)| head);
    clean_text(first_tag_body(before_pm, "name").unwrap_or_default())
}

fn resolve_callsign(track_name: &str, doc_title: &str) -> String {
    let candidate = track_name.trim();
    if !candidate.is_empty() && candidate.len() <= 12 && !candidate.contains(' ') {
        return candidate.to_ascii_uppercase();
    }
    for word in doc_title.split_whitespace() {
        let trimmed = word.trim_matches(|c: char| !c.is_ascii_alphanumeric());
        if (3..=8).contains(&trimmed.len())
            && trimmed.chars().any(|c| c.is_ascii_digit())
            && trimmed.chars().all(|c| c.is_ascii_alphanumeric())
        {
            return trimmed.to_ascii_uppercase();
        }
    }
    if candidate.is_empty() {
        "KML".to_owned()
    } else {
        candidate
            .chars()
            .take(10)
            .collect::<String>()
            .to_ascii_uppercase()
    }
}

fn resolve_route_label(desc: &str, waypoints: &[RouteWaypoint], doc_title: &str) -> String {
    if let Some((lhs, rhs)) = desc.split_once('-') {
        let a = lhs.trim();
        let b = rhs.trim();
        if !a.is_empty() && !b.is_empty() && a.len() <= 8 && b.len() <= 8 {
            return format!("{} → {}", a.to_ascii_uppercase(), b.to_ascii_uppercase());
        }
    }
    if let (Some(first), Some(last)) = (waypoints.first(), waypoints.last())
        && waypoints.len() >= 2
    {
        return format!("{} → {}", first.ident, last.ident);
    }
    if let Some((_, paren)) = doc_title.rsplit_once('(')
        && let Some((inside, _)) = paren.split_once(')')
        && let Some((a, b)) = inside.split_once('-')
    {
        return format!(
            "{} → {}",
            a.trim().to_ascii_uppercase(),
            b.trim().to_ascii_uppercase()
        );
    }
    String::new()
}

fn clean_text(raw: &str) -> String {
    let unwrapped = raw
        .trim()
        .strip_prefix("<![CDATA[")
        .and_then(|s| s.strip_suffix("]]>"))
        .unwrap_or(raw);
    unwrapped.trim().to_owned()
}

fn first_tag_body<'a>(haystack: &'a str, local_tag: &str) -> Option<&'a str> {
    extract_tag_bodies(haystack, local_tag).into_iter().next()
}

fn extract_tag_bodies<'a>(haystack: &'a str, tag: &str) -> Vec<&'a str> {
    let open_pat = format!("<{tag}");
    let close_pat = format!("</{tag}>");
    collect_bodies(haystack, &open_pat, &close_pat)
}

fn extract_tag_bodies_ns<'a>(haystack: &'a str, local_tag: &str) -> Vec<&'a str> {
    let gx_open = format!("<gx:{local_tag}");
    let gx_close = format!("</gx:{local_tag}>");
    let mut out = collect_bodies(haystack, &gx_open, &gx_close);
    if out.is_empty() {
        let plain_open = format!("<{local_tag}");
        let plain_close = format!("</{local_tag}>");
        out = collect_bodies(haystack, &plain_open, &plain_close);
    }
    out
}

fn collect_bodies<'a>(haystack: &'a str, open_pat: &str, close_pat: &str) -> Vec<&'a str> {
    let mut out = Vec::new();
    let mut rest = haystack;
    while let Some((_, after_open)) = rest.split_once(open_pat) {
        let next_ch = after_open.chars().next();
        if !matches!(next_ch, Some('>' | ' ' | '\t' | '\r' | '\n')) {
            rest = after_open;
            continue;
        }
        let Some((_, after_gt)) = after_open.split_once('>') else {
            break;
        };
        let Some((body, tail)) = after_gt.split_once(close_pat) else {
            break;
        };
        out.push(body);
        rest = tail;
    }
    out
}

fn extract_attr<'a>(attrs: &'a str, name: &str) -> Option<&'a str> {
    for quote in ['"', '\''] {
        let pat = format!("{name}={quote}");
        if let Some((_, after)) = attrs.split_once(&pat)
            && let Some((val, _)) = after.split_once(quote)
        {
            return Some(val);
        }
    }
    None
}

/// Parses an ISO-8601 timestamp (`YYYY-MM-DDTHH:MM:SS[.fff][Z|±HH:MM]`) to Unix epoch seconds.
pub(super) fn parse_iso8601_utc_seconds(raw: &str) -> Option<f64> {
    let s = raw.trim();
    let (date_part, time_and_tz) = s.split_once('T').or_else(|| s.split_once(' '))?;
    let mut d_it = date_part.split('-');
    let year = d_it.next()?.parse::<i64>().ok()?;
    let month = d_it.next()?.parse::<u32>().ok()?;
    let day = d_it.next()?.parse::<u32>().ok()?;
    if !(1..=12).contains(&month) || !(1..=31).contains(&day) {
        return None;
    }

    let (time_part, tz_offset_s) = split_tz_offset(time_and_tz)?;
    let mut t_it = time_part.split(':');
    let hour = t_it.next()?.parse::<u32>().ok()?;
    let minute = t_it.next()?.parse::<u32>().ok()?;
    let sec = t_it.next().map_or(Some(0.0), |v| v.parse::<f64>().ok())?;
    if hour > 23 || minute > 59 || !(0.0..61.0).contains(&sec) {
        return None;
    }

    let days = days_from_civil(year, month, day);
    let total_s = (days * 86_400 + i64::from(hour) * 3_600 + i64::from(minute) * 60 - tz_offset_s)
        as f64
        + sec;
    Some(total_s)
}

fn split_tz_offset(time_and_tz: &str) -> Option<(&str, i64)> {
    if let Some(stripped) = time_and_tz
        .strip_suffix('Z')
        .or_else(|| time_and_tz.strip_suffix('z'))
    {
        return Some((stripped, 0));
    }
    for (sign_ch, sign_mul) in [('+', 1_i64), ('-', -1_i64)] {
        if let Some((lhs, rhs)) = time_and_tz.rsplit_once(sign_ch) {
            let mut it = rhs.split(':');
            let hh = it.next()?.parse::<i64>().ok()?;
            let mm = it.next().map_or(Some(0_i64), |m| m.parse::<i64>().ok())?;
            return Some((lhs, sign_mul * (hh * 3_600 + mm * 60)));
        }
    }
    Some((time_and_tz, 0))
}

/// Howard Hinnant's civil-to-days algorithm relative to `1970-01-01`.
const fn days_from_civil(year: i64, month: u32, day: u32) -> i64 {
    let y = year - if month <= 2 { 1 } else { 0 };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = (y - era * 400) as u64;
    let m = month as u64;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + (day as u64) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + (doe as i64) - 719_468
}
