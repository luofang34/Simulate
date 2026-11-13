//! Flight-plan route tokens, omni-search parsing, and proximity-ranked autocomplete.
//!
//! Modelled after `Pilotage`'s modern chip-based route editor (`RouteChipCell` and
//! `RouteEntryEditor`), adapted to portable Rust for both desktop and WebAssembly EFB views.

use sim_maplibre::FixId;
use sim_scenery::MATHIAS_RUST_1987_WAYPOINTS;

use crate::esc_menu::catalog::{AerodromeEntry, AerodromeKind, HubCatalog};

/// Visual badge category of a [`RouteToken`] chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipBadge {
    /// Surveyed or OSM aerodrome (`EFHF/18`, `KEWR/22L`).
    Airport,
    /// Runtime-delineated or historical ad-hoc strip (`ZMRS/15`, `ZZ01/09`).
    AdHocRunway,
    /// Named navigation fix or enroute waypoint (`KOHTL`, `KREML`).
    Waypoint,
    /// Pilot map-click or explicit coordinate pin (`WP01`, `55.75N`).
    MapPin,
}

/// Turn direction of a racetrack holding pattern attached to a [`RouteToken`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldTurn {
    /// Standard right-hand turns.
    Right,
    /// Non-standard left-hand turns.
    Left,
}

/// Holding pattern specification attached to a [`RouteToken`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoldSpec {
    /// Turn direction (`Right` standard, `Left` non-standard).
    pub turn: HoldTurn,
    /// Inbound leg timing, seconds (typically `60.0`).
    pub leg_seconds: f64,
    /// Optional explicit inbound true course, degrees (`None` uses inbound leg course).
    pub inbound_course_deg: Option<f64>,
}

impl Default for HoldSpec {
    fn default() -> Self {
        Self {
            turn: HoldTurn::Right,
            leg_seconds: 60.0,
            inbound_course_deg: None,
        }
    }
}

/// Semantic kind of one [`RouteToken`] in the EFB flight plan.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RouteTokenKind {
    /// Origin, destination, or intermediate aerodrome with optional runway end.
    Airport {
        /// Primary identifier (`KEWR`, `EFHF`, `ZMRS`).
        code: String,
        /// Optional runway designator (`22L`, `18`, `15`).
        runway: Option<String>,
        /// Whether this aerodrome is an ad-hoc / unimproved landing strip.
        ad_hoc: bool,
    },
    /// Named enroute or terminal fix (`KOHTL`, `KREML`).
    Waypoint {
        /// Five-letter or short identifier.
        ident: String,
    },
    /// Coordinate or map-click waypoint (`WP01`, `PIN1`).
    MapPin {
        /// Short display label.
        ident: String,
    },
}

/// One editable chip in an EFB flight plan.
#[derive(Debug, Clone, PartialEq)]
pub struct RouteToken {
    /// Unique identifier matching [`sim_maplibre::RouteFix::id`].
    pub id: FixId,
    /// Semantic token classification.
    pub kind: RouteTokenKind,
    /// Latitude, degrees (`[-90, 90]`).
    pub lat_deg: f64,
    /// Longitude, degrees (`[-180, 180]`).
    pub lon_deg: f64,
    /// Optional target crossing altitude in feet MSL (`100 ft` increments).
    pub altitude_ft: Option<i32>,
    /// Whether ownship must overfly the fix before sequencing (`true`) vs fly-by (`false`).
    pub fly_over: bool,
    /// Optional holding pattern anchored at this fix.
    pub hold: Option<HoldSpec>,
}

impl RouteToken {
    /// Short identifier for [`sim_maplibre::RouteFix`] and `#v=2` link export.
    #[must_use]
    pub fn ident(&self) -> String {
        match &self.kind {
            RouteTokenKind::Airport { code, .. } => code.clone(),
            RouteTokenKind::Waypoint { ident } | RouteTokenKind::MapPin { ident } => ident.clone(),
        }
    }

    /// Compact glyph-safe chip label (`EFHF/18`, `ZMRS/15 H-R`, `KREML 1500`).
    #[must_use]
    pub fn chip_label(&self) -> String {
        let mut base = match &self.kind {
            RouteTokenKind::Airport {
                code,
                runway: Some(rwy),
                ..
            } if !rwy.is_empty() => format!("{code}/{rwy}"),
            RouteTokenKind::Airport { code, .. } => code.clone(),
            RouteTokenKind::Waypoint { ident } | RouteTokenKind::MapPin { ident } => ident.clone(),
        };
        if let Some(hold) = self.hold {
            let dir = match hold.turn {
                HoldTurn::Right => "HR",
                HoldTurn::Left => "HL",
            };
            base.push(' ');
            base.push_str(dir);
        } else if let Some(alt_ft) = self.altitude_ft {
            base.push(' ');
            base.push_str(&format!("{}F", alt_ft / 100));
        }
        base
    }

    /// Visual badge style of this chip.
    #[must_use]
    pub const fn badge(&self) -> ChipBadge {
        match &self.kind {
            RouteTokenKind::Airport { ad_hoc: true, .. } => ChipBadge::AdHocRunway,
            RouteTokenKind::Airport { ad_hoc: false, .. } => ChipBadge::Airport,
            RouteTokenKind::Waypoint { .. } => ChipBadge::Waypoint,
            RouteTokenKind::MapPin { .. } => ChipBadge::MapPin,
        }
    }

    /// Airport code and runway designator when this token is an [`RouteTokenKind::Airport`].
    #[must_use]
    pub fn airport_and_runway(&self) -> Option<(&str, Option<&str>)> {
        match &self.kind {
            RouteTokenKind::Airport { code, runway, .. } => {
                Some((code.as_str(), runway.as_deref()))
            }
            RouteTokenKind::Waypoint { .. } | RouteTokenKind::MapPin { .. } => None,
        }
    }
}

/// One candidate row in the EFB omni-search autocomplete list.
#[derive(Debug, Clone, PartialEq)]
pub struct AutocompleteItem {
    /// Insertable token query (`ZMRS/15`, `EFHF/18`, `KREML`).
    pub insert_text: String,
    /// Primary display label (`ZMRS/15  BOLSHOY MOSKVORETSKY BRIDGE`).
    pub label: String,
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Badge category.
    pub badge: ChipBadge,
}

/// Built-in named enroute waypoints searchable from the EFB omni-box.
const EXTRA_WAYPOINTS: &[(&str, f64, f64, &str)] = &[
    ("ENO", 39.2316, -75.5159, "SMYRNA VORTAC DE"),
    ("SIE", 39.0955, -74.8003, "SEA ISLE VORTAC NJ"),
    ("RBV", 40.2024, -74.4950, "ROBBINSVILLE VORTAC NJ"),
    ("NP90S", -89.8800, 139.2729, "SOUTH POLE FINAL GATE"),
];

fn all_named_waypoints() -> impl Iterator<Item = (&'static str, f64, f64, &'static str)> {
    MATHIAS_RUST_1987_WAYPOINTS
        .iter()
        .map(|&(ident, lat, lon)| (ident, lat, lon, "MATHIAS RUST 1987 CORRIDOR"))
        .chain(EXTRA_WAYPOINTS.iter().copied())
}

/// Parses one space-delimited token (`EFHF/18`, `ZMRS/15`, `KREML`, `NAME:lat/lon`, or
/// `lat,lon`) into a [`RouteToken`] with `id`.
#[must_use]
pub fn parse_single_token(raw: &str, id: FixId, catalog: &HubCatalog) -> Option<RouteToken> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return None;
    }
    if let Some((ident, coords)) = trimmed.split_once(':')
        && let Some([lat_deg, lon_deg]) = parse_lat_lon_pair(coords)
    {
        return Some(RouteToken {
            id,
            kind: RouteTokenKind::Waypoint {
                ident: sanitize_ident(ident, "WPT"),
            },
            lat_deg,
            lon_deg,
            altitude_ft: None,
            fly_over: false,
            hold: None,
        });
    }
    if let Some([lat_deg, lon_deg]) = parse_lat_lon_pair(trimmed) {
        return Some(RouteToken {
            id,
            kind: RouteTokenKind::MapPin {
                ident: format!("P{:02}", id.0 % 100),
            },
            lat_deg,
            lon_deg,
            altitude_ft: None,
            fly_over: false,
            hold: None,
        });
    }
    let (code_part, rwy_part) = match trimmed.split_once('/') {
        Some((c, r)) => (c.trim(), Some(r.trim().to_ascii_uppercase())),
        None => (trimmed, None),
    };
    let upper = code_part.to_ascii_uppercase();
    if let Some(entry) = catalog
        .aerodromes
        .iter()
        .find(|a| a.code.eq_ignore_ascii_case(&upper) && a.lat_deg.is_some())
    {
        return token_from_aerodrome(entry, rwy_part.as_deref(), id);
    }
    let (_, lat_deg, lon_deg, _) = all_named_waypoints().find(|(w, ..)| *w == upper)?;
    Some(RouteToken {
        id,
        kind: RouteTokenKind::Waypoint { ident: upper },
        lat_deg,
        lon_deg,
        altitude_ft: None,
        fly_over: false,
        hold: None,
    })
}

fn token_from_aerodrome(
    entry: &AerodromeEntry,
    requested_rwy: Option<&str>,
    id: FixId,
) -> Option<RouteToken> {
    let (lat_deg, lon_deg) = (entry.lat_deg?, entry.lon_deg?);
    let chosen = requested_rwy
        .filter(|r| !r.is_empty())
        .and_then(|req| {
            entry
                .runways
                .iter()
                .find(|r| r.designator.eq_ignore_ascii_case(req))
                .map(|r| r.designator.clone())
        })
        .or_else(|| entry.runways.first().map(|r| r.designator.clone()));
    Some(RouteToken {
        id,
        kind: RouteTokenKind::Airport {
            code: entry.code.clone(),
            runway: chosen,
            ad_hoc: matches!(entry.kind, AerodromeKind::AdHoc(_)),
        },
        lat_deg,
        lon_deg,
        altitude_ft: None,
        fly_over: false,
        hold: None,
    })
}

/// Parses `lat,lon` or `lat/lon` decimal degrees when valid.
#[must_use]
pub fn parse_lat_lon_pair(raw: &str) -> Option<[f64; 2]> {
    let (a, b) = raw.split_once(',').or_else(|| raw.split_once('/'))?;
    let lat: f64 = a.trim().parse().ok()?;
    let lon: f64 = b.trim().parse().ok()?;
    (lat.is_finite()
        && lon.is_finite()
        && (-90.0..=90.0).contains(&lat)
        && (-180.0..=180.0).contains(&lon))
    .then_some([lat, lon])
}

/// Ranked autocomplete suggestions for `query` near `near_deg`, up to `limit` items.
#[must_use]
pub fn autocomplete_candidates(
    query: &str,
    near_deg: Option<[f64; 2]>,
    catalog: &HubCatalog,
    limit: usize,
) -> Vec<AutocompleteItem> {
    let token = query
        .split_whitespace()
        .last()
        .unwrap_or("")
        .trim()
        .to_ascii_uppercase();
    let code_prefix = token.split('/').next().unwrap_or("").trim();
    let mut out = Vec::new();
    for (ident, lat_deg, lon_deg, desc) in all_named_waypoints() {
        if code_prefix.is_empty() || ident.contains(code_prefix) || desc.contains(code_prefix) {
            out.push(AutocompleteItem {
                insert_text: ident.to_owned(),
                label: format!("{ident}  {desc}"),
                lat_deg,
                lon_deg,
                badge: ChipBadge::Waypoint,
            });
        }
    }
    let indices = catalog.filter_aerodromes(code_prefix);
    for idx in indices.into_iter().take(24) {
        let Some(entry) = catalog.aerodromes.get(idx) else {
            continue;
        };
        let (Some(lat_deg), Some(lon_deg)) = (entry.lat_deg, entry.lon_deg) else {
            continue;
        };
        let rwy = entry.runways.first().map(|r| r.designator.as_str());
        let insert_text = match rwy {
            Some(r) => format!("{}/{r}", entry.code),
            None => entry.code.clone(),
        };
        out.push(AutocompleteItem {
            label: format!("{insert_text}  {}", entry.name.to_ascii_uppercase()),
            insert_text,
            lat_deg,
            lon_deg,
            badge: if matches!(entry.kind, AerodromeKind::AdHoc(_)) {
                ChipBadge::AdHocRunway
            } else {
                ChipBadge::Airport
            },
        });
    }
    sort_candidates(&mut out, code_prefix, near_deg);
    out.truncate(limit);
    out
}

fn sort_candidates(items: &mut [AutocompleteItem], prefix: &str, near_deg: Option<[f64; 2]>) {
    items.sort_by(|a, b| {
        let a_exact = !prefix.is_empty() && a.insert_text.starts_with(prefix);
        let b_exact = !prefix.is_empty() && b.insert_text.starts_with(prefix);
        b_exact
            .cmp(&a_exact)
            .then_with(|| {
                (a.badge == ChipBadge::AdHocRunway)
                    .cmp(&(b.badge == ChipBadge::AdHocRunway))
                    .reverse()
            })
            .then_with(|| {
                let Some([nlat, nlon]) = near_deg else {
                    return std::cmp::Ordering::Equal;
                };
                let da = (a.lat_deg - nlat).hypot(a.lon_deg - nlon);
                let db = (b.lat_deg - nlat).hypot(b.lon_deg - nlon);
                da.total_cmp(&db)
            })
    });
}

pub(super) fn sanitize_ident(raw: &str, fallback: &str) -> String {
    let cleaned: String = raw
        .chars()
        .filter(|c| c.is_ascii_alphanumeric() || *c == '_' || *c == '-')
        .take(12)
        .collect::<String>()
        .to_ascii_uppercase();
    if cleaned.is_empty() {
        fallback.to_owned()
    } else {
        cleaned
    }
}
