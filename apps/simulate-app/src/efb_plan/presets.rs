//! Curated EFB mission and ad-hoc landing presets, including Mathias Rust's May 28, 1987
//! Helsinki-Malmi (`EFHF`) to Bolshoy Moskvoretsky Bridge (`ZMRS`, Red Square) flight and
//! stabilized short-final autoland demonstration.

use sim_maplibre::FixId;
use sim_scenery::MATHIAS_RUST_1987_WAYPOINTS;

use super::tokens::{RouteToken, RouteTokenKind, parse_single_token};
use crate::esc_menu::catalog::HubCatalog;

/// Identifier of a 1-click EFB mission preset card (`PRESETS` tab).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EfbPresetId {
    /// Mathias Rust's full May 28, 1987 VFR corridor from Helsinki-Malmi (`EFHF/18`) across
    /// Kohtla-Jarve, Gdov, Dno, Seliger, Torzhok, Klin, Khimki, Red Square (`KREML`), and
    /// Bolshaya Ordynka (`ORDNK`) to south-to-north landing on Bolshoy Moskvoretsky Bridge
    /// & Vasilevsky Descent (`ZMRS/33`).
    MathiasRust1987Corridor,
    /// Photo-verified 1987 stabilized south-to-north short final (`ZMRS RWY 33`, `456 × 36 m`,
    /// `128 m` MSL) onto Bolshoy Moskvoretsky Bridge rolling up Vasilevsky Descent to park
    /// beside St. Basil's Cathedral.
    RedSquareBridgeFinal,
    /// Stabilized short final directly onto **Red Square Cobblestone Plaza** (`ZMRQ RWY 30`,
    /// `55.75300° N, 37.62208° E -> 55.75475° N, 37.61857° E`, `294 × 45 m`, `144 m` MSL)
    /// between St. Basil's Cathedral and the State Historical Museum.
    RedSquarePlazaFinal,
    /// Regional visual-navigation evaluation corridor from Dover (`KDOV/01`) via Central
    /// Jersey (`47N/07`) to Newark (`KEWR/22L`).
    DelawareCorridor,
    /// Polar True-north (`GRID/TRUE`) approach to Amundsen-Scott South Pole Station
    /// (`NP90S -> NZSP/02T`, `2,835 m` MSL).
    SouthPoleSkiway,
}

impl EfbPresetId {
    /// All presets in `PRESETS` tab display order.
    pub const ALL: [Self; 5] = [
        Self::MathiasRust1987Corridor,
        Self::RedSquareBridgeFinal,
        Self::RedSquarePlazaFinal,
        Self::DelawareCorridor,
        Self::SouthPoleSkiway,
    ];

    /// Short card title rendered in the EFB `PRESETS` tab.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::MathiasRust1987Corridor => "MATHIAS RUST 1987: EFHF/18 TO ZMRS/33",
            Self::RedSquareBridgeFinal => "MOSKVORETSKY BRIDGE + DESCENT: ZMRS RWY 33",
            Self::RedSquarePlazaFinal => "RED SQUARE PLAZA DIRECT: ZMRQ RWY 30 AUTOLAND",
            Self::DelawareCorridor => "DELAWARE VNAV CORRIDOR: KDOV TO 47N TO KEWR",
            Self::SouthPoleSkiway => "AMUNDSEN-SCOTT POLAR SKIWAY: NP90S TO NZSP/02T",
        }
    }

    /// Subtitle describing distance, key waypoints, and spawn behaviour.
    #[must_use]
    pub const fn subtitle(self) -> &'static str {
        match self {
            Self::MathiasRust1987Corridor => "490 NM  C-172P D-ECJB  EFHF TO BRIDGE + ST BASIL'S",
            Self::RedSquareBridgeFinal => "1987 TOUCHDOWN  SOUTH-TO-NORTH BRIDGE TO ST BASIL'S",
            Self::RedSquarePlazaFinal => "55.7530N 37.6221E TO 55.7548N 37.6186E  294M PLAZA",
            Self::DelawareCorridor => "108 NM  KDOV/01 - ENO - 47N/07 - KEWR/22L",
            Self::SouthPoleSkiway => "TRUE RWY 02T  9301 FT MSL ICE PLATEAU",
        }
    }

    /// Builds the [`RouteToken`] sequence for `self` against `catalog`, advancing `next_id`.
    pub fn build_tokens(self, next_id: &mut u32, catalog: &HubCatalog) -> Vec<RouteToken> {
        match self {
            Self::MathiasRust1987Corridor => build_mathias_rust_tokens(next_id, catalog),
            Self::RedSquareBridgeFinal => {
                parse_token_list(&["ORDNK", "ZMRS/33"], Some(1_100), next_id, catalog)
            }
            Self::RedSquarePlazaFinal => {
                parse_token_list(&["ORDNK", "ZMRQ/30"], Some(1_150), next_id, catalog)
            }
            Self::DelawareCorridor => parse_token_list(
                &["KDOV/01", "ENO", "RBV", "47N/07", "KEWR/22L"],
                Some(3_500),
                next_id,
                catalog,
            ),
            Self::SouthPoleSkiway => {
                parse_token_list(&["NP90S", "NZSP/02T"], Some(10_500), next_id, catalog)
            }
        }
    }
}

fn alloc_id(next_id: &mut u32) -> FixId {
    let id = FixId(*next_id);
    *next_id = next_id.wrapping_add(1);
    id
}

fn build_mathias_rust_tokens(next_id: &mut u32, catalog: &HubCatalog) -> Vec<RouteToken> {
    let mut out = Vec::with_capacity(MATHIAS_RUST_1987_WAYPOINTS.len().saturating_add(2));
    if let Some(mut efhf) = parse_single_token("EFHF/18", alloc_id(next_id), catalog) {
        efhf.altitude_ft = Some(1_500);
        out.push(efhf);
    }
    for &(ident, lat_deg, lon_deg) in &MATHIAS_RUST_1987_WAYPOINTS {
        let alt_ft = if ident == "ORDNK" {
            Some(950)
        } else if ident == "KREML" {
            Some(1_150)
        } else if ident == "KHIMK" {
            Some(1_500)
        } else {
            Some(2_500)
        };
        out.push(RouteToken {
            id: alloc_id(next_id),
            kind: RouteTokenKind::Waypoint {
                ident: ident.to_owned(),
            },
            lat_deg,
            lon_deg,
            altitude_ft: alt_ft,
            fly_over: matches!(ident, "KREML" | "ORDNK"),
            hold: None,
        });
    }
    if let Some(mut zmrs) = parse_single_token("ZMRS/33", alloc_id(next_id), catalog) {
        zmrs.altitude_ft = Some(420);
        zmrs.fly_over = true;
        out.push(zmrs);
    }
    out
}

fn parse_token_list(
    raw_list: &[&str],
    cruise_ft: Option<i32>,
    next_id: &mut u32,
    catalog: &HubCatalog,
) -> Vec<RouteToken> {
    let last_idx = raw_list.len().saturating_sub(1);
    raw_list
        .iter()
        .enumerate()
        .filter_map(|(i, raw)| {
            let mut tok = parse_single_token(raw, alloc_id(next_id), catalog)?;
            if i > 0 && i < last_idx {
                tok.altitude_ft = cruise_ft;
            }
            Some(tok)
        })
        .collect()
}
