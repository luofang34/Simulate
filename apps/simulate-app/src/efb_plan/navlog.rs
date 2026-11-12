//! Spherical geodesy, Navigation Log (`NavLog`) metrics, `sim_maplibre::FlightPlan`
//! assembly, lateral leg sequencing, and `#v=2` [`sim_link::SimulationLink`] export/import.

use sim_link::{
    Bearing, Duration as LinkDuration, Fix as LinkFix, Height, HeightDatum, HeightUnit, HoldLength,
    Leg as LinkLeg, Limit, Means, Micro, Milli, NavSet, North, PathTerminator, SimulationLink,
    StartTime, TimeUnit, TurnDirection, Vehicle, decode_fragment,
};
use sim_maplibre::{FixId, FlightPlan, HoldPattern, RouteFix, RouteProgress};

use super::tokens::{HoldSpec, HoldTurn, RouteToken, RouteTokenKind, sanitize_ident};
use crate::esc_menu::catalog::HubCatalog;

const EARTH_RADIUS_NM: f64 = 3_440.065;
const METRES_PER_NM: f64 = 1_852.0;
const METRES_PER_FOOT: f64 = 0.3048;

/// Computed leg row in the EFB Navigation Log (`NAVLOG` tab).
#[derive(Debug, Clone, PartialEq)]
pub struct NavLogRow {
    /// Zero-based leg index (`fixes[leg_index] -> fixes[leg_index + 1]`).
    pub leg_index: usize,
    /// Start fix label.
    pub from_ident: String,
    /// End fix label.
    pub to_ident: String,
    /// Initial true course, degrees `[0, 360)`.
    pub course_true_deg: f64,
    /// Leg great-circle length, nautical miles.
    pub leg_nm: f64,
    /// Cumulative route distance at the end of this leg, nautical miles.
    pub cumulative_nm: f64,
    /// Estimated leg time in seconds at the reference ground speed.
    pub ete_s: f64,
    /// Optional target altitude in feet MSL.
    pub altitude_ft: Option<i32>,
    /// Whether this leg carries a hold at its termination fix.
    pub has_hold: bool,
}

/// Live active-leg navigation summary displayed in the EFB header pill and cockpit glance bar.
#[derive(Debug, Clone, PartialEq)]
pub struct ActiveLegSummary {
    /// Active `TO` fix identifier.
    pub to_ident: String,
    /// Direct true bearing from ownship to the active fix, degrees `[0, 360)`.
    pub bearing_deg: f64,
    /// Distance from ownship to the active fix, nautical miles.
    pub distance_nm: f64,
    /// Signed cross-track error from the active leg (`> 0` right of course), metres.
    pub cross_track_m: f64,
    /// Remaining total route distance from ownship through destination, nautical miles.
    pub remaining_nm: f64,
    /// Estimated time to destination, seconds.
    pub ete_total_s: f64,
    /// Active RNP lateral containment scale (`2.0` ENR, `1.0` TERM, `0.3` APR), nautical miles.
    pub rnp_nm: f64,
    /// Flight phase badge (`ENR`, `TERM`, `APR`).
    pub phase_badge: &'static str,
}

/// Great-circle distance in nautical miles and initial true bearing `[0, 360)` between two
/// `[lat_deg, lon_deg]` points.
#[must_use]
pub fn distance_and_bearing_nm(from_deg: [f64; 2], to_deg: [f64; 2]) -> (f64, f64) {
    let [lat1, lon1] = from_deg.map(f64::to_radians);
    let [lat2, lon2] = to_deg.map(f64::to_radians);
    let dlat = lat2 - lat1;
    let dlon = lon2 - lon1;
    let half_dlat = (dlat * 0.5).sin();
    let half_dlon = (dlon * 0.5).sin();
    let a =
        (half_dlat * half_dlat + lat1.cos() * lat2.cos() * half_dlon * half_dlon).clamp(0.0, 1.0);
    let central = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    let y = dlon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    let bearing_deg = y.atan2(x).to_degrees().rem_euclid(360.0);
    (central * EARTH_RADIUS_NM, bearing_deg)
}

/// Cross-track distance in metres (`> 0` right of course `from_deg -> to_deg`) and along-track
/// distance in nautical miles from `from_deg`.
#[must_use]
pub fn cross_and_along_track(
    from_deg: [f64; 2],
    to_deg: [f64; 2],
    own_deg: [f64; 2],
) -> (f64, f64) {
    let (d13_nm, brg13_deg) = distance_and_bearing_nm(from_deg, own_deg);
    let (_, course_deg) = distance_and_bearing_nm(from_deg, to_deg);
    let d13_rad = d13_nm / EARTH_RADIUS_NM;
    let diff_rad = (brg13_deg - course_deg).to_radians();
    let xtk_rad = (d13_rad.sin() * diff_rad.sin()).clamp(-1.0, 1.0).asin();
    let atk_rad = (d13_rad.cos() / xtk_rad.cos().max(1e-9))
        .clamp(-1.0, 1.0)
        .acos();
    let sign = if diff_rad.cos() >= 0.0 { 1.0 } else { -1.0 };
    (
        xtk_rad * EARTH_RADIUS_NM * METRES_PER_NM,
        sign * atk_rad * EARTH_RADIUS_NM,
    )
}

/// Builds the full Navigation Log rows for `tokens` at `ground_speed_kt`.
#[must_use]
pub fn compute_navlog(tokens: &[RouteToken], ground_speed_kt: f64) -> Vec<NavLogRow> {
    let gs_kt = ground_speed_kt.clamp(35.0, 600.0);
    let mut cumulative_nm = 0.0;
    let mut rows = Vec::new();
    for (leg_index, pair) in tokens.windows(2).enumerate() {
        let [from, to] = pair else {
            continue;
        };
        let (leg_nm, course_true_deg) =
            distance_and_bearing_nm([from.lat_deg, from.lon_deg], [to.lat_deg, to.lon_deg]);
        cumulative_nm += leg_nm;
        let ete_s = (leg_nm / gs_kt) * 3_600.0;
        rows.push(NavLogRow {
            leg_index,
            from_ident: from.chip_label(),
            to_ident: to.chip_label(),
            course_true_deg,
            leg_nm,
            cumulative_nm,
            ete_s,
            altitude_ft: to.altitude_ft,
            has_hold: to.hold.is_some(),
        });
    }
    rows
}

/// Zero-based active leg index (`tokens[k] -> tokens[k + 1]`) for `progress`.
#[must_use]
pub fn active_leg_of(progress: RouteProgress, token_count: usize) -> usize {
    let leg_count = token_count.saturating_sub(1);
    if leg_count == 0 {
        return 0;
    }
    progress
        .to_fix
        .unwrap_or(1)
        .saturating_sub(1)
        .min(leg_count.saturating_sub(1))
}

/// Converts `tokens` into a `sim_maplibre::FlightPlan` and portrayal `RouteProgress`.
#[must_use]
pub fn build_maplibre_plan(
    tokens: &[RouteToken],
    progress: RouteProgress,
    selected_chip: Option<usize>,
    cruise_kt: f64,
) -> (FlightPlan, RouteProgress) {
    let speed_mps = (cruise_kt.clamp(45.0, 450.0) * METRES_PER_NM) / 3_600.0;
    let fixes: Vec<RouteFix> = tokens
        .iter()
        .map(|t| RouteFix {
            id: t.id,
            ident: t.chip_label(),
            lat_deg: t.lat_deg,
            lon_deg: t.lon_deg,
            altitude_m: t.altitude_ft.map(|ft| f64::from(ft) * METRES_PER_FOOT),
        })
        .collect();
    let preview_hold = progress.hold.or_else(|| {
        let candidate = selected_chip
            .filter(|&i| tokens.get(i).and_then(|t| t.hold).is_some())
            .or(progress.to_fix)
            .or_else(|| tokens.iter().position(|t| t.hold.is_some()))?;
        hold_pattern_at(tokens, candidate, speed_mps)
    });
    let shown = RouteProgress {
        to_fix: if tokens.len() >= 2 {
            Some(
                progress
                    .to_fix
                    .unwrap_or(1)
                    .clamp(1, tokens.len().saturating_sub(1)),
            )
        } else {
            None
        },
        complete: progress.complete,
        hold: preview_hold,
    };
    (
        FlightPlan {
            origin: None,
            fixes,
        },
        shown,
    )
}

fn hold_pattern_at(tokens: &[RouteToken], idx: usize, speed_mps: f64) -> Option<HoldPattern> {
    let to = tokens.get(idx)?;
    let hold = to.hold?;
    let prev = tokens.get(idx.saturating_sub(1)).unwrap_or(to);
    let (_, inbound_deg) = if idx > 0 {
        distance_and_bearing_nm([prev.lat_deg, prev.lon_deg], [to.lat_deg, to.lon_deg])
    } else {
        (0.0, 0.0)
    };
    Some(HoldPattern {
        fix_deg: [to.lat_deg, to.lon_deg],
        inbound_course_deg: hold.inbound_course_deg.unwrap_or(inbound_deg),
        right_turns: hold.turn == HoldTurn::Right,
        turn_radius_m: ((speed_mps * speed_mps) / (9.806_65 * 22.0_f64.to_radians().tan()))
            .clamp(350.0, 3_500.0),
        leg_length_m: (speed_mps * hold.leg_seconds).clamp(900.0, 15_000.0),
    })
}

/// Advances `progress` along `tokens` when `own_deg` passes the active `TO` fix's capture
/// boundary (unless holding at that fix).
#[must_use]
pub fn sequence_progress(
    tokens: &[RouteToken],
    current: RouteProgress,
    own_deg: [f64; 2],
) -> RouteProgress {
    let leg_count = tokens.len().saturating_sub(1);
    if leg_count == 0 {
        return RouteProgress::default();
    }
    let active = active_leg_of(current, tokens.len());
    let to_idx = active.wrapping_add(1);
    let (Some(from), Some(to)) = (tokens.get(active), tokens.get(to_idx)) else {
        return RouteProgress {
            to_fix: Some(to_idx),
            complete: false,
            hold: None,
        };
    };
    let (dist_to_nm, _) = distance_and_bearing_nm(own_deg, [to.lat_deg, to.lon_deg]);
    if to.hold.is_some() && dist_to_nm < 0.85 {
        return RouteProgress {
            to_fix: Some(to_idx),
            complete: false,
            hold: hold_pattern_at(tokens, to_idx, 54.0),
        };
    }
    let (leg_nm, _) =
        distance_and_bearing_nm([from.lat_deg, from.lon_deg], [to.lat_deg, to.lon_deg]);
    let (_, along_nm) = cross_and_along_track(
        [from.lat_deg, from.lon_deg],
        [to.lat_deg, to.lon_deg],
        own_deg,
    );
    let capture_nm = if to.fly_over { 0.18 } else { 0.45 };
    if active.wrapping_add(1) < leg_count && (dist_to_nm <= capture_nm || along_nm > leg_nm + 0.15)
    {
        return RouteProgress {
            to_fix: Some(to_idx.wrapping_add(1)),
            complete: false,
            hold: None,
        };
    }
    RouteProgress {
        to_fix: Some(to_idx),
        complete: current.complete,
        hold: if to.hold.is_some() {
            current.hold
        } else {
            None
        },
    }
}

/// Computes the live [`ActiveLegSummary`] for `own_deg` at `ground_speed_kt`.
#[must_use]
pub fn active_leg_summary(
    tokens: &[RouteToken],
    progress: RouteProgress,
    own_deg: [f64; 2],
    ground_speed_kt: f64,
) -> Option<ActiveLegSummary> {
    if tokens.len() < 2 {
        return None;
    }
    let active = active_leg_of(progress, tokens.len());
    let from = tokens.get(active)?;
    let to = tokens.get(active.wrapping_add(1))?;
    let (distance_nm, bearing_deg) = distance_and_bearing_nm(own_deg, [to.lat_deg, to.lon_deg]);
    let (cross_track_m, _) = cross_and_along_track(
        [from.lat_deg, from.lon_deg],
        [to.lat_deg, to.lon_deg],
        own_deg,
    );
    let mut remaining_nm = distance_nm;
    for pair in tokens
        .get(active.wrapping_add(1)..)
        .unwrap_or(&[])
        .windows(2)
    {
        if let [a, b] = pair {
            remaining_nm +=
                distance_and_bearing_nm([a.lat_deg, a.lon_deg], [b.lat_deg, b.lon_deg]).0;
        }
    }
    let dest_is_rwy = tokens
        .last()
        .and_then(RouteToken::airport_and_runway)
        .is_some_and(|(_, rwy)| rwy.is_some());
    let (rnp_nm, phase_badge) = if dest_is_rwy && remaining_nm <= 5.0 {
        (0.3, "APR")
    } else if remaining_nm <= 30.0 {
        (1.0, "TERM")
    } else {
        (2.0, "ENR")
    };
    let gs_kt = ground_speed_kt.clamp(40.0, 550.0);
    Some(ActiveLegSummary {
        to_ident: to.chip_label(),
        bearing_deg,
        distance_nm,
        cross_track_m,
        remaining_nm,
        ete_total_s: (remaining_nm / gs_kt) * 3_600.0,
        rnp_nm,
        phase_badge,
    })
}

/// Builds [`LinkLeg`]s (`IF`, `TF`, and optional `HM` holds) for `tokens`.
pub fn build_simulation_link_legs(tokens: &[RouteToken]) -> Result<Vec<LinkLeg>, String> {
    if tokens.len() < 2 {
        return Err("at least 2 route fixes are required to export a #v=2 link".to_owned());
    }
    let mut legs = Vec::with_capacity(tokens.len().saturating_add(2));
    for (idx, token) in tokens.iter().enumerate() {
        let fix = LinkFix {
            name: Some(sanitize_ident(&token.ident(), "FIX")),
            lat: Micro::from_scaled((token.lat_deg.clamp(-90.0, 90.0) * 1_000_000.0).round() as i64),
            lon: Micro::from_scaled(
                (token.lon_deg.clamp(-180.0, 180.0) * 1_000_000.0).round() as i64
            ),
        };
        let mut leg = LinkLeg::new(
            if idx == 0 {
                PathTerminator::If
            } else {
                PathTerminator::Tf
            },
            Some(fix.clone()),
        );
        leg.fly_over = token.fly_over;
        if let Some(alt_ft) = token.altitude_ft.filter(|&ft| ft > 0) {
            leg.altitude = Some(Limit::At(Height {
                value: Milli::from_whole(i64::from(alt_ft)),
                unit: HeightUnit::Foot,
            }));
        }
        legs.push(leg);
        if let Some(hold) = token.hold {
            let prev = tokens.get(idx.saturating_sub(1)).unwrap_or(token);
            legs.push(build_hold_link_leg(prev, token, fix, hold));
        }
    }
    Ok(legs)
}

fn build_hold_link_leg(
    prev: &RouteToken,
    token: &RouteToken,
    fix: LinkFix,
    hold: HoldSpec,
) -> LinkLeg {
    let (_, inbound) =
        distance_and_bearing_nm([prev.lat_deg, prev.lon_deg], [token.lat_deg, token.lon_deg]);
    let raw_deg = hold.inbound_course_deg.unwrap_or(inbound).rem_euclid(360.0);
    let deg_milli = ((raw_deg * 1_000.0).round() as i64).rem_euclid(360_000);
    let sec_milli = (hold.leg_seconds.clamp(30.0, 300.0) * 1_000.0).round() as i64;
    let mut hm = LinkLeg::new(PathTerminator::Hm, Some(fix));
    hm.course = Some(Bearing {
        degrees: Milli::from_scaled(deg_milli),
        north: North::True,
    });
    hm.turn = Some(match hold.turn {
        HoldTurn::Right => TurnDirection::Right,
        HoldTurn::Left => TurnDirection::Left,
    });
    hm.hold_length = Some(HoldLength::Time(LinkDuration {
        value: Milli::from_scaled(sec_milli),
        unit: TimeUnit::Second,
    }));
    hm
}

/// Exports `tokens` (requires `>= 2` tokens) into a canonical `#v=2` [`SimulationLink`]
/// URL fragment (`v=2&r=IF,...;TF,...`).
pub fn export_simulation_link_fragment(
    tokens: &[RouteToken],
    cruise_alt_m: i32,
) -> Result<String, String> {
    let legs = build_simulation_link_legs(tokens)?;
    let link = SimulationLink {
        route: legs,
        default_height: Some(Milli::from_whole(i64::from(
            cruise_alt_m.clamp(100, 12_000),
        ))),
        datum: HeightDatum::MeanSeaLevel,
        start: None,
        time: StartTime {
            year: 2026,
            month: 5,
            day: 28,
            hour: 12,
            minute: 0,
            second: 0,
        },
        vehicle: Vehicle::C172,
        cruise_speed: None,
        seed: 42,
        sets: vec![NavSet(vec![
            Means::Ins,
            Means::Baro,
            Means::Gnss,
            Means::Visual,
        ])],
        steering_set: 0,
        outages: Vec::new(),
        spoof: None,
        camera: None,
        auto_start: false,
        budget_mb: 256,
    };
    link.fragment().map_err(|e| e.to_string())
}

/// Imports a `#v=2` URL or fragment into [`RouteToken`]s resolved against `catalog`.
pub fn import_simulation_link_fragment(
    fragment_or_url: &str,
    next_id: &mut u32,
    catalog: &HubCatalog,
) -> Result<Vec<RouteToken>, String> {
    let trimmed = fragment_or_url.trim();
    let raw_frag = trimmed.strip_prefix('#').unwrap_or(trimmed);
    let link = decode_fragment(raw_frag).map_err(|e| e.to_string())?;
    let mut tokens: Vec<RouteToken> = Vec::new();
    for leg in &link.route {
        if matches!(
            leg.path,
            PathTerminator::Ha | PathTerminator::Hf | PathTerminator::Hm
        ) {
            if let Some(last) = tokens.last_mut() {
                last.hold = Some(HoldSpec {
                    turn: match leg.turn {
                        Some(TurnDirection::Left) => HoldTurn::Left,
                        _ => HoldTurn::Right,
                    },
                    leg_seconds: 60.0,
                    inbound_course_deg: leg.course.map(|c| c.degrees.as_f64()),
                });
            }
            continue;
        }
        let Some(fix) = &leg.fix else {
            continue;
        };
        let [lat_deg, lon_deg] = [fix.lat_deg(), fix.lon_deg()];
        let id = FixId(*next_id);
        *next_id = next_id.wrapping_add(1);
        let ident = fix
            .name
            .clone()
            .unwrap_or_else(|| format!("P{:02}", id.0 % 100));
        let kind = if let Some(entry) = catalog
            .aerodromes
            .iter()
            .find(|a| a.code.eq_ignore_ascii_case(&ident))
        {
            RouteTokenKind::Airport {
                code: entry.code.clone(),
                runway: entry.runways.first().map(|r| r.designator.clone()),
                ad_hoc: matches!(
                    entry.kind,
                    crate::esc_menu::catalog::AerodromeKind::AdHoc(_)
                ),
            }
        } else {
            RouteTokenKind::Waypoint { ident }
        };
        tokens.push(RouteToken {
            id,
            kind,
            lat_deg,
            lon_deg,
            altitude_ft: None,
            fly_over: leg.fly_over,
            hold: None,
        });
    }
    Ok(tokens)
}
