//! ICAO PBN / RNP error decomposition (`NSE`, `FTE`, `TSE`) and containment
//! bounds from Pilotage's active navigation display and exact truth.

use glam::DVec3;
use navigate_contract::GeodeticPosition;
use navigate_geodesy::{BodyEllipsoid, cross_track_m};
use sim_flight_chain::{FlightPhase, LegDisplay, NavigationDisplay};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::types::{GuidanceRnpTelemetry, NavigatorTelemetry};

/// Exact international nautical mile in metres.
pub const METRES_PER_NM: f64 = 1_852.0;

/// Inputs needed to evaluate [`GuidanceRnpTelemetry`] at one instant.
pub(super) struct RnpContext<'a> {
    pub(super) steered_set: &'a str,
    pub(super) display: Option<&'a NavigationDisplay>,
    pub(super) start_fix: GeodeticPosition,
    pub(super) truth_ecef: DVec3,
    pub(super) steered_est_ecef: Option<DVec3>,
    pub(super) steered_nav: Option<&'a NavigatorTelemetry>,
    pub(super) configured_rnp_nm: Option<f64>,
}

/// Evaluates active guidance modes and the RNP (`NSE` / `FTE` / `TSE`) triad.
pub(super) fn evaluate_rnp(ctx: &RnpContext<'_>) -> GuidanceRnpTelemetry {
    let (flight_phase, default_rnp_nm) = match ctx.display.map(|d| d.phase) {
        Some(FlightPhase::Enroute) => ("enroute", 2.0),
        Some(FlightPhase::Terminal) => ("terminal", 1.0),
        Some(FlightPhase::Approach) => ("approach", 0.3),
        Some(FlightPhase::Missed) => ("missed", 1.0),
        Some(_) => ("other", 1.0),
        None => ("unloaded", 2.0),
    };
    let display_rnp_nm = ctx
        .display
        .map(|d| d.rnp_m / METRES_PER_NM)
        .filter(|value| value.is_finite() && *value > 0.0);
    let rnp_limit_nm = ctx
        .configured_rnp_nm
        .or(display_rnp_nm)
        .unwrap_or(default_rnp_nm);
    let rnp_limit_m = rnp_limit_nm * METRES_PER_NM;

    let Some(display) = ctx.display else {
        return GuidanceRnpTelemetry {
            steered_set: ctx.steered_set.to_owned(),
            active_leg_index: None,
            leg_kind: "none".to_owned(),
            lateral_mode: "none".to_owned(),
            vertical_mode: "none".to_owned(),
            flight_phase: flight_phase.to_owned(),
            rnp_limit_nm,
            rnp_limit_m,
            nse_cross_track_m: None,
            nse_horiz_m: ctx.steered_nav.map(|n| n.horizontal_error_m),
            fte_cross_track_m: None,
            tse_cross_track_m: None,
            rnp_95_bound_m: None,
            within_rnp: None,
        };
    };

    let lateral_mode = format!("{:?}", display.lateral.mode);
    let vertical_mode = format!("{:?}", display.vertical.mode);
    let (leg_kind, tse_cross_track_m, fte_cross_track_m) =
        leg_deviations(display, ctx.start_fix, ctx.truth_ecef, ctx.steered_est_ecef);
    let nse_cross_track_m = tse_cross_track_m
        .zip(fte_cross_track_m)
        .map(|(tse, fte)| tse - fte);
    let nse_horiz_m = ctx.steered_nav.map(|n| n.horizontal_error_m);
    let rnp_95_bound_m = ctx
        .steered_nav
        .and_then(|n| n.epu_95_m)
        .zip(fte_cross_track_m)
        .map(|(epu, fte)| epu.hypot(fte));
    let within_rnp = tse_cross_track_m.map(|tse| {
        tse.abs() <= rnp_limit_m && rnp_95_bound_m.is_none_or(|bound| bound <= rnp_limit_m)
    });

    GuidanceRnpTelemetry {
        steered_set: ctx.steered_set.to_owned(),
        active_leg_index: display.active_waypoint,
        leg_kind: leg_kind.to_owned(),
        lateral_mode,
        vertical_mode,
        flight_phase: flight_phase.to_owned(),
        rnp_limit_nm,
        rnp_limit_m,
        nse_cross_track_m,
        nse_horiz_m,
        fte_cross_track_m,
        tse_cross_track_m,
        rnp_95_bound_m,
        within_rnp,
    }
}

fn leg_deviations(
    display: &NavigationDisplay,
    start_fix: GeodeticPosition,
    truth_ecef: DVec3,
    steered_est_ecef: Option<DVec3>,
) -> (&'static str, Option<f64>, Option<f64>) {
    if let Some(hold) = &display.holding_pattern {
        let course = hold.pattern.inbound_course_rad;
        let fix = hold.pattern.fix;
        let tse = Some(tangent_cross_track_m(fix, course, truth_ecef));
        let fte = steered_est_ecef.map(|est| tangent_cross_track_m(fix, course, est));
        return ("holding_pattern", tse, fte);
    }
    let Some(leg) = &display.leg else {
        return ("none", None, None);
    };
    let (kind, from) = match &leg.from {
        Some(from) => ("track_to_fix", from.position),
        None => ("direct_to_fix", start_fix),
    };
    let tse = Some(segment_or_course_xtk_m(from, leg, truth_ecef));
    let fte = steered_est_ecef.map(|est| segment_or_course_xtk_m(from, leg, est));
    (kind, tse, fte)
}

fn segment_or_course_xtk_m(from: GeodeticPosition, leg: &LegDisplay, pos_ecef: DVec3) -> f64 {
    let geo = ecef_to_geodetic(pos_ecef);
    let gp = GeodeticPosition::new(geo.lat_rad, geo.lon_rad, geo.height_m);
    cross_track_m(&gp, &from, &leg.to.position, BodyEllipsoid::WGS84)
        .unwrap_or_else(|_| tangent_cross_track_m(leg.to.position, leg.desired_track_rad, pos_ecef))
}

fn tangent_cross_track_m(fix: GeodeticPosition, course_rad: f64, pos_ecef: DVec3) -> f64 {
    let fix_geo = Geodetic {
        lat_rad: fix.latitude_rad,
        lon_rad: fix.longitude_rad,
        height_m: fix.altitude_m,
    };
    let offset = LocalFrame::at(&fix_geo).ecef_to_ned_vector(pos_ecef - geodetic_to_ecef(&fix_geo));
    let (sin, cos) = course_rad.sin_cos();
    offset.y * cos - offset.x * sin
}
