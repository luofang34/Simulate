//! Converts [`crate::FlyableRoute`] CIFP metadata into [`sim_chain_trial::ChainLegSpec`]
//! entries when a `#v=2` link carries non-default leg paths, altitude windows,
//! forced turn directions, flight-phase roles, or RNP bounds.

use sim_chain_trial::{
    ChainAltitudeSpec, ChainHoldLengthSpec, ChainHoldSpec, ChainLegRole, ChainLegSpec,
    ChainPathSpec, HoldTurn,
};
use sim_link::{SimulationLink, TurnDirection};
use sim_navigate::trial::WaypointSpec;

use crate::request::{ellipsoid_height_m, fix_ident, stated_height_m};
use crate::{
    FlyableAltitude, FlyableFix, FlyableHold, FlyableHoldLength, FlyableLegRole, FlyablePath,
    FlyableRoute, HostError,
};

/// Builds `ChainTrialSpec::planned_legs` when `route` uses any CIFP attribute
/// beyond plain `IF`/`TF` (or initial `DF`) tracks with `At` altitudes; returns
/// an empty vector on plain routes so existing trial digests stay unchanged.
pub(super) fn planned_legs(
    link: &SimulationLink,
    route: &FlyableRoute,
    waypoints: &[WaypointSpec],
) -> Result<Vec<ChainLegSpec>, HostError> {
    if !needs_planned_legs(route, waypoints) {
        return Ok(Vec::new());
    }
    route
        .fixes
        .iter()
        .enumerate()
        .map(|(index, fix)| build_chain_leg(link, index, fix))
        .collect()
}

fn needs_planned_legs(route: &FlyableRoute, waypoints: &[WaypointSpec]) -> bool {
    if route.fixes.len() != waypoints.len() {
        return true;
    }
    route.fixes.iter().enumerate().any(|(index, fix)| {
        let plain_path = matches!(fix.path, FlyablePath::InitialFix | FlyablePath::TrackToFix)
            || (index == 0 && fix.path == FlyablePath::DirectToFix);
        let plain_alt = matches!(fix.altitude, None | Some(FlyableAltitude::At(_)));
        !plain_path
            || !plain_alt
            || fix.turn_direction.is_some()
            || fix.leg_role.is_some()
            || fix.rnp_nm.is_some()
    })
}

fn build_chain_leg(
    link: &SimulationLink,
    index: usize,
    fix: &FlyableFix,
) -> Result<ChainLegSpec, HostError> {
    let ident = fix_ident(fix.name.as_deref(), index);
    let (lat_deg, lon_deg) = (fix.lat.as_f64(), fix.lon.as_f64());
    let to_ellipsoid = |h: f64| ellipsoid_height_m(link.datum, lat_deg, lon_deg, h);
    let altitude = match fix.altitude {
        Some(FlyableAltitude::At(h)) => ChainAltitudeSpec::At {
            altitude_m: to_ellipsoid(h),
        },
        Some(FlyableAltitude::AtOrAbove(h)) => ChainAltitudeSpec::AtOrAbove {
            altitude_m: to_ellipsoid(h),
        },
        Some(FlyableAltitude::AtOrBelow(h)) => ChainAltitudeSpec::AtOrBelow {
            altitude_m: to_ellipsoid(h),
        },
        Some(FlyableAltitude::Window { lower_m, upper_m }) => ChainAltitudeSpec::Window {
            lower_m: to_ellipsoid(lower_m),
            upper_m: to_ellipsoid(upper_m),
        },
        None => {
            let stated = stated_height_m(link, None).ok_or_else(|| HostError::MissingHeight {
                what: ident.clone(),
            })?;
            ChainAltitudeSpec::At {
                altitude_m: to_ellipsoid(stated),
            }
        }
    };
    Ok(ChainLegSpec {
        ident,
        lat_deg,
        lon_deg,
        altitude,
        fly_over: fix.fly_over,
        turn_direction: fix.turn_direction.map(hold_turn),
        path: chain_path(fix.path),
        role: fix.leg_role.map(chain_role),
        rnp_nm: fix.rnp_nm,
    })
}

const fn hold_turn(turn: TurnDirection) -> HoldTurn {
    match turn {
        TurnDirection::Left => HoldTurn::Left,
        TurnDirection::Right => HoldTurn::Right,
    }
}

fn chain_hold(hold: FlyableHold) -> ChainHoldSpec {
    ChainHoldSpec {
        inbound_course_deg: hold.inbound_course_deg,
        magnetic: hold.magnetic,
        turn: hold_turn(hold.turn),
        length: match hold.length {
            FlyableHoldLength::Minutes(minutes) => ChainHoldLengthSpec::Minutes { minutes },
            FlyableHoldLength::DistanceM(meters) => ChainHoldLengthSpec::DistanceM { meters },
        },
    }
}

fn chain_path(path: FlyablePath) -> ChainPathSpec {
    match path {
        FlyablePath::InitialFix => ChainPathSpec::InitialFix,
        FlyablePath::TrackToFix => ChainPathSpec::TrackToFix,
        FlyablePath::DirectToFix => ChainPathSpec::DirectToFix,
        FlyablePath::CourseToFix {
            course_deg,
            magnetic,
        } => ChainPathSpec::CourseToFix {
            course_deg,
            magnetic,
        },
        FlyablePath::CourseToAltitude {
            course_deg,
            magnetic,
        } => ChainPathSpec::CourseToAltitude {
            course_deg,
            magnetic,
        },
        FlyablePath::HoldToAltitude(hold) => ChainPathSpec::HoldToAltitude {
            hold: chain_hold(hold),
        },
        FlyablePath::HoldToFix(hold) => ChainPathSpec::HoldToFix {
            hold: chain_hold(hold),
        },
        FlyablePath::HoldManual(hold) => ChainPathSpec::HoldManual {
            hold: chain_hold(hold),
        },
    }
}

const fn chain_role(role: FlyableLegRole) -> ChainLegRole {
    match role {
        FlyableLegRole::Departure => ChainLegRole::Departure,
        FlyableLegRole::Enroute => ChainLegRole::Enroute,
        FlyableLegRole::Arrival => ChainLegRole::Arrival,
        FlyableLegRole::Approach => ChainLegRole::Approach,
        FlyableLegRole::MissedApproach => ChainLegRole::MissedApproach,
    }
}
