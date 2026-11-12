//! Enroute great-circle LNAV, VNAV target height, and racetrack hold target construction
//! for [`super::EfbAutopilot`].

use std::f64::consts::PI;

use glam::DVec3;
use sim_core::VehicleState;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_maplibre::RouteProgress;

use super::{AeroState, GuidanceTarget};
use crate::efb_plan::navlog::{active_leg_of, distance_and_bearing_nm};
use crate::efb_plan::tokens::{HoldTurn, RouteToken};

const METRES_PER_NM: f64 = 1_852.0;
const METRES_PER_FOOT: f64 = 0.3048;

pub(super) fn build_enroute_target(
    aero: &AeroState,
    tokens: &[RouteToken],
    progress: RouteProgress,
    geoid_undulation_m: f64,
    hold_elapsed_s: &mut f64,
    dt_s: f64,
) -> Option<GuidanceTarget> {
    let active = active_leg_of(progress, tokens.len());
    let from = tokens.get(active)?;
    let to = tokens.get(active.wrapping_add(1))?;
    let target_ellip_m = to.altitude_ft.map_or(aero.geo.height_m, |ft| {
        f64::from(ft) * METRES_PER_FOOT + geoid_undulation_m
    });
    let (_, inbound_deg) =
        distance_and_bearing_nm([from.lat_deg, from.lon_deg], [to.lat_deg, to.lon_deg]);
    let to_geo = Geodetic::from_degrees(to.lat_deg, to.lon_deg, target_ellip_m);
    if progress.hold.is_some()
        && let Some(hold) = to.hold
    {
        return Some(build_hold_target(
            hold,
            inbound_deg,
            to_geo,
            target_ellip_m,
            hold_elapsed_s,
            dt_s,
        ));
    }
    *hold_elapsed_s = 0.0;
    let from_geo = Geodetic::from_degrees(from.lat_deg, from.lon_deg, target_ellip_m);
    let (dist_nm, direct_deg) = distance_and_bearing_nm(
        [aero.geo.lat_deg(), aero.geo.lon_deg()],
        [to.lat_deg, to.lon_deg],
    );
    let use_direct = dist_nm > 12.0;
    Some(GuidanceTarget {
        line_origin_ecef: if use_direct {
            geodetic_to_ecef(&to_geo)
        } else {
            geodetic_to_ecef(&from_geo)
        },
        line_frame: if use_direct {
            LocalFrame::at(&to_geo)
        } else {
            LocalFrame::at(&from_geo)
        },
        line_bearing_rad: if use_direct {
            direct_deg.to_radians()
        } else {
            inbound_deg.to_radians()
        },
        lookahead_m: (dist_nm * METRES_PER_NM * 0.25).clamp(260.0, 1_200.0),
        target_ellip_m,
        height_gain: 0.20,
        target_speed_m_s: 52.0,
        flaps_norm: 0.0,
    })
}

fn build_hold_target(
    hold: crate::efb_plan::tokens::HoldSpec,
    inbound_deg: f64,
    to_geo: Geodetic,
    target_ellip_m: f64,
    hold_elapsed_s: &mut f64,
    dt_s: f64,
) -> GuidanceTarget {
    *hold_elapsed_s += dt_s;
    let inbound_rad = hold.inbound_course_deg.unwrap_or(inbound_deg).to_radians();
    let cycle_s = (2.0 * hold.leg_seconds).max(60.0_f64);
    let outbound = (*hold_elapsed_s % cycle_s) < hold.leg_seconds;
    let turn_sign = if hold.turn == HoldTurn::Right {
        1.0
    } else {
        -1.0
    };
    let frame = LocalFrame::at(&to_geo);
    let (sin_i, cos_i) = inbound_rad.sin_cos();
    let right_vec = frame.north() * (-sin_i) + frame.east() * cos_i;
    let origin = if outbound {
        geodetic_to_ecef(&to_geo) + right_vec * (turn_sign * 950.0)
    } else {
        geodetic_to_ecef(&to_geo)
    };
    GuidanceTarget {
        line_origin_ecef: origin,
        line_frame: frame,
        line_bearing_rad: if outbound {
            inbound_rad + PI
        } else {
            inbound_rad
        },
        lookahead_m: 320.0,
        target_ellip_m,
        height_gain: 0.20,
        target_speed_m_s: 46.0,
        flaps_norm: 0.0,
    }
}

pub(super) fn observe_aero(state: &VehicleState) -> AeroState {
    let geo = ecef_to_geodetic(state.position_ecef);
    let local = LocalFrame::at(&geo);
    let (north, east, up) = (local.north(), local.east(), local.up());
    let vel = state.velocity_ecef;
    let speed_m_s = vel.length();
    let nose = state.attitude * DVec3::X;
    let right_axis = state.attitude * DVec3::Y;
    let heading_rad = nose.dot(east).atan2(nose.dot(north));
    let track_rad = if speed_m_s > 2.0 {
        vel.dot(east).atan2(vel.dot(north))
    } else {
        heading_rad
    };
    let up_speed_m_s = vel.dot(up);
    AeroState {
        geo,
        speed_m_s,
        track_rad,
        heading_rad,
        flight_path_rad: (up_speed_m_s / speed_m_s.max(1.0)).clamp(-1.0, 1.0).asin(),
        bank_rad: (-right_axis.dot(up)).atan2(-state.down_ecef().dot(up)),
        pitch_rad: state.forward_ecef().dot(up).clamp(-1.0, 1.0).asin(),
        body_rate: state.angular_rate_body,
    }
}
