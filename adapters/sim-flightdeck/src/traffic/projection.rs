//! Display target and HUD contact projection from [`TrackSnapshot`] records.

use super::{METRES_PER_NM, TrafficDisplayTarget, TrafficThreat};
use crate::telemetry::{FEET_TO_METRES, FPM_TO_MPS, KNOTS_TO_MPS};
use indicate_instrument_hud::{Contact, Domain, Identity};
use indicate_instrument_state::SignalStatus;
use world::surveillance::{TrackPhase, TrackSnapshot};
use world::traffic::RelativeTraffic;
use world_core::Ned;

pub(super) fn build_display_target(
    track: &TrackSnapshot,
    rel: &RelativeTraffic,
) -> TrafficDisplayTarget {
    let address_hex = format!("{:06X}", track.key.address & 0x00FF_FFFF);
    let callsign = track
        .callsign
        .as_ref()
        .map_or_else(|| address_hex.clone(), |cs| cs.value.text.clone());
    let up_m = rel.relative_altitude.map_or(0.0, |a| a.above_ownship_m);
    let slant_range_m = libm::hypot(rel.horizontal_range_m, up_m);
    let slant_range_nm = (slant_range_m / METRES_PER_NM) as f32;
    let rel_alt_ft = up_m / FEET_TO_METRES;
    let rel_altitude_hundreds_ft = (rel_alt_ft / 100.0).round() as i32;
    let vel_ned = snapshot_velocity_ned(track);
    let climb_fpm = (-vel_ned.down_m) / FPM_TO_MPS;
    let vertical_trend = if climb_fpm >= 300.0 {
        1
    } else if climb_fpm <= -300.0 {
        -1
    } else {
        0
    };
    let trend_arrow = match vertical_trend {
        1 => "↑",
        -1 => "↓",
        _ => "",
    };
    let placard = format!(
        "{callsign} {:+03}{trend_arrow} {:.1}NM",
        rel_altitude_hundreds_ft, slant_range_nm
    );
    let threat = classify_threat(rel);
    let (time_to_cpa_s, cpa_horizontal_m) = rel.cpa.map_or((0.0, rel.horizontal_range_m), |cpa| {
        (cpa.time_to_cpa_s, cpa.horizontal_miss_m)
    });
    let on_ground = track.air_ground.and_then(|ag| ag.value.is_airborne()) == Some(false);
    TrafficDisplayTarget {
        track_id: track.id.get(),
        address_hex,
        callsign,
        placard,
        offset_enu_m: [rel.east_m as f32, rel.north_m as f32, up_m as f32],
        velocity_enu_mps: [
            vel_ned.east_m as f32,
            vel_ned.north_m as f32,
            (-vel_ned.down_m) as f32,
        ],
        track_true_deg: track
            .velocity
            .and_then(|v| v.value.track_angle_deg_true.or(v.value.heading_deg))
            .map(|deg| deg.rem_euclid(360.0) as f32),
        slant_range_nm,
        rel_altitude_hundreds_ft,
        vertical_trend,
        time_to_cpa_s: time_to_cpa_s as f32,
        cpa_horizontal_m: cpa_horizontal_m as f32,
        threat,
        on_ground,
        active: track.phase == TrackPhase::Active,
    }
}

pub(super) fn to_hud_contact(
    track: &TrackSnapshot,
    rel: &RelativeTraffic,
    own_vel_ned: &Ned,
) -> Contact {
    let threat = classify_threat(rel);
    let vel_ned = snapshot_velocity_ned(track);
    let up_m = rel.relative_altitude.map_or(0.0, |a| a.above_ownship_m);
    let on_ground = track.air_ground.and_then(|ag| ag.value.is_airborne()) == Some(false);
    Contact {
        relative_position_m: [rel.north_m as f32, rel.east_m as f32, (-up_m) as f32],
        relative_velocity_mps: [
            (vel_ned.north_m - own_vel_ned.north_m) as f32,
            (vel_ned.east_m - own_vel_ned.east_m) as f32,
            (vel_ned.down_m - own_vel_ned.down_m) as f32,
        ],
        identity: if threat >= TrafficThreat::TrafficAdvisory {
            Identity::Hostile
        } else {
            Identity::Friendly
        },
        domain: if on_ground {
            Domain::Surface
        } else {
            Domain::Air
        },
        status: match track.phase {
            TrackPhase::Active => SignalStatus::Valid,
            _ => SignalStatus::Stale,
        },
    }
}

fn snapshot_velocity_ned(track: &TrackSnapshot) -> Ned {
    let Some(vel) = track.velocity.as_ref().map(|v| v.value) else {
        return Ned {
            north_m: 0.0,
            east_m: 0.0,
            down_m: 0.0,
        };
    };
    let spd_mps = vel.ground_speed_kt.unwrap_or(0.0) * KNOTS_TO_MPS;
    let angle_deg = vel.track_angle_deg_true.or(vel.heading_deg);
    let (vn, ve) = angle_deg.map_or((0.0, 0.0), |deg| {
        let rad = deg.to_radians();
        (spd_mps * libm::cos(rad), spd_mps * libm::sin(rad))
    });
    let vd = -vel
        .vertical_rate_fpm
        .map_or(0.0, |f| f64::from(f) * FPM_TO_MPS);
    Ned {
        north_m: vn,
        east_m: ve,
        down_m: vd,
    }
}

fn classify_threat(rel: &RelativeTraffic) -> TrafficThreat {
    let horiz_nm = rel.horizontal_range_m / METRES_PER_NM;
    let abs_alt_ft = rel
        .relative_altitude
        .map_or(0.0, |a| (a.above_ownship_m / FEET_TO_METRES).abs());
    let (time_to_cpa_s, cpa_nm) = rel.cpa.map_or((0.0, horiz_nm), |cpa| {
        (cpa.time_to_cpa_s, cpa.horizontal_miss_m / METRES_PER_NM)
    });

    if (horiz_nm <= 0.45 && abs_alt_ft <= 600.0)
        || (time_to_cpa_s > 0.0 && time_to_cpa_s <= 25.0 && cpa_nm <= 0.45 && abs_alt_ft <= 600.0)
    {
        TrafficThreat::ResolutionAdvisory
    } else if (horiz_nm <= 0.80 && abs_alt_ft <= 850.0)
        || (time_to_cpa_s > 0.0 && time_to_cpa_s <= 45.0 && cpa_nm <= 1.20 && abs_alt_ft <= 850.0)
    {
        TrafficThreat::TrafficAdvisory
    } else if horiz_nm <= 6.0 && abs_alt_ft <= 1_200.0 {
        TrafficThreat::Proximate
    } else {
        TrafficThreat::Other
    }
}
