//! Causal light-time solver, exact range-rate chain rule, topocentric ENU angles, and occultation checks.

use core::f64::consts::TAU;

use glam::{DMat3, DVec3};
use sim_geo::LocalFrame;
use sim_spacetime::{
    BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, FrameAxesSpec,
    FramedStateTransformExt, NaifId, SPEED_OF_LIGHT_M_S, TICKS_PER_SECOND, TdbEpoch,
    TranslationalTrajectorySource, surface_station_state_icrf,
};

use super::types::{
    GroundTrackerError, GroundTrackerStationSpec, TrackingLinkMode, TrackingTruthMeasurement,
};

const MAX_LIGHT_TIME_ITERATIONS: usize = 32;
const CONVERGENCE_TOL_S: f64 = 1.0e-14;
const DEGENERATE_RANGE_M: f64 = 1.0e-12;
const HOST_CHORD_MARGIN: f64 = 1.0e-4;

struct StationSsbState {
    pos_ssb_m: DVec3,
    vel_ssb_m_s: DVec3,
    local_enu: LocalFrame,
    r_icrf_to_fixed: DMat3,
}

struct DownlinkLegSolution {
    emit_epoch: TdbEpoch,
    tau_s: f64,
    range_m: f64,
    range_rate_m_s: f64,
    dt_emit_d_rx: f64,
    sat_pos_ssb_m: DVec3,
    sat_vel_ssb_m_s: DVec3,
}

#[inline]
fn vec_norm(v: DVec3) -> f64 {
    libm::sqrt(v.length_squared())
}

/// Shifts `base` by `delta_s` at picosecond tick resolution (`TICKS_PER_SECOND`),
/// returning `(nearest_tick_epoch, sub_tick_remainder_s)` so `base + delta_s == epoch + sub_tick`.
pub(super) fn shift_epoch_seconds(
    base: TdbEpoch,
    delta_s: f64,
) -> Result<(TdbEpoch, f64), GroundTrackerError> {
    if !delta_s.is_finite() || libm::fabs(delta_s) > 1.0e15 {
        return Err(GroundTrackerError::LightTimeDidNotConverge);
    }
    let whole_s = libm::floor(delta_s);
    let frac_s = (delta_s - whole_s).clamp(0.0, 1.0);
    let tps_f64 = TICKS_PER_SECOND as f64;
    let delta_ticks = libm::round(frac_s * tps_f64) as i128;
    let sub_tick_s = frac_s - (delta_ticks as f64) / tps_f64;

    let tps = TICKS_PER_SECOND as i128;
    let total_sub_ticks = (base.subsec_ticks() as i128) + delta_ticks;
    let carry = total_sub_ticks.div_euclid(tps);
    let rem_ticks = total_sub_ticks.rem_euclid(tps) as u64;
    let secs_i128 = (base.j2000_seconds() as i128) + (whole_s as i128) + carry;
    let secs = i64::try_from(secs_i128).map_err(|_| GroundTrackerError::LightTimeDidNotConverge)?;
    Ok((TdbEpoch::new(secs, rem_ticks), sub_tick_s))
}

fn station_state_ssb(
    station: &GroundTrackerStationSpec,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<StationSsbState, GroundTrackerError> {
    let (rel_pos, rel_vel, local_enu) = surface_station_state_icrf(
        station.body,
        station.ellipsoid,
        &station.geodetic,
        epoch,
        orientation,
    )?;
    let rot = orientation.body_rotation(station.body, epoch)?;
    let (body_pos_ssb, body_vel_ssb) = if station.body == NaifId::SSB {
        (DVec3::ZERO, DVec3::ZERO)
    } else {
        let host = ephemeris.geometric_state_icrf(
            station.body,
            NaifId::SSB,
            epoch,
            DerivativeOrder::Velocity,
        )?;
        (host.pos_m, host.vel_m_s)
    };
    Ok(StationSsbState {
        pos_ssb_m: body_pos_ssb + rel_pos,
        vel_ssb_m_s: body_vel_ssb + rel_vel,
        local_enu,
        r_icrf_to_fixed: rot.r_icrf_to_fixed,
    })
}

fn target_state_ssb(
    target: &dyn TranslationalTrajectorySource,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<(DVec3, DVec3), GroundTrackerError> {
    let framed = target.sample_translational_icrf(epoch)?;
    let ssb = framed.transform_to(
        NaifId::SSB,
        FrameAxesSpec::IcrfInertial,
        ephemeris,
        orientation,
    )?;
    Ok((ssb.pos, ssb.vel))
}

fn solve_downlink_leg(
    target: &dyn TranslationalTrajectorySource,
    rx_epoch: TdbEpoch,
    gs_rx: &StationSsbState,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<DownlinkLegSolution, GroundTrackerError> {
    let (r_sat_0, _) = target_state_ssb(target, rx_epoch, ephemeris, orientation)?;
    let mut tau_s = vec_norm(r_sat_0 - gs_rx.pos_ssb_m) / SPEED_OF_LIGHT_M_S;

    for _ in 0..MAX_LIGHT_TIME_ITERATIONS {
        let (emit_epoch_tick, dt_sub_s) = shift_epoch_seconds(rx_epoch, -tau_s)?;
        let (r_sat_tick, v_sat) =
            target_state_ssb(target, emit_epoch_tick, ephemeris, orientation)?;
        let r_sat = r_sat_tick + v_sat * dt_sub_s;
        let los = r_sat - gs_rx.pos_ssb_m;
        let range_m = vec_norm(los);
        let tau_new_s = range_m / SPEED_OF_LIGHT_M_S;
        let tol_s = CONVERGENCE_TOL_S.max(8.0 * f64::EPSILON * tau_new_s);
        if libm::fabs(tau_new_s - tau_s) < tol_s {
            let (emit_epoch, _) = shift_epoch_seconds(rx_epoch, -tau_new_s)?;
            let (range_rate_m_s, dt_emit_d_rx) =
                downlink_range_rate(los, range_m, v_sat, gs_rx.vel_ssb_m_s)?;
            return Ok(DownlinkLegSolution {
                emit_epoch,
                tau_s: tau_new_s,
                range_m,
                range_rate_m_s,
                dt_emit_d_rx,
                sat_pos_ssb_m: r_sat,
                sat_vel_ssb_m_s: v_sat,
            });
        }
        tau_s = tau_new_s;
    }
    Err(GroundTrackerError::LightTimeDidNotConverge)
}

fn downlink_range_rate(
    los_gs_to_sat: DVec3,
    range_m: f64,
    v_sat: DVec3,
    v_gs_rx: DVec3,
) -> Result<(f64, f64), GroundTrackerError> {
    if range_m <= DEGENERATE_RANGE_M {
        return Ok((0.0, 1.0));
    }
    let u = los_gs_to_sat / range_m;
    let denom = 1.0 + u.dot(v_sat) / SPEED_OF_LIGHT_M_S;
    if !denom.is_finite() || denom <= 0.0 {
        return Err(GroundTrackerError::LightTimeDidNotConverge);
    }
    let r_dot = u.dot(v_sat - v_gs_rx) / denom;
    Ok((r_dot, 1.0 - r_dot / SPEED_OF_LIGHT_M_S))
}

fn solve_uplink_leg(
    station: &GroundTrackerStationSpec,
    rx_epoch: TdbEpoch,
    down: &DownlinkLegSolution,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<(TdbEpoch, f64, f64, f64, DVec3), GroundTrackerError> {
    let mut tau_up_s = down.tau_s;
    for _ in 0..MAX_LIGHT_TIME_ITERATIONS {
        let total_lt_s = down.tau_s + tau_up_s;
        let (tx_epoch_tick, dt_sub_s) = shift_epoch_seconds(rx_epoch, -total_lt_s)?;
        let gs_tx = station_state_ssb(station, tx_epoch_tick, ephemeris, orientation)?;
        let r_gs_tx = gs_tx.pos_ssb_m + gs_tx.vel_ssb_m_s * dt_sub_s;
        let los_up = down.sat_pos_ssb_m - r_gs_tx;
        let r_up_m = vec_norm(los_up);
        let tau_up_new_s = r_up_m / SPEED_OF_LIGHT_M_S;
        let tol_s = CONVERGENCE_TOL_S.max(8.0 * f64::EPSILON * (down.tau_s + tau_up_new_s));
        if libm::fabs(tau_up_new_s - tau_up_s) < tol_s {
            let (tx_epoch, _) = shift_epoch_seconds(rx_epoch, -(down.tau_s + tau_up_new_s))?;
            let r_up_dot_bounce =
                uplink_range_rate_bounce(los_up, r_up_m, down.sat_vel_ssb_m_s, gs_tx.vel_ssb_m_s)?;
            return Ok((tx_epoch, tau_up_new_s, r_up_m, r_up_dot_bounce, r_gs_tx));
        }
        tau_up_s = tau_up_new_s;
    }
    Err(GroundTrackerError::LightTimeDidNotConverge)
}

fn uplink_range_rate_bounce(
    los_gs_to_sat: DVec3,
    range_m: f64,
    v_sat_bounce: DVec3,
    v_gs_tx: DVec3,
) -> Result<f64, GroundTrackerError> {
    if range_m <= DEGENERATE_RANGE_M {
        return Ok(0.0);
    }
    let u_up = los_gs_to_sat / range_m;
    let denom = 1.0 - u_up.dot(v_gs_tx) / SPEED_OF_LIGHT_M_S;
    if !denom.is_finite() || denom <= 0.0 {
        return Err(GroundTrackerError::LightTimeDidNotConverge);
    }
    Ok(u_up.dot(v_sat_bounce - v_gs_tx) / denom)
}

pub(super) fn evaluate_truth(
    station: &GroundTrackerStationSpec,
    target: &dyn TranslationalTrajectorySource,
    rx_epoch: TdbEpoch,
    mode: TrackingLinkMode,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    enforce_mask: bool,
) -> Result<TrackingTruthMeasurement, GroundTrackerError> {
    let gs_rx = station_state_ssb(station, rx_epoch, ephemeris, orientation)?;
    let down = solve_downlink_leg(target, rx_epoch, &gs_rx, ephemeris, orientation)?;

    let los_enu = gs_rx
        .local_enu
        .ecef_to_enu_vector(gs_rx.r_icrf_to_fixed * (down.sat_pos_ssb_m - gs_rx.pos_ssb_m));
    let (azimuth_rad, elevation_rad) = enu_to_az_el(los_enu);

    if enforce_mask && elevation_rad < station.min_elevation_rad {
        return Err(GroundTrackerError::BelowElevationMask {
            elevation_rad,
            min_elevation_rad: station.min_elevation_rad,
        });
    }

    match mode {
        TrackingLinkMode::OneWayDownlink => build_one_way_truth(
            station,
            rx_epoch,
            &gs_rx,
            &down,
            azimuth_rad,
            elevation_rad,
            ephemeris,
        ),
        TrackingLinkMode::TwoWayCoherent => build_two_way_truth(
            station,
            rx_epoch,
            &gs_rx,
            &down,
            azimuth_rad,
            elevation_rad,
            ephemeris,
            orientation,
        ),
    }
}

fn build_one_way_truth(
    station: &GroundTrackerStationSpec,
    rx_epoch: TdbEpoch,
    gs_rx: &StationSsbState,
    down: &DownlinkLegSolution,
    azimuth_rad: f64,
    elevation_rad: f64,
    ephemeris: &dyn EphemerisCatalog,
) -> Result<TrackingTruthMeasurement, GroundTrackerError> {
    check_leg_occultation(
        station,
        down.emit_epoch,
        rx_epoch,
        down.sat_pos_ssb_m,
        gs_rx.pos_ssb_m,
        false,
        true,
        ephemeris,
    )?;
    let doppler_hz = doppler_from_range_rate(
        station.carrier_frequency_hz,
        down.range_rate_m_s,
        TrackingLinkMode::OneWayDownlink,
    );
    Ok(TrackingTruthMeasurement {
        rx_epoch,
        tx_epoch: down.emit_epoch,
        bounce_epoch: None,
        light_time_s: down.tau_s,
        range_m: down.range_m,
        range_rate_m_s: down.range_rate_m_s,
        doppler_hz,
        azimuth_rad,
        elevation_rad,
    })
}

#[allow(clippy::too_many_arguments)]
fn build_two_way_truth(
    station: &GroundTrackerStationSpec,
    rx_epoch: TdbEpoch,
    gs_rx: &StationSsbState,
    down: &DownlinkLegSolution,
    azimuth_rad: f64,
    elevation_rad: f64,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<TrackingTruthMeasurement, GroundTrackerError> {
    let (tx_epoch, tau_up_s, r_up_m, r_up_dot_bounce, r_gs_tx) =
        solve_uplink_leg(station, rx_epoch, down, ephemeris, orientation)?;
    check_leg_occultation(
        station,
        tx_epoch,
        down.emit_epoch,
        r_gs_tx,
        down.sat_pos_ssb_m,
        true,
        false,
        ephemeris,
    )?;
    check_leg_occultation(
        station,
        down.emit_epoch,
        rx_epoch,
        down.sat_pos_ssb_m,
        gs_rx.pos_ssb_m,
        false,
        true,
        ephemeris,
    )?;
    let light_time_s = down.tau_s + tau_up_s;
    let range_m = 0.5 * (down.range_m + r_up_m);
    let range_rate_m_s = 0.5 * (down.range_rate_m_s + r_up_dot_bounce * down.dt_emit_d_rx);
    let doppler_hz = doppler_from_range_rate(
        station.carrier_frequency_hz,
        range_rate_m_s,
        TrackingLinkMode::TwoWayCoherent,
    );
    Ok(TrackingTruthMeasurement {
        rx_epoch,
        tx_epoch,
        bounce_epoch: Some(down.emit_epoch),
        light_time_s,
        range_m,
        range_rate_m_s,
        doppler_hz,
        azimuth_rad,
        elevation_rad,
    })
}

#[allow(clippy::too_many_arguments)]
fn check_leg_occultation(
    station: &GroundTrackerStationSpec,
    emit_epoch: TdbEpoch,
    rx_epoch: TdbEpoch,
    emit_pos_ssb_m: DVec3,
    rx_pos_ssb_m: DVec3,
    emitter_is_station: bool,
    receiver_is_station: bool,
    ephemeris: &dyn EphemerisCatalog,
) -> Result<(), GroundTrackerError> {
    if station.occulting_bodies.is_empty() {
        return Ok(());
    }
    let light_time_s = rx_epoch.signed_offset_seconds_from(emit_epoch);
    let chord = rx_pos_ssb_m - emit_pos_ssb_m;
    let chord_sq = chord.length_squared();

    for occ in &station.occulting_bodies {
        let body_at = |epoch| -> Result<DVec3, GroundTrackerError> {
            if occ.body == NaifId::SSB {
                Ok(DVec3::ZERO)
            } else {
                Ok(ephemeris
                    .geometric_state_icrf(occ.body, NaifId::SSB, epoch, DerivativeOrder::Position)?
                    .pos_m)
            }
        };
        let mut body_ssb = body_at(rx_epoch)?;
        for _ in 0..2 {
            let frac = if chord_sq > DEGENERATE_RANGE_M * DEGENERATE_RANGE_M {
                ((body_ssb - emit_pos_ssb_m).dot(chord) / chord_sq).clamp(0.0, 1.0)
            } else {
                1.0
            };
            let (pass_epoch, _) = shift_epoch_seconds(emit_epoch, frac * light_time_s)?;
            body_ssb = body_at(pass_epoch)?;
        }

        let p0 = emit_pos_ssb_m - body_ssb;
        let emitter_host = emitter_is_station && occ.body == station.body;
        let receiver_host = receiver_is_station && occ.body == station.body;
        if sphere_chord_occulted(
            p0,
            chord,
            chord_sq,
            occ.radius_m,
            emitter_host,
            receiver_host,
        ) {
            return Err(GroundTrackerError::BodyOcculted { body: occ.body });
        }
    }
    Ok(())
}

fn sphere_chord_occulted(
    p0: DVec3,
    chord: DVec3,
    chord_sq: f64,
    radius_m: f64,
    emitter_host: bool,
    receiver_host: bool,
) -> bool {
    if chord_sq <= DEGENERATE_RANGE_M * DEGENERATE_RANGE_M {
        return !emitter_host && !receiver_host && vec_norm(p0) < radius_m;
    }
    let nearest = -p0.dot(chord) / chord_sq;
    if emitter_host && nearest <= HOST_CHORD_MARGIN {
        return false;
    }
    if receiver_host && nearest >= 1.0 - HOST_CHORD_MARGIN {
        return false;
    }
    let s = nearest.clamp(0.0, 1.0);
    vec_norm(p0 + s * chord) < radius_m
}

/// Converts a topocentric ENU line-of-sight vector into `(azimuth_rad, elevation_rad)`.
pub(super) fn enu_to_az_el(rho_enu: DVec3) -> (f64, f64) {
    let r = vec_norm(rho_enu);
    if r <= DEGENERATE_RANGE_M {
        return (0.0, 0.0);
    }
    let horiz = libm::hypot(rho_enu.x, rho_enu.y);
    let el = libm::atan2(rho_enu.z, horiz);
    let az = wrap_azimuth_rad(libm::atan2(rho_enu.x, rho_enu.y));
    (az, el)
}

/// Wraps an azimuth angle in radians onto `[0, 2*pi)`.
pub(super) fn wrap_azimuth_rad(az_rad: f64) -> f64 {
    let rem = libm::fmod(az_rad, TAU);
    let pos = if rem < 0.0 { rem + TAU } else { rem };
    if pos >= TAU || pos == -0.0 { 0.0 } else { pos }
}

/// Converts range-rate (`m/s`) to receiver Doppler shift (`Hz`) according to `mode`.
pub(super) fn doppler_from_range_rate(
    carrier_hz: f64,
    range_rate_m_s: f64,
    mode: TrackingLinkMode,
) -> f64 {
    if carrier_hz == 0.0 || range_rate_m_s == 0.0 {
        return 0.0;
    }
    let factor = match mode {
        TrackingLinkMode::OneWayDownlink => 1.0,
        TrackingLinkMode::TwoWayCoherent => 2.0,
    };
    -factor * carrier_hz * range_rate_m_s / SPEED_OF_LIGHT_M_S
}
