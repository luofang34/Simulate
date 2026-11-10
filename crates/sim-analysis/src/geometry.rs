//! Reusable instantaneous link geometry (`LinkGeometrySample`), line-of-sight
//! planetary occultation evaluation, and electromagnetic medium query contracts (`PropagationMediumQuery`).

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use sim_spacetime::{
    AttitudeSample, BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, NaifId, TdbEpoch,
    asin_clamped, atan2, serde_dvec3, sqrt, surface_station_state_icrf,
};

use crate::models::{
    GroundStationModel, OccultationEnvironmentModel, OccultingShape, SensorFovModel,
    VehicleSensorModel,
};

/// Instantaneous geometric relation between a [`GroundStationModel`] and a [`VehicleSensorModel`].
///
/// Designed to be directly consumed by both geometric access window calculators and
/// future RF/optical electromagnetic link-budget, Doppler, and polarization analyzers.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkGeometrySample {
    /// Sample epoch (`TDB`).
    pub epoch: TdbEpoch,
    /// Ground station antenna phase-center position in `SSB ICRF` (`m`).
    #[serde(with = "serde_dvec3")]
    pub station_pos_icrf_m: DVec3,
    /// Ground station antenna phase-center velocity in `SSB ICRF` (`m/s`).
    #[serde(with = "serde_dvec3")]
    pub station_vel_icrf_m_s: DVec3,
    /// Vehicle sensor phase-center position in `SSB ICRF` (`m`, including lever-arm offset).
    #[serde(with = "serde_dvec3")]
    pub vehicle_sensor_pos_icrf_m: DVec3,
    /// Vehicle sensor phase-center velocity in `SSB ICRF` (`m/s`, including $\boldsymbol{\omega}\times\mathbf{r}$ transport).
    #[serde(with = "serde_dvec3")]
    pub vehicle_sensor_vel_icrf_m_s: DVec3,
    /// Euclidean slant range $\|\mathbf{r}_{\text{veh}} - \mathbf{r}_{\text{sta}}\|$ (`m`).
    pub slant_range_m: f64,
    /// Line-of-sight range rate $\dot{\rho} = \hat{\boldsymbol{\rho}} \cdot (\mathbf{v}_{\text{veh}} - \mathbf{v}_{\text{sta}})$ (`m/s`).
    pub range_rate_m_s: f64,
    /// Topocentric horizon elevation angle at the ground station (`rad`, $[-\pi/2, \pi/2]$).
    pub elevation_rad: f64,
    /// Topocentric azimuth angle at the ground station (`rad`, $[0, 2\pi)$ clockwise from North).
    pub azimuth_rad: f64,
    /// Off-boresight angle from the station mount `+Z_mount` axis to the vehicle (`rad`, $[0, \pi]$).
    pub station_off_boresight_rad: f64,
    /// Off-boresight angle from the vehicle sensor `+Z_mount` axis to the station (`rad`, `Some` when attitude is available).
    pub vehicle_off_boresight_rad: Option<f64>,
    /// Line-of-sight vector from station to vehicle expressed in station mount coordinates (`m`).
    #[serde(with = "serde_dvec3")]
    pub los_in_station_mount_m: DVec3,
    /// Line-of-sight vector from vehicle to station expressed in vehicle sensor mount coordinates (`m`).
    pub los_in_vehicle_mount_m: Option<[f64; 3]>,
    /// Orthogonal $3\times 3$ rotation matrix $R_{\text{veh,mount}\to\text{sta,mount}}$ mapping a vector
    /// from vehicle sensor mount axes into ground station mount axes (used for antenna polarization alignment).
    pub r_vehicle_mount_to_station_mount: Option<[[f64; 3]; 3]>,
}

/// Evaluates [`LinkGeometrySample`] at `epoch` given the vehicle CG state in `SSB ICRF` and optional [`AttitudeSample`].
#[allow(clippy::too_many_arguments)]
pub fn compute_link_geometry_sample(
    epoch: TdbEpoch,
    vehicle_cg_pos_ssb_m: DVec3,
    vehicle_cg_vel_ssb_m_s: DVec3,
    vehicle_attitude: Option<&AttitudeSample>,
    station: &GroundStationModel,
    sensor: &VehicleSensorModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<LinkGeometrySample, LinkGeometryEvalError> {
    let (station_pos_icrf_m, station_vel_icrf_m_s, r_enu_to_icrf, r_icrf_to_sta_mount) =
        evaluate_station_mount_kinematics(epoch, station, ephemeris, orientation)?;

    let (
        vehicle_sensor_pos_icrf_m,
        vehicle_sensor_vel_icrf_m_s,
        veh_off_boresight,
        los_veh_mount,
        r_v2s,
    ) = evaluate_vehicle_mount_kinematics(
        vehicle_cg_pos_ssb_m,
        vehicle_cg_vel_ssb_m_s,
        vehicle_attitude,
        sensor,
        station_pos_icrf_m,
        r_icrf_to_sta_mount,
    );

    let rho_icrf = vehicle_sensor_pos_icrf_m - station_pos_icrf_m;
    let drho_icrf = vehicle_sensor_vel_icrf_m_s - station_vel_icrf_m_s;
    let slant_range_m = sqrt(rho_icrf.length_squared());
    let range_rate_m_s = if slant_range_m > 1.0e-12 {
        rho_icrf.dot(drho_icrf) / slant_range_m
    } else {
        0.0
    };

    let rho_enu = r_enu_to_icrf.transpose() * rho_icrf;
    let (elevation_rad, azimuth_rad) = enu_to_el_az(rho_enu);
    let los_in_station_mount_m = r_icrf_to_sta_mount * rho_icrf;
    let station_off_boresight_rad = SensorFovModel::off_boresight_angle_rad(los_in_station_mount_m);

    Ok(LinkGeometrySample {
        epoch,
        station_pos_icrf_m,
        station_vel_icrf_m_s,
        vehicle_sensor_pos_icrf_m,
        vehicle_sensor_vel_icrf_m_s,
        slant_range_m,
        range_rate_m_s,
        elevation_rad,
        azimuth_rad,
        station_off_boresight_rad,
        vehicle_off_boresight_rad: veh_off_boresight,
        los_in_station_mount_m,
        los_in_vehicle_mount_m: los_veh_mount,
        r_vehicle_mount_to_station_mount: r_v2s,
    })
}

fn evaluate_station_mount_kinematics(
    epoch: TdbEpoch,
    station: &GroundStationModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<(DVec3, DVec3, DMat3, DMat3), LinkGeometryEvalError> {
    let host_ssb = ephemeris
        .geometric_state_icrf(station.body, NaifId::SSB, epoch, DerivativeOrder::Velocity)
        .map_err(|_| LinkGeometryEvalError::EphemerisMissing(station.body))?;
    let rot = orientation
        .body_rotation(station.body, epoch)
        .map_err(|_| LinkGeometryEvalError::OrientationMissing(station.body))?;
    let (sta_rel_pos, sta_rel_vel, local_enu) = surface_station_state_icrf(
        station.body,
        station.ellipsoid,
        &station.geodetic,
        epoch,
        orientation,
    )
    .map_err(|_| LinkGeometryEvalError::OrientationMissing(station.body))?;

    let r_f2i = rot.r_icrf_to_fixed.transpose();
    let r_enu_to_icrf = r_f2i * local_enu.enu_to_ecef();
    let omega_icrf = r_f2i * rot.omega_body_rad_s;
    let sta_mount_offset_icrf = r_enu_to_icrf * station.mount.offset_body_m;
    let station_pos_icrf_m = host_ssb.pos_m + sta_rel_pos + sta_mount_offset_icrf;
    let station_vel_icrf_m_s =
        host_ssb.vel_m_s + sta_rel_vel + omega_icrf.cross(sta_mount_offset_icrf);
    let r_sta_mount_to_enu = DMat3::from_quat(station.mount.mount_to_body());
    let r_icrf_to_sta_mount = (r_enu_to_icrf * r_sta_mount_to_enu).transpose();
    Ok((
        station_pos_icrf_m,
        station_vel_icrf_m_s,
        r_enu_to_icrf,
        r_icrf_to_sta_mount,
    ))
}

type VehicleMountEval = (
    DVec3,
    DVec3,
    Option<f64>,
    Option<[f64; 3]>,
    Option<[[f64; 3]; 3]>,
);

fn evaluate_vehicle_mount_kinematics(
    vehicle_cg_pos_ssb_m: DVec3,
    vehicle_cg_vel_ssb_m_s: DVec3,
    vehicle_attitude: Option<&AttitudeSample>,
    sensor: &VehicleSensorModel,
    station_pos_icrf_m: DVec3,
    r_icrf_to_sta_mount: DMat3,
) -> VehicleMountEval {
    let Some(att) = vehicle_attitude else {
        return (
            vehicle_cg_pos_ssb_m,
            vehicle_cg_vel_ssb_m_s,
            None,
            None,
            None,
        );
    };
    let mounted = sensor.mount.evaluate_world_kinematics(
        vehicle_cg_pos_ssb_m,
        vehicle_cg_vel_ssb_m_s,
        att.q_body_to_icrf,
        att.omega_body_rad_s,
        sensor.cg_body_m,
    );
    let r_icrf_to_v_mount = mounted.r_mount_to_world.transpose();
    let los_v_to_s_icrf = station_pos_icrf_m - mounted.position_world_m;
    let los_in_v_mount = r_icrf_to_v_mount * los_v_to_s_icrf;
    let off_rad = SensorFovModel::off_boresight_angle_rad(los_in_v_mount);
    let r_veh_to_sta = r_icrf_to_sta_mount * mounted.r_mount_to_world;
    (
        mounted.position_world_m,
        mounted.velocity_world_m_s,
        Some(off_rad),
        Some(los_in_v_mount.to_array()),
        Some(r_veh_to_sta.to_cols_array_2d()),
    )
}

fn enu_to_el_az(rho_enu: DVec3) -> (f64, f64) {
    let r = sqrt(rho_enu.length_squared());
    if r <= 1.0e-12 {
        return (0.0, 0.0);
    }
    let el = asin_clamped(rho_enu.z / r);
    let raw_az = atan2(rho_enu.x, rho_enu.y);
    let az = if raw_az < 0.0 {
        raw_az + core::f64::consts::TAU
    } else {
        raw_az
    };
    (el, az)
}

/// Internal lookup failure while evaluating instantaneous link geometry or occultation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkGeometryEvalError {
    /// Ephemeris missing for the specified body at the query epoch.
    EphemerisMissing(NaifId),
    /// Orientation missing for the specified body at the query epoch.
    OrientationMissing(NaifId),
}

/// Tests whether any celestial body in `env` occults the line-of-sight chord from
/// `station_pos_ssb_m` to `vehicle_pos_ssb_m` at `epoch`.
///
/// Returns `Ok(Some((occulting_body, min_altitude_above_surface_m)))` when occulted
/// (`min_altitude_above_surface_m <= 0.0`), or `Ok(None)` when unobstructed.
pub fn check_line_of_sight_occultation(
    epoch: TdbEpoch,
    station_host_body: NaifId,
    station_pos_ssb_m: DVec3,
    vehicle_pos_ssb_m: DVec3,
    env: &OccultationEnvironmentModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<Option<(NaifId, f64)>, LinkGeometryEvalError> {
    for spec in &env.bodies {
        let s_min = if spec.body == station_host_body {
            1.0e-4
        } else {
            0.0
        };
        let body_ssb = ephemeris
            .geometric_state_icrf(spec.body, NaifId::SSB, epoch, DerivativeOrder::Position)
            .map_err(|_| LinkGeometryEvalError::EphemerisMissing(spec.body))?
            .pos_m;

        let min_alt_m = match spec.shape {
            OccultingShape::Sphere { radius_m } => chord_min_altitude_above_sphere_m(
                station_pos_ssb_m - body_ssb,
                vehicle_pos_ssb_m - body_ssb,
                s_min,
                radius_m,
            ),
            OccultingShape::Ellipsoid(ell) => {
                let rot = orientation
                    .body_rotation(spec.body, epoch)
                    .map_err(|_| LinkGeometryEvalError::OrientationMissing(spec.body))?;
                let p0_fixed = rot.r_icrf_to_fixed * (station_pos_ssb_m - body_ssb);
                let p1_fixed = rot.r_icrf_to_fixed * (vehicle_pos_ssb_m - body_ssb);
                chord_min_altitude_above_ellipsoid_m(p0_fixed, p1_fixed, s_min, &ell)
            }
        };

        if min_alt_m < -1.0 {
            return Ok(Some((spec.body, min_alt_m)));
        }
    }
    Ok(None)
}

/// Tests whether any body in `env` blocks a signal that left `emit_pos_ssb_m` at
/// `emit_epoch` and arrives at `receive_pos_ssb_m` at `receive_epoch`.
///
/// Each body is taken where it is as the signal passes it, which for an interplanetary
/// link is minutes of orbital motion from where it is at either end. An end standing on
/// a body (`emitter_host`, `receiver_host`) is not blocked by that body's bulk here: its
/// horizon belongs to the end's elevation mask.
///
/// Returns `Ok(Some((occulting_body, min_altitude_above_surface_m)))` when blocked.
#[allow(clippy::too_many_arguments)]
pub fn check_signal_path_occultation(
    emit_epoch: TdbEpoch,
    receive_epoch: TdbEpoch,
    emitter_host: Option<NaifId>,
    receiver_host: Option<NaifId>,
    emit_pos_ssb_m: DVec3,
    receive_pos_ssb_m: DVec3,
    env: &OccultationEnvironmentModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<Option<(NaifId, f64)>, LinkGeometryEvalError> {
    let light_time_s = receive_epoch.signed_offset_seconds_from(emit_epoch);
    let chord = receive_pos_ssb_m - emit_pos_ssb_m;
    let chord_sq = chord.length_squared();
    for spec in &env.bodies {
        let body_at = |epoch| {
            ephemeris
                .geometric_state_icrf(spec.body, NaifId::SSB, epoch, DerivativeOrder::Position)
                .map(|state| state.pos_m)
                .map_err(|_| LinkGeometryEvalError::EphemerisMissing(spec.body))
        };
        // Where along the path the signal is nearest the body, first with the body as it
        // is on arrival, then with it as it is at that moment.
        let mut fraction = 1.0_f64;
        let mut pass_epoch = receive_epoch;
        let mut body_ssb = body_at(pass_epoch)?;
        for _ in 0..2 {
            if chord_sq > 1.0e-12 {
                fraction = ((body_ssb - emit_pos_ssb_m).dot(chord) / chord_sq).clamp(0.0, 1.0);
            }
            pass_epoch = emit_epoch.advanced_seconds(fraction * light_time_s);
            body_ssb = body_at(pass_epoch)?;
        }
        let from = if emitter_host == Some(spec.body) {
            HOST_BODY_CHORD_MARGIN
        } else {
            0.0
        };
        let until = if receiver_host == Some(spec.body) {
            1.0 - HOST_BODY_CHORD_MARGIN
        } else {
            1.0
        };
        let (p0, p1) = (emit_pos_ssb_m - body_ssb, receive_pos_ssb_m - body_ssb);
        let min_alt_m = match spec.shape {
            OccultingShape::Sphere { radius_m } => {
                chord_min_altitude_between_m(p0, p1, from, until, DVec3::ONE, radius_m)
            }
            OccultingShape::Ellipsoid(ell) => {
                let rot = orientation
                    .body_rotation(spec.body, pass_epoch)
                    .map_err(|_| LinkGeometryEvalError::OrientationMissing(spec.body))?;
                let (a, b) = (ell.semi_major_m.max(1.0), ell.semi_minor_m().max(1.0));
                chord_min_altitude_between_m(
                    rot.r_icrf_to_fixed * p0,
                    rot.r_icrf_to_fixed * p1,
                    from,
                    until,
                    DVec3::new(b / a, b / a, 1.0),
                    b,
                )
            }
        };
        if min_alt_m < -1.0 {
            return Ok(Some((spec.body, min_alt_m)));
        }
    }
    Ok(None)
}

/// Fraction of a signal path next to a body that an end standing on that body does not
/// answer to the body's bulk for.
const HOST_BODY_CHORD_MARGIN: f64 = 1.0e-4;

/// Least height of the chord `p0 -> p1` above a spheroid, over the fractions
/// `[from, until]` of its length: the points are scaled by `axis_scale` so the spheroid
/// becomes a sphere of `radius_m`. Infinite when the nearest point lies outside that span.
fn chord_min_altitude_between_m(
    p0: DVec3,
    p1: DVec3,
    from: f64,
    until: f64,
    axis_scale: DVec3,
    radius_m: f64,
) -> f64 {
    let (q0, q1) = (p0 * axis_scale, p1 * axis_scale);
    let d = q1 - q0;
    let d_sq = d.length_squared();
    if d_sq <= 1.0e-12 {
        return sqrt(q0.length_squared()) - radius_m;
    }
    let nearest = -q0.dot(d) / d_sq;
    if nearest <= from || nearest >= until {
        return f64::INFINITY;
    }
    sqrt((q0 + nearest * d).length_squared()) - radius_m
}

fn chord_min_altitude_above_sphere_m(p0: DVec3, p1: DVec3, s_min: f64, radius_m: f64) -> f64 {
    let d = p1 - p0;
    let d_sq = d.length_squared();
    if d_sq <= 1.0e-12 {
        return sqrt(p0.length_squared()) - radius_m;
    }
    let s = (-p0.dot(d) / d_sq).clamp(s_min, 1.0);
    if s <= s_min {
        return f64::INFINITY;
    }
    let closest = p0 + s * d;
    sqrt(closest.length_squared()) - radius_m
}

fn chord_min_altitude_above_ellipsoid_m(
    p0_fixed: DVec3,
    p1_fixed: DVec3,
    s_min: f64,
    ell: &sim_geo::BodyEllipsoid,
) -> f64 {
    let a = ell.semi_major_m.max(1.0);
    let b = ell.semi_minor_m().max(1.0);
    let scale = DVec3::new(1.0 / a, 1.0 / a, 1.0 / b);
    let q0 = p0_fixed * scale;
    let q1 = p1_fixed * scale;
    let dq = q1 - q0;
    let dq_sq = dq.length_squared();
    if dq_sq <= 1.0e-18 {
        return (sqrt(q0.length_squared()) - 1.0) * b;
    }
    let s = (-q0.dot(dq) / dq_sq).clamp(s_min, 1.0);
    if s <= s_min {
        return f64::INFINITY;
    }
    let closest_q = q0 + s * dq;
    (sqrt(closest_q.length_squared()) - 1.0) * b
}

/// Path-integrated propagation medium observables along a station-to-vehicle ray.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MediumIntegratedSample {
    /// Excess group path delay (`m`).
    pub excess_path_delay_m: f64,
    /// Slant Total Electron Content (`TECU`, $10^{16}\text{ e}^-/\text{m}^2$).
    pub slant_tec_tecu: f64,
    /// Atmospheric/rain/gaseous path attenuation (`dB`, `>= 0`).
    pub atmospheric_loss_db: f64,
}

/// Entry interface for electromagnetic/atmospheric medium models to evaluate path delay and
/// attenuation along a [`LinkGeometrySample`] line of sight without coupling `sim-analysis`
/// to a specific ionosphere or troposphere implementation.
pub trait PropagationMediumQuery: Send + Sync {
    /// Integrates medium properties along the ray from `station_pos_icrf_m` to `vehicle_pos_icrf_m` at `epoch`.
    fn sample_medium_along_los(
        &self,
        epoch: TdbEpoch,
        station_pos_icrf_m: DVec3,
        vehicle_pos_icrf_m: DVec3,
    ) -> Result<MediumIntegratedSample, String>;
}
