//! Endpoint world-state kinematics (`EndpointWorldState`) for surface stations and moving platforms.

use glam::{DMat3, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_spacetime::{
    BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, NaifId, TdbEpoch, UnitSemantics,
    surface_station_state_icrf,
};

use crate::{
    access::{AccessAnalysisError, DataCoverageError, ModelNotApplicableReason},
    models::SensorFovModel,
    relay::AnalysisEndpoint,
    terrain::{TerrainElevationSource, resolve_altitude_ellipsoidal_height_m},
};

pub(super) struct EndpointWorldState {
    pub(super) endpoint_id: u32,
    pub(super) is_surface_station: bool,
    pub(super) host_body: NaifId,
    pub(super) pos_ssb_m: DVec3,
    pub(super) vel_ssb_m_s: DVec3,
    pub(super) r_enu_to_icrf: DMat3,
    pub(super) r_icrf_to_mount: Option<DMat3>,
    pub(super) fov: SensorFovModel,
    pub(super) min_elevation_rad: Option<f64>,
}

pub(super) fn evaluate_endpoint_world_state(
    endpoint: &AnalysisEndpoint<'_>,
    epoch: TdbEpoch,
    dem: Option<&dyn TerrainElevationSource>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<EndpointWorldState, AccessAnalysisError> {
    match *endpoint {
        AnalysisEndpoint::SurfaceStation {
            station,
            altitude_spec,
        } => evaluate_surface_station_world_state(
            station,
            altitude_spec,
            epoch,
            dem,
            ephemeris,
            orientation,
        ),
        AnalysisEndpoint::MovingPlatform {
            endpoint_id,
            host_body,
            ellipsoid,
            trajectory,
            attitude,
            sensor,
            min_local_elevation_rad,
            ..
        } => evaluate_moving_platform_world_state(
            endpoint_id,
            host_body,
            ellipsoid,
            trajectory,
            attitude,
            sensor,
            min_local_elevation_rad,
            epoch,
            ephemeris,
            orientation,
        ),
    }
}

fn evaluate_surface_station_world_state(
    station: &crate::models::GroundStationModel,
    altitude_spec: Option<&crate::terrain::AltitudeSpec>,
    epoch: TdbEpoch,
    dem: Option<&dyn TerrainElevationSource>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<EndpointWorldState, AccessAnalysisError> {
    let resolved_h_m = if let Some(spec) = altitude_spec {
        resolve_altitude_ellipsoidal_height_m(
            spec,
            station.body,
            station.geodetic.lat_rad,
            station.geodetic.lon_rad,
            epoch,
            dem,
        )?
    } else {
        station.geodetic.height_m
    };
    let geo = Geodetic {
        lat_rad: station.geodetic.lat_rad,
        lon_rad: station.geodetic.lon_rad,
        height_m: resolved_h_m,
    };
    let host_ssb = ephemeris
        .geometric_state_icrf(station.body, NaifId::SSB, epoch, DerivativeOrder::Velocity)
        .map_err(|_| DataCoverageError::EphemerisOutOfCoverage {
            body: station.body,
            requested_epoch: epoch,
        })?;
    let rot = orientation
        .body_rotation(station.body, epoch)
        .map_err(|_| DataCoverageError::OrientationOutOfCoverage {
            body: station.body,
            requested_epoch: epoch,
        })?;
    let (rel_pos, rel_vel, local_enu) =
        surface_station_state_icrf(station.body, station.ellipsoid, &geo, epoch, orientation)
            .map_err(|_| DataCoverageError::OrientationOutOfCoverage {
                body: station.body,
                requested_epoch: epoch,
            })?;
    let r_f2i = rot.r_icrf_to_fixed.transpose();
    let r_enu_to_icrf = r_f2i * local_enu.enu_to_ecef();
    let omega_icrf = r_f2i * rot.omega_body_rad_s;
    let mount_off_icrf = r_enu_to_icrf * station.mount.offset_body_m;
    let r_mount_to_enu = DMat3::from_quat(station.mount.mount_to_body());
    let r_icrf_to_mount = (r_enu_to_icrf * r_mount_to_enu).transpose();
    Ok(EndpointWorldState {
        endpoint_id: station.station_id,
        is_surface_station: true,
        host_body: station.body,
        pos_ssb_m: host_ssb.pos_m + rel_pos + mount_off_icrf,
        vel_ssb_m_s: host_ssb.vel_m_s + rel_vel + omega_icrf.cross(mount_off_icrf),
        r_enu_to_icrf,
        r_icrf_to_mount: Some(r_icrf_to_mount),
        fov: station.fov,
        min_elevation_rad: Some(station.min_elevation_rad),
    })
}

#[allow(clippy::too_many_arguments)]
fn evaluate_moving_platform_world_state(
    endpoint_id: u32,
    host_body: NaifId,
    ellipsoid: BodyEllipsoid,
    trajectory: &dyn sim_spacetime::TranslationalTrajectorySource,
    attitude: Option<&dyn sim_spacetime::AttitudeTrajectorySource>,
    sensor: &crate::models::VehicleSensorModel,
    min_local_elevation_rad: Option<f64>,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<EndpointWorldState, AccessAnalysisError> {
    let framed = trajectory.sample_translational_icrf(epoch).map_err(|_| {
        AccessAnalysisError::DataCoverage(DataCoverageError::TrajectoryOutOfCoverage {
            requested_epoch: epoch,
            valid_interval: trajectory.valid_interval(),
        })
    })?;
    if framed.spec.units != UnitSemantics::SiMetric {
        return Err(ModelNotApplicableReason::NonSiTrajectoryUnits {
            actual: framed.spec.units,
        }
        .into());
    }
    let (cg_pos_ssb, cg_vel_ssb) = match framed.spec.origin {
        NaifId::SSB => (framed.pos, framed.vel),
        body => {
            let c = ephemeris
                .geometric_state_icrf(body, NaifId::SSB, epoch, DerivativeOrder::Velocity)
                .map_err(|_| DataCoverageError::EphemerisOutOfCoverage {
                    body,
                    requested_epoch: epoch,
                })?;
            (c.pos_m + framed.pos, c.vel_m_s + framed.vel)
        }
    };
    let (pos_ssb_m, vel_ssb_m_s, r_icrf_to_mount) =
        resolve_platform_mount_kinematics(cg_pos_ssb, cg_vel_ssb, attitude, sensor, epoch)?;
    let host_ssb = ephemeris
        .geometric_state_icrf(host_body, NaifId::SSB, epoch, DerivativeOrder::Position)
        .map_err(|_| DataCoverageError::EphemerisOutOfCoverage {
            body: host_body,
            requested_epoch: epoch,
        })?
        .pos_m;
    let rot = orientation.body_rotation(host_body, epoch).map_err(|_| {
        DataCoverageError::OrientationOutOfCoverage {
            body: host_body,
            requested_epoch: epoch,
        }
    })?;
    let pos_fixed = rot.r_icrf_to_fixed * (pos_ssb_m - host_ssb);
    let geo = ellipsoid.fixed_to_geodetic(pos_fixed);
    let local_enu = LocalFrame::at(&geo);
    let r_enu_to_icrf = rot.r_icrf_to_fixed.transpose() * local_enu.enu_to_ecef();
    Ok(EndpointWorldState {
        endpoint_id,
        is_surface_station: false,
        host_body,
        pos_ssb_m,
        vel_ssb_m_s,
        r_enu_to_icrf,
        r_icrf_to_mount,
        fov: sensor.fov,
        min_elevation_rad: min_local_elevation_rad,
    })
}

fn resolve_platform_mount_kinematics(
    cg_pos_ssb: DVec3,
    cg_vel_ssb: DVec3,
    attitude: Option<&dyn sim_spacetime::AttitudeTrajectorySource>,
    sensor: &crate::models::VehicleSensorModel,
    epoch: TdbEpoch,
) -> Result<(DVec3, DVec3, Option<DMat3>), AccessAnalysisError> {
    if let Some(att_src) = attitude {
        match att_src.sample_attitude_icrf(epoch) {
            Ok(att) => {
                let m = sensor.mount.evaluate_world_kinematics(
                    cg_pos_ssb,
                    cg_vel_ssb,
                    att.q_body_to_icrf,
                    att.omega_body_rad_s,
                    sensor.cg_body_m,
                );
                Ok((
                    m.position_world_m,
                    m.velocity_world_m_s,
                    Some(m.r_mount_to_world.transpose()),
                ))
            }
            Err(_) if sensor.require_attitude => Err(DataCoverageError::AttitudeOutOfCoverage {
                requested_epoch: epoch,
                valid_interval: Some(att_src.attitude_valid_interval()),
            }
            .into()),
            Err(_) => Ok((cg_pos_ssb, cg_vel_ssb, None)),
        }
    } else {
        Ok((cg_pos_ssb, cg_vel_ssb, None))
    }
}
