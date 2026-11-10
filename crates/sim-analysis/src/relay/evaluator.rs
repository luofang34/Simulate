//! Internal kinematics, constraint evaluation, and window root-refinement helpers for [`super`].

use glam::DVec3;
use sim_spacetime::{
    BodyOrientationCatalog, DataProvenance, EphemerisCatalog, NaifId, TICKS_PER_SECOND, TdbEpoch,
    asin_clamped, sqrt,
};

use crate::{
    access::{
        AccessAnalysisError, AccessRestrictionReason, DataCoverageError, ModelNotApplicableReason,
        validate_query_grid_step_count,
    },
    geometry::{
        LinkGeometryEvalError, check_line_of_sight_occultation, check_signal_path_occultation,
    },
    models::{AccessQuerySpec, OccultationEnvironmentModel, SensorFovModel, SignalTimeModel},
    relay::{
        AnalysisEndpoint, EndpointLinkSample, RelayAccessSample, RelayHopKind,
        RelayRestrictionEntry,
        kinematics::{EndpointWorldState, append_terrain_occultation},
        light_time::resolve_signal_events,
    },
    terrain::{TerrainElevationSource, TerrainLosConfig},
};

pub(super) struct EndpointPairCtx<'a> {
    pub(super) source: AnalysisEndpoint<'a>,
    pub(super) target: AnalysisEndpoint<'a>,
    pub(super) occultation_env: &'a OccultationEnvironmentModel,
    pub(super) terrain_dem: Option<(&'a dyn TerrainElevationSource, &'a TerrainLosConfig)>,
    pub(super) query: &'a AccessQuerySpec,
    pub(super) ephemeris: &'a dyn EphemerisCatalog,
    pub(super) orientation: &'a dyn BodyOrientationCatalog,
}

pub(super) struct RelayEvalCtx<'a> {
    pub(super) hop1_ctx: &'a EndpointPairCtx<'a>,
    pub(super) hop2_ctx: &'a EndpointPairCtx<'a>,
    pub(super) direct_ctx: Option<&'a EndpointPairCtx<'a>>,
    pub(super) query: &'a AccessQuerySpec,
}

pub(super) fn validate_query_and_terrain(
    query: &AccessQuerySpec,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
) -> Result<(), AccessAnalysisError> {
    let min_step_s = 1.0 / (TICKS_PER_SECOND as f64);
    if !query.sample_step_s.is_finite()
        || query.sample_step_s < min_step_s
        || !query.boundary_time_tolerance_s.is_finite()
        || query.boundary_time_tolerance_s <= 0.0
    {
        return Err(ModelNotApplicableReason::InvalidQueryParameters {
            sample_step_s: query.sample_step_s,
            boundary_time_tolerance_s: query.boundary_time_tolerance_s,
        }
        .into());
    }
    validate_query_grid_step_count(query.query_interval, query.sample_step_s)?;
    if let Some((_, cfg)) = terrain_dem
        && (!cfg.sample_step_m.is_finite()
            || cfg.sample_step_m <= 0.0
            || !cfg.min_clearance_m.is_finite()
            || !cfg.max_terrain_interaction_altitude_m.is_finite()
            || cfg.max_ray_samples == 0)
    {
        return Err(ModelNotApplicableReason::InvalidTerrainParameters {
            reason: "non-finite, non-positive, or zero-budget TerrainLosConfig parameters"
                .to_owned(),
        }
        .into());
    }
    Ok(())
}

pub(super) fn validate_endpoint(
    endpoint: &AnalysisEndpoint<'_>,
    query: &AccessQuerySpec,
) -> Result<(), AccessAnalysisError> {
    match *endpoint {
        AnalysisEndpoint::SurfaceStation { station, .. } => {
            if !station.min_elevation_rad.is_finite()
                || station.min_elevation_rad < -core::f64::consts::FRAC_PI_2
                || station.min_elevation_rad > core::f64::consts::FRAC_PI_2
            {
                return Err(ModelNotApplicableReason::InvalidStationParameters {
                    min_elevation_rad: station.min_elevation_rad,
                }
                .into());
            }
            Ok(())
        }
        AnalysisEndpoint::MovingPlatform {
            trajectory,
            attitude,
            sensor,
            min_local_elevation_rad,
            ..
        } => {
            let model = trajectory.model_identity();
            if !model.supports_physical_ephemeris_analysis() {
                return Err(ModelNotApplicableReason::UnsupportedDynamicalModel { model }.into());
            }
            if let Some(min_el) = min_local_elevation_rad
                && (!min_el.is_finite()
                    || min_el < -core::f64::consts::FRAC_PI_2
                    || min_el > core::f64::consts::FRAC_PI_2)
            {
                return Err(ModelNotApplicableReason::InvalidStationParameters {
                    min_elevation_rad: min_el,
                }
                .into());
            }
            let traj_cov = trajectory.valid_interval();
            for ep in [
                query.query_interval.start_epoch,
                query.query_interval.end_epoch,
            ] {
                if !traj_cov.contains(ep) {
                    return Err(DataCoverageError::TrajectoryOutOfCoverage {
                        requested_epoch: ep,
                        valid_interval: traj_cov,
                    }
                    .into());
                }
            }
            if sensor.require_attitude {
                let Some(att) = attitude else {
                    return Err(DataCoverageError::AttitudeOutOfCoverage {
                        requested_epoch: query.query_interval.start_epoch,
                        valid_interval: None,
                    }
                    .into());
                };
                let att_cov = att.attitude_valid_interval();
                for ep in [
                    query.query_interval.start_epoch,
                    query.query_interval.end_epoch,
                ] {
                    if !att_cov.contains(ep) {
                        return Err(DataCoverageError::AttitudeOutOfCoverage {
                            requested_epoch: ep,
                            valid_interval: Some(att_cov),
                        }
                        .into());
                    }
                }
            }
            Ok(())
        }
    }
}

pub(super) fn collect_endpoint_provenance(
    endpoint: &AnalysisEndpoint<'_>,
    out: &mut Vec<DataProvenance>,
) {
    if let AnalysisEndpoint::MovingPlatform { trajectory, .. } = *endpoint {
        let p = trajectory.provenance();
        if !out.iter().any(|existing| existing == p) {
            out.push(p.clone());
        }
    }
}

pub(super) fn evaluate_pair_at_epoch(
    ctx: &EndpointPairCtx<'_>,
    epoch: TdbEpoch,
) -> Result<EndpointLinkSample, AccessAnalysisError> {
    let events = resolve_signal_events(ctx, epoch)?;
    let (src, dst) = (events.source, events.target);

    let rho_icrf = dst.pos_ssb_m - src.pos_ssb_m;
    let drho_icrf = dst.vel_ssb_m_s - src.vel_ssb_m_s;
    let slant_range_m = sqrt(rho_icrf.length_squared());
    let range_rate_m_s = if slant_range_m > 1.0e-12 {
        rho_icrf.dot(drho_icrf) / slant_range_m
    } else {
        0.0
    };

    let el_from_src_rad = elevation_in_enu_rad(src.r_enu_to_icrf.transpose() * rho_icrf);
    let el_from_dst_rad = elevation_in_enu_rad(dst.r_enu_to_icrf.transpose() * (-rho_icrf));
    let src_los_mount = src.r_icrf_to_mount.map(|r| r * rho_icrf);
    let dst_los_mount = dst.r_icrf_to_mount.map(|r| r * (-rho_icrf));
    let source_off_boresight_rad = src_los_mount.map(SensorFovModel::off_boresight_angle_rad);
    let target_off_boresight_rad = dst_los_mount.map(SensorFovModel::off_boresight_angle_rad);

    let restrictions = collect_pair_restrictions(
        ctx,
        events.emit_epoch,
        epoch,
        &src,
        &dst,
        slant_range_m,
        el_from_src_rad,
        el_from_dst_rad,
        src_los_mount,
        dst_los_mount,
        source_off_boresight_rad,
        target_off_boresight_rad,
    )?;
    let accessible = restrictions.is_empty();

    Ok(EndpointLinkSample {
        epoch,
        source_pos_icrf_m: src.pos_ssb_m,
        target_pos_icrf_m: dst.pos_ssb_m,
        slant_range_m,
        range_rate_m_s,
        elevation_from_source_rad: el_from_src_rad,
        elevation_from_target_rad: el_from_dst_rad,
        source_off_boresight_rad,
        target_off_boresight_rad,
        light_time_s: events.light_time_s,
        active_restrictions: restrictions,
        accessible,
    })
}

#[allow(clippy::too_many_arguments)]
fn collect_pair_restrictions(
    ctx: &EndpointPairCtx<'_>,
    emit_epoch: TdbEpoch,
    epoch: TdbEpoch,
    src: &EndpointWorldState,
    dst: &EndpointWorldState,
    slant_range_m: f64,
    el_from_src_rad: f64,
    el_from_dst_rad: f64,
    src_los_mount: Option<DVec3>,
    dst_los_mount: Option<DVec3>,
    src_off_rad: Option<f64>,
    dst_off_rad: Option<f64>,
) -> Result<Vec<AccessRestrictionReason>, AccessAnalysisError> {
    let mut restrictions = Vec::new();
    if let Some(min_el) = src.min_elevation_rad
        && el_from_src_rad < min_el
    {
        restrictions.push(AccessRestrictionReason::BelowMinimumElevation {
            elevation_rad: el_from_src_rad,
            min_elevation_rad: min_el,
        });
    }
    if let Some(min_el_dst) = dst.min_elevation_rad
        && el_from_dst_rad < min_el_dst
    {
        restrictions.push(
            AccessRestrictionReason::BelowTargetEndpointMinimumElevation {
                elevation_rad: el_from_dst_rad,
                min_elevation_rad: min_el_dst,
            },
        );
    }
    append_fov_and_range_restrictions(
        ctx,
        src,
        dst,
        slant_range_m,
        src_los_mount,
        dst_los_mount,
        src_off_rad,
        dst_off_rad,
        &mut restrictions,
    );
    let below_dem_horizon = |dem_body: NaifId| {
        (src.is_surface_station && src.host_body == dem_body && el_from_src_rad < 0.0)
            || (dst.is_surface_station && dst.host_body == dem_body && el_from_dst_rad < 0.0)
    };
    append_body_and_terrain_occultation(
        ctx,
        emit_epoch,
        epoch,
        src,
        dst,
        below_dem_horizon,
        &mut restrictions,
    )?;
    Ok(restrictions)
}

#[allow(clippy::too_many_arguments)]
fn append_fov_and_range_restrictions(
    ctx: &EndpointPairCtx<'_>,
    src: &EndpointWorldState,
    dst: &EndpointWorldState,
    slant_range_m: f64,
    src_los_mount: Option<DVec3>,
    dst_los_mount: Option<DVec3>,
    src_off_rad: Option<f64>,
    dst_off_rad: Option<f64>,
    restrictions: &mut Vec<AccessRestrictionReason>,
) {
    if let (Some(los_m), Some(off_rad)) = (src_los_mount, src_off_rad)
        && !src.fov.contains_direction_in_mount(los_m)
    {
        restrictions.push(if src.is_surface_station {
            AccessRestrictionReason::OutsideStationFov {
                off_boresight_rad: off_rad,
            }
        } else {
            AccessRestrictionReason::OutsideVehicleSensorFov {
                off_boresight_rad: off_rad,
            }
        });
    }
    if let (Some(los_m), Some(off_rad)) = (dst_los_mount, dst_off_rad)
        && !dst.fov.contains_direction_in_mount(los_m)
    {
        restrictions.push(if dst.is_surface_station {
            AccessRestrictionReason::OutsideStationFov {
                off_boresight_rad: off_rad,
            }
        } else {
            AccessRestrictionReason::OutsideTargetSensorFov {
                endpoint_id: dst.endpoint_id,
                off_boresight_rad: off_rad,
            }
        });
    }
    if let Some(min_r) = ctx.query.min_range_m
        && slant_range_m < min_r
    {
        restrictions.push(AccessRestrictionReason::RangeBelowMinimum {
            slant_range_m,
            min_range_m: min_r,
        });
    }
    if let Some(max_r) = ctx.query.max_range_m
        && slant_range_m > max_r
    {
        restrictions.push(AccessRestrictionReason::RangeAboveMaximum {
            slant_range_m,
            max_range_m: max_r,
        });
    }
}

fn append_body_and_terrain_occultation(
    ctx: &EndpointPairCtx<'_>,
    emit_epoch: TdbEpoch,
    epoch: TdbEpoch,
    src: &EndpointWorldState,
    dst: &EndpointWorldState,
    below_dem_horizon: impl FnOnce(NaifId) -> bool,
    restrictions: &mut Vec<AccessRestrictionReason>,
) -> Result<(), AccessAnalysisError> {
    if ctx.query.signal_time == SignalTimeModel::LightTime {
        let host = |end: &EndpointWorldState| end.is_surface_station.then_some(end.host_body);
        if let Some((body, min_altitude_above_surface_m)) = check_signal_path_occultation(
            emit_epoch,
            epoch,
            host(src),
            host(dst),
            src.pos_ssb_m,
            dst.pos_ssb_m,
            ctx.occultation_env,
            ctx.ephemeris,
            ctx.orientation,
        )
        .map_err(|e| map_geom_err(e, epoch))?
        {
            restrictions.push(AccessRestrictionReason::OccultedByBody {
                body,
                min_altitude_above_surface_m,
            });
        }
        return append_terrain_occultation(
            ctx,
            emit_epoch,
            epoch,
            src,
            dst,
            below_dem_horizon,
            restrictions,
        );
    }
    let station_host = if src.is_surface_station {
        src.host_body
    } else if dst.is_surface_station {
        dst.host_body
    } else {
        NaifId::SSB
    };
    let (p0, p1) = if dst.is_surface_station && !src.is_surface_station {
        (dst.pos_ssb_m, src.pos_ssb_m)
    } else {
        (src.pos_ssb_m, dst.pos_ssb_m)
    };
    if let Some((occ_body, min_alt)) = check_line_of_sight_occultation(
        epoch,
        station_host,
        p0,
        p1,
        ctx.occultation_env,
        ctx.ephemeris,
        ctx.orientation,
    )
    .map_err(|e| map_geom_err(e, epoch))?
    {
        restrictions.push(AccessRestrictionReason::OccultedByBody {
            body: occ_body,
            min_altitude_above_surface_m: min_alt,
        });
    }
    append_terrain_occultation(
        ctx,
        emit_epoch,
        epoch,
        src,
        dst,
        below_dem_horizon,
        restrictions,
    )
}

pub(super) fn evaluate_relay_at_epoch(
    ctx: &RelayEvalCtx<'_>,
    epoch: TdbEpoch,
) -> Result<RelayAccessSample, AccessAnalysisError> {
    let hop2 = evaluate_pair_at_epoch(ctx.hop2_ctx, epoch)?;
    let hop1_epoch = match (ctx.query.signal_time, hop2.light_time_s) {
        (SignalTimeModel::LightTime, Some(lt2_s)) => epoch.advanced_seconds(-lt2_s),
        _ => epoch,
    };
    let hop1 = evaluate_pair_at_epoch(ctx.hop1_ctx, hop1_epoch)?;
    let direct = if let Some(d_ctx) = ctx.direct_ctx {
        Some(evaluate_pair_at_epoch(d_ctx, epoch)?)
    } else {
        None
    };

    let mut relay_restrictions = Vec::new();
    for &r in &hop1.active_restrictions {
        relay_restrictions.push(RelayRestrictionEntry {
            hop: RelayHopKind::AircraftToSatellite,
            reason: r,
        });
    }
    for &r in &hop2.active_restrictions {
        relay_restrictions.push(RelayRestrictionEntry {
            hop: RelayHopKind::SatelliteToGround,
            reason: r,
        });
    }
    if let Some(d) = &direct {
        for &r in &d.active_restrictions {
            relay_restrictions.push(RelayRestrictionEntry {
                hop: RelayHopKind::DirectAircraftToGround,
                reason: r,
            });
        }
    }
    let relay_accessible = hop1.accessible && hop2.accessible;
    Ok(RelayAccessSample {
        epoch,
        hop1_aircraft_to_satellite: hop1,
        hop2_satellite_to_ground: hop2,
        direct_aircraft_to_ground: direct,
        relay_restrictions,
        relay_accessible,
    })
}

fn elevation_in_enu_rad(rho_enu: DVec3) -> f64 {
    let r = sqrt(rho_enu.length_squared());
    if r <= 1.0e-12 {
        0.0
    } else {
        asin_clamped(rho_enu.z / r)
    }
}

const fn map_geom_err(err: LinkGeometryEvalError, epoch: TdbEpoch) -> AccessAnalysisError {
    match err {
        LinkGeometryEvalError::EphemerisMissing(body) => {
            AccessAnalysisError::DataCoverage(DataCoverageError::EphemerisOutOfCoverage {
                body,
                requested_epoch: epoch,
            })
        }
        LinkGeometryEvalError::OrientationMissing(body) => {
            AccessAnalysisError::DataCoverage(DataCoverageError::OrientationOutOfCoverage {
                body,
                requested_epoch: epoch,
            })
        }
    }
}
