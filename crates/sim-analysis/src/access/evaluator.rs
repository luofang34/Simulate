//! Internal evaluation helpers for [`evaluate_station_access`](super::evaluate_station_access).

use glam::DVec3;
use sim_spacetime::{
    AttitudeTrajectorySource, BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, NaifId,
    TICKS_PER_SECOND, TdbEpoch, TickSpan, TranslationalTrajectorySource, UnitSemantics,
    ValidEpochInterval,
};

use crate::{
    access::{
        AccessAnalysisError, AccessRestrictionReason, AccessWindow, DataCoverageError,
        MAX_ACCESS_GRID_STEPS, ModelNotApplicableReason, StationAccessSample,
        WindowSearchCompleteness,
    },
    geometry::{
        LinkGeometryEvalError, LinkGeometrySample, check_line_of_sight_occultation,
        compute_link_geometry_sample,
    },
    models::{
        AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, VehicleSensorModel,
    },
};

pub(super) struct AccessEvalCtx<'a> {
    pub(super) trajectory: &'a dyn TranslationalTrajectorySource,
    pub(super) attitude: Option<&'a dyn AttitudeTrajectorySource>,
    pub(super) station: &'a GroundStationModel,
    pub(super) sensor: &'a VehicleSensorModel,
    pub(super) occultation_env: &'a OccultationEnvironmentModel,
    pub(super) query: &'a AccessQuerySpec,
    pub(super) ephemeris: &'a dyn EphemerisCatalog,
    pub(super) orientation: &'a dyn BodyOrientationCatalog,
}

pub(super) fn validate_applicability_and_coverage(
    trajectory: &dyn TranslationalTrajectorySource,
    attitude: Option<&dyn AttitudeTrajectorySource>,
    station: &GroundStationModel,
    sensor: &VehicleSensorModel,
    query: &AccessQuerySpec,
) -> Result<(), AccessAnalysisError> {
    let model = trajectory.model_identity();
    if !model.supports_physical_ephemeris_analysis() {
        return Err(ModelNotApplicableReason::UnsupportedDynamicalModel { model }.into());
    }
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
    if !station.min_elevation_rad.is_finite()
        || station.min_elevation_rad < -core::f64::consts::FRAC_PI_2
        || station.min_elevation_rad > core::f64::consts::FRAC_PI_2
    {
        return Err(ModelNotApplicableReason::InvalidStationParameters {
            min_elevation_rad: station.min_elevation_rad,
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

pub(crate) fn validate_query_grid_step_count(
    interval: ValidEpochInterval,
    step_s: f64,
) -> Result<usize, AccessAnalysisError> {
    let dur_s = interval.duration_s();
    if dur_s <= 0.0 {
        return Ok(0);
    }
    let raw_steps = (dur_s / step_s).ceil();
    if !raw_steps.is_finite() || raw_steps > (MAX_ACCESS_GRID_STEPS as f64) {
        let req = if raw_steps.is_finite() && raw_steps >= 0.0 {
            raw_steps as u64
        } else {
            u64::MAX
        };
        return Err(ModelNotApplicableReason::QueryStepCountExceeded {
            requested_steps: req,
            max_steps: MAX_ACCESS_GRID_STEPS as u64,
            interval_duration_s: dur_s,
            sample_step_s: step_s,
        }
        .into());
    }
    Ok(raw_steps as usize)
}

pub(crate) fn build_query_grid_epochs(
    interval: ValidEpochInterval,
    step_s: f64,
) -> Result<Vec<TdbEpoch>, AccessAnalysisError> {
    let n_steps = validate_query_grid_step_count(interval, step_s)?;
    if n_steps == 0 {
        return Ok(vec![interval.start_epoch]);
    }
    let mut epochs = Vec::with_capacity(n_steps.wrapping_add(1));
    let mut i = 0_usize;
    while i < n_steps {
        let tau_s = (i as f64) * step_s;
        let ticks = (tau_s * (TICKS_PER_SECOND as f64)).round() as u128;
        let ep = interval
            .start_epoch
            .add_span(TickSpan::from_ticks_u128(ticks));
        if ep < interval.end_epoch {
            epochs.push(ep);
        }
        i = i.wrapping_add(1);
    }
    epochs.push(interval.end_epoch);
    Ok(epochs)
}

pub(super) fn evaluate_single_epoch(
    ctx: &AccessEvalCtx<'_>,
    epoch: TdbEpoch,
) -> Result<StationAccessSample, AccessAnalysisError> {
    let (veh_pos_ssb, veh_vel_ssb) = sample_vehicle_ssb_icrf(ctx, epoch)?;
    let att_sample = if let Some(att_src) = ctx.attitude {
        match att_src.sample_attitude_icrf(epoch) {
            Ok(s) => Some(s),
            Err(_) if ctx.sensor.require_attitude => {
                return Err(DataCoverageError::AttitudeOutOfCoverage {
                    requested_epoch: epoch,
                    valid_interval: Some(att_src.attitude_valid_interval()),
                }
                .into());
            }
            Err(_) => None,
        }
    } else {
        None
    };

    let geom = compute_link_geometry_sample(
        epoch,
        veh_pos_ssb,
        veh_vel_ssb,
        att_sample.as_ref(),
        ctx.station,
        ctx.sensor,
        ctx.ephemeris,
        ctx.orientation,
    )
    .map_err(|e| map_geom_err(e, epoch))?;

    let restrictions = collect_epoch_restrictions(ctx, epoch, &geom)?;
    let accessible = restrictions.is_empty();
    Ok(StationAccessSample {
        geometry: geom,
        active_restrictions: restrictions,
        accessible,
    })
}

fn collect_epoch_restrictions(
    ctx: &AccessEvalCtx<'_>,
    epoch: TdbEpoch,
    geom: &LinkGeometrySample,
) -> Result<Vec<AccessRestrictionReason>, AccessAnalysisError> {
    let mut restrictions = Vec::new();
    if geom.elevation_rad < ctx.station.min_elevation_rad {
        restrictions.push(AccessRestrictionReason::BelowMinimumElevation {
            elevation_rad: geom.elevation_rad,
            min_elevation_rad: ctx.station.min_elevation_rad,
        });
    }
    if !ctx
        .station
        .fov
        .contains_direction_in_mount(geom.los_in_station_mount_m)
    {
        restrictions.push(AccessRestrictionReason::OutsideStationFov {
            off_boresight_rad: geom.station_off_boresight_rad,
        });
    }
    if let (Some(los_v), Some(off_v)) =
        (geom.los_in_vehicle_mount_m, geom.vehicle_off_boresight_rad)
        && !ctx
            .sensor
            .fov
            .contains_direction_in_mount(DVec3::from_array(los_v))
    {
        restrictions.push(AccessRestrictionReason::OutsideVehicleSensorFov {
            off_boresight_rad: off_v,
        });
    }
    if let Some(min_r) = ctx.query.min_range_m
        && geom.slant_range_m < min_r
    {
        restrictions.push(AccessRestrictionReason::RangeBelowMinimum {
            slant_range_m: geom.slant_range_m,
            min_range_m: min_r,
        });
    }
    if let Some(max_r) = ctx.query.max_range_m
        && geom.slant_range_m > max_r
    {
        restrictions.push(AccessRestrictionReason::RangeAboveMaximum {
            slant_range_m: geom.slant_range_m,
            max_range_m: max_r,
        });
    }
    if let Some((occ_body, min_alt)) = check_line_of_sight_occultation(
        epoch,
        ctx.station.body,
        geom.station_pos_icrf_m,
        geom.vehicle_sensor_pos_icrf_m,
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
    Ok(restrictions)
}

fn sample_vehicle_ssb_icrf(
    ctx: &AccessEvalCtx<'_>,
    epoch: TdbEpoch,
) -> Result<(DVec3, DVec3), AccessAnalysisError> {
    let framed = ctx
        .trajectory
        .sample_translational_icrf(epoch)
        .map_err(|_| {
            AccessAnalysisError::DataCoverage(DataCoverageError::TrajectoryOutOfCoverage {
                requested_epoch: epoch,
                valid_interval: ctx.trajectory.valid_interval(),
            })
        })?;
    if framed.spec.units != UnitSemantics::SiMetric {
        return Err(ModelNotApplicableReason::NonSiTrajectoryUnits {
            actual: framed.spec.units,
        }
        .into());
    }
    match framed.spec.origin {
        NaifId::SSB => Ok((framed.pos, framed.vel)),
        body => {
            let center_ssb = ctx
                .ephemeris
                .geometric_state_icrf(body, NaifId::SSB, epoch, DerivativeOrder::Velocity)
                .map_err(|_| {
                    AccessAnalysisError::DataCoverage(DataCoverageError::EphemerisOutOfCoverage {
                        body,
                        requested_epoch: epoch,
                    })
                })?;
            Ok((
                center_ssb.pos_m + framed.pos,
                center_ssb.vel_m_s + framed.vel,
            ))
        }
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

pub(super) fn extract_access_windows(
    ctx: &AccessEvalCtx<'_>,
    samples: &[StationAccessSample],
    refine: bool,
) -> Result<Vec<AccessWindow>, AccessAnalysisError> {
    let mut windows = Vec::new();
    let mut active_start: Option<TdbEpoch> = None;
    let mut max_el = f64::NEG_INFINITY;
    let mut min_range = f64::INFINITY;

    for (idx, s) in samples.iter().enumerate() {
        if s.accessible {
            if active_start.is_none() {
                let start_ep = match (refine, idx.checked_sub(1).and_then(|j| samples.get(j))) {
                    (true, Some(prev)) => {
                        bisect_transition_epoch(ctx, prev.geometry.epoch, s.geometry.epoch, false)?
                    }
                    _ => s.geometry.epoch,
                };
                active_start = Some(start_ep);
                max_el = s.geometry.elevation_rad;
                min_range = s.geometry.slant_range_m;
            } else {
                if refine && let Some(prev) = idx.checked_sub(1).and_then(|j| samples.get(j)) {
                    scan_access_interior_dropouts(
                        ctx,
                        prev.geometry.epoch,
                        s.geometry.epoch,
                        &mut windows,
                        &mut active_start,
                        &mut max_el,
                        &mut min_range,
                    )?;
                }
                max_el = max_el.max(s.geometry.elevation_rad);
                min_range = min_range.min(s.geometry.slant_range_m);
            }
        } else if let Some(start_ep) = active_start.take() {
            let end_ep = match (refine, idx.checked_sub(1).and_then(|j| samples.get(j))) {
                (true, Some(prev)) => {
                    bisect_transition_epoch(ctx, prev.geometry.epoch, s.geometry.epoch, true)?
                }
                (false, Some(prev)) => prev.geometry.epoch,
                _ => s.geometry.epoch,
            };
            let dur_s = end_ep.signed_offset_seconds_from(start_ep).max(0.0);
            windows.push(AccessWindow {
                start_epoch: start_ep,
                end_epoch: end_ep,
                duration_s: dur_s,
                max_elevation_rad: max_el,
                min_slant_range_m: min_range,
            });
        }
    }

    if let (Some(start_ep), Some(last)) = (active_start, samples.last()) {
        let end_ep = last.geometry.epoch;
        let dur_s = end_ep.signed_offset_seconds_from(start_ep).max(0.0);
        windows.push(AccessWindow {
            start_epoch: start_ep,
            end_epoch: end_ep,
            duration_s: dur_s,
            max_elevation_rad: max_el,
            min_slant_range_m: min_range,
        });
    }
    Ok(windows)
}

fn scan_access_interior_dropouts(
    ctx: &AccessEvalCtx<'_>,
    prev_ep: TdbEpoch,
    next_ep: TdbEpoch,
    windows: &mut Vec<AccessWindow>,
    active_start: &mut Option<TdbEpoch>,
    max_el: &mut f64,
    min_range: &mut f64,
) -> Result<(), AccessAnalysisError> {
    let substeps = WindowSearchCompleteness::INTERIOR_DROPOUT_SUBSTEPS;
    let span_s = next_ep.signed_offset_seconds_from(prev_ep);
    let tol_s = ctx.query.boundary_time_tolerance_s.max(1.0e-6);
    if span_s <= tol_s {
        return Ok(());
    }
    let mut cur_ep = prev_ep;
    let mut cur_acc = true;
    let mut k = 1_u32;
    while k < substeps {
        let frac = f64::from(k) / f64::from(substeps);
        let sub_ticks = ((span_s * frac) * (TICKS_PER_SECOND as f64)).round() as u128;
        let sub_ep = prev_ep.add_span(TickSpan::from_ticks_u128(sub_ticks));
        if sub_ep > cur_ep && sub_ep < next_ep {
            let sub = evaluate_single_epoch(ctx, sub_ep)?;
            if cur_acc && !sub.accessible {
                let end_ep = bisect_transition_epoch(ctx, cur_ep, sub_ep, true)?;
                if let Some(start_ep) = active_start.take() {
                    windows.push(AccessWindow {
                        start_epoch: start_ep,
                        end_epoch: end_ep,
                        duration_s: end_ep.signed_offset_seconds_from(start_ep).max(0.0),
                        max_elevation_rad: *max_el,
                        min_slant_range_m: *min_range,
                    });
                }
            } else if !cur_acc && sub.accessible {
                let start_ep = bisect_transition_epoch(ctx, cur_ep, sub_ep, false)?;
                *active_start = Some(start_ep);
                *max_el = sub.geometry.elevation_rad;
                *min_range = sub.geometry.slant_range_m;
            } else if sub.accessible {
                *max_el = max_el.max(sub.geometry.elevation_rad);
                *min_range = min_range.min(sub.geometry.slant_range_m);
            }
            cur_ep = sub_ep;
            cur_acc = sub.accessible;
        }
        k = k.wrapping_add(1);
    }
    if !cur_acc {
        let start_ep = bisect_transition_epoch(ctx, cur_ep, next_ep, false)?;
        *active_start = Some(start_ep);
        *max_el = f64::NEG_INFINITY;
        *min_range = f64::INFINITY;
    }
    Ok(())
}

fn bisect_transition_epoch(
    ctx: &AccessEvalCtx<'_>,
    mut lo: TdbEpoch,
    mut hi: TdbEpoch,
    lo_accessible: bool,
) -> Result<TdbEpoch, AccessAnalysisError> {
    let tol_s = ctx.query.boundary_time_tolerance_s.max(1.0e-6);
    let mut step = 0_u32;
    while step < 48 {
        let span_s = hi.signed_offset_seconds_from(lo);
        if span_s <= tol_s {
            break;
        }
        let half_ticks = ((0.5 * span_s) * (TICKS_PER_SECOND as f64)).round() as u128;
        let mid = lo.add_span(TickSpan::from_ticks_u128(half_ticks));
        let sample = evaluate_single_epoch(ctx, mid)?;
        if sample.accessible == lo_accessible {
            lo = mid;
        } else {
            hi = mid;
        }
        step = step.wrapping_add(1);
    }
    Ok(if lo_accessible { lo } else { hi })
}
