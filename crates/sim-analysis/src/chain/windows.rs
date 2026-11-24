//! Interval sampling, window extraction, and provenance assembly for N-hop chains.

use sim_spacetime::{
    BodyOrientationCatalog, EphemerisCatalog, NaifId, TICKS_PER_SECOND, TdbEpoch,
    ValidEpochInterval,
};

use crate::{
    access::{
        AccessWindow, ModelNotApplicableReason, WindowSearchCompleteness,
        append_environment_provenance, build_query_grid_epochs,
    },
    chain::{
        ChainEvaluationError, ChainEvaluationReport, ChainHopRfSpec, ChainNodeSpec, ChainQuerySpec,
        ChainWindowReport, solver::evaluate_chain_epoch_internal,
    },
    relay::{
        VerificationScope, bisect_epoch_condition, collect_endpoint_provenance,
        scan_interior_dropouts,
    },
    terrain::{TerrainElevationSource, TerrainLosConfig},
};

struct ChainWindowCtx<'a> {
    nodes: &'a [ChainNodeSpec<'a>],
    hop_rf: Option<&'a [Option<ChainHopRfSpec<'a>>]>,
    query: &'a ChainQuerySpec,
    boundary_time_tolerance_s: f64,
    terrain_dem: Option<(&'a dyn TerrainElevationSource, &'a TerrainLosConfig)>,
    ephemeris: &'a dyn EphemerisCatalog,
    orientation: &'a dyn BodyOrientationCatalog,
}

pub(super) fn validate_window_sampling_params(
    query_interval: ValidEpochInterval,
    sample_step_s: f64,
    boundary_time_tolerance_s: f64,
) -> Result<(), ChainEvaluationError> {
    if query_interval.end_epoch < query_interval.start_epoch {
        return Err(ChainEvaluationError::Access(
            ModelNotApplicableReason::InvalidQueryParameters {
                sample_step_s,
                boundary_time_tolerance_s,
            }
            .into(),
        ));
    }
    let min_step_s = 1.0 / (TICKS_PER_SECOND as f64);
    if !sample_step_s.is_finite()
        || sample_step_s < min_step_s
        || !boundary_time_tolerance_s.is_finite()
        || boundary_time_tolerance_s <= 0.0
    {
        return Err(ChainEvaluationError::Access(
            ModelNotApplicableReason::InvalidQueryParameters {
                sample_step_s,
                boundary_time_tolerance_s,
            }
            .into(),
        ));
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub(super) fn run_chain_windows(
    nodes: &[ChainNodeSpec<'_>],
    hop_rf: Option<&[Option<ChainHopRfSpec<'_>>]>,
    query_interval: ValidEpochInterval,
    sample_step_s: f64,
    boundary_time_tolerance_s: f64,
    query: &ChainQuerySpec,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<ChainWindowReport, ChainEvaluationError> {
    let epochs = build_query_grid_epochs(query_interval, sample_step_s)?;
    let ctx = ChainWindowCtx {
        nodes,
        hop_rf,
        query,
        boundary_time_tolerance_s,
        terrain_dem,
        ephemeris,
        orientation,
    };

    let mut samples = Vec::with_capacity(epochs.len());
    for &ep in &epochs {
        samples.push(evaluate_chain_epoch_internal(
            nodes,
            hop_rf,
            ep,
            query,
            terrain_dem,
            ephemeris,
            orientation,
        )?);
    }

    let refine = boundary_time_tolerance_s < sample_step_s;
    let windows = extract_chain_windows(&ctx, &samples, refine)?;
    let completeness = if refine {
        WindowSearchCompleteness::BracketRefinedOnGrid {
            sample_step_s,
            boundary_tolerance_s: boundary_time_tolerance_s,
            may_miss_subgrid_windows_shorter_than_s: sample_step_s,
        }
    } else {
        WindowSearchCompleteness::SampledGridOnly { sample_step_s }
    };

    let has_rf = hop_rf.is_some_and(|s| s.iter().any(Option::is_some));
    let rf_verified = has_rf
        && samples
            .iter()
            .all(|s| !s.end_to_end_accessible || s.min_margin_db.is_some());
    let verification_scope = if rf_verified {
        VerificationScope::geometric_visibility_with_rf_link_budget()
    } else {
        VerificationScope::geometric_visibility_only()
    };
    let provenance = collect_chain_provenance(nodes, query_interval, ephemeris, orientation);

    Ok(ChainWindowReport {
        valid_interval: query_interval,
        completeness,
        verification_scope,
        provenance,
        samples,
        windows,
    })
}

fn collect_chain_provenance(
    nodes: &[ChainNodeSpec<'_>],
    query_interval: ValidEpochInterval,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Vec<sim_spacetime::DataProvenance> {
    let mut provenance = Vec::new();
    let mut env_body = NaifId::EARTH;
    for node in nodes {
        let ep = node.as_analysis_endpoint();
        collect_endpoint_provenance(&ep, &mut provenance);
        if let ChainNodeSpec::GroundStation { station, .. } = node {
            env_body = station.body;
        }
    }
    append_environment_provenance(
        &mut provenance,
        env_body,
        query_interval,
        ephemeris,
        orientation,
    );
    provenance
}

fn chain_window_metrics(
    nodes: &[ChainNodeSpec<'_>],
    report: &ChainEvaluationReport,
) -> (bool, f64, f64) {
    let mut min_station_el = f64::INFINITY;
    let mut has_station = false;
    let mut min_any_el = f64::INFINITY;
    let mut total_range_m = 0.0_f64;

    for (idx, hop) in report.hops.iter().enumerate() {
        total_range_m += hop.slant_range_m;
        min_any_el = min_any_el
            .min(hop.elevation_from_source_rad)
            .min(hop.elevation_from_target_rad);
        if matches!(nodes.get(idx), Some(ChainNodeSpec::GroundStation { .. })) {
            has_station = true;
            min_station_el = min_station_el.min(hop.elevation_from_source_rad);
        }
        if matches!(
            nodes.get(idx.wrapping_add(1)),
            Some(ChainNodeSpec::GroundStation { .. })
        ) {
            has_station = true;
            min_station_el = min_station_el.min(hop.elevation_from_target_rad);
        }
    }

    let el = if has_station {
        min_station_el
    } else if min_any_el.is_finite() {
        min_any_el
    } else {
        0.0
    };
    (report.end_to_end_accessible, el, total_range_m)
}

fn eval_chain_accessible_at(
    ctx: &ChainWindowCtx<'_>,
    epoch: TdbEpoch,
) -> Result<(bool, f64, f64), ChainEvaluationError> {
    let rep = evaluate_chain_epoch_internal(
        ctx.nodes,
        ctx.hop_rf,
        epoch,
        ctx.query,
        ctx.terrain_dem,
        ctx.ephemeris,
        ctx.orientation,
    )?;
    Ok(chain_window_metrics(ctx.nodes, &rep))
}

fn extract_chain_windows(
    ctx: &ChainWindowCtx<'_>,
    samples: &[ChainEvaluationReport],
    refine: bool,
) -> Result<Vec<AccessWindow>, ChainEvaluationError> {
    let mut windows = Vec::new();
    let mut active_start: Option<TdbEpoch> = None;
    let mut max_el = f64::NEG_INFINITY;
    let mut min_range = f64::INFINITY;

    for (idx, s) in samples.iter().enumerate() {
        let (acc, el, rng) = chain_window_metrics(ctx.nodes, s);
        if acc {
            if active_start.is_none() {
                let start_ep = match (refine, idx.checked_sub(1).and_then(|j| samples.get(j))) {
                    (true, Some(prev)) => {
                        bisect_chain_boundary(ctx, prev.query_epoch, s.query_epoch, false)?
                    }
                    _ => s.query_epoch,
                };
                active_start = Some(start_ep);
                max_el = el;
                min_range = rng;
            } else {
                if refine && let Some(prev) = idx.checked_sub(1).and_then(|j| samples.get(j)) {
                    scan_interior_dropouts(
                        prev.query_epoch,
                        s.query_epoch,
                        ctx.boundary_time_tolerance_s,
                        &mut windows,
                        &mut active_start,
                        &mut max_el,
                        &mut min_range,
                        |ep| eval_chain_accessible_at(ctx, ep),
                        |lo, hi, lo_acc| bisect_chain_boundary(ctx, lo, hi, lo_acc),
                    )?;
                }
                max_el = max_el.max(el);
                min_range = min_range.min(rng);
            }
        } else if let Some(start_ep) = active_start.take() {
            let end_ep = match (refine, idx.checked_sub(1).and_then(|j| samples.get(j))) {
                (true, Some(prev)) => {
                    bisect_chain_boundary(ctx, prev.query_epoch, s.query_epoch, true)?
                }
                _ => s.query_epoch,
            };
            push_window(&mut windows, start_ep, end_ep, max_el, min_range);
        }
    }
    if let (Some(start_ep), Some(last)) = (active_start, samples.last()) {
        push_window(&mut windows, start_ep, last.query_epoch, max_el, min_range);
    }
    Ok(windows)
}

fn push_window(
    windows: &mut Vec<AccessWindow>,
    start_epoch: TdbEpoch,
    end_epoch: TdbEpoch,
    max_elevation_rad: f64,
    min_slant_range_m: f64,
) {
    windows.push(AccessWindow {
        start_epoch,
        end_epoch,
        duration_s: end_epoch.signed_offset_seconds_from(start_epoch).max(0.0),
        max_elevation_rad,
        min_slant_range_m,
    });
}

fn bisect_chain_boundary(
    ctx: &ChainWindowCtx<'_>,
    lo: TdbEpoch,
    hi: TdbEpoch,
    lo_acc: bool,
) -> Result<TdbEpoch, ChainEvaluationError> {
    bisect_epoch_condition(lo, hi, lo_acc, ctx.boundary_time_tolerance_s, |ep| {
        Ok(evaluate_chain_epoch_internal(
            ctx.nodes,
            ctx.hop_rf,
            ep,
            ctx.query,
            ctx.terrain_dem,
            ctx.ephemeris,
            ctx.orientation,
        )?
        .end_to_end_accessible)
    })
}
