//! Window extraction and bracket-bisection root refinement for two-endpoint and multi-hop relay access.

use sim_spacetime::{TICKS_PER_SECOND, TdbEpoch, TickSpan};

use crate::{
    access::{AccessAnalysisError, AccessWindow},
    relay::{
        EndpointLinkSample, RelayAccessSample,
        evaluator::{
            EndpointPairCtx, RelayEvalCtx, evaluate_pair_at_epoch, evaluate_relay_at_epoch,
        },
    },
};

pub(super) fn extract_pair_windows(
    ctx: &EndpointPairCtx<'_>,
    samples: &[EndpointLinkSample],
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
                    (true, Some(prev)) => bisect_pair_epoch(ctx, prev.epoch, s.epoch, false)?,
                    _ => s.epoch,
                };
                active_start = Some(start_ep);
                max_el = s.elevation_from_source_rad;
                min_range = s.slant_range_m;
            } else {
                max_el = max_el.max(s.elevation_from_source_rad);
                min_range = min_range.min(s.slant_range_m);
            }
        } else if let Some(start_ep) = active_start.take() {
            let end_ep = match (refine, idx.checked_sub(1).and_then(|j| samples.get(j))) {
                (true, Some(prev)) => bisect_pair_epoch(ctx, prev.epoch, s.epoch, true)?,
                _ => s.epoch,
            };
            windows.push(AccessWindow {
                start_epoch: start_ep,
                end_epoch: end_ep,
                duration_s: end_ep.signed_offset_seconds_from(start_ep).max(0.0),
                max_elevation_rad: max_el,
                min_slant_range_m: min_range,
            });
        }
    }
    if let (Some(start_ep), Some(last)) = (active_start, samples.last()) {
        windows.push(AccessWindow {
            start_epoch: start_ep,
            end_epoch: last.epoch,
            duration_s: last.epoch.signed_offset_seconds_from(start_ep).max(0.0),
            max_elevation_rad: max_el,
            min_slant_range_m: min_range,
        });
    }
    Ok(windows)
}

pub(super) fn extract_relay_windows(
    ctx: &RelayEvalCtx<'_>,
    samples: &[RelayAccessSample],
    refine: bool,
) -> Result<Vec<AccessWindow>, AccessAnalysisError> {
    let mut windows = Vec::new();
    let mut active_start: Option<TdbEpoch> = None;
    let mut max_el = f64::NEG_INFINITY;
    let mut min_range = f64::INFINITY;
    for (idx, s) in samples.iter().enumerate() {
        let el = s
            .hop1_aircraft_to_satellite
            .elevation_from_source_rad
            .min(s.hop2_satellite_to_ground.elevation_from_target_rad);
        let rng =
            s.hop1_aircraft_to_satellite.slant_range_m + s.hop2_satellite_to_ground.slant_range_m;
        if s.relay_accessible {
            if active_start.is_none() {
                let start_ep = match (refine, idx.checked_sub(1).and_then(|j| samples.get(j))) {
                    (true, Some(prev)) => bisect_relay_epoch(ctx, prev.epoch, s.epoch, false)?,
                    _ => s.epoch,
                };
                active_start = Some(start_ep);
                max_el = el;
                min_range = rng;
            } else {
                max_el = max_el.max(el);
                min_range = min_range.min(rng);
            }
        } else if let Some(start_ep) = active_start.take() {
            let end_ep = match (refine, idx.checked_sub(1).and_then(|j| samples.get(j))) {
                (true, Some(prev)) => bisect_relay_epoch(ctx, prev.epoch, s.epoch, true)?,
                _ => s.epoch,
            };
            windows.push(AccessWindow {
                start_epoch: start_ep,
                end_epoch: end_ep,
                duration_s: end_ep.signed_offset_seconds_from(start_ep).max(0.0),
                max_elevation_rad: max_el,
                min_slant_range_m: min_range,
            });
        }
    }
    if let (Some(start_ep), Some(last)) = (active_start, samples.last()) {
        windows.push(AccessWindow {
            start_epoch: start_ep,
            end_epoch: last.epoch,
            duration_s: last.epoch.signed_offset_seconds_from(start_ep).max(0.0),
            max_elevation_rad: max_el,
            min_slant_range_m: min_range,
        });
    }
    Ok(windows)
}

fn bisect_pair_epoch(
    ctx: &EndpointPairCtx<'_>,
    mut t_lo: TdbEpoch,
    mut t_hi: TdbEpoch,
    lo_is_accessible: bool,
) -> Result<TdbEpoch, AccessAnalysisError> {
    let tol_s = ctx
        .query
        .boundary_time_tolerance_s
        .max(1.0 / (TICKS_PER_SECOND as f64));
    for _ in 0..48 {
        let span_s = t_hi.signed_offset_seconds_from(t_lo);
        if span_s <= tol_s {
            break;
        }
        let mid_ticks = ((span_s * 0.5) * (TICKS_PER_SECOND as f64)).round() as u128;
        if mid_ticks == 0 {
            break;
        }
        let t_mid = t_lo.add_span(TickSpan::from_ticks_u128(mid_ticks));
        let mid_acc = evaluate_pair_at_epoch(ctx, t_mid)?.accessible;
        if mid_acc == lo_is_accessible {
            t_lo = t_mid;
        } else {
            t_hi = t_mid;
        }
    }
    Ok(if lo_is_accessible { t_lo } else { t_hi })
}

fn bisect_relay_epoch(
    ctx: &RelayEvalCtx<'_>,
    mut t_lo: TdbEpoch,
    mut t_hi: TdbEpoch,
    lo_is_accessible: bool,
) -> Result<TdbEpoch, AccessAnalysisError> {
    let tol_s = ctx
        .query
        .boundary_time_tolerance_s
        .max(1.0 / (TICKS_PER_SECOND as f64));
    for _ in 0..48 {
        let span_s = t_hi.signed_offset_seconds_from(t_lo);
        if span_s <= tol_s {
            break;
        }
        let mid_ticks = ((span_s * 0.5) * (TICKS_PER_SECOND as f64)).round() as u128;
        if mid_ticks == 0 {
            break;
        }
        let t_mid = t_lo.add_span(TickSpan::from_ticks_u128(mid_ticks));
        let mid_acc = evaluate_relay_at_epoch(ctx, t_mid)?.relay_accessible;
        if mid_acc == lo_is_accessible {
            t_lo = t_mid;
        } else {
            t_hi = t_mid;
        }
    }
    Ok(if lo_is_accessible { t_lo } else { t_hi })
}
