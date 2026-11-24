//! Per-hop causal light-time solver, geometric/occultation evaluation, and RF link budget composition.

use sim_spacetime::{
    BodyOrientationCatalog, EphemerisCatalog, SPEED_OF_LIGHT_M_S, TICKS_PER_SECOND, TdbEpoch,
    ValidEpochInterval, sqrt,
};

use crate::{
    access::AccessAnalysisError,
    chain::{
        ChainEvaluationError, ChainEvaluationReport, ChainHopRfSpec, ChainNodeSpec,
        ChainPropagationDirection, ChainQuerySpec, HopEvaluation,
    },
    models::SignalTimeModel,
    relay::{
        AnalysisEndpoint, EndpointLinkSample, EndpointWorldState, PairConstraintEnv,
        evaluate_endpoint_world_state, evaluate_resolved_pair, validate_endpoint_static,
        validate_terrain_config,
    },
    signal::{LinkBudgetModels, budget_sample},
    terrain::{TerrainElevationSource, TerrainLosConfig},
};

struct SolvedHop {
    tx_epoch: TdbEpoch,
    rx_epoch: TdbEpoch,
    src_state: EndpointWorldState,
    dst_state: EndpointWorldState,
    light_time_s: Option<f64>,
}

pub(super) fn validate_chain_inputs(
    nodes: &[ChainNodeSpec<'_>],
    hop_rf: Option<&[Option<ChainHopRfSpec<'_>>]>,
    query: &ChainQuerySpec,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
) -> Result<(), ChainEvaluationError> {
    if nodes.len() < 2 {
        return Err(ChainEvaluationError::TooFewNodes {
            node_count: nodes.len(),
        });
    }
    let expected_hops = nodes.len().saturating_sub(1);
    if let Some(rf_slice) = hop_rf
        && rf_slice.len() != expected_hops
    {
        return Err(ChainEvaluationError::HopRfSpecCountMismatch {
            expected: expected_hops,
            actual: rf_slice.len(),
        });
    }
    if !query.light_time_tolerance_s.is_finite()
        || query.light_time_tolerance_s <= 0.0
        || query.max_light_time_iterations == 0
    {
        return Err(ChainEvaluationError::InvalidLightTimeParameters {
            tolerance_s: query.light_time_tolerance_s,
            max_iterations: query.max_light_time_iterations,
        });
    }
    if query.min_range_m.is_some_and(|r| !r.is_finite() || r < 0.0)
        || query
            .max_range_m
            .is_some_and(|r| !r.is_finite() || r <= 0.0)
    {
        return Err(ChainEvaluationError::InvalidRangeBounds {
            min_range_m: query.min_range_m,
            max_range_m: query.max_range_m,
        });
    }
    if let (Some(min_r), Some(max_r)) = (query.min_range_m, query.max_range_m)
        && min_r > max_r
    {
        return Err(ChainEvaluationError::InvalidRangeBounds {
            min_range_m: query.min_range_m,
            max_range_m: query.max_range_m,
        });
    }
    validate_terrain_config(terrain_dem)?;
    for node in nodes {
        validate_endpoint_static(&node.as_analysis_endpoint())?;
    }
    if let Some(rf_slice) = hop_rf {
        for rf_opt in rf_slice.iter().flatten() {
            rf_opt.link.validate().map_err(AccessAnalysisError::from)?;
        }
    }
    Ok(())
}

pub(super) fn evaluate_chain_epoch_internal(
    nodes: &[ChainNodeSpec<'_>],
    hop_rf: Option<&[Option<ChainHopRfSpec<'_>>]>,
    epoch: TdbEpoch,
    query: &ChainQuerySpec,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<ChainEvaluationReport, ChainEvaluationError> {
    let dem = terrain_dem.map(|(d, _)| d);
    let pair_env = PairConstraintEnv {
        occultation_env: &query.occultation_env,
        terrain_dem,
        min_range_m: query.min_range_m,
        max_range_m: query.max_range_m,
        signal_time: query.signal_time,
        ephemeris,
        orientation,
    };

    let n_hops = nodes.len().saturating_sub(1);
    let mut hops = Vec::with_capacity(n_hops);
    match query.propagation_direction {
        ChainPropagationDirection::ForwardFromTx => {
            let mut cur_tx = epoch;
            for (hop_idx, pair) in nodes.windows(2).enumerate() {
                let (Some(src_node), Some(dst_node)) = (pair.first(), pair.get(1)) else {
                    continue;
                };
                let solved = solve_hop_forward(
                    hop_idx,
                    &src_node.as_analysis_endpoint(),
                    &dst_node.as_analysis_endpoint(),
                    cur_tx,
                    query,
                    dem,
                    ephemeris,
                    orientation,
                )?;
                cur_tx = solved.rx_epoch;
                let rf_opt = hop_rf.and_then(|s| s.get(hop_idx)).and_then(|x| x.as_ref());
                hops.push(compose_hop_eval(hop_idx, &pair_env, solved, rf_opt)?);
            }
        }
        ChainPropagationDirection::BackwardFromRx => {
            let mut cur_rx = epoch;
            for (hop_idx, pair) in nodes.windows(2).enumerate().rev() {
                let (Some(src_node), Some(dst_node)) = (pair.first(), pair.get(1)) else {
                    continue;
                };
                let solved = solve_hop_backward(
                    hop_idx,
                    &src_node.as_analysis_endpoint(),
                    &dst_node.as_analysis_endpoint(),
                    cur_rx,
                    query,
                    dem,
                    ephemeris,
                    orientation,
                )?;
                cur_rx = solved.tx_epoch;
                let rf_opt = hop_rf.and_then(|s| s.get(hop_idx)).and_then(|x| x.as_ref());
                hops.push(compose_hop_eval(hop_idx, &pair_env, solved, rf_opt)?);
            }
            hops.reverse();
        }
    }

    Ok(assemble_chain_report(epoch, query, hop_rf, hops))
}

#[allow(clippy::too_many_arguments)]
fn solve_hop_backward(
    hop_index: usize,
    src_ep: &AnalysisEndpoint<'_>,
    dst_ep: &AnalysisEndpoint<'_>,
    rx_epoch: TdbEpoch,
    query: &ChainQuerySpec,
    dem: Option<&dyn TerrainElevationSource>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<SolvedHop, ChainEvaluationError> {
    let dst_state = evaluate_endpoint_world_state(dst_ep, rx_epoch, dem, ephemeris, orientation)?;
    if query.signal_time == SignalTimeModel::Geometric {
        let src_state =
            evaluate_endpoint_world_state(src_ep, rx_epoch, dem, ephemeris, orientation)?;
        return Ok(SolvedHop {
            tx_epoch: rx_epoch,
            rx_epoch,
            src_state,
            dst_state,
            light_time_s: None,
        });
    }
    let tx_seed = initial_causal_epoch_seed(src_ep, rx_epoch);
    let mut src_state =
        evaluate_endpoint_world_state(src_ep, tx_seed, dem, ephemeris, orientation)?;
    let mut tau_s =
        sqrt((dst_state.pos_ssb_m - src_state.pos_ssb_m).length_squared()) / SPEED_OF_LIGHT_M_S;
    let mut tx_epoch = rx_epoch.advanced_seconds(-tau_s);
    let tick_floor_s = 1.0 / (TICKS_PER_SECOND as f64);
    let mut last_delta_s = f64::INFINITY;
    for _ in 0..query.max_light_time_iterations {
        src_state = evaluate_endpoint_world_state(src_ep, tx_epoch, dem, ephemeris, orientation)?;
        let next_tau_s =
            sqrt((dst_state.pos_ssb_m - src_state.pos_ssb_m).length_squared()) / SPEED_OF_LIGHT_M_S;
        last_delta_s = (next_tau_s - tau_s).abs();
        let next_tx = rx_epoch.advanced_seconds(-next_tau_s);
        tau_s = next_tau_s;
        let converged = last_delta_s <= query.light_time_tolerance_s
            || (next_tx == tx_epoch && last_delta_s <= tick_floor_s);
        if converged {
            if next_tx != tx_epoch {
                tx_epoch = next_tx;
                src_state =
                    evaluate_endpoint_world_state(src_ep, tx_epoch, dem, ephemeris, orientation)?;
                tau_s = sqrt((dst_state.pos_ssb_m - src_state.pos_ssb_m).length_squared())
                    / SPEED_OF_LIGHT_M_S;
            }
            return Ok(SolvedHop {
                tx_epoch,
                rx_epoch,
                src_state,
                dst_state,
                light_time_s: Some(tau_s),
            });
        }
        tx_epoch = next_tx;
    }
    Err(ChainEvaluationError::LightTimeDidNotConverge {
        hop_index,
        max_iterations: query.max_light_time_iterations,
        last_delta_s,
        tolerance_s: query.light_time_tolerance_s,
    })
}

#[allow(clippy::too_many_arguments)]
fn solve_hop_forward(
    hop_index: usize,
    src_ep: &AnalysisEndpoint<'_>,
    dst_ep: &AnalysisEndpoint<'_>,
    tx_epoch: TdbEpoch,
    query: &ChainQuerySpec,
    dem: Option<&dyn TerrainElevationSource>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<SolvedHop, ChainEvaluationError> {
    let src_state = evaluate_endpoint_world_state(src_ep, tx_epoch, dem, ephemeris, orientation)?;
    if query.signal_time == SignalTimeModel::Geometric {
        let dst_state =
            evaluate_endpoint_world_state(dst_ep, tx_epoch, dem, ephemeris, orientation)?;
        return Ok(SolvedHop {
            tx_epoch,
            rx_epoch: tx_epoch,
            src_state,
            dst_state,
            light_time_s: None,
        });
    }
    let rx_seed = initial_causal_epoch_seed(dst_ep, tx_epoch);
    let mut dst_state =
        evaluate_endpoint_world_state(dst_ep, rx_seed, dem, ephemeris, orientation)?;
    let mut tau_s =
        sqrt((dst_state.pos_ssb_m - src_state.pos_ssb_m).length_squared()) / SPEED_OF_LIGHT_M_S;
    let mut rx_epoch = tx_epoch.advanced_seconds(tau_s);
    let tick_floor_s = 1.0 / (TICKS_PER_SECOND as f64);
    let mut last_delta_s = f64::INFINITY;
    for _ in 0..query.max_light_time_iterations {
        dst_state = evaluate_endpoint_world_state(dst_ep, rx_epoch, dem, ephemeris, orientation)?;
        let next_tau_s =
            sqrt((dst_state.pos_ssb_m - src_state.pos_ssb_m).length_squared()) / SPEED_OF_LIGHT_M_S;
        last_delta_s = (next_tau_s - tau_s).abs();
        let next_rx = tx_epoch.advanced_seconds(next_tau_s);
        tau_s = next_tau_s;
        let converged = last_delta_s <= query.light_time_tolerance_s
            || (next_rx == rx_epoch && last_delta_s <= tick_floor_s);
        if converged {
            if next_rx != rx_epoch {
                rx_epoch = next_rx;
                dst_state =
                    evaluate_endpoint_world_state(dst_ep, rx_epoch, dem, ephemeris, orientation)?;
                tau_s = sqrt((dst_state.pos_ssb_m - src_state.pos_ssb_m).length_squared())
                    / SPEED_OF_LIGHT_M_S;
            }
            return Ok(SolvedHop {
                tx_epoch,
                rx_epoch,
                src_state,
                dst_state,
                light_time_s: Some(tau_s),
            });
        }
        rx_epoch = next_rx;
    }
    Err(ChainEvaluationError::LightTimeDidNotConverge {
        hop_index,
        max_iterations: query.max_light_time_iterations,
        last_delta_s,
        tolerance_s: query.light_time_tolerance_s,
    })
}

fn initial_causal_epoch_seed(endpoint: &AnalysisEndpoint<'_>, anchor: TdbEpoch) -> TdbEpoch {
    let AnalysisEndpoint::MovingPlatform {
        trajectory,
        attitude,
        sensor,
        ..
    } = *endpoint
    else {
        return anchor;
    };
    let traj_cov = trajectory.valid_interval();
    let (start, end) = match (sensor.require_attitude, attitude) {
        (true, Some(att)) => {
            let att_cov = att.attitude_valid_interval();
            let s = traj_cov.start_epoch.max(att_cov.start_epoch);
            let e = traj_cov.end_epoch.min(att_cov.end_epoch);
            if s <= e {
                (s, e)
            } else {
                (traj_cov.start_epoch, traj_cov.end_epoch)
            }
        }
        _ => (traj_cov.start_epoch, traj_cov.end_epoch),
    };
    clamp_epoch(anchor, ValidEpochInterval::new(start, end))
}

fn clamp_epoch(epoch: TdbEpoch, interval: ValidEpochInterval) -> TdbEpoch {
    if epoch < interval.start_epoch {
        interval.start_epoch
    } else if epoch > interval.end_epoch {
        interval.end_epoch
    } else {
        epoch
    }
}

fn compose_hop_eval(
    hop_index: usize,
    pair_env: &PairConstraintEnv<'_>,
    solved: SolvedHop,
    rf_spec: Option<&ChainHopRfSpec<'_>>,
) -> Result<HopEvaluation, AccessAnalysisError> {
    let sample: EndpointLinkSample = evaluate_resolved_pair(
        pair_env,
        solved.tx_epoch,
        solved.rx_epoch,
        &solved.src_state,
        &solved.dst_state,
        solved.light_time_s,
    )?;
    let doppler_factor = 1.0 - sample.range_rate_m_s / SPEED_OF_LIGHT_M_S;
    let (link_budget, budget_unavailable) = match rf_spec {
        Some(rf) => {
            let default_models;
            let models = match &rf.models {
                Some(m) => m,
                None => {
                    default_models = LinkBudgetModels::from_spec(rf.link);
                    &default_models
                }
            };
            let b_sample = budget_sample(
                &sample,
                rf.link,
                models,
                solved.src_state.is_surface_station,
                solved.dst_state.is_surface_station,
            );
            (b_sample.budget, b_sample.unavailable)
        }
        None => (None, None),
    };
    let eb_n0_db = link_budget.as_ref().map(|b| b.eb_n0_db);
    let margin_db = link_budget.as_ref().map(|b| b.margin_db);

    Ok(HopEvaluation {
        hop_index,
        source_endpoint_id: solved.src_state.endpoint_id,
        target_endpoint_id: solved.dst_state.endpoint_id,
        tx_epoch: solved.tx_epoch,
        rx_epoch: solved.rx_epoch,
        light_time_s: solved.light_time_s.unwrap_or(0.0),
        slant_range_m: sample.slant_range_m,
        range_rate_m_s: sample.range_rate_m_s,
        doppler_factor,
        source_pos_icrf_m: sample.source_pos_icrf_m,
        target_pos_icrf_m: sample.target_pos_icrf_m,
        elevation_from_source_rad: sample.elevation_from_source_rad,
        elevation_from_target_rad: sample.elevation_from_target_rad,
        source_off_boresight_rad: sample.source_off_boresight_rad,
        target_off_boresight_rad: sample.target_off_boresight_rad,
        accessible: sample.accessible,
        active_restrictions: sample.active_restrictions,
        link_budget,
        eb_n0_db,
        margin_db,
        budget_unavailable,
    })
}

fn assemble_chain_report(
    query_epoch: TdbEpoch,
    query: &ChainQuerySpec,
    hop_rf: Option<&[Option<ChainHopRfSpec<'_>>]>,
    hops: Vec<HopEvaluation>,
) -> ChainEvaluationReport {
    let tx_epoch = hops.first().map_or(query_epoch, |h| h.tx_epoch);
    let rx_epoch = hops.last().map_or(query_epoch, |h| h.rx_epoch);
    let total_light_time_s: f64 = hops.iter().map(|h| h.light_time_s).sum();
    let net_doppler_factor = hops.iter().fold(1.0_f64, |acc, h| acc * h.doppler_factor);
    let blocking_hop = hops.iter().position(|h| !h.accessible);
    let end_to_end_accessible = blocking_hop.is_none();

    let has_any_rf = hop_rf.is_some_and(|s| s.iter().any(Option::is_some));
    let min_margin_db = if end_to_end_accessible && has_any_rf {
        let mut all_rf_resolved = true;
        let mut min_margin = f64::INFINITY;
        let mut count = 0_usize;
        for (idx, h) in hops.iter().enumerate() {
            let expects_rf = hop_rf.and_then(|s| s.get(idx)).is_some_and(Option::is_some);
            if expects_rf {
                if let Some(m) = h.margin_db {
                    min_margin = min_margin.min(m);
                    count = count.wrapping_add(1);
                } else {
                    all_rf_resolved = false;
                    break;
                }
            }
        }
        (all_rf_resolved && count > 0).then_some(min_margin)
    } else {
        None
    };

    ChainEvaluationReport {
        query_epoch,
        propagation_direction: query.propagation_direction,
        signal_time: query.signal_time,
        end_to_end_accessible,
        tx_epoch,
        rx_epoch,
        total_light_time_s,
        net_doppler_factor,
        min_margin_db,
        hops,
        blocking_hop,
    }
}
