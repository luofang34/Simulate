//! Per-epoch visibility evaluation and revisit-gap accumulation for [`super`].

use core::f64::consts::FRAC_PI_2;

use glam::{DMat3, DVec3};
use sim_geo::{Geodetic, LocalFrame};
use sim_spacetime::{
    BodyOrientationCatalog, BodyRotationState, DerivativeOrder, EphemerisCatalog, NaifId,
    TICKS_PER_SECOND, TdbEpoch, TickSpan, ValidEpochInterval,
};

use crate::{
    access::{AccessAnalysisError, DataCoverageError, build_query_grid_epochs},
    coverage::{
        CellCoverageSummary, CoverageAssetSpec, CoverageGridError, CoverageGridReport,
        CoverageQuerySpec, region::ResolvedH3Cell,
    },
    geometry::{
        chord_min_altitude_above_ellipsoid_m, chord_min_altitude_above_sphere_m, enu_to_el_az,
    },
    models::OccultingShape,
    relay::{
        resolve_platform_mount_kinematics, sample_platform_cg_ssb_icrf,
        validate_moving_platform_interval,
    },
};

struct PrecomputedCell {
    h3_index: u64,
    lat_rad: f64,
    lon_rad: f64,
    pos_fixed_m: DVec3,
    enu_to_fixed: DMat3,
}

struct CellAccumulator {
    covered_samples: usize,
    min_fold_observed: usize,
    max_fold_observed: usize,
    active_gap_start_idx: Option<usize>,
    gaps: Vec<ValidEpochInterval>,
}

struct AssetEpochState<'a> {
    pos_ssb_m: DVec3,
    r_icrf_to_mount: Option<DMat3>,
    spec: &'a CoverageAssetSpec<'a>,
}

enum OccultingEpochShape {
    Sphere {
        body: NaifId,
        body_ssb_m: DVec3,
        radius_m: f64,
    },
    Ellipsoid {
        body: NaifId,
        body_ssb_m: DVec3,
        r_icrf_to_fixed: DMat3,
        ellipsoid: sim_geo::BodyEllipsoid,
    },
}

struct EpochEnvironment {
    host_pos_ssb_m: DVec3,
    host_rot: BodyRotationState,
    occulting_bodies: Vec<OccultingEpochShape>,
}

pub(super) fn validate_coverage_inputs(
    assets: &[CoverageAssetSpec<'_>],
    query: &CoverageQuerySpec,
) -> Result<(), CoverageGridError> {
    if assets.is_empty() {
        return Err(CoverageGridError::EmptyConstellation);
    }
    if query.min_fold == 0 || query.min_fold > assets.len() {
        return Err(CoverageGridError::InvalidRequiredFold {
            required_fold: query.min_fold,
            asset_count: assets.len(),
        });
    }
    if !query.min_elevation_rad.is_finite()
        || query.min_elevation_rad < -FRAC_PI_2
        || query.min_elevation_rad > FRAC_PI_2
    {
        return Err(CoverageGridError::InvalidMinElevation {
            min_elevation_rad: query.min_elevation_rad,
        });
    }
    if query.query_interval.end_epoch < query.query_interval.start_epoch {
        return Err(CoverageGridError::InvalidTimeSpecification {
            reason: "query_interval end_epoch precedes start_epoch".to_owned(),
        });
    }
    let min_step_s = 1.0 / (TICKS_PER_SECOND as f64);
    if !query.sample_step_s.is_finite() || query.sample_step_s < min_step_s {
        return Err(CoverageGridError::InvalidTimeSpecification {
            reason: format!(
                "sample_step_s ({}) must be finite and >= {min_step_s} s",
                query.sample_step_s
            ),
        });
    }
    for asset in assets {
        validate_moving_platform_interval(
            asset.trajectory,
            asset.attitude,
            asset.sensor,
            query.query_interval,
        )?;
    }
    Ok(())
}

pub(super) fn run_coverage_grid(
    resolved_cells: &[ResolvedH3Cell],
    assets: &[CoverageAssetSpec<'_>],
    query: &CoverageQuerySpec,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<CoverageGridReport, CoverageGridError> {
    let epochs = build_query_grid_epochs(query.query_interval, query.sample_step_s)?;
    let precomputed = precompute_cells(resolved_cells, query.ellipsoid);
    let mut accumulators: Vec<CellAccumulator> = (0..precomputed.len())
        .map(|_| CellAccumulator {
            covered_samples: 0,
            min_fold_observed: usize::MAX,
            max_fold_observed: 0,
            active_gap_start_idx: None,
            gaps: Vec::new(),
        })
        .collect();

    for (epoch_idx, &epoch) in epochs.iter().enumerate() {
        let env = fetch_epoch_environment(query, epoch, ephemeris, orientation)?;
        let asset_states = fetch_epoch_assets(assets, epoch, ephemeris)?;
        evaluate_epoch_over_cells(
            query,
            &env,
            &asset_states,
            &precomputed,
            &epochs,
            epoch_idx,
            &mut accumulators,
        );
    }

    Ok(assemble_coverage_report(
        &precomputed,
        &mut accumulators,
        &epochs,
        query.min_fold,
    ))
}

fn precompute_cells(
    resolved_cells: &[ResolvedH3Cell],
    ellipsoid: sim_geo::BodyEllipsoid,
) -> Vec<PrecomputedCell> {
    resolved_cells
        .iter()
        .map(|cell| {
            let geo = Geodetic {
                lat_rad: cell.lat_rad,
                lon_rad: cell.lon_rad,
                height_m: 0.0,
            };
            PrecomputedCell {
                h3_index: cell.h3_index,
                lat_rad: cell.lat_rad,
                lon_rad: cell.lon_rad,
                pos_fixed_m: ellipsoid.geodetic_to_fixed(&geo),
                enu_to_fixed: LocalFrame::at(&geo).enu_to_ecef(),
            }
        })
        .collect()
}

fn fetch_epoch_environment(
    query: &CoverageQuerySpec,
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<EpochEnvironment, AccessAnalysisError> {
    let host_pos_ssb_m = ephemeris
        .geometric_state_icrf(query.body, NaifId::SSB, epoch, DerivativeOrder::Position)
        .map_err(|_| DataCoverageError::EphemerisOutOfCoverage {
            body: query.body,
            requested_epoch: epoch,
        })?
        .pos_m;
    let host_rot = orientation.body_rotation(query.body, epoch).map_err(|_| {
        DataCoverageError::OrientationOutOfCoverage {
            body: query.body,
            requested_epoch: epoch,
        }
    })?;
    let mut occulting_bodies = Vec::with_capacity(query.occultation_env.bodies.len());
    for spec in &query.occultation_env.bodies {
        let body_ssb_m = ephemeris
            .geometric_state_icrf(spec.body, NaifId::SSB, epoch, DerivativeOrder::Position)
            .map_err(|_| DataCoverageError::EphemerisOutOfCoverage {
                body: spec.body,
                requested_epoch: epoch,
            })?
            .pos_m;
        let entry = match spec.shape {
            OccultingShape::Sphere { radius_m } => OccultingEpochShape::Sphere {
                body: spec.body,
                body_ssb_m,
                radius_m,
            },
            OccultingShape::Ellipsoid(ellipsoid) => {
                let rot = orientation.body_rotation(spec.body, epoch).map_err(|_| {
                    DataCoverageError::OrientationOutOfCoverage {
                        body: spec.body,
                        requested_epoch: epoch,
                    }
                })?;
                OccultingEpochShape::Ellipsoid {
                    body: spec.body,
                    body_ssb_m,
                    r_icrf_to_fixed: rot.r_icrf_to_fixed,
                    ellipsoid,
                }
            }
        };
        occulting_bodies.push(entry);
    }
    Ok(EpochEnvironment {
        host_pos_ssb_m,
        host_rot,
        occulting_bodies,
    })
}

fn fetch_epoch_assets<'a>(
    assets: &'a [CoverageAssetSpec<'a>],
    epoch: TdbEpoch,
    ephemeris: &dyn EphemerisCatalog,
) -> Result<Vec<AssetEpochState<'a>>, AccessAnalysisError> {
    let mut states = Vec::with_capacity(assets.len());
    for spec in assets {
        let (cg_pos, cg_vel) = sample_platform_cg_ssb_icrf(spec.trajectory, epoch, ephemeris)?;
        let (pos_ssb_m, _, r_icrf_to_mount) =
            resolve_platform_mount_kinematics(cg_pos, cg_vel, spec.attitude, spec.sensor, epoch)?;
        states.push(AssetEpochState {
            pos_ssb_m,
            r_icrf_to_mount,
            spec,
        });
    }
    Ok(states)
}

fn evaluate_epoch_over_cells(
    query: &CoverageQuerySpec,
    env: &EpochEnvironment,
    asset_states: &[AssetEpochState<'_>],
    cells: &[PrecomputedCell],
    epochs: &[TdbEpoch],
    epoch_idx: usize,
    accumulators: &mut [CellAccumulator],
) {
    let r_f2i = env.host_rot.r_icrf_to_fixed.transpose();
    for (cell, acc) in cells.iter().zip(accumulators.iter_mut()) {
        let sta_pos_icrf = env.host_pos_ssb_m + r_f2i * cell.pos_fixed_m;
        let r_enu_to_icrf = r_f2i * cell.enu_to_fixed;
        let mut fold = 0_usize;
        for asset in asset_states {
            if is_cell_asset_visible(query, env, sta_pos_icrf, r_enu_to_icrf, asset) {
                fold = fold.wrapping_add(1);
            }
        }
        acc.min_fold_observed = acc.min_fold_observed.min(fold);
        acc.max_fold_observed = acc.max_fold_observed.max(fold);
        if fold >= query.min_fold {
            acc.covered_samples = acc.covered_samples.wrapping_add(1);
            if let Some(start_idx) = acc.active_gap_start_idx.take() {
                acc.gaps
                    .push(build_closed_gap(epochs, start_idx, epoch_idx));
            }
        } else if acc.active_gap_start_idx.is_none() {
            acc.active_gap_start_idx = Some(epoch_idx);
        }
    }
}

fn is_cell_asset_visible(
    query: &CoverageQuerySpec,
    env: &EpochEnvironment,
    sta_pos_icrf: DVec3,
    r_enu_to_icrf: DMat3,
    asset: &AssetEpochState<'_>,
) -> bool {
    let rho_icrf = asset.pos_ssb_m - sta_pos_icrf;
    let rho_enu = r_enu_to_icrf.transpose() * rho_icrf;
    let (el_rad, _) = enu_to_el_az(rho_enu);
    if el_rad < query.min_elevation_rad {
        return false;
    }
    if let Some(r_icrf_to_mount) = asset.r_icrf_to_mount
        && !asset
            .spec
            .sensor
            .fov
            .contains_direction_in_mount(r_icrf_to_mount * (-rho_icrf))
    {
        return false;
    }
    for occ in &env.occulting_bodies {
        let (body, body_ssb_m) = match *occ {
            OccultingEpochShape::Sphere {
                body, body_ssb_m, ..
            }
            | OccultingEpochShape::Ellipsoid {
                body, body_ssb_m, ..
            } => (body, body_ssb_m),
        };
        let s_min = if body == query.body { 1.0e-4 } else { 0.0 };
        let min_alt_m = match *occ {
            OccultingEpochShape::Sphere { radius_m, .. } => chord_min_altitude_above_sphere_m(
                sta_pos_icrf - body_ssb_m,
                asset.pos_ssb_m - body_ssb_m,
                s_min,
                radius_m,
            ),
            OccultingEpochShape::Ellipsoid {
                r_icrf_to_fixed,
                ellipsoid,
                ..
            } => chord_min_altitude_above_ellipsoid_m(
                r_icrf_to_fixed * (sta_pos_icrf - body_ssb_m),
                r_icrf_to_fixed * (asset.pos_ssb_m - body_ssb_m),
                s_min,
                &ellipsoid,
            ),
        };
        if min_alt_m < -1.0 {
            return false;
        }
    }
    true
}

fn midpoint_epoch(a: TdbEpoch, b: TdbEpoch) -> TdbEpoch {
    let ticks = b.ticks_since(a);
    a.add_span(TickSpan::from_ticks_u128(ticks / 2))
}

fn build_closed_gap(
    epochs: &[TdbEpoch],
    gap_start_idx: usize,
    next_covered_idx: usize,
) -> ValidEpochInterval {
    let start_epoch = match gap_start_idx.checked_sub(1) {
        Some(prev) => {
            if let (Some(&ep_prev), Some(&ep_curr)) = (epochs.get(prev), epochs.get(gap_start_idx))
            {
                midpoint_epoch(ep_prev, ep_curr)
            } else {
                epochs.first().copied().unwrap_or(TdbEpoch::J2000)
            }
        }
        None => epochs.first().copied().unwrap_or(TdbEpoch::J2000),
    };
    let end_epoch = match next_covered_idx.checked_sub(1) {
        Some(last_uncovered) => {
            if let (Some(&ep_last), Some(&ep_next)) =
                (epochs.get(last_uncovered), epochs.get(next_covered_idx))
            {
                midpoint_epoch(ep_last, ep_next)
            } else {
                epochs.last().copied().unwrap_or(start_epoch)
            }
        }
        None => epochs.first().copied().unwrap_or(start_epoch),
    };
    ValidEpochInterval::new(start_epoch, end_epoch)
}

fn build_trailing_gap(epochs: &[TdbEpoch], gap_start_idx: usize) -> ValidEpochInterval {
    let start_epoch = match gap_start_idx.checked_sub(1) {
        Some(prev) => {
            if let (Some(&ep_prev), Some(&ep_curr)) = (epochs.get(prev), epochs.get(gap_start_idx))
            {
                midpoint_epoch(ep_prev, ep_curr)
            } else {
                epochs.first().copied().unwrap_or(TdbEpoch::J2000)
            }
        }
        None => epochs.first().copied().unwrap_or(TdbEpoch::J2000),
    };
    let end_epoch = epochs.last().copied().unwrap_or(start_epoch);
    ValidEpochInterval::new(start_epoch, end_epoch)
}

fn summarize_cell(
    cell: &PrecomputedCell,
    acc: &mut CellAccumulator,
    epochs: &[TdbEpoch],
) -> CellCoverageSummary {
    let total_samples = epochs.len();
    if let Some(start_idx) = acc.active_gap_start_idx.take() {
        acc.gaps.push(build_trailing_gap(epochs, start_idx));
    }
    let coverage_fraction = if total_samples > 0 {
        (acc.covered_samples as f64) / (total_samples as f64)
    } else {
        0.0
    };
    let max_revisit_gap_s = acc
        .gaps
        .iter()
        .map(ValidEpochInterval::duration_s)
        .fold(0.0_f64, f64::max);
    let mean_revisit_gap_s = if acc.gaps.is_empty() {
        0.0
    } else {
        let sum_gap_s: f64 = acc.gaps.iter().map(ValidEpochInterval::duration_s).sum();
        sum_gap_s / (acc.gaps.len() as f64)
    };
    let min_fold_observed = if total_samples == 0 {
        0
    } else {
        acc.min_fold_observed
    };
    CellCoverageSummary {
        h3_index: cell.h3_index,
        lat_rad: cell.lat_rad,
        lon_rad: cell.lon_rad,
        covered_samples: acc.covered_samples,
        total_samples,
        coverage_fraction,
        min_fold_observed,
        max_fold_observed: acc.max_fold_observed,
        max_revisit_gap_s,
        mean_revisit_gap_s,
        gaps: core::mem::take(&mut acc.gaps),
    }
}

fn assemble_coverage_report(
    cells: &[PrecomputedCell],
    accumulators: &mut [CellAccumulator],
    epochs: &[TdbEpoch],
    _min_fold: usize,
) -> CoverageGridReport {
    let total_samples = epochs.len();
    let total_cells = cells.len();
    let mut summaries = Vec::with_capacity(total_cells);
    let mut sum_coverage_fraction = 0.0_f64;
    let mut min_coverage_fraction = f64::INFINITY;
    let mut global_max_revisit_gap_s = 0.0_f64;
    let mut total_covered_instants = 0_usize;

    for (cell, acc) in cells.iter().zip(accumulators.iter_mut()) {
        let summary = summarize_cell(cell, acc, epochs);
        sum_coverage_fraction += summary.coverage_fraction;
        min_coverage_fraction = min_coverage_fraction.min(summary.coverage_fraction);
        global_max_revisit_gap_s = global_max_revisit_gap_s.max(summary.max_revisit_gap_s);
        total_covered_instants = total_covered_instants.wrapping_add(summary.covered_samples);
        summaries.push(summary);
    }

    let mean_coverage_fraction = if total_cells > 0 {
        sum_coverage_fraction / (total_cells as f64)
    } else {
        0.0
    };
    if total_cells == 0 {
        min_coverage_fraction = 0.0;
    }
    let total_cell_instants = total_cells.saturating_mul(total_samples);
    let n_fold_fraction = if total_cell_instants > 0 {
        (total_covered_instants as f64) / (total_cell_instants as f64)
    } else {
        0.0
    };

    CoverageGridReport {
        cells: summaries,
        mean_coverage_fraction,
        min_coverage_fraction,
        global_max_revisit_gap_s,
        n_fold_fraction,
        total_cells,
    }
}
