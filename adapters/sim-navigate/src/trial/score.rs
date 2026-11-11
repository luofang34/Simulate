//! Scoring a replay's estimates against truth: the only place truth and an
//! estimate meet.

use navigate_contract::{NavigationSolution, SymmetricCov3};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{LocalFrame, ecef_to_geodetic};

use super::TrialError;
use super::availability::ScheduledInterval;
use super::replay::IMU_SOURCE;
use super::replay::SensorTallies;
use super::report::{
    ConsistencyNote, ErrorStats, IntervalErrors, IntervalReport, SeriesSample, SetSummary,
};
use crate::{
    AblationEvaluation, Estimate, NavigationOrigin, NavigationOutput, Percentiles, SimTruth, Truth,
    compare_attitude_with_truth,
};

/// Scores one estimate against the truth at its instant, recording its
/// position score under `origin` in `evaluation`.
pub fn score_sample(
    evaluation: &mut AblationEvaluation,
    origin: NavigationOrigin,
    truth: &Truth<(SimInstant, VehicleState)>,
    estimate: &Estimate<NavigationSolution>,
) -> Result<SeriesSample, TrialError> {
    let (at, vehicle) = *truth.get();
    let solution = estimate.get();
    let output = Estimate::new(NavigationOutput::from_solution(
        origin, IMU_SOURCE, solution, at, at,
    ));
    let position = evaluation.record(&Truth::new(SimTruth::vehicle(at, vehicle)), &output)?;
    let attitude = compare_attitude_with_truth(solution, vehicle.attitude)?;
    let frame = LocalFrame::at(&ecef_to_geodetic(vehicle.position_ecef));
    let true_velocity = frame.ecef_to_ned_vector(vehicle.velocity_ecef);
    let v = solution.velocity;
    let velocity_error = glam::DVec3::new(v.north_mps, v.east_mps, v.down_mps) - true_velocity;
    let (pn, pe, pd) = solution.position_cov.diagonal();
    Ok(SeriesSample {
        time_s: at.duration_since(SimInstant::START).as_seconds(),
        horizontal_error_m: position.horizontal_error_m,
        vertical_error_m: position.vertical_error_m,
        velocity_error_m_s: velocity_error.length(),
        attitude_error_rad: attitude.error_rad,
        horizontal_sigma_m: (pn + pe).sqrt(),
        vertical_sigma_m: pd.sqrt(),
        velocity_sigma_m_s: trace(&solution.velocity_cov).sqrt(),
        attitude_sigma_rad: solution
            .attitude_cov
            .map_or(f64::NAN, |cov| trace(&cov).sqrt()),
        position_nees: position.position_nees,
        attitude_nees: attitude.attitude_nees,
    })
}

fn trace(covariance: &SymmetricCov3) -> f64 {
    let (a, b, c) = covariance.diagonal();
    a + b + c
}

/// RMS, percentiles and last value of a sample.
pub(super) fn error_stats(values: &[f64]) -> ErrorStats {
    let count = values.len().max(1) as f64;
    let rms = (values.iter().map(|v| v * v).sum::<f64>() / count).sqrt();
    let spread = Percentiles::of(values).unwrap_or(Percentiles {
        p50: 0.0,
        p95: 0.0,
        max: 0.0,
    });
    ErrorStats {
        rms,
        p50: spread.p50,
        p95: spread.p95,
        max: spread.max,
        last: values.last().copied().unwrap_or(0.0),
    }
}

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let (sum, count) = values.fold((0.0, 0_u64), |(s, n), v| (s + v, n.wrapping_add(1)));
    sum / count.max(1) as f64
}

/// The summary of a set's series, with each scheduled interval and the
/// tallies inside it.
pub fn summarise(
    series: &[SeriesSample],
    intervals: &[(ScheduledInterval, SensorTallies)],
) -> SetSummary {
    let column = |f: fn(&SeriesSample) -> f64| -> Vec<f64> { series.iter().map(f).collect() };
    SetSummary {
        samples: series.len() as u64,
        horizontal_error_m: error_stats(&column(|s| s.horizontal_error_m)),
        vertical_error_m: error_stats(&column(|s| s.vertical_error_m)),
        velocity_error_m_s: error_stats(&column(|s| s.velocity_error_m_s)),
        attitude_error_rad: error_stats(&column(|s| s.attitude_error_rad)),
        mean_position_nees: mean(series.iter().map(|s| s.position_nees)),
        mean_attitude_nees: mean(series.iter().map(|s| s.attitude_nees)),
        unavailable: intervals
            .iter()
            .map(|(scheduled, tallies)| IntervalReport {
                sensor: scheduled.sensor,
                schedule: scheduled.schedule.clone(),
                reason: scheduled.interval.reason.clone(),
                start_s: scheduled.interval.start_s,
                end_s: scheduled.interval.end_s,
                errors: interval_errors(series, scheduled),
                tallies: tallies.clone(),
            })
            .collect(),
        consistency: consistency(series),
    }
}

/// Mean 3-DOF position NEES of a consistent filter lies in this band.
const NEES_BAND: [f64; 2] = [0.5, 6.0];
/// Attitude error beyond which an error-state filter's small-angle
/// linearisation no longer holds, radians.
const SMALL_ANGLE_LIMIT_RAD: f64 = 0.1;

/// The consistency verdict of a series.
fn consistency(series: &[SeriesSample]) -> ConsistencyNote {
    let nees = mean(series.iter().map(|s| s.position_nees));
    let attitude = series
        .iter()
        .map(|s| s.attitude_error_rad)
        .fold(0.0, f64::max);
    let [low, high] = NEES_BAND;
    let (verdict, statement) = if attitude > SMALL_ANGLE_LIMIT_RAD {
        (
            "diverged",
            format!(
                "filter diverged: attitude error reached {attitude:.2} rad, beyond the {SMALL_ANGLE_LIMIT_RAD} rad small-angle regime of an error-state filter; mean NEES {nees:.3}"
            ),
        )
    } else if nees > high {
        (
            "overconfident",
            format!("filter overconfident: mean NEES {nees:.3} against a band of {low}-{high}"),
        )
    } else if nees < low {
        (
            "conservative",
            format!("filter conservative: mean NEES {nees:.3} against a band of {low}-{high}"),
        )
    } else {
        (
            "consistent",
            format!("filter consistent: mean NEES {nees:.3} within the band {low}-{high}"),
        )
    };
    ConsistencyNote {
        verdict: verdict.into(),
        mean_position_nees: nees,
        band: NEES_BAND,
        max_attitude_error_rad: attitude,
        statement,
    }
}

/// The errors from the last sample at or before the interval's start to
/// the last sample inside it; `None` when the run does not cover it.
fn interval_errors(
    series: &[SeriesSample],
    scheduled: &ScheduledInterval,
) -> Option<IntervalErrors> {
    let interval = &scheduled.interval;
    let start = series.iter().rev().find(|s| s.time_s <= interval.start_s)?;
    let end = series.iter().rev().find(|s| interval.contains(s.time_s))?;
    let duration = end.time_s - start.time_s;
    Some(IntervalErrors {
        horizontal_error_at_start_m: start.horizontal_error_m,
        horizontal_error_at_end_m: end.horizontal_error_m,
        horizontal_sigma_at_end_m: end.horizontal_sigma_m,
        horizontal_growth_m_s: if duration > 0.0 {
            (end.horizontal_error_m - start.horizontal_error_m) / duration
        } else {
            0.0
        },
        mean_position_nees: mean(
            series
                .iter()
                .filter(|s| interval.contains(s.time_s))
                .map(|s| s.position_nees),
        ),
    })
}
