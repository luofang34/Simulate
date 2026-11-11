//! The result's side-by-side table: one row per set, and what each aided
//! set gains over the sets with fewer of its means.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::availability::SensorKind;
use super::replay::SensorTallies;
use super::report::{SeriesSample, SetReport};
use crate::NavMeans;

/// Every set side by side.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComparisonTable {
    /// The horizontal error the time-above column counts, metres.
    pub horizontal_error_threshold_m: f64,
    /// How a row's baseline is chosen.
    pub baseline_rule: String,
    /// One row per set, in request order.
    pub rows: Vec<ComparisonRow>,
}

/// One set's row.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComparisonRow {
    /// The set's name.
    pub set: String,
    /// Its means.
    pub means: Vec<NavMeans>,
    /// The schedule withholding each of its sensors.
    pub availability: BTreeMap<SensorKind, String>,
    /// Horizontal RMS error, metres.
    pub horizontal_rms_m: f64,
    /// Horizontal 95th percentile error, metres.
    pub horizontal_p95_m: f64,
    /// Horizontal error at the last sample, metres.
    pub horizontal_final_m: f64,
    /// Route time with the horizontal error above the threshold, seconds,
    /// each sample standing for the time since the one before.
    pub time_above_threshold_s: f64,
    /// Readings and observations per sensor.
    pub tallies: SensorTallies,
    /// The consistency verdict.
    pub consistency: String,
    /// The baseline the set is compared against, when the run has one.
    pub versus: Option<Baseline>,
}

/// A row's baseline and the difference from it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Baseline {
    /// The baseline set.
    pub set: String,
    /// Its horizontal RMS error, metres.
    pub horizontal_rms_m: f64,
    /// This set's horizontal RMS minus the baseline's, metres: negative is
    /// better.
    pub horizontal_rms_change_m: f64,
    /// This set's horizontal RMS over the baseline's; absent when the
    /// baseline's is zero.
    pub horizontal_rms_ratio: Option<f64>,
}

/// How [`ComparisonTable::rows`] choose their baselines.
const BASELINE_RULE: &str = "a set fusing two or more aiding means is compared with the best (lowest horizontal RMS) set aided by exactly one of them; a set aided by one means with the inertial core alone; a candidate qualifies only if it withholds each sensor it shares with the set by the same schedule";

/// The table of `reports`, counting time above `threshold_m`.
pub fn comparison(reports: &[SetReport], threshold_m: f64) -> ComparisonTable {
    let rows = reports
        .iter()
        .map(|report| ComparisonRow {
            set: report.name.clone(),
            means: report.means.clone(),
            availability: report.availability.clone(),
            horizontal_rms_m: report.summary.horizontal_error_m.rms,
            horizontal_p95_m: report.summary.horizontal_error_m.p95,
            horizontal_final_m: report.summary.horizontal_error_m.last,
            time_above_threshold_s: time_above(&report.series, threshold_m),
            tallies: report.observations.clone(),
            consistency: report.summary.consistency.verdict.clone(),
            versus: baseline(report, reports),
        })
        .collect();
    ComparisonTable {
        horizontal_error_threshold_m: threshold_m,
        baseline_rule: BASELINE_RULE.into(),
        rows,
    }
}

fn time_above(series: &[SeriesSample], threshold_m: f64) -> f64 {
    let mut previous_s = 0.0;
    let mut above_s = 0.0;
    for sample in series {
        if sample.horizontal_error_m > threshold_m {
            above_s += sample.time_s - previous_s;
        }
        previous_s = sample.time_s;
    }
    above_s
}

fn aiding(report: &SetReport) -> Vec<NavMeans> {
    report
        .means
        .iter()
        .copied()
        .filter(|m| *m != NavMeans::DeadReckoning)
        .collect()
}

fn schedule_of(report: &SetReport, means: NavMeans) -> Option<&String> {
    SensorKind::ALL
        .into_iter()
        .find(|kind| kind.means() == means)
        .and_then(|kind| report.availability.get(&kind))
}

fn baseline(report: &SetReport, reports: &[SetReport]) -> Option<Baseline> {
    let mine = aiding(report);
    let wanted = match mine.len() {
        0 => return None,
        1 => 0,
        _ => 1,
    };
    let best = reports
        .iter()
        .filter(|other| {
            let theirs = aiding(other);
            theirs.len() == wanted
                && theirs
                    .iter()
                    .all(|m| mine.contains(m) && schedule_of(other, *m) == schedule_of(report, *m))
        })
        .min_by(|a, b| {
            let rms = |r: &SetReport| r.summary.horizontal_error_m.rms;
            rms(a).total_cmp(&rms(b))
        })?;
    let own = report.summary.horizontal_error_m.rms;
    let theirs = best.summary.horizontal_error_m.rms;
    Some(Baseline {
        set: best.name.clone(),
        horizontal_rms_m: theirs,
        horizontal_rms_change_m: own - theirs,
        horizontal_rms_ratio: (theirs > 0.0).then(|| own / theirs),
    })
}
