use super::{Estimate, NavigationOutput, SimTruth, Truth};
use crate::NavigateBridgeError;
use crate::truth::compare_position_with_truth;

#[cfg(test)]
mod tests;

/// One output scored against the truth at the instant it describes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionScore {
    /// Truth minus estimate in the estimate's east, north, up axes, metres.
    pub error_enu_m: [f64; 3],
    /// Horizontal position error, metres.
    pub horizontal_error_m: f64,
    /// Vertical position error, metres.
    pub vertical_error_m: f64,
    /// Position normalised estimation error squared, 3 degrees of freedom.
    pub position_nees: f64,
    /// Master-clock latency from the instant described to delivery, seconds.
    pub latency_s: f64,
}

/// Scores `output` against `truth`, which must be taken at the instant the
/// output describes: a delayed output is scored for where the vehicle was,
/// and its delay is reported as latency rather than as error. Truth from
/// another instant is refused.
pub fn score_position(
    truth: &Truth<SimTruth>,
    output: &Estimate<NavigationOutput>,
) -> Result<PositionScore, NavigateBridgeError> {
    let output = output.get();
    if truth.get().instant != output.valid_at {
        return Err(NavigateBridgeError::TruthInstantMismatch {
            truth_at: truth.get().instant,
            valid_at: output.valid_at,
        });
    }
    let (ned, comparison) = compare_position_with_truth(
        output.position,
        &output.position_cov,
        truth.get().vehicle.position_ecef,
    )?;
    Ok(PositionScore {
        error_enu_m: [ned.y, ned.x, -ned.z],
        horizontal_error_m: comparison.horizontal_error_m,
        vertical_error_m: comparison.vertical_error_m,
        position_nees: comparison.position_nees,
        latency_s: output.latency().as_seconds(),
    })
}

/// The median, 95th percentile, and largest of a sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Percentiles {
    /// Median.
    pub p50: f64,
    /// 95th percentile.
    pub p95: f64,
    /// Largest value.
    pub max: f64,
}

impl Percentiles {
    /// Nearest-rank percentiles of `values`, or `None` for an empty sample.
    pub fn of(values: &[f64]) -> Option<Self> {
        let mut sorted = values.to_vec();
        sorted.sort_by(f64::total_cmp);
        let rank = |fraction: f64| {
            let index = (fraction * sorted.len() as f64).ceil() as usize;
            sorted.get(index.saturating_sub(1)).copied()
        };
        Some(Self {
            p50: rank(0.5)?,
            p95: rank(0.95)?,
            max: sorted.last().copied()?,
        })
    }
}

/// The scores of one origin's outputs across a run.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MeansEvaluation {
    expected: u64,
    scores: Vec<PositionScore>,
}

impl MeansEvaluation {
    /// Counts one instant at which an output was due, for availability.
    pub fn expect(&mut self) {
        self.expected = self.expected.wrapping_add(1);
    }

    /// Adds one scored output.
    pub fn record(&mut self, score: PositionScore) {
        self.scores.push(score);
    }

    /// Every score so far, in recording order.
    pub fn scores(&self) -> &[PositionScore] {
        &self.scores
    }

    /// The run so far, summarised.
    pub fn summary(&self) -> EvaluationSummary {
        let column =
            |f: fn(&PositionScore) -> f64| -> Vec<f64> { self.scores.iter().map(f).collect() };
        let nees = column(|s| s.position_nees);
        let delivered = self.scores.len() as u64;
        EvaluationSummary {
            expected: self.expected,
            delivered,
            availability: (self.expected > 0)
                .then(|| (delivered as f64 / self.expected as f64).min(1.0)),
            horizontal_error_m: Percentiles::of(&column(|s| s.horizontal_error_m)),
            vertical_error_m: Percentiles::of(&column(|s| s.vertical_error_m)),
            mean_nees: (!nees.is_empty()).then(|| nees.iter().sum::<f64>() / nees.len() as f64),
            nees: Percentiles::of(&nees),
            latency_s: Percentiles::of(&column(|s| s.latency_s)),
        }
    }
}

/// How one origin did over a run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EvaluationSummary {
    /// Instants at which an output was due.
    pub expected: u64,
    /// Outputs scored.
    pub delivered: u64,
    /// Delivered over expected, capped at one; `None` when nothing was due.
    pub availability: Option<f64>,
    /// Horizontal error, metres.
    pub horizontal_error_m: Option<Percentiles>,
    /// Vertical error, metres.
    pub vertical_error_m: Option<Percentiles>,
    /// Mean position NEES: consistent outputs average 3.
    pub mean_nees: Option<f64>,
    /// Position NEES spread.
    pub nees: Option<Percentiles>,
    /// Master-clock latency, seconds.
    pub latency_s: Option<Percentiles>,
}
