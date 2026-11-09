use std::collections::BTreeMap;

use super::{
    ComputeMeter, ComputeStats, Estimate, EvaluationSummary, MeansEvaluation, NavigationOutput,
    PositionScore, SimTruth, Truth, score_position,
};
use crate::NavigateBridgeError;

#[cfg(test)]
mod tests;

/// One independent way of knowing where the vehicle is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum NavMeans {
    /// Inertial or kinematic propagation alone.
    DeadReckoning,
    /// Visible-band terrain-relative navigation.
    Visual,
    /// Long-wave infrared terrain-relative navigation.
    Thermal,
    /// Star and celestial-body sightings.
    Celestial,
    /// Magnetic-anomaly or heading aiding.
    Magnetic,
    /// Barometric altitude.
    Barometric,
    /// Satellite navigation.
    Gnss,
}

impl NavMeans {
    /// Every means, in a fixed order.
    pub const ALL: [Self; 7] = [
        Self::DeadReckoning,
        Self::Visual,
        Self::Thermal,
        Self::Celestial,
        Self::Magnetic,
        Self::Barometric,
        Self::Gnss,
    ];

    const fn bit(self) -> u16 {
        1 << self as u16
    }
}

/// The means a fusion run is allowed to take observations from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct AblationSet(u16);

impl AblationSet {
    /// No means.
    pub const EMPTY: Self = Self(0);

    /// Every means.
    pub fn all() -> Self {
        NavMeans::ALL.into_iter().fold(Self::EMPTY, Self::with)
    }

    /// `means` alone.
    pub const fn only(means: NavMeans) -> Self {
        Self(means.bit())
    }

    /// This set and `means`.
    #[must_use]
    pub const fn with(self, means: NavMeans) -> Self {
        Self(self.0 | means.bit())
    }

    /// This set without `means`.
    #[must_use]
    pub const fn without(self, means: NavMeans) -> Self {
        Self(self.0 & !means.bit())
    }

    /// Whether fusion may take observations from `means`.
    pub const fn contains(self, means: NavMeans) -> bool {
        self.0 & means.bit() != 0
    }

    /// The means in the set, in [`NavMeans::ALL`] order.
    pub fn means(self) -> impl Iterator<Item = NavMeans> {
        NavMeans::ALL.into_iter().filter(move |m| self.contains(*m))
    }
}

/// What produced an output: one means on its own, or a fusion of a set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum NavigationOrigin {
    /// One means without fusion.
    Standalone(NavMeans),
    /// A filter fed only from the set's means.
    Fused(AblationSet),
}

/// Scores of every origin in a run, for comparing each means alone against
/// fusions with and without it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AblationEvaluation {
    origins: BTreeMap<NavigationOrigin, MeansEvaluation>,
}

impl AblationEvaluation {
    /// Counts one instant at which `origin` was due to publish.
    pub fn expect(&mut self, origin: NavigationOrigin) {
        self.origins.entry(origin).or_default().expect();
    }

    /// Scores `output` against `truth` under the output's origin.
    pub fn record(
        &mut self,
        truth: &Truth<SimTruth>,
        output: &Estimate<NavigationOutput>,
    ) -> Result<PositionScore, NavigateBridgeError> {
        let score = score_position(truth, output)?;
        self.origins
            .entry(output.get().origin)
            .or_default()
            .record(score);
        Ok(score)
    }

    /// The scores of one origin, if any were expected or recorded.
    pub fn origin(&self, origin: NavigationOrigin) -> Option<&MeansEvaluation> {
        self.origins.get(&origin)
    }

    /// The run summarised, with each origin's compute cost from `meter`.
    pub fn summary(&self, meter: &ComputeMeter) -> AblationSummary {
        let mut summary = AblationSummary::default();
        for (origin, evaluation) in &self.origins {
            let entry = (evaluation.summary(), meter.stats(*origin));
            match origin {
                NavigationOrigin::Standalone(means) => {
                    summary.standalone.insert(*means, entry);
                }
                NavigationOrigin::Fused(set) => {
                    summary.fused.insert(*set, entry);
                }
            }
        }
        summary
    }
}

/// Every origin's accuracy and cost over a run.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AblationSummary {
    /// Each means on its own.
    pub standalone: BTreeMap<NavMeans, (EvaluationSummary, ComputeStats)>,
    /// Each fusion, by the means it was allowed.
    pub fused: BTreeMap<AblationSet, (EvaluationSummary, ComputeStats)>,
}
