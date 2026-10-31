use std::collections::{BTreeSet, VecDeque};

use glam::{DMat3, DVec3};

use crate::{CameraFrameCapture, RateAdapter, SimInstant, TICKS_PER_SECOND, TickSpan};

#[cfg(test)]
mod tests;

const WARM_RATE_HZ: u64 = 30;
const ANCHOR_RATE_HZ: u64 = 2;
const TICKS_PER_MILLISECOND: u64 = TICKS_PER_SECOND / 1_000;
const WARM_DELAY_MS: u64 = 35;
const ANCHOR_DELAY_MS: u64 = 300;
const MAP_CELL_M: f64 = 500.0;
// Every default must land on whole master-clock ticks, or delivery times
// would be rounded rather than exact.
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(WARM_RATE_HZ));
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(ANCHOR_RATE_HZ));
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(1_000));

/// How the fusion filter may use a fix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum FusionEligibility {
    /// A warm-track pose, correlated with the anchors it descends from.
    CorrelatedTrack,
    /// The first map anchor in its cell, independent of earlier fixes.
    IndependentAnchor,
    /// A map anchor in a visited cell, sharing that cell's map error.
    SharedMapCell,
}

impl FusionEligibility {
    /// Whether an EKF may treat this fix as independent evidence.
    pub const fn independent_ekf_fix(self) -> bool {
        matches!(self, Self::IndependentAnchor)
    }
}

/// A square of the catalog map that shares one error bias.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MapCell {
    /// Column, counted east from the anchor.
    pub cell_x: i64,
    /// Row, counted north from the anchor.
    pub cell_y: i64,
}

impl MapCell {
    /// The cell holding an ENU position, for cells `cell_size_m` wide.
    pub fn containing(position_enu_m: DVec3, cell_size_m: f64) -> Self {
        Self {
            cell_x: (position_enu_m.x / cell_size_m).floor() as i64,
            cell_y: (position_enu_m.y / cell_size_m).floor() as i64,
        }
    }
}

/// A position estimate from the visual stack for one camera frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisualEstimate {
    /// Estimated eye position in the anchor frame, metres.
    pub position_enu_m: DVec3,
    /// Claimed error covariance of that position, m².
    pub covariance_enu_m2: DMat3,
}

/// One camera frame offered to the harness, with what each tier made of it. A
/// tier that found nothing gives `None`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisualFrame {
    /// The camera truth.
    pub capture: CameraFrameCapture,
    /// Warm-track result.
    pub warm_track: Option<VisualEstimate>,
    /// Corridor map anchor result.
    pub corridor_anchor: Option<VisualEstimate>,
}

/// A visual fix as the fusion filter receives it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisualFix {
    /// The exposure the fix describes.
    pub captured_at: SimInstant,
    /// When the fix reaches fusion: `captured_at` plus the tier's delay.
    pub delivered_at: SimInstant,
    /// How fusion may use it.
    pub eligibility: FusionEligibility,
    /// The map cell of an anchor; a warm-track fix has none.
    pub map_cell: Option<MapCell>,
    /// The estimate.
    pub estimate: VisualEstimate,
    /// True eye position at the exposure, metres.
    pub true_eye_enu_m: DVec3,
    /// Normalised estimation error squared of position, 3 degrees of freedom:
    /// consistent estimates average 3.
    pub position_nees: f64,
}

/// Running position NEES over the fixes of one eligibility.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct NeesSummary {
    /// Fixes counted.
    pub count: u64,
    /// Sum of their NEES.
    pub sum: f64,
    /// Largest NEES.
    pub max: f64,
}

impl NeesSummary {
    /// Mean NEES, or `None` before any fix.
    pub fn mean(&self) -> Option<f64> {
        (self.count > 0).then(|| self.sum / self.count as f64)
    }

    fn record(&mut self, nees: f64) {
        self.count = self.count.wrapping_add(1);
        self.sum += nees;
        self.max = self.max.max(nees);
    }
}

/// Fixed setup of a [`TwoTierVisualHarness`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwoTierConfig {
    /// Warm-track loop period.
    pub warm_period: TickSpan,
    /// Warm-track processing delay.
    pub warm_delay: TickSpan,
    /// Corridor anchor loop period.
    pub anchor_period: TickSpan,
    /// Corridor anchor processing delay.
    pub anchor_delay: TickSpan,
    /// Map cell width, metres.
    pub map_cell_m: f64,
}

impl Default for TwoTierConfig {
    /// `Navigate`'s nominal loops: 30 Hz warm tracking with 35 ms of latency,
    /// 2 Hz anchors with 300 ms, and 500 m map cells.
    fn default() -> Self {
        Self {
            warm_period: TickSpan::from_ticks(TICKS_PER_SECOND / WARM_RATE_HZ),
            warm_delay: TickSpan::from_ticks(WARM_DELAY_MS * TICKS_PER_MILLISECOND),
            anchor_period: TickSpan::from_ticks(TICKS_PER_SECOND / ANCHOR_RATE_HZ),
            anchor_delay: TickSpan::from_ticks(ANCHOR_DELAY_MS * TICKS_PER_MILLISECOND),
            map_cell_m: MAP_CELL_M,
        }
    }
}

/// Why the harness refused a frame.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum VisualHarnessError {
    /// A period is zero or the cell width is not a positive number.
    #[error("invalid harness configuration: {detail}")]
    InvalidConfig {
        /// What was wrong.
        detail: String,
    },
    /// Frames must arrive in capture order.
    #[error("frame captured at {captured_at:?} does not follow {previous:?}")]
    OutOfOrderFrame {
        /// The offending frame's capture instant.
        captured_at: SimInstant,
        /// The previous frame's capture instant.
        previous: SimInstant,
    },
    /// An estimate has no usable NEES: a non-finite position, or a covariance
    /// that is not positive definite.
    #[error("frame captured at {captured_at:?} has an unusable estimate: {estimate:?}")]
    UnusableEstimate {
        /// The frame's capture instant.
        captured_at: SimInstant,
        /// The offending estimate.
        estimate: VisualEstimate,
    },
}

/// `eᵀ P⁻¹ e`, or `None` unless `P` is positive definite by its leading minors.
fn position_nees(estimate: &VisualEstimate, truth_m: DVec3) -> Option<f64> {
    let p = estimate.covariance_enu_m2;
    let error = estimate.position_enu_m - truth_m;
    let leading_2x2 = p.x_axis.x * p.y_axis.y - p.y_axis.x * p.x_axis.y;
    let definite = p.is_finite() && p.x_axis.x > 0.0 && leading_2x2 > 0.0 && p.determinant() > 0.0;
    let nees = error.dot(p.inverse() * error);
    (definite && nees.is_finite()).then_some(nees)
}

/// Compares `Navigate`'s two-tier visual positioning with exact truth on the
/// master clock.
///
/// Frames go in at their capture instants. A warm-track loop and a slower
/// corridor anchor loop each pick the frames they process by their own
/// [`RateAdapter`], the first frame going to both, and each fix comes back
/// after its tier's constant delay. The NEES compares an estimate with the
/// truth at its exposure, not at delivery, since that is what it describes.
///
/// As in `Navigate`, an anchor's map cell comes from its estimated position,
/// because a filter has nothing else to key on, and a first anchor claims its
/// cell whether or not it was accurate. A tier that found nothing claims
/// nothing. The whole harness is plain data; a clone is its snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct TwoTierVisualHarness {
    config: TwoTierConfig,
    warm_loop: RateAdapter,
    anchor_loop: RateAdapter,
    previous_capture: Option<SimInstant>,
    visited_cells: BTreeSet<MapCell>,
    warm_in_flight: VecDeque<VisualFix>,
    anchors_in_flight: VecDeque<VisualFix>,
    correlated_nees: NeesSummary,
    independent_nees: NeesSummary,
    shared_nees: NeesSummary,
}

impl TwoTierVisualHarness {
    /// A harness with nothing in flight and no cell visited.
    pub fn new(config: TwoTierConfig) -> Result<Self, VisualHarnessError> {
        let periods_usable =
            config.warm_period != TickSpan::ZERO && config.anchor_period != TickSpan::ZERO;
        if !periods_usable || !config.map_cell_m.is_finite() || config.map_cell_m <= 0.0 {
            return Err(VisualHarnessError::InvalidConfig {
                detail: format!("{config:?}"),
            });
        }
        Ok(Self {
            config,
            warm_loop: RateAdapter::new(config.warm_period),
            anchor_loop: RateAdapter::new(config.anchor_period),
            previous_capture: None,
            visited_cells: BTreeSet::new(),
            warm_in_flight: VecDeque::new(),
            anchors_in_flight: VecDeque::new(),
            correlated_nees: NeesSummary::default(),
            independent_nees: NeesSummary::default(),
            shared_nees: NeesSummary::default(),
        })
    }

    /// Offers one frame to both loops. A refused frame changes nothing.
    pub fn submit(&mut self, frame: &VisualFrame) -> Result<(), VisualHarnessError> {
        let captured_at = frame.capture.timestamp;
        if let Some(previous) = self.previous_capture
            && captured_at <= previous
        {
            return Err(VisualHarnessError::OutOfOrderFrame {
                captured_at,
                previous,
            });
        }
        let truth = DVec3::from_array(frame.capture.eye_enu);
        let nees_of = |estimate: VisualEstimate| {
            position_nees(&estimate, truth)
                .map(|nees| (estimate, nees))
                .ok_or(VisualHarnessError::UnusableEstimate {
                    captured_at,
                    estimate,
                })
        };
        let warm = frame.warm_track.map(nees_of).transpose()?;
        let anchor = frame.corridor_anchor.map(nees_of).transpose()?;

        let elapsed = self
            .previous_capture
            .map(|previous| captured_at.duration_since(previous));
        self.previous_capture = Some(captured_at);
        let warm_due = elapsed.is_none_or(|span| self.warm_loop.steps_for_u128(span) > 0);
        let anchor_due = elapsed.is_none_or(|span| self.anchor_loop.steps_for_u128(span) > 0);

        if let (true, Some((estimate, position_nees))) = (warm_due, warm) {
            self.warm_in_flight.push_back(VisualFix {
                captured_at,
                delivered_at: captured_at.advanced(self.config.warm_delay),
                eligibility: FusionEligibility::CorrelatedTrack,
                map_cell: None,
                estimate,
                true_eye_enu_m: truth,
                position_nees,
            });
        }
        if let (true, Some((estimate, position_nees))) = (anchor_due, anchor) {
            let cell = MapCell::containing(estimate.position_enu_m, self.config.map_cell_m);
            let eligibility = if self.visited_cells.insert(cell) {
                FusionEligibility::IndependentAnchor
            } else {
                FusionEligibility::SharedMapCell
            };
            self.anchors_in_flight.push_back(VisualFix {
                captured_at,
                delivered_at: captured_at.advanced(self.config.anchor_delay),
                eligibility,
                map_cell: Some(cell),
                estimate,
                true_eye_enu_m: truth,
                position_nees,
            });
        }
        Ok(())
    }

    /// Hands over every fix due by `now`, in delivery order, warm tracks first
    /// within one instant, and adds them to the NEES summaries.
    pub fn take_due(&mut self, now: SimInstant) -> Vec<VisualFix> {
        let mut due = Vec::new();
        loop {
            let warm = self.warm_in_flight.front().map(|fix| fix.delivered_at);
            let anchor = self.anchors_in_flight.front().map(|fix| fix.delivered_at);
            let next = match (warm, anchor) {
                (Some(w), Some(a)) if w <= a && w <= now => self.warm_in_flight.pop_front(),
                (Some(_), Some(a)) if a <= now => self.anchors_in_flight.pop_front(),
                (Some(w), None) if w <= now => self.warm_in_flight.pop_front(),
                (None, Some(a)) if a <= now => self.anchors_in_flight.pop_front(),
                _ => None,
            };
            let Some(fix) = next else {
                return due;
            };
            self.summary_mut(fix.eligibility).record(fix.position_nees);
            due.push(fix);
        }
    }

    /// Position NEES so far over delivered fixes of `eligibility`.
    pub fn nees_summary(&self, eligibility: FusionEligibility) -> NeesSummary {
        match eligibility {
            FusionEligibility::CorrelatedTrack => self.correlated_nees,
            FusionEligibility::IndependentAnchor => self.independent_nees,
            FusionEligibility::SharedMapCell => self.shared_nees,
        }
    }

    fn summary_mut(&mut self, eligibility: FusionEligibility) -> &mut NeesSummary {
        match eligibility {
            FusionEligibility::CorrelatedTrack => &mut self.correlated_nees,
            FusionEligibility::IndependentAnchor => &mut self.independent_nees,
            FusionEligibility::SharedMapCell => &mut self.shared_nees,
        }
    }
}
