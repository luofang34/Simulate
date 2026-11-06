//! The corridor's guidance filter in image-driven runs: velocity aiding
//! carries the position, and only map anchors first in their cell correct it.

use std::collections::VecDeque;

use glam::DVec3;
use sim_core::{FusionEligibility, SimInstant, VisualFix};

/// The share of an anchor's variance its successors repeat: the attitude
/// reference's bias and the map's own error do not average away, so fusing
/// more anchors never claims better than this.
const CORRELATED_FRACTION: f64 = 0.5;

/// How far back fixes are matched against the dead-reckoned path; longer
/// than any tier's delay.
const HISTORY_S: f64 = 2.0;

/// Position from velocity aiding and independent anchors.
///
/// An anchor describes the eye at its exposure, so it is compared with the
/// dead-reckoned position then, and the correction carried to the present.
/// Warm tracks descend from anchors and shared-cell anchors repeat their
/// cell's map error, so neither is fused as independent evidence; they are
/// only counted. Each axis is filtered on its own.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct GuidanceFilter {
    position_enu_m: DVec3,
    variance_m2: DVec3,
    /// Cumulative dead-reckoned displacement by instant.
    path: VecDeque<(SimInstant, DVec3)>,
    travelled_m: DVec3,
    velocity_var_m2_s2: f64,
    pub(super) fused_anchors: u64,
    pub(super) refused_shared_cell: u64,
    pub(super) warm_fixes: u64,
}

impl GuidanceFilter {
    pub(super) fn new(seed_enu_m: DVec3, seed_sigma_m: f64, velocity_sigma_m_s: f64) -> Self {
        Self {
            position_enu_m: seed_enu_m,
            variance_m2: DVec3::splat(seed_sigma_m * seed_sigma_m),
            path: VecDeque::new(),
            travelled_m: DVec3::ZERO,
            velocity_var_m2_s2: velocity_sigma_m_s * velocity_sigma_m_s,
            fused_anchors: 0,
            refused_shared_cell: 0,
            warm_fixes: 0,
        }
    }

    pub(super) fn position_enu_m(&self) -> DVec3 {
        self.position_enu_m
    }

    /// One-sigma horizontal uncertainty, metres.
    pub(super) fn horizontal_sigma_m(&self) -> f64 {
        (0.5 * (self.variance_m2.x + self.variance_m2.y)).sqrt()
    }

    /// Dead-reckons `dt_s` on a measured velocity, ending at `now`.
    pub(super) fn propagate(&mut self, velocity_enu_m_s: DVec3, dt_s: f64, now: SimInstant) {
        let step = velocity_enu_m_s * dt_s;
        self.position_enu_m += step;
        self.travelled_m += step;
        self.variance_m2 += DVec3::splat(self.velocity_var_m2_s2 * dt_s * dt_s);
        self.path.push_back((now, self.travelled_m));
        while self
            .path
            .front()
            .is_some_and(|(at, _)| now.duration_since(*at).as_seconds() > HISTORY_S)
        {
            self.path.pop_front();
        }
    }

    /// Takes in delivered fixes.
    pub(super) fn ingest(&mut self, fixes: &[VisualFix]) {
        for fix in fixes {
            match fix.eligibility {
                FusionEligibility::CorrelatedTrack => {
                    self.warm_fixes = self.warm_fixes.wrapping_add(1);
                }
                FusionEligibility::SharedMapCell => {
                    self.refused_shared_cell = self.refused_shared_cell.wrapping_add(1);
                }
                FusionEligibility::IndependentAnchor => self.fuse(fix),
            }
        }
    }

    fn fuse(&mut self, fix: &VisualFix) {
        let since = self
            .path
            .iter()
            .find(|(at, _)| *at >= fix.captured_at)
            .map_or(DVec3::ZERO, |(_, travelled)| self.travelled_m - *travelled);
        let predicted = self.position_enu_m - since;
        let covariance = fix.estimate.covariance_enu_m2;
        let measured = DVec3::new(
            covariance.x_axis.x,
            covariance.y_axis.y,
            covariance.z_axis.z,
        );
        let gain = self.variance_m2 / (self.variance_m2 + measured);
        self.position_enu_m += gain * (fix.estimate.position_enu_m - predicted);
        self.variance_m2 =
            (self.variance_m2 * (DVec3::ONE - gain)).max(measured * CORRELATED_FRACTION);
        self.fused_anchors = self.fused_anchors.wrapping_add(1);
    }
}
