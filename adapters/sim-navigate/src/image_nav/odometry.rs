//! The warm track: Shi-Tomasi corners of a keyframe ground patch followed
//! into each new patch by ZNCC, their median shift chained onto the
//! keyframe's position.

use glam::{DMat3, DVec2, DVec3};
use sim_core::VisualEstimate;

use super::ImageCorridorEstimator;
use super::patch::GroundPatch;
use super::zncc::{Window, best_match};

/// Template side, cells.
const TEMPLATE: usize = 9;
/// Search radius around the predicted shift, cells.
const SEARCH: i64 = 3;
/// Corners keep this far from the keyframe's edge, cells, so a template
/// still lies inside the new patch after a full keyframe hop.
const CORNER_MARGIN: usize = 14;
/// Corners are picked one per block of this many cells a side.
const CORNER_BLOCK: usize = 8;
const MAX_CORNERS: usize = 16;
const MIN_TRACKED: usize = 4;
const MIN_TRACK_SCORE: f64 = 0.7;
/// Motion after which the current patch becomes the keyframe, metres.
const KEYFRAME_HOP_M: f64 = 60.0;
/// Doubt added when the track is lost and restarts from the prior, metres.
const LOST_SIGMA_M: f64 = 30.0;

/// A patch the warm track measures motion from.
#[derive(Debug, Clone)]
pub(super) struct Keyframe {
    patch: GroundPatch,
    /// The patch centre's offset from the point below the camera, metres.
    centre_m: DVec2,
    corners: Vec<(usize, usize)>,
    position_enu_m: DVec3,
    horizontal_var_m2: f64,
    /// Feature shift in the last two tracked frames, cells.
    last_shift: DVec2,
    previous_shift: DVec2,
}

impl Keyframe {
    fn new(
        patch: GroundPatch,
        centre_m: DVec2,
        position_enu_m: DVec3,
        horizontal_var_m2: f64,
    ) -> Self {
        Self {
            corners: corners(&patch),
            centre_m,
            patch,
            position_enu_m,
            horizontal_var_m2,
            last_shift: DVec2::ZERO,
            previous_shift: DVec2::ZERO,
        }
    }

    /// How far the camera moved from the keyframe to `observed`, centred
    /// `centre_m` from the point below it, metres east and north: the
    /// patch centre's motion, the negated median shift of the tracked
    /// corners, less the change in where the patch is centred.
    fn displacement(&mut self, observed: &GroundPatch, centre_m: DVec2) -> Option<DVec2> {
        let predicted = self.last_shift * 2.0 - self.previous_shift;
        let half = (TEMPLATE / 2) as i64;
        let mut shifts: Vec<DVec2> = self
            .corners
            .iter()
            .filter_map(|&(ci, cj)| {
                let window = Window {
                    i: ci - TEMPLATE / 2,
                    j: cj - TEMPLATE / 2,
                    size: TEMPLATE,
                };
                let centre = (
                    ci as i64 - half + predicted.x.round() as i64,
                    cj as i64 - half + predicted.y.round() as i64,
                );
                let found = best_match(&self.patch, window, observed, centre, SEARCH)?;
                (found.score >= MIN_TRACK_SCORE)
                    .then(|| found.top_left - DVec2::new(window.i as f64, window.j as f64))
            })
            .collect();
        if shifts.len() < MIN_TRACKED {
            return None;
        }
        let shift = DVec2::new(
            median(shifts.iter().map(|s| s.x).collect())?,
            median(shifts.iter_mut().map(|s| s.y).collect())?,
        );
        self.previous_shift = self.last_shift;
        self.last_shift = shift;
        // Ground features move opposite to the camera; rows count south.
        Some(DVec2::new(-shift.x, shift.y) * self.patch.gsd_m() - (centre_m - self.centre_m))
    }
}

fn median(mut values: Vec<f64>) -> Option<f64> {
    values.sort_by(f64::total_cmp);
    let middle = values.len() / 2;
    if values.len().is_multiple_of(2) {
        Some(0.5 * (values.get(middle.checked_sub(1)?)? + values.get(middle)?))
    } else {
        values.get(middle).copied()
    }
}

/// The strongest corner in each block of the patch's interior, strongest
/// first.
fn corners(patch: &GroundPatch) -> Vec<(usize, usize)> {
    let end = patch.size().saturating_sub(CORNER_MARGIN);
    let mut found: Vec<(f64, usize, usize)> = Vec::new();
    for block_j in (CORNER_MARGIN..end).step_by(CORNER_BLOCK) {
        for block_i in (CORNER_MARGIN..end).step_by(CORNER_BLOCK) {
            let best = (block_j..(block_j + CORNER_BLOCK).min(end))
                .flat_map(|j| (block_i..(block_i + CORNER_BLOCK).min(end)).map(move |i| (i, j)))
                .filter_map(|(i, j)| patch.corner_strength(i, j).map(|s| (s, i, j)))
                .max_by(|a, b| a.0.total_cmp(&b.0));
            found.extend(best.filter(|(strength, _, _)| *strength > 1.0));
        }
    }
    found.sort_by(|a, b| b.0.total_cmp(&a.0));
    found
        .into_iter()
        .take(MAX_CORNERS)
        .map(|(_, i, j)| (i, j))
        .collect()
}

impl ImageCorridorEstimator {
    fn warm_estimate(&self, position_enu_m: DVec3, horizontal_var_m2: f64) -> VisualEstimate {
        let vertical = self.anchor_sigmas(DVec2::ZERO, 1.0).1;
        VisualEstimate {
            position_enu_m,
            covariance_enu_m2: DMat3::from_diagonal(DVec3::new(
                horizontal_var_m2,
                horizontal_var_m2,
                vertical,
            )),
        }
    }

    /// One warm-track step: restarts on a fresh anchor, otherwise chains the
    /// keyframe's motion. Nothing is reported before the first anchor, when
    /// the track would only restate the flight plan.
    pub(super) fn track(
        &mut self,
        observed: GroundPatch,
        centre_m: DVec2,
        anchor: Option<VisualEstimate>,
        agl_m: f64,
    ) -> Option<VisualEstimate> {
        if let Some(fix) = anchor {
            let var = fix.covariance_enu_m2.x_axis.x;
            self.keyframe = Some(Keyframe::new(observed, centre_m, fix.position_enu_m, var));
            return Some(fix);
        }
        let Some(keyframe) = self.keyframe.as_mut() else {
            self.keyframe = Some(Keyframe::new(
                observed,
                centre_m,
                self.prior_enu_m,
                self.prior_horizontal_var_m2,
            ));
            return None;
        };
        let Some(moved_m) = keyframe.displacement(&observed, centre_m) else {
            self.prior_horizontal_var_m2 += LOST_SIGMA_M * LOST_SIGMA_M;
            self.keyframe = Some(Keyframe::new(
                observed,
                centre_m,
                self.prior_enu_m,
                self.prior_horizontal_var_m2,
            ));
            return None;
        };
        let (from, from_var) = (keyframe.position_enu_m, keyframe.horizontal_var_m2);
        let gsd = self.config.gsd_m;
        let scale_error = moved_m.length() * self.config.agl_sigma_m / agl_m;
        let var = from_var + (0.15 * gsd).powi(2) + scale_error.powi(2) + 0.25;
        let position = self.eye_at(&self.enu_to_geodetic(from), moved_m, agl_m);
        self.prior_enu_m = position;
        self.prior_horizontal_var_m2 = var;
        if moved_m.length() > KEYFRAME_HOP_M {
            self.keyframe = Some(Keyframe::new(observed, centre_m, position, var));
        }
        self.anchored.then(|| self.warm_estimate(position, var))
    }
}
