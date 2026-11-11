//! A visual set's latest terrain match as the page shows it: the reference
//! view and the camera frame the navigator compared, the correspondences it
//! found, what it made of them, and the fix it offered, copied out of the
//! navigator's own record so the page can draw them after the flight moves
//! on.

use std::collections::BTreeMap;

use sim_navigate::trial::{TerrainMatch, outcome_key};

#[cfg(test)]
mod tests;

/// A grey image: row-major, top row first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GreyImage {
    /// Width, pixels.
    pub width: u32,
    /// Height, pixels.
    pub height: u32,
    /// One grey level per pixel.
    pub pixels: Vec<u8>,
}

/// One correspondence, in the pixels of the two images.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchLine {
    /// Where it lies in the reference view.
    pub reference_px: [f64; 2],
    /// Where it lies in the frame.
    pub frame_px: [f64; 2],
    /// Whether the fit kept it.
    pub inlier: bool,
}

/// How well the matched pose was solved.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolvedFit {
    /// Matches with a depth from the reference view.
    pub depth_matches: usize,
    /// Matches the fit kept.
    pub inliers: usize,
    /// Image cells the inliers spread over.
    pub spatial_support: usize,
    /// Root-mean-square reprojection error of the inliers, pixels.
    pub reprojection_rms_px: f64,
}

/// The fix the match offered the filter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OfferedPosition {
    /// Latitude and longitude, degrees.
    pub at_deg: (f64, f64),
    /// Height above the ellipsoid, metres.
    pub height_m: f64,
    /// Horizontal one-sigma it was offered with, `sqrt(σN² + σE²)`, metres.
    pub sigma_horizontal_m: f64,
}

/// A set's latest terrain match.
#[derive(Debug, Clone, PartialEq)]
pub struct MatchShot {
    /// The set.
    pub set: String,
    /// Route time of the frame, seconds.
    pub at_s: f64,
    /// What the navigator did with the frame, as the tally keys it.
    pub outcome: String,
    /// Where the reference view came from: `render` or `keyframe`.
    pub path: Option<String>,
    /// The reference view, at the working size.
    pub reference: Option<GreyImage>,
    /// The frame as matched, at the working size.
    pub frame: Option<GreyImage>,
    /// The correspondences.
    pub lines: Vec<MatchLine>,
    /// The solved pose's quality, when the fit was accepted.
    pub solved: Option<SolvedFit>,
    /// The fix offered, when the match gave one.
    pub fix: Option<OfferedPosition>,
}

impl MatchShot {
    /// A copy of `found`, `set`'s latest match.
    pub fn of(set: &str, found: &TerrainMatch) -> Self {
        let grey = |luma: sim_navigate::trial::Luma<'_>| GreyImage {
            width: luma.width,
            height: luma.height,
            pixels: luma.pixels.to_vec(),
        };
        Self {
            set: set.to_owned(),
            at_s: found.acquired_at().as_nanos() as f64 * 1e-9,
            outcome: outcome_key(found.outcome(), found.path().is_some()),
            path: found.path().map(str::to_owned),
            reference: found.reference().map(grey),
            frame: found.frame().map(grey),
            lines: found
                .pairs()
                .iter()
                .map(|pair| MatchLine {
                    reference_px: pair.reference_px,
                    frame_px: pair.frame_px,
                    inlier: pair.inlier,
                })
                .collect(),
            solved: found.solved().map(|solved| SolvedFit {
                depth_matches: solved.depth_matches,
                inliers: solved.inliers,
                spatial_support: solved.spatial_support,
                reprojection_rms_px: solved.reprojection_rms_px,
            }),
            fix: found.fix().map(|fix| {
                let (nn, ee, _) = fix.covariance_ned.diagonal();
                OfferedPosition {
                    at_deg: (
                        fix.position.latitude_rad.to_degrees(),
                        fix.position.longitude_rad.to_degrees(),
                    ),
                    height_m: fix.position.altitude_m,
                    sigma_horizontal_m: (nn + ee).max(0.0).sqrt(),
                }
            }),
        }
    }
}

/// Each visual set's latest terrain match, and its latest one with images
/// to compare: a frame skipped before the match leaves only its outcome, and
/// the page keeps showing the last comparison beside it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MatchLog {
    latest: BTreeMap<String, MatchShot>,
    compared: BTreeMap<String, MatchShot>,
}

impl MatchLog {
    /// Takes `shot`, its set's newest match.
    pub(super) fn take(&mut self, shot: MatchShot) {
        if shot.reference.is_some() && shot.frame.is_some() {
            self.compared.insert(shot.set.clone(), shot.clone());
        }
        self.latest.insert(shot.set.clone(), shot);
    }

    /// `set`'s latest match.
    pub fn latest(&self, set: &str) -> Option<&MatchShot> {
        self.latest.get(set)
    }

    /// `set`'s latest match with a reference view and a frame to compare.
    pub fn compared(&self, set: &str) -> Option<&MatchShot> {
        self.compared.get(set)
    }
}
