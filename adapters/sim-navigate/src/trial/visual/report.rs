//! The visual part of the result: what the camera's frames became, and the
//! accepted fixes scored against truth.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_core::SimInstant;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::{FixRecord, VisualAid, VisualTally};
use crate::package::PackageIdentity;
use crate::trial::ErrorStats;
use crate::trial::score::error_stats;

/// Most frame records a result keeps per set.
const MAX_RECORDS: usize = 240;

/// One frame's record, scored.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScoredFix {
    /// Route time, seconds.
    pub time_s: f64,
    /// The navigator-side record.
    pub record: FixRecord,
    /// Horizontal distance from the fix to the true centre of gravity,
    /// metres; scoring only.
    pub residual_h_m: Option<f64>,
    /// The fix minus truth, north and east, metres; scoring only.
    pub residual_ne_m: Option<[f64; 2]>,
}

/// A set's visual navigation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualReport {
    /// Frames and their outcomes, layer by layer.
    pub tally: VisualTally,
    /// Accepted fixes' horizontal residual against truth, metres.
    pub accepted_residual_h_m: Option<ErrorStats>,
    /// RMS of each accepted fix's residual over its declared horizontal
    /// 1-sigma; about 1.4 when the declared covariance is honest (2-D).
    pub accepted_residual_over_sigma_rms: Option<f64>,
    /// Mean route time between accepted fixes, seconds.
    pub mean_s_between_accepted: Option<f64>,
    /// Longest route time between accepted fixes, seconds.
    pub max_s_between_accepted: Option<f64>,
    /// Frame records, every one when there are few, evenly thinned when
    /// there are many.
    pub records: Vec<ScoredFix>,
}

/// The maps of a run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualMapsReport {
    /// The onboard package the navigator matches against.
    pub onboard: MapIdentity,
    /// The world imagery only the vehicle camera sees.
    pub world: MapIdentity,
    /// Mean distance from the camera to the ground below it, metres.
    pub mean_ground_distance_m: Option<f64>,
    /// Ground footprint of one camera pixel at that distance, metres; the
    /// live frame and the reference share the camera, so both have it.
    pub camera_pixel_footprint_m: Option<f64>,
    /// What the visual test does and does not test.
    pub statement: String,
}

/// One map's identity.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapIdentity {
    /// Release identifier.
    pub release_id: String,
    /// SHA-256 of the manifest.
    pub manifest_sha256: String,
    /// Imagery zoom.
    pub imagery_zoom: u8,
    /// Imagery ground sample distance at the anchor, metres.
    pub imagery_gsd_m: f64,
    /// Elevation zoom.
    pub terrain_zoom: u8,
    /// Imagery tiles.
    pub imagery_tiles: usize,
    /// Elevation tiles.
    pub terrain_tiles: usize,
    /// West, south, east, north, degrees.
    pub bbox: [f64; 4],
    /// Where the tiles came from.
    pub source: String,
}

impl From<&PackageIdentity> for MapIdentity {
    fn from(identity: &PackageIdentity) -> Self {
        Self {
            release_id: identity.info.release_id.clone(),
            manifest_sha256: identity.manifest_sha256.clone(),
            imagery_zoom: identity.imagery_zoom,
            imagery_gsd_m: identity.info.imagery_gsd_m,
            terrain_zoom: identity.terrain_zoom,
            imagery_tiles: identity.imagery_tiles,
            terrain_tiles: identity.terrain_tiles,
            bbox: identity.bbox,
            source: identity.info.source.clone(),
        }
    }
}

/// Scores `aid`'s frames: `truth_at` gives a step's instant and true centre
/// of gravity, ECEF.
pub fn visual_report(
    aid: &VisualAid,
    truth_at: impl Fn(usize) -> Option<(SimInstant, DVec3)>,
) -> VisualReport {
    let scored: Vec<ScoredFix> = aid
        .records()
        .iter()
        .map(|record| {
            let truth = truth_at(record.step);
            let residual_ne_m = record.fix.zip(truth).map(|(fix, (_, true_ecef))| {
                let estimate = geodetic_to_ecef(&Geodetic::from_degrees(fix[0], fix[1], fix[2]));
                let frame = LocalFrame::at(&ecef_to_geodetic(true_ecef));
                let d = frame.ecef_to_ned_vector(estimate - true_ecef);
                [d.x, d.y]
            });
            let residual_h_m = residual_ne_m.map(|[n, e]| n.hypot(e));
            ScoredFix {
                time_s: truth.map_or(0.0, |(at, _)| {
                    at.duration_since(SimInstant::START).as_seconds()
                }),
                record: record.clone(),
                residual_h_m,
                residual_ne_m,
            }
        })
        .collect();
    let accepted: Vec<&ScoredFix> = scored
        .iter()
        .filter(|s| s.record.outcome == "accepted")
        .collect();
    let residuals: Vec<f64> = accepted.iter().filter_map(|s| s.residual_h_m).collect();
    let normalised: Vec<f64> = accepted
        .iter()
        .filter_map(|s| Some(s.residual_h_m? / s.record.fix_sigma_h_m?))
        .collect();
    let gaps: Vec<f64> = accepted
        .windows(2)
        .filter_map(|pair| match pair {
            [a, b] => Some(b.time_s - a.time_s),
            _ => None,
        })
        .collect();
    let thin = scored.len().div_ceil(MAX_RECORDS).max(1);
    VisualReport {
        tally: aid.tally().clone(),
        accepted_residual_h_m: (!residuals.is_empty()).then(|| error_stats(&residuals)),
        accepted_residual_over_sigma_rms: (!normalised.is_empty()).then(|| {
            (normalised.iter().map(|v| v * v).sum::<f64>() / normalised.len() as f64).sqrt()
        }),
        mean_s_between_accepted: (!gaps.is_empty())
            .then(|| gaps.iter().sum::<f64>() / gaps.len() as f64),
        max_s_between_accepted: gaps.iter().copied().reduce(f64::max),
        records: scored.into_iter().step_by(thin).collect(),
    }
}
