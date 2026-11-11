//! The navigator side of the camera: Navigate renders a reference view from
//! the onboard package at the navigator's own estimated pose, its localizer
//! matches the live frame against it, its fusion bridge turns the pose into
//! an observation, and the filter admits or refuses it. Nothing here reads
//! truth.

use std::collections::BTreeMap;
use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use nalgebra::{Quaternion, UnitQuaternion};
use navigate_contract::{NavigationSolution, SourceEpoch, SourceId};
use navigate_visual::fusion_bridge::{
    VerticalDatum, VisualErrorBudget, VisualFixSource, VisualSourceIdentity,
};
use navigate_visual::{Localizer, LocalizerConfig, PyramidalMatcher};
use serde::{Deserialize, Serialize};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};

use super::{CameraFrame, VisualSpec};
use crate::package::MapPackage;
use crate::trial::TrialError;
use crate::trial::replay::ObservationTally;
use crate::{MasterClockBridge, solution_position_ecef};

/// Navigate source identifier of the camera's fixes.
pub const VISUAL_SOURCE: SourceId = SourceId::new(95);

/// What became of the camera's frames, layer by layer.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct VisualTally {
    /// Frames the camera delivered.
    pub frames: u64,
    /// Frames with no navigator solution to search around.
    pub no_solution: u64,
    /// Frames whose search window lies outside the package's coverage.
    pub outside_coverage: u64,
    /// Frames Navigate's reference renderer refused, by reason.
    pub render_refusals: BTreeMap<String, u64>,
    /// Frames Navigate's localizer rejected, by reason.
    pub localizer_rejections: BTreeMap<String, u64>,
    /// Poses Navigate's fusion bridge refused, by reason.
    pub bridge_refusals: BTreeMap<String, u64>,
    /// Fixes offered to the filter, and its verdicts.
    pub filter: ObservationTally,
}

/// One frame's outcome, with the fix when there was one.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FixRecord {
    /// Simulation step.
    pub step: usize,
    /// `accepted`, or the layer and reason that stopped it.
    pub outcome: String,
    /// Search radius around the estimate, metres.
    pub prior_radius_m: f64,
    /// The fix: latitude, longitude in degrees, height above the ellipsoid
    /// in metres, of the vehicle's centre of gravity.
    pub fix: Option<[f64; 3]>,
    /// The fix's declared horizontal 1-sigma, `√(P_nn + P_ee)`, metres.
    pub fix_sigma_h_m: Option<f64>,
    /// Localizer inliers.
    pub inliers: Option<usize>,
}

/// Wall time of one frame's navigator-side work, seconds. Not part of the
/// deterministic result.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct FrameTiming {
    /// Simulate rendering the live frame.
    pub live_render_s: f64,
    /// Building the scene window and rendering the reference.
    pub reference_s: f64,
    /// Matching and pose solving.
    pub matching_s: f64,
}

/// Navigate's visual chain on one capture stream, fed from the onboard
/// package and the navigator's own solution.
pub struct VisualAid {
    pub(super) package: Arc<MapPackage>,
    pub(super) spec: VisualSpec,
    identity: VisualSourceIdentity,
    /// Built on first use, and again after a clone: neither the localizer's
    /// frame order nor the source's evidence ledger can be copied, and a fork
    /// only meets later frames.
    chain: Option<(Localizer<PyramidalMatcher>, VisualFixSource)>,
    pub(super) tally: VisualTally,
    pub(super) records: Vec<FixRecord>,
    pub(super) timings: Vec<FrameTiming>,
}

impl std::fmt::Debug for VisualAid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VisualAid")
            .field("map", &self.package.map_revision())
            .field("spec", &self.spec)
            .field("tally", &self.tally)
            .finish_non_exhaustive()
    }
}

impl Clone for VisualAid {
    /// A copy whose localizer and fusion source start afresh.
    fn clone(&self) -> Self {
        Self {
            package: Arc::clone(&self.package),
            spec: self.spec.clone(),
            identity: self.identity,
            chain: None,
            tally: self.tally.clone(),
            records: self.records.clone(),
            timings: self.timings.clone(),
        }
    }
}

pub(super) fn visual_error(what: impl std::fmt::Display) -> TrialError {
    TrialError::Visual {
        what: what.to_string(),
    }
}

/// A rejection's variant name, without its values.
pub(super) fn kind(reason: &impl std::fmt::Debug) -> String {
    let text = format!("{reason:?}");
    text.split([' ', '{', '('])
        .next()
        .unwrap_or_default()
        .to_owned()
}

pub(super) fn bump(map: &mut BTreeMap<String, u64>, key: String) {
    let count = map.entry(key).or_default();
    *count = count.wrapping_add(1);
}

/// The vehicle as the navigator believes it to be, for the search.
pub(super) struct Belief {
    pub(super) eye: Geodetic,
    pub(super) body_to_ecef: DQuat,
    pub(super) sigma_h_m: f64,
}

pub(super) fn belief(solution: &NavigationSolution, offset_body_m: DVec3) -> Option<Belief> {
    let attitude = solution.attitude?;
    let position = solution_position_ecef(solution);
    let ned_to_ecef = LocalFrame::at(&ecef_to_geodetic(position)).ned_to_ecef();
    let body_to_ned = DQuat::from_xyzw(attitude.x, attitude.y, attitude.z, attitude.w);
    let body_to_ecef = (DQuat::from_mat3(&ned_to_ecef) * body_to_ned).normalize();
    let (pn, pe, _) = solution.position_cov.diagonal();
    Some(Belief {
        eye: ecef_to_geodetic(position + body_to_ecef * offset_body_m),
        body_to_ecef,
        sigma_h_m: (pn + pe).sqrt(),
    })
}

/// The rotation from ECEF axes into the east, north, up axes at `at`.
pub(super) fn enu_from_ecef(at: &Geodetic) -> DQuat {
    let frame = LocalFrame::at(at);
    DQuat::from_mat3(&DMat3::from_cols(frame.east(), frame.north(), frame.up()).transpose())
}

pub(super) fn to_nalgebra(q: DQuat) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z))
}

impl VisualAid {
    /// The chain for `package`, identified on the navigator's clock.
    pub fn new(package: Arc<MapPackage>, spec: VisualSpec) -> Self {
        let identity = VisualSourceIdentity {
            source: VISUAL_SOURCE,
            epoch: SourceEpoch::new(0),
            clock: MasterClockBridge::default().domain(),
        };
        Self {
            package,
            spec,
            identity,
            chain: None,
            tally: VisualTally::default(),
            records: Vec::new(),
            timings: Vec::new(),
        }
    }

    /// The frames' outcomes so far.
    pub const fn tally(&self) -> &VisualTally {
        &self.tally
    }

    /// Every frame's record so far.
    pub fn records(&self) -> &[FixRecord] {
        &self.records
    }

    /// Every frame's wall time so far.
    pub fn timings(&self) -> &[FrameTiming] {
        &self.timings
    }

    fn budget(&self) -> VisualErrorBudget {
        let b = self.spec.budget;
        VisualErrorBudget {
            map_horizontal_m: b.map_horizontal_m,
            map_vertical_m: b.map_vertical_m,
            calibration_m: b.calibration_m,
            geometry_model_m: b.geometry_model_m,
            // The package converts Terrarium heights to the ellipsoid when it
            // is read, so the scene and the fix are both ellipsoidal.
            vertical_datum: VerticalDatum::Wgs84Ellipsoid,
            vertical_datum_m: b.vertical_datum_m,
        }
    }

    pub(super) fn chain(
        &mut self,
    ) -> Result<&mut (Localizer<PyramidalMatcher>, VisualFixSource), TrialError> {
        if self.chain.is_none() {
            let localizer = Localizer::new(PyramidalMatcher, LocalizerConfig::default())
                .map_err(visual_error)?;
            let source =
                VisualFixSource::new(self.identity, self.budget()).map_err(visual_error)?;
            self.chain = Some((localizer, source));
        }
        self.chain
            .as_mut()
            .ok_or_else(|| visual_error("visual chain unavailable"))
    }

    pub(super) fn record(&mut self, frame: &CameraFrame, outcome: String, prior_radius_m: f64) {
        self.records.push(FixRecord {
            step: frame.step,
            outcome,
            prior_radius_m,
            fix: None,
            fix_sigma_h_m: None,
            inliers: None,
        });
    }
}
