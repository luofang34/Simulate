//! The navigator side of the star camera: Navigate's tracker identifies each
//! frame, lost in space or with a prior taken from the navigator's own
//! solution, solves the attitude, and offers every star as a line of sight.

use glam::DQuat;
use nalgebra::{Quaternion, UnitQuaternion};
use navigate_camera::{CameraMount as NavigateMount, ImagerModel};
use navigate_contract::{NavigationSolution, SourceEpoch, SourceId, WrappingSequence};
use navigate_fusion::{IngestOutcome, RejectionReason};
use navigate_startrack::{
    AttitudePrior, IdentifyConfig, IdentifyMode, PairCatalogue, SolveConfig, StarTracker,
    TrackerConfig, catalogue_ecef, identify_stars, line_of_sight_observations, solve_attitude,
};
use sim_core::SimInstant;
use sim_geo::{LocalFrame, UtcTime, ecef_to_geodetic};

use super::{StarImage, roof_star_mount, star_camera_config};
use crate::trial::TrialError;
use crate::{
    InertialNavigator, MasterClockBridge, imager_model, navigate_mount, solution_position_ecef,
    star_camera_epoch,
};

/// Faintest star of the navigator's identification catalogue, Johnson V.
pub const CATALOGUE_FAINTEST_V: f64 = 6.0;
/// Faintest star of the lost-in-space pair table: brighter than the
/// catalogue, so the stars a pyramid is built from are ones the camera
/// detects reliably.
pub const LIS_FAINTEST_V: f64 = 5.5;

/// The lost-in-space pair table for the star camera's diagonal field of
/// view, from the catalogue at `start`; pair angles do not change with the
/// Earth's rotation.
pub fn pair_table(start: UtcTime) -> Result<PairCatalogue, TrialError> {
    let intrinsics = star_camera_config().intrinsics;
    let half_diagonal_px = f64::from(intrinsics.width).hypot(f64::from(intrinsics.height)) / 2.0;
    let diagonal_rad = 2.0 * (half_diagonal_px / intrinsics.fx).atan();
    let catalogue = catalogue_ecef(star_camera_epoch(start, SimInstant::START), LIS_FAINTEST_V)
        .map_err(TrialError::StarTrack)?;
    PairCatalogue::new(&catalogue, diagonal_rad, LIS_FAINTEST_V).map_err(TrialError::StarTrack)
}

/// The navigator's own attitude belief as an identification prior: its
/// published body-to-NED attitude carried into ECEF at its own position, and
/// the RMS of its published attitude covariance. `None` before the solution
/// carries an attitude.
pub fn own_prior(solution: &NavigationSolution) -> Option<AttitudePrior> {
    let (attitude, covariance) = (solution.attitude?, solution.attitude_cov?);
    let body_to_ned = DQuat::from_xyzw(attitude.x, attitude.y, attitude.z, attitude.w);
    let frame = LocalFrame::at(&ecef_to_geodetic(solution_position_ecef(solution)));
    let ecef = DQuat::from_mat3(&frame.ned_to_ecef()) * body_to_ned;
    let (xx, yy, zz) = covariance.diagonal();
    Some(AttitudePrior {
        ecef_from_body: UnitQuaternion::from_quaternion(Quaternion::new(
            ecef.w, ecef.x, ecef.y, ecef.z,
        )),
        sigma_rad: (xx + yy + zz).sqrt(),
    })
}

/// What one frame gave the navigator.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SightingOutcome {
    /// The tracker chose lost-in-space identification.
    pub lost_in_space: bool,
    /// Identification and the attitude solve succeeded.
    pub solved: bool,
    /// Stars in the solution, each offered as one line of sight.
    pub stars: usize,
    /// Lines of sight the filter accepted.
    pub accepted: u64,
    /// Why the filter refused the others.
    pub rejected: Vec<RejectionReason>,
}

/// Navigate's star tracker on the roof camera, fed from the navigator's own
/// solution and never from truth.
#[derive(Debug, Clone)]
pub struct StarSighting<'a> {
    imager: ImagerModel,
    mount: NavigateMount,
    pairs: &'a PairCatalogue,
    start: UtcTime,
    source: SourceId,
    mount_variance_rad2: f64,
    next: WrappingSequence,
    held_prior: Option<AttitudePrior>,
    tracker: StarTracker,
}

impl<'a> StarSighting<'a> {
    /// A tracker with no lock, identifying against `pairs`, whose sightings
    /// come from `source` with the mount known to `mount_alignment_sigma_rad`.
    pub fn new(
        pairs: &'a PairCatalogue,
        start: UtcTime,
        source: SourceId,
        mount_alignment_sigma_rad: f64,
    ) -> Result<Self, TrialError> {
        let (mount, cg_body_m) = roof_star_mount();
        Ok(Self {
            imager: imager_model(&star_camera_config().intrinsics)?,
            mount: navigate_mount(&mount, cg_body_m),
            pairs,
            start,
            source,
            mount_variance_rad2: mount_alignment_sigma_rad.powi(2),
            next: WrappingSequence::new(0),
            held_prior: None,
            tracker: StarTracker::new(TrackerConfig::default()).map_err(TrialError::StarTrack)?,
        })
    }

    /// At mid-exposure: the navigator's own attitude belief. The tracker
    /// uses it only while it holds a lock and the belief is narrow.
    pub fn hold_prior(
        &mut self,
        navigator: &mut InertialNavigator,
        at: SimInstant,
    ) -> Result<(), TrialError> {
        self.held_prior = navigator.solution(at)?.as_ref().and_then(own_prior);
        Ok(())
    }

    /// At readout, `now`: sights `image` in the mode the tracker chooses and
    /// offers every line of sight stamped at mid-exposure.
    pub fn aid(
        &mut self,
        image: &StarImage,
        navigator: &mut InertialNavigator,
        now: SimInstant,
    ) -> Result<SightingOutcome, TrialError> {
        let mode = self.tracker.mode(self.held_prior.take(), self.pairs);
        let mut outcome = SightingOutcome {
            lost_in_space: matches!(mode, IdentifyMode::LostInSpace(_)),
            ..SightingOutcome::default()
        };
        let catalogue = catalogue_ecef(
            star_camera_epoch(self.start, image.mid),
            CATALOGUE_FAINTEST_V,
        )
        .map_err(TrialError::StarTrack)?;
        let solved = identify_stars(
            &image.centroids,
            &self.imager,
            &self.mount,
            mode,
            &catalogue,
            &IdentifyConfig::default(),
        )
        .and_then(|matches| {
            solve_attitude(
                &matches,
                &image.centroids,
                &self.imager,
                &self.mount,
                &catalogue,
                &SolveConfig::default(),
            )
        });
        self.tracker.record(solved.is_ok());
        let Ok(sighting) = solved else {
            return Ok(outcome);
        };
        outcome.solved = true;
        outcome.stars = sighting.matches.len();
        let stamp = MasterClockBridge::default().stamp(
            self.source,
            SourceEpoch::new(0),
            self.next,
            image.mid,
        )?;
        let frame = navigator.filter().frame();
        let observations =
            line_of_sight_observations(&sighting, stamp, frame, self.mount_variance_rad2)
                .map_err(TrialError::StarTrack)?;
        for observation in &observations {
            self.next = self.next.next();
            match navigator.observe(observation, now)? {
                IngestOutcome::Accepted => outcome.accepted = outcome.accepted.wrapping_add(1),
                IngestOutcome::Rejected(reason) => outcome.rejected.push(reason),
            }
        }
        Ok(outcome)
    }
}
