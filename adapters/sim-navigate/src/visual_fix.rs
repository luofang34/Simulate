use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use nalgebra::{Isometry3, Translation3};
use navigate_visual::fusion_bridge::{
    EvidenceIndependence, VisualErrorBudget, VisualFix as FusionVisualFix, VisualFixSource,
    VisualFusionError, VisualSourceIdentity,
};
use navigate_visual::session::{
    AnchorDecision, AnchorObservation, Calibration, CameraMount as SessionCameraMount, CaptureTime,
    ContinuityEpoch, FrameKey, FrameRecord, FusionEligibility as SessionEligibility, LensModel,
    MapReliability, Pose, RelativeMotion, ScaleSource, SessionConfig, SessionError, StreamId,
    VerticalReference, VisualSession, from_camera,
};
use navigate_visual::{CameraPose, Estimate, FrameStamp, MapRevision};
use sim_core::{CameraMount, FusionEligibility, SensorIntrinsics, SimInstant, VisualFix};
use sim_geo::Geodetic;

use crate::estimate::{
    EstimateParts, camera_model, estimate, evidence_digest, lever_arm_ecef, to_nalgebra, to_vector,
};
use crate::{CorridorFrame, MasterClockBridge, NavigateBridgeError};

#[cfg(test)]
mod tests;

/// Bound on the difference between a capture stamp and the exposure: the
/// master clock rounds down to whole nanoseconds.
const CAPTURE_ERROR_BOUND_NS: u64 = 1;

/// Fixed setup of a [`VisualFixAdapter`].
#[derive(Debug, Clone, PartialEq)]
pub struct VisualFixConfig {
    /// The corridor catalog's anchor.
    pub anchor: Geodetic,
    /// Where the camera sits on the airframe.
    pub mount: CameraMount,
    /// Centre of gravity in the mount's axes: the filter tracks this point.
    pub cg_body_m: DVec3,
    /// The camera's optics.
    pub intrinsics: SensorIntrinsics,
    /// The map release the corridor anchors are solved against.
    pub map: MapRevision,
    /// Error terms image geometry cannot measure.
    pub budget: VisualErrorBudget,
    /// Declared error of the map near an anchor.
    pub reliability: MapReliability,
    /// Identity of the capture stream.
    pub identity: VisualSourceIdentity,
    /// The host's assessment of all shared error outside the session ledger.
    /// A session label can only narrow this.
    pub host_independence: EvidenceIndependence,
    /// RMS error of the attitude supplied with each fix, radians. Times the
    /// lever arm it is a position error, which `budget.calibration_m` must
    /// cover.
    pub attitude_rms_rad: f64,
    /// Limits and declared drift of the visual session.
    pub session: SessionConfig,
}

/// What became of one harness fix.
#[derive(Debug, Clone, PartialEq)]
pub enum FixDisposition {
    /// A warm-track pose, kept as odometry between anchors. It depends on the
    /// anchors behind it, so it can never be a position fix.
    Tracked,
    /// The anchor is independent evidence: offer the fix to the filter.
    Admitted(Box<FusionVisualFix>),
    /// `VisualFixSource` refused the fix, because its evidence is correlated
    /// with what the filter already holds.
    RefusedCorrelated {
        /// The session's label.
        session_label: SessionEligibility,
        /// The harness's label.
        harness_label: FusionEligibility,
    },
    /// The anchor never entered the session's trajectory.
    NotAnchored(AnchorDecision),
}

/// A warm-track pose held until an anchor needs it as odometry.
#[derive(Debug, Clone, Copy)]
struct TrackPose {
    eye_ecef: DVec3,
    body_to_ecef: DQuat,
}

/// The latest frame in the session, with what its odometry continues from.
#[derive(Debug, Clone)]
struct AnchoredFrame {
    key: FrameKey,
    digest: String,
    track: Option<Pose>,
}

/// Carries harness fixes into Navigate's real admission path.
///
/// Each anchor becomes a `navigate_visual::Estimate` at the camera, is labelled
/// by a real [`VisualSession`], and has the camera lever arm taken off before
/// [`VisualFixSource`] converts it. Independence starts from the host's
/// assessment and is narrowed by both the session's ledger and the harness's
/// `MapCell` bookkeeping, so either one alone can refuse a fix.
pub struct VisualFixAdapter {
    config: VisualFixConfig,
    clock: MasterClockBridge,
    frame: CorridorFrame,
    source: VisualFixSource,
    session: VisualSession,
    tracks: BTreeMap<SimInstant, TrackPose>,
    previous: Option<AnchoredFrame>,
    next_index: u64,
}

impl VisualFixAdapter {
    /// An adapter with nothing tracked yet. Refuses an unusable budget,
    /// session, or anchor, and an attitude allowance whose lever-arm error the
    /// budget does not cover.
    pub fn new(
        config: VisualFixConfig,
        clock: MasterClockBridge,
    ) -> Result<Self, NavigateBridgeError> {
        let rms = config.attitude_rms_rad;
        if !rms.is_finite() || rms < 0.0 {
            return Err(NavigateBridgeError::InvalidAttitudeAllowance {
                attitude_rms_rad: rms,
            });
        }
        let lever_arm_error_m = (config.mount.offset_body_m - config.cg_body_m).length() * rms;
        if config.budget.calibration_m < lever_arm_error_m {
            return Err(NavigateBridgeError::LeverArmUncovered {
                calibration_m: config.budget.calibration_m,
                lever_arm_error_m,
            });
        }
        let frame = CorridorFrame::new(&config.anchor)?;
        let source = VisualFixSource::new(config.identity, config.budget)
            .map_err(NavigateBridgeError::VisualFusion)?;
        let calibration = session_calibration(&config);
        let session = VisualSession::new(config.session, calibration).map_err(|source| {
            NavigateBridgeError::Session {
                captured_at: SimInstant::START,
                source,
            }
        })?;
        Ok(Self {
            config,
            clock,
            frame,
            source,
            session,
            tracks: BTreeMap::new(),
            previous: None,
            next_index: 0,
        })
    }

    /// The session behind the labels.
    pub const fn session(&self) -> &VisualSession {
        &self.session
    }

    /// Takes one delivered fix. `body_to_ecef` is the attitude the host holds
    /// for the fix's capture instant, within the configured allowance; passing
    /// the true attitude here would hide the lever-arm error the budget
    /// claims to cover.
    pub fn offer(
        &mut self,
        fix: &VisualFix,
        body_to_ecef: DQuat,
    ) -> Result<FixDisposition, NavigateBridgeError> {
        if fix.eligibility == FusionEligibility::CorrelatedTrack {
            self.tracks.insert(
                fix.captured_at,
                TrackPose {
                    eye_ecef: self.frame.enu_to_ecef(fix.estimate.position_enu_m),
                    body_to_ecef,
                },
            );
            return Ok(FixDisposition::Tracked);
        }
        let sequence = self.next_index;
        self.next_index = sequence.wrapping_add(1);
        let captured_at = fix.captured_at;
        let at_camera = self.camera_estimate(fix, body_to_ecef, sequence)?;
        let key = FrameKey {
            stream: StreamId(self.config.identity.source.get()),
            continuity: ContinuityEpoch(self.config.identity.epoch.get()),
            index: sequence,
        };
        let session_error = |source| NavigateBridgeError::Session {
            captured_at,
            source,
        };
        self.session
            .observe_frame(FrameRecord {
                key,
                media: None,
                capture: CaptureTime {
                    clock: self.clock.domain(),
                    at_ns: at_camera.stamp.capture_time_ns,
                    error_bound_ns: CAPTURE_ERROR_BOUND_NS,
                },
                observation_sha256: at_camera.observation_sha256.clone(),
            })
            .map_err(session_error)?;
        let track = self.track_pose(captured_at)?;
        self.extend_odometry(key, &at_camera.observation_sha256, track.as_ref())
            .map_err(session_error)?;
        self.previous = Some(AnchoredFrame {
            key,
            digest: at_camera.observation_sha256.clone(),
            track,
        });
        self.tracks = self.tracks.split_off(&captured_at);

        let anchor = AnchorObservation::from_estimate(key, &at_camera, self.config.reliability);
        let decision = self.session.submit_anchor(anchor).map_err(session_error)?;
        let AnchorDecision::Accepted {
            eligibility: session_label,
        } = decision
        else {
            return Ok(FixDisposition::NotAnchored(decision));
        };
        self.convert(fix, body_to_ecef, at_camera, session_label)
    }

    /// `fix` as an estimate of the vehicle reference point rather than the
    /// camera, the form `VisualFixSource` must receive for a vehicle-position
    /// filter.
    pub fn vehicle_estimate(
        &self,
        fix: &VisualFix,
        body_to_ecef: DQuat,
        sequence: u64,
    ) -> Result<Estimate, NavigateBridgeError> {
        let at_camera = self.camera_estimate(fix, body_to_ecef, sequence)?;
        self.without_lever_arm(fix, body_to_ecef, at_camera)
    }

    fn convert(
        &mut self,
        fix: &VisualFix,
        body_to_ecef: DQuat,
        at_camera: Estimate,
        session_label: SessionEligibility,
    ) -> Result<FixDisposition, NavigateBridgeError> {
        let independence = self.independence(fix.eligibility, &session_label);
        let at_vehicle = self.without_lever_arm(fix, body_to_ecef, at_camera)?;
        match self.source.convert(&at_vehicle, independence) {
            Ok(admitted) => Ok(FixDisposition::Admitted(Box::new(admitted))),
            Err(VisualFusionError::UnknownCorrelation) => Ok(FixDisposition::RefusedCorrelated {
                session_label,
                harness_label: fix.eligibility,
            }),
            Err(error) => Err(NavigateBridgeError::VisualFusion(error)),
        }
    }

    fn independence(
        &self,
        harness_label: FusionEligibility,
        session_label: &SessionEligibility,
    ) -> EvidenceIndependence {
        let restricted = self.config.host_independence.restricted_by(session_label);
        if harness_label.independent_ekf_fix() {
            restricted
        } else {
            EvidenceIndependence::Unknown
        }
    }

    /// The camera's orientation under the host's attitude. Simulate's mount
    /// gives camera axes right, up, back, as Navigate's eye axes are.
    fn camera_to_ecef(&self, body_to_ecef: DQuat) -> DQuat {
        (body_to_ecef * self.config.mount.body_to_camera.inverse()).normalize()
    }

    fn camera_estimate(
        &self,
        fix: &VisualFix,
        body_to_ecef: DQuat,
        sequence: u64,
    ) -> Result<Estimate, NavigateBridgeError> {
        let capture_time_ns = self.clock.to_monotonic(fix.captured_at)?.as_nanos();
        let eye_ecef = self.frame.enu_to_ecef(fix.estimate.position_enu_m);
        Ok(estimate(EstimateParts {
            stamp: FrameStamp {
                sequence,
                capture_time_ns,
            },
            observation_sha256: evidence_digest(
                sequence,
                capture_time_ns,
                fix.captured_at,
                &self.config.intrinsics,
            ),
            map: &self.config.map,
            frame: self.frame.local(),
            pose: self
                .frame
                .camera_pose(eye_ecef, self.camera_to_ecef(body_to_ecef))?,
            covariance_enu_m2: fix.estimate.covariance_enu_m2,
        }))
    }

    fn without_lever_arm(
        &self,
        fix: &VisualFix,
        body_to_ecef: DQuat,
        at_camera: Estimate,
    ) -> Result<Estimate, NavigateBridgeError> {
        let eye_ecef = self.frame.enu_to_ecef(fix.estimate.position_enu_m);
        let arm = lever_arm_ecef(
            self.config.mount.offset_body_m,
            self.config.cg_body_m,
            body_to_ecef,
        );
        let vehicle: CameraPose = self
            .frame
            .camera_pose(eye_ecef - arm, self.camera_to_ecef(body_to_ecef))?;
        Ok(Estimate {
            pose: vehicle,
            ..at_camera
        })
    }

    fn track_pose(&self, captured_at: SimInstant) -> Result<Option<Pose>, NavigateBridgeError> {
        self.tracks
            .get(&captured_at)
            .map(|track| {
                let pose = self
                    .frame
                    .camera_pose(track.eye_ecef, self.camera_to_ecef(track.body_to_ecef))?;
                Ok(from_camera(&pose))
            })
            .transpose()
    }

    /// Joins this frame to the previous anchor's by the warm track's relative
    /// motion. Without a warm pose at both ends the session sees a gap and the
    /// anchor starts a new segment, which is what a lost track is. Scale comes
    /// from seeded map points, because that is what warm tracks are built on.
    fn extend_odometry(
        &mut self,
        key: FrameKey,
        digest: &str,
        track: Option<&Pose>,
    ) -> Result<(), SessionError> {
        let Some(previous) = &self.previous else {
            return Ok(());
        };
        let (Some(from_pose), Some(to_pose)) = (previous.track.as_ref(), track) else {
            return Ok(());
        };
        self.session
            .apply_motion(RelativeMotion {
                from: previous.key,
                to: key,
                from_to: from_pose.inv_mul(to_pose),
                scale: ScaleSource::SeededMapPoints {
                    map: self.config.map.clone(),
                },
                from_observation_sha256: previous.digest.clone(),
                to_observation_sha256: digest.to_owned(),
            })
            .map(drop)
    }
}

/// The session's view of the camera, with its fixed mount.
fn session_calibration(config: &VisualFixConfig) -> Calibration {
    let translation = to_vector(config.mount.offset_body_m - config.cg_body_m);
    let rotation = to_nalgebra(config.mount.body_to_camera.inverse());
    Calibration {
        intrinsics: camera_model(&config.intrinsics),
        lens: LensModel::Undistorted {
            residual_rms_px: 0.0,
        },
        mount: SessionCameraMount::Fixed {
            body_from_camera: Isometry3::from_parts(Translation3::from(translation), rotation),
        },
        map_vertical: VerticalReference::Wgs84Ellipsoid,
    }
}
