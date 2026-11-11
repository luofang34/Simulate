//! One frame through Navigate's visual chain.

use glam::DVec3;
use image::GrayImage;
use navigate_fusion::{IngestOutcome, ObservationValue};
use navigate_visual::fusion_bridge::EvidenceIndependence;
use navigate_visual::{
    CameraPose, Estimate as VisualEstimate, Frame, FrameStamp, PosePrior, ReferenceView,
};
use navigate_visual_render::SparseSceneSampler;
use sim_core::SimInstant;
use sim_geo::Geodetic;

use super::aid::{
    Belief, FixRecord, FrameTiming, VisualAid, belief, bump, enu_from_ecef, kind, to_nalgebra,
    visual_error,
};
use super::{CameraFrame, CameraSpec};
use crate::trial::TrialError;
use crate::{InertialNavigator, MasterClockBridge};

/// Share of a scene window's imagery the package must hold for a frame to
/// be matched at all.
const MIN_WINDOW_COVERAGE: f64 = 0.5;
/// Reference-render thresholds: luma steps and relative depth steps that
/// the adaptive renderer refines rather than interpolates.
const LUMA_EDGE: u8 = 8;
const DEPTH_EDGE: f32 = 0.02;

impl VisualAid {
    /// Matches `frame`, delivered at `at`, against the package around the
    /// navigator's own estimate, and offers any fix to the navigator.
    pub fn aid(
        &mut self,
        frame: &CameraFrame,
        navigator: &mut InertialNavigator,
        at: SimInstant,
    ) -> Result<(), TrialError> {
        self.tally.frames = self.tally.frames.wrapping_add(1);
        let offset = DVec3::from_array(self.spec.camera.mount_offset_body_m);
        let Some(belief) = navigator.solution(at)?.and_then(|s| belief(&s, offset)) else {
            self.tally.no_solution = self.tally.no_solution.wrapping_add(1);
            self.record(frame, "no_solution".into(), 0.0);
            return Ok(());
        };
        let p = self.spec.prior;
        let radius = (p.sigma_multiple * belief.sigma_h_m).clamp(p.min_radius_m, p.max_radius_m);
        let started = web_time::Instant::now();
        let Some((reference, prior)) = self.reference(frame, &belief, radius)? else {
            return Ok(());
        };
        let reference_s = started.elapsed().as_secs_f64();
        let camera = self.spec.camera.camera_model();
        let image = GrayImage::from_raw(camera.width, camera.height, frame.pixels.clone())
            .ok_or_else(|| visual_error("frame size does not match the camera"))?;
        let stamp = FrameStamp {
            sequence: frame.index,
            capture_time_ns: MasterClockBridge::default().to_monotonic(at)?.as_nanos(),
        };
        let query = Frame {
            stamp,
            camera,
            image,
        };
        let started = web_time::Instant::now();
        let estimate = self
            .chain()?
            .0
            .estimate_blocking(&query, &reference, &prior);
        self.timings.push(FrameTiming {
            live_render_s: frame.render_s,
            reference_s,
            matching_s: started.elapsed().as_secs_f64(),
        });
        match estimate {
            Err(error) => {
                let reason = kind(&error);
                bump(&mut self.tally.localizer_rejections, reason.clone());
                self.record(frame, format!("localizer:{reason}"), radius);
                Ok(())
            }
            Ok(estimate) => self.admit(frame, estimate, &belief, radius, navigator, at),
        }
    }

    /// The reference view at the believed camera pose and the search around
    /// it, or `None` when the package cannot supply one.
    fn reference(
        &mut self,
        frame: &CameraFrame,
        belief: &Belief,
        radius: f64,
    ) -> Result<Option<(ReferenceView, PosePrior)>, TrialError> {
        let footprint = belief.eye.height_m.max(1.0) * self.spec.camera.half_diagonal_rad().tan();
        let window = self.package.scene_around(
            belief.eye.lat_deg(),
            belief.eye.lon_deg(),
            footprint + radius,
        )?;
        if window.imagery_coverage < MIN_WINDOW_COVERAGE {
            self.tally.outside_coverage = self.tally.outside_coverage.wrapping_add(1);
            self.record(frame, "coverage:outside_package".into(), radius);
            return Ok(None);
        }
        let anchor = Geodetic {
            height_m: 0.0,
            ..belief.eye
        };
        let position = window
            .scene
            .frame
            .local([
                belief.eye.lat_deg(),
                belief.eye.lon_deg(),
                belief.eye.height_m,
            ])
            .map_err(visual_error)?;
        let camera_to_enu =
            enu_from_ecef(&anchor) * belief.body_to_ecef * CameraSpec::camera_to_body();
        let pose = CameraPose {
            position,
            orientation: to_nalgebra(camera_to_enu.normalize()),
        };
        let r = self.spec.render;
        let rendered = SparseSceneSampler::new(&window.scene, r.far_m, r.steps, r.refinements)
            .and_then(|sampler| {
                sampler.render_adaptive(
                    &self.spec.camera.camera_model(),
                    &pose,
                    r.stride,
                    LUMA_EDGE,
                    DEPTH_EDGE,
                )
            });
        match rendered {
            Ok((reference, _)) => Ok(Some((
                reference,
                PosePrior {
                    pose,
                    position_radius_m: radius,
                    attitude_radius_rad: self.spec.prior.attitude_radius_rad,
                },
            ))),
            Err(error) => {
                let reason = kind(&error);
                bump(&mut self.tally.render_refusals, reason.clone());
                self.record(frame, format!("render:{reason}"), radius);
                Ok(None)
            }
        }
    }

    /// Takes the lever arm off a solved camera pose, converts it through
    /// Navigate's fusion bridge, and offers it to the navigator.
    fn admit(
        &mut self,
        frame: &CameraFrame,
        mut estimate: VisualEstimate,
        belief: &Belief,
        radius: f64,
        navigator: &mut InertialNavigator,
        at: SimInstant,
    ) -> Result<(), TrialError> {
        let anchor = Geodetic {
            height_m: 0.0,
            ..belief.eye
        };
        let lever_enu = (enu_from_ecef(&anchor) * belief.body_to_ecef)
            * DVec3::from_array(self.spec.camera.mount_offset_body_m);
        estimate.pose.position -= nalgebra::Vector3::new(lever_enu.x, lever_enu.y, lever_enu.z);
        let inliers = estimate.quality.inliers;
        let converted = self
            .chain()?
            .1
            .convert(&estimate, EvidenceIndependence::ValidatedIndependent);
        let fix = match converted {
            Ok(fix) => fix,
            Err(error) => {
                let reason = kind(&error);
                bump(&mut self.tally.bridge_refusals, reason.clone());
                self.record(frame, format!("bridge:{reason}"), radius);
                return Ok(());
            }
        };
        let outcome = match navigator.observe(&fix.observation, at)? {
            IngestOutcome::Accepted => {
                self.tally.filter.count(Ok(()));
                "accepted".to_owned()
            }
            IngestOutcome::Rejected(reason) => {
                let text = format!("filter:{}", kind(&reason));
                self.tally.filter.count(Err(&reason));
                text
            }
        };
        let (fix_position, sigma) = match fix.observation.value {
            ObservationValue::PositionFix {
                position,
                covariance,
            } => {
                let (pn, pe, _) = covariance.diagonal();
                (
                    Some([
                        position.latitude_rad.to_degrees(),
                        position.longitude_rad.to_degrees(),
                        position.altitude_m,
                    ]),
                    Some((pn + pe).sqrt()),
                )
            }
            _ => (None, None),
        };
        self.records.push(FixRecord {
            step: frame.step,
            outcome,
            prior_radius_m: radius,
            fix: fix_position,
            fix_sigma_h_m: sigma,
            inliers: Some(inliers),
        });
        Ok(())
    }
}
