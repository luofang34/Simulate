//! Image-driven corridor runs: a nadir camera's frames are rendered on the
//! CPU from a catalog, an injected estimator positions the aircraft from
//! those pixels alone, and the autopilot steers on a filter fed by the
//! delivered fixes. Truth stays on this side, in the sensor models and the
//! harness's scoring.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_core::{
    CameraFrameCapture, CameraMount, RateAdapter, SimInstant, TICKS_PER_SECOND, TickSpan,
    TwoTierConfig, VehicleState, VisualFix, VisualFrame,
};
use sim_devices::WhiteNoise;
use sim_geo::{Geodetic, LocalFrame, SensorIntrinsics, ecef_to_geodetic, geodetic_to_ecef};
use sim_scenery::{CatalogScenery, render_luma_view};

use super::dynamics::{CorridorStart, FlightDynamics, Source};
use super::navigation::GuidanceFilter;
use super::visual::{
    CorridorVisualEstimator, RenderedSensorFrame, SensorObservation, SyntheticVisualEstimator,
};
use super::{CorridorRun, anchor_position, start_of};
use crate::{ScenarioError, ScenarioProfile};

const TICKS_PER_MILLISECOND: u64 = TICKS_PER_SECOND / 1_000;

/// How an image-driven corridor run senses and is scored.
#[derive(Debug, Clone)]
pub struct ImageDrivenConfig {
    /// The world the camera renders.
    pub world: Arc<CatalogScenery>,
    /// The nadir camera's optics; kept small, since every pixel is a CPU ray.
    pub intrinsics: SensorIntrinsics,
    /// How often a frame is rendered and handed to the estimator.
    pub render_period: TickSpan,
    /// The start moved this far right of the profile's, metres, which the
    /// estimator and filter are not told.
    pub start_lateral_offset_m: f64,
    /// One-sigma horizontal doubt the filter starts with about the planned
    /// start, metres.
    pub seed_sigma_m: f64,
    /// One-sigma constant attitude reference error per axis, radians.
    pub attitude_bias_sigma_rad: f64,
    /// One-sigma frame-to-frame attitude jitter per axis, radians.
    pub attitude_noise_rad: f64,
    /// One-sigma radar altimeter noise, metres.
    pub agl_sigma_m: f64,
    /// One-sigma velocity aiding noise per axis, m/s.
    pub velocity_sigma_m_s: f64,
    /// Seeds every sensor error.
    pub sensor_seed: u64,
    /// The harness's tier rates and delays.
    pub harness: TwoTierConfig,
}

impl ImageDrivenConfig {
    /// A 96 × 96 nadir camera at 10 Hz over `world`, a 1 Hz anchor tier
    /// delivered 200 ms late, and tactical-grade attitude and altimeter
    /// errors.
    #[must_use]
    pub fn new(world: Arc<CatalogScenery>) -> Self {
        Self {
            world,
            intrinsics: SensorIntrinsics {
                width: 96,
                height: 96,
                fx: 90.0,
                fy: 90.0,
                cx: 47.5,
                cy: 47.5,
            },
            render_period: TickSpan::from_ticks(TICKS_PER_SECOND / 10),
            start_lateral_offset_m: 0.0,
            seed_sigma_m: 150.0,
            attitude_bias_sigma_rad: 0.002,
            attitude_noise_rad: 0.000_2,
            agl_sigma_m: 3.0,
            velocity_sigma_m_s: 0.3,
            sensor_seed: 0x1A6E_5EED,
            harness: TwoTierConfig {
                anchor_period: TickSpan::from_ticks(TICKS_PER_SECOND),
                anchor_delay: TickSpan::from_ticks(200 * TICKS_PER_MILLISECOND),
                ..TwoTierConfig::default()
            },
        }
    }
}

/// What an image-driven run's navigation has done so far.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ImageNavigationReport {
    /// Frames rendered and handed to the estimator.
    pub rendered_frames: u64,
    /// Independent anchors fused by the guidance filter.
    pub fused_anchors: u64,
    /// Shared-cell anchors the filter refused as repeated evidence.
    pub refused_shared_cell: u64,
    /// Warm-track fixes delivered, counted but not fused.
    pub warm_fixes: u64,
    /// Latest horizontal distance between the filter and the truth, metres.
    pub navigation_error_m: f64,
    /// Latest one-sigma horizontal uncertainty the filter claims, metres.
    pub navigation_sigma_m: f64,
    /// Smallest claimed one-sigma horizontal uncertainty so far, metres.
    pub min_navigation_sigma_m: f64,
}

/// Where a corridor run's visual estimates come from.
#[derive(Debug, Clone)]
pub(super) enum VisualSource {
    /// Truth plus declared noise.
    Synthetic(SyntheticVisualEstimator),
    /// Rendered pixels through an injected estimator.
    Image(Box<ImageMode>),
}

/// The image-driven side of a [`super::CorridorRun`].
#[derive(Debug)]
pub(super) struct ImageMode {
    config: ImageDrivenConfig,
    estimator: Box<dyn CorridorVisualEstimator>,
    render_rate: RateAdapter,
    last_render: Option<SimInstant>,
    noise: WhiteNoise,
    attitude_bias: DVec3,
    filter: GuidanceFilter,
    report: ImageNavigationReport,
}

impl Clone for ImageMode {
    fn clone(&self) -> Self {
        Self {
            config: self.config.clone(),
            estimator: self.estimator.boxed_clone(),
            render_rate: self.render_rate,
            last_render: self.last_render,
            noise: self.noise,
            attitude_bias: self.attitude_bias,
            filter: self.filter.clone(),
            report: self.report,
        }
    }
}

impl ImageMode {
    pub(super) fn new(
        config: ImageDrivenConfig,
        estimator: Box<dyn CorridorVisualEstimator>,
        seed_enu_m: DVec3,
    ) -> Self {
        let mut noise = WhiteNoise::from_seed(config.sensor_seed);
        let attitude_bias = noise.next_vec3(DVec3::splat(config.attitude_bias_sigma_rad));
        Self {
            filter: GuidanceFilter::new(seed_enu_m, config.seed_sigma_m, config.velocity_sigma_m_s),
            render_rate: RateAdapter::new(config.render_period),
            last_render: None,
            noise,
            attitude_bias,
            estimator,
            report: ImageNavigationReport {
                min_navigation_sigma_m: f64::INFINITY,
                ..ImageNavigationReport::default()
            },
            config,
        }
    }

    pub(super) fn report(&self) -> ImageNavigationReport {
        self.report
    }

    /// Renders `capture` when a frame is due, has the estimator position
    /// it, and returns the frame for the harness.
    pub(super) fn observe(
        &mut self,
        capture: &CameraFrameCapture,
    ) -> Result<Option<VisualFrame>, ScenarioError> {
        let elapsed = self
            .last_render
            .map(|last| capture.timestamp.duration_since(last));
        if !elapsed.is_none_or(|span| self.render_rate.steps_for_u128(span) > 0) {
            return Ok(None);
        }
        self.last_render = Some(capture.timestamp);
        let world = &self.config.world;
        let image = render_luma_view(
            world,
            capture.pose.position_ecef,
            capture.pose.orientation,
            &capture.intrinsics,
        )?;
        let eye = ecef_to_geodetic(capture.pose.position_ecef);
        let ground = world
            .ellipsoidal_ground_height_m(&eye)?
            .map_or(0.0, |h| h.0);
        let jitter = self
            .noise
            .next_vec3(DVec3::splat(self.config.attitude_noise_rad));
        let attitude_error = DQuat::from_scaled_axis(self.attitude_bias + jitter);
        let observation = SensorObservation {
            frame: RenderedSensorFrame {
                timestamp: capture.timestamp,
                intrinsics: capture.intrinsics,
                image,
            },
            camera_to_ecef_estimate: (attitude_error * capture.pose.orientation).normalize(),
            agl_estimate_m: eye.height_m - ground
                + self.config.agl_sigma_m * self.noise.next_gaussian(),
        };
        let tiers = self.estimator.estimate(&observation);
        self.report.rendered_frames = self.report.rendered_frames.wrapping_add(1);
        Ok(Some(VisualFrame {
            capture: *capture,
            warm_track: tiers.warm_track,
            corridor_anchor: tiers.corridor_anchor,
        }))
    }

    /// Dead-reckons one step on measured velocity, then takes in the fixes
    /// delivered by `now`, and scores the filter against `truth_enu_m`.
    pub(super) fn navigate(
        &mut self,
        truth: &VehicleState,
        truth_enu_m: DVec3,
        anchor_frame: &LocalFrame,
        dt_s: f64,
        now: SimInstant,
        fixes: &[VisualFix],
    ) {
        let measured = anchor_frame.ecef_to_enu_vector(truth.velocity_ecef)
            + self
                .noise
                .next_vec3(DVec3::splat(self.config.velocity_sigma_m_s));
        self.filter.propagate(measured, dt_s, now);
        self.filter.ingest(fixes);
        let report = &mut self.report;
        report.fused_anchors = self.filter.fused_anchors;
        report.refused_shared_cell = self.filter.refused_shared_cell;
        report.warm_fixes = self.filter.warm_fixes;
        report.navigation_error_m = (self.filter.position_enu_m() - truth_enu_m)
            .truncate()
            .length();
        report.navigation_sigma_m = self.filter.horizontal_sigma_m();
        if report.fused_anchors > 0 {
            report.min_navigation_sigma_m =
                report.min_navigation_sigma_m.min(report.navigation_sigma_m);
        }
    }

    /// `truth` with the filter's position in place of the true one; the
    /// attitude and velocity are the inertial reference's.
    pub(super) fn navigated(
        &self,
        truth: &VehicleState,
        origin_ecef: DVec3,
        anchor_frame: &LocalFrame,
    ) -> VehicleState {
        VehicleState {
            position_ecef: origin_ecef + anchor_frame.enu_to_ecef() * self.filter.position_enu_m(),
            ..*truth
        }
    }
}

impl CorridorRun {
    /// `profile` on `dynamics` with a nadir camera whose frames are rendered
    /// from `config.world` and positioned by `estimator` from their pixels
    /// alone. The autopilot steers on a filter of the delivered fixes and
    /// velocity aiding, never on the true position. The start is moved
    /// `config.start_lateral_offset_m` right of the profile's, which neither
    /// the estimator nor the filter is told: both begin at
    /// [`Self::planned_start_enu_m`].
    pub fn image_driven(
        profile: ScenarioProfile,
        dynamics: FlightDynamics,
        config: ImageDrivenConfig,
        estimator: Box<dyn CorridorVisualEstimator>,
    ) -> Result<Self, ScenarioError> {
        let planned = start_of(profile, dynamics);
        let frame = LocalFrame::at(&planned.position);
        let (sin, cos) = planned.heading_rad.sin_cos();
        let right = frame.east() * cos - frame.north() * sin;
        let moved = CorridorStart {
            position: ecef_to_geodetic(
                geodetic_to_ecef(&planned.position) + right * config.start_lateral_offset_m,
            ),
            ..planned
        };
        let (intrinsics, harness) = (config.intrinsics, config.harness);
        let image = ImageMode::new(config, estimator, Self::planned_start_enu_m(profile));
        Self::assemble(
            profile,
            dynamics,
            Source::new(dynamics, &moved)?,
            CameraMount::nadir_belly(DVec3::ZERO),
            intrinsics,
            harness,
            VisualSource::Image(Box::new(image)),
        )
    }

    /// Where the flight plan starts `profile`, in the corridor's
    /// east-north-up frame, metres: what an image-driven run's estimator and
    /// filter are seeded with. Only the start speed depends on the dynamics.
    #[must_use]
    pub fn planned_start_enu_m(profile: ScenarioProfile) -> DVec3 {
        let anchor = anchor_position();
        LocalFrame::at(&anchor).ecef_to_enu_vector(
            geodetic_to_ecef(&start_of(profile, FlightDynamics::Kinematic).position)
                - geodetic_to_ecef(&anchor),
        )
    }

    /// The origin of the corridor's east-north-up frame.
    #[must_use]
    pub fn enu_anchor() -> Geodetic {
        anchor_position()
    }

    /// The image-driven navigation so far; `None` for a run built by
    /// [`Self::new`].
    #[must_use]
    pub fn image_navigation_report(&self) -> Option<ImageNavigationReport> {
        match &self.visual {
            VisualSource::Image(mode) => Some(mode.report()),
            VisualSource::Synthetic(_) => None,
        }
    }
}
