//! Image-driven corridor positioning: a nadir camera's rendered pixels,
//! the attitude reference and the radar altimeter in, east-north-up fixes
//! out, with no access to the true position.
//!
//! Every frame is rectified onto level ground with the attitude and height.
//! The corridor anchor registers that ground patch against the reference
//! orthoimagery by coarse-to-fine ZNCC inside the current uncertainty
//! window; the warm track follows Shi-Tomasi corners from a keyframe patch
//! by ZNCC and chains the motion onto the last anchor.

use std::sync::Arc;

use glam::{DMat3, DVec2, DVec3};
use sim_core::{RateAdapter, SimInstant, TickSpan, VisualEstimate};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_scenario::{CorridorVisualEstimator, SensorObservation, VisualTiers};
use sim_scenery::CatalogScenery;

mod odometry;
mod patch;
mod zncc;

#[cfg(test)]
mod tests;

use odometry::Keyframe;
use patch::{boresight_offset, metres_per_radian, rectified_patch, reference_patch};

/// Fixed setup of an [`ImageCorridorEstimator`].
#[derive(Debug, Clone)]
pub struct ImageEstimatorConfig {
    /// The reference orthoimagery and DEM the anchor registers against.
    pub reference: Arc<CatalogScenery>,
    /// Origin of the corridor's east-north-up frame.
    pub anchor: Geodetic,
    /// Where the flight plan says the camera starts, in that frame, metres.
    pub seed_position_enu_m: DVec3,
    /// One-sigma horizontal uncertainty of the seed, metres.
    pub seed_sigma_m: f64,
    /// How often the anchor tier runs.
    pub anchor_period: TickSpan,
    /// Cells along a ground patch's side.
    pub patch_cells: usize,
    /// Ground sample distance of the patches, metres.
    pub gsd_m: f64,
    /// Widest anchor search either side of the prior, metres.
    pub max_search_m: f64,
    /// Lowest ZNCC an anchor registration is accepted at.
    pub min_anchor_score: f64,
    /// One-sigma attitude reference error, radians.
    pub attitude_sigma_rad: f64,
    /// One-sigma radar altimeter error, metres.
    pub agl_sigma_m: f64,
}

impl ImageEstimatorConfig {
    /// A 64-cell, 10 m patch, 1 Hz anchor setup over `reference`, seeded at
    /// `seed_position_enu_m` with `seed_sigma_m` of doubt.
    #[must_use]
    pub fn new(
        reference: Arc<CatalogScenery>,
        anchor: Geodetic,
        seed_position_enu_m: DVec3,
        seed_sigma_m: f64,
    ) -> Self {
        Self {
            reference,
            anchor,
            seed_position_enu_m,
            seed_sigma_m,
            anchor_period: TickSpan::from_ticks(sim_core::TICKS_PER_SECOND),
            patch_cells: 64,
            gsd_m: 10.0,
            max_search_m: 400.0,
            min_anchor_score: 0.5,
            attitude_sigma_rad: 0.002,
            agl_sigma_m: 3.0,
        }
    }
}

/// Positions a nadir camera from its pixels; see the module documentation.
#[derive(Debug, Clone)]
pub struct ImageCorridorEstimator {
    config: ImageEstimatorConfig,
    origin_ecef: DVec3,
    anchor_frame: LocalFrame,
    anchor_rate: RateAdapter,
    last_frame: Option<SimInstant>,
    /// The estimator's own belief, which sets the anchor search window.
    prior_enu_m: DVec3,
    prior_horizontal_var_m2: f64,
    anchored: bool,
    keyframe: Option<Keyframe>,
    stats: ImageEstimatorStats,
}

/// What an [`ImageCorridorEstimator`] has done so far.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ImageEstimatorStats {
    /// Frames processed.
    pub frames: u64,
    /// Anchor registrations attempted.
    pub anchor_attempts: u64,
    /// Anchor registrations accepted.
    pub anchors: u64,
    /// Warm-track estimates produced.
    pub warm_tracks: u64,
}

impl ImageCorridorEstimator {
    /// An estimator that has seen nothing yet.
    #[must_use]
    pub fn new(config: ImageEstimatorConfig) -> Self {
        let sigma = config.seed_sigma_m;
        Self {
            origin_ecef: geodetic_to_ecef(&config.anchor),
            anchor_frame: LocalFrame::at(&config.anchor),
            anchor_rate: RateAdapter::new(config.anchor_period),
            last_frame: None,
            prior_enu_m: config.seed_position_enu_m,
            prior_horizontal_var_m2: sigma * sigma,
            anchored: false,
            keyframe: None,
            stats: ImageEstimatorStats::default(),
            config,
        }
    }

    /// What it has done so far.
    #[must_use]
    pub fn stats(&self) -> ImageEstimatorStats {
        self.stats
    }

    fn enu_to_geodetic(&self, enu: DVec3) -> Geodetic {
        ecef_to_geodetic(self.origin_ecef + self.anchor_frame.enu_to_ecef() * enu)
    }

    fn geodetic_to_enu(&self, position: &Geodetic) -> DVec3 {
        self.anchor_frame
            .ecef_to_enu_vector(geodetic_to_ecef(position) - self.origin_ecef)
    }

    /// The eye `east_m`, `north_m` from the ground point under `from`, at
    /// `agl_m` above the ground there.
    fn eye_at(&self, from: &Geodetic, offset_m: DVec2, agl_m: f64) -> DVec3 {
        let radii = metres_per_radian(from.lat_rad);
        let mut eye = Geodetic {
            lat_rad: from.lat_rad + offset_m.y / radii.x,
            lon_rad: from.lon_rad + offset_m.x / radii.y,
            height_m: 0.0,
        };
        let ground = self
            .config
            .reference
            .ellipsoidal_ground_height_m(&eye)
            .ok()
            .flatten()
            .map_or(from.height_m - agl_m, |height| height.0);
        eye.height_m = ground + agl_m;
        self.geodetic_to_enu(&eye)
    }

    /// The reference DEM's ground relative to an eye `agl_m` above the
    /// ground under `eye`, at an east and north offset from that point.
    fn ground_relief(&self, eye: &Geodetic, agl_m: f64) -> impl Fn(DVec2) -> f64 + '_ {
        let radii = metres_per_radian(eye.lat_rad);
        let (lat_rad, lon_rad) = (eye.lat_rad, eye.lon_rad);
        let height_at = move |offset: DVec2| {
            let point = Geodetic {
                lat_rad: lat_rad + offset.y / radii.x,
                lon_rad: lon_rad + offset.x / radii.y,
                height_m: 0.0,
            };
            self.config
                .reference
                .ellipsoidal_ground_height_m(&point)
                .ok()
                .flatten()
                .map(|height| height.0)
        };
        let below = height_at(DVec2::ZERO);
        move |offset| match (below, height_at(offset)) {
            (Some(below), Some(there)) => there - below - agl_m,
            _ => -agl_m,
        }
    }

    /// Anchor variances, horizontal and vertical, for a patch centred
    /// `centre_m` from the point below a camera `agl_m` up: an attitude
    /// error moves an oblique patch further than a vertical one.
    fn anchor_sigmas(&self, centre_m: DVec2, agl_m: f64) -> (f64, f64) {
        let config = &self.config;
        let lever_m = (agl_m * agl_m + centre_m.length_squared()) / agl_m.max(1.0);
        let horizontal = (0.3 * config.gsd_m).powi(2)
            + (lever_m * config.attitude_sigma_rad).powi(2)
            + RELIEF_SIGMA_M * RELIEF_SIGMA_M;
        let vertical = config.agl_sigma_m.powi(2) + DEM_SIGMA_M * DEM_SIGMA_M;
        (horizontal, vertical)
    }

    /// Registers the observed patch, centred `centre_m` from the point below
    /// the camera, against the reference around the prior.
    fn anchor(
        &mut self,
        observed: &patch::GroundPatch,
        centre_m: DVec2,
        agl_m: f64,
    ) -> Option<VisualEstimate> {
        self.stats.anchor_attempts = self.stats.anchor_attempts.wrapping_add(1);
        let config = &self.config;
        let reach_m = (3.0 * self.prior_horizontal_var_m2.sqrt())
            .clamp(3.0 * config.gsd_m, config.max_search_m);
        let margin = (reach_m / config.gsd_m).ceil() as usize;
        let prior = self.enu_to_geodetic(self.prior_enu_m);
        let radii = metres_per_radian(prior.lat_rad);
        let look = Geodetic {
            lat_rad: prior.lat_rad + centre_m.y / radii.x,
            lon_rad: prior.lon_rad + centre_m.x / radii.y,
            ..prior
        };
        let search = reference_patch(
            &config.reference,
            &look,
            observed.size() + 2 * margin,
            config.gsd_m,
        );
        let (offset_cells, score) = zncc::register(observed, &search, margin)?;
        if score < config.min_anchor_score {
            return None;
        }
        let offset_m = DVec2::new(offset_cells.x, -offset_cells.y) * config.gsd_m;
        let position = self.eye_at(&prior, offset_m, agl_m);
        let (horizontal, vertical) = self.anchor_sigmas(centre_m, agl_m);
        self.prior_enu_m = position;
        self.prior_horizontal_var_m2 = horizontal;
        self.anchored = true;
        self.stats.anchors = self.stats.anchors.wrapping_add(1);
        Some(VisualEstimate {
            position_enu_m: position,
            covariance_enu_m2: DMat3::from_diagonal(DVec3::new(horizontal, horizontal, vertical)),
        })
    }

    /// Whether the anchor tier runs on a frame at `now`, on the same clock
    /// rule the harness's anchor loop uses.
    fn anchor_due(&mut self, now: SimInstant) -> bool {
        let elapsed = self.last_frame.map(|last| now.duration_since(last));
        self.last_frame = Some(now);
        elapsed.is_none_or(|span| self.anchor_rate.steps_for_u128(span) > 0)
    }
}

/// Unmodelled relief displacement in a level-ground rectification, metres.
const RELIEF_SIGMA_M: f64 = 4.0;
/// The reference DEM's own height error, metres.
const DEM_SIGMA_M: f64 = 2.0;

impl CorridorVisualEstimator for ImageCorridorEstimator {
    fn estimate(&mut self, observation: &SensorObservation) -> VisualTiers {
        self.stats.frames = self.stats.frames.wrapping_add(1);
        let anchor_due = self.anchor_due(observation.frame.timestamp);
        let agl_m = observation.agl_estimate_m;
        if !(agl_m.is_finite() && agl_m > 1.0) {
            return VisualTiers::default();
        }
        let prior = self.enu_to_geodetic(self.prior_enu_m);
        let camera_to_level = LocalFrame::at(&prior).enu_to_ecef().transpose()
            * DMat3::from_quat(observation.camera_to_ecef_estimate);
        let Some(centre_m) = boresight_offset(camera_to_level, agl_m) else {
            self.keyframe = None;
            return VisualTiers::default();
        };
        let observed = rectified_patch(
            &observation.frame.image,
            &observation.frame.intrinsics,
            camera_to_level,
            centre_m,
            self.ground_relief(&prior, agl_m),
            self.config.patch_cells,
            self.config.gsd_m,
        );
        let corridor_anchor = if anchor_due && observed.coverage() > 0.8 {
            self.anchor(&observed, centre_m, agl_m)
        } else {
            None
        };
        let warm_track = self.track(observed, centre_m, corridor_anchor, agl_m);
        if warm_track.is_some() {
            self.stats.warm_tracks = self.stats.warm_tracks.wrapping_add(1);
        }
        VisualTiers {
            warm_track,
            corridor_anchor,
        }
    }

    fn boxed_clone(&self) -> Box<dyn CorridorVisualEstimator> {
        Box::new(self.clone())
    }
}
