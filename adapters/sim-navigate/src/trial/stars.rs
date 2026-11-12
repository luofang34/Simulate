//! The star camera's scene side: frames rendered from the true poses on the
//! scenario clock, and the centroids Navigate's detector finds in them.
//! The navigator receives the centroids and does the rest.

use glam::DVec3;
use navigate_camera::{ImageView, Pixels};
use navigate_startrack::{DetectConfig, StarCentroid, detect_stars};
use sim_core::{CameraMount, SimInstant, VehicleState};
use sim_geo::{LocalFrame, UtcTime, ecef_to_geodetic};
use sim_render::{CameraPose, StarCamera, StarCameraConfig};

use super::TrialError;
use super::record::TruthAtStep;
use super::sky::{SkySample, sky_brightness, sun_direction_ecef};
use crate::{Truth, camera_stars};

/// Faintest star the scene renders, Johnson V: fainter than the navigator's
/// catalogue, so the detector also meets stars it cannot identify.
pub const SKY_FAINTEST_V: f64 = 6.5;

/// A star camera on the cabin roof, a little off the zenith toward the nose
/// and right wing, and the centre of gravity in the mount's axes.
pub fn roof_star_mount() -> (CameraMount, DVec3) {
    let forward_body = DVec3::new(0.25, 0.12, -1.0).normalize();
    let mount = CameraMount::looking_along(DVec3::new(4.2, 0.6, -2.1), forward_body, DVec3::X);
    (mount, DVec3::new(1.1, 0.0, 0.3))
}

/// How the scene renders an exposure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StarExposure {
    /// The camera moves from its true pose at the shutter's opening to its
    /// true pose at its closing, so stars streak.
    Moving,
    /// The camera holds its true mid-exposure pose, for comparison.
    Frozen,
}

/// Where the sky brightness behind the stars comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StarSky {
    /// The camera configuration's own constant sky.
    Fixed,
    /// The trial's sky-brightness model, for the Sun's place at the frame's
    /// epoch and the camera's line of sight.
    SunLit,
}

/// How frames are taken along a flight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StarScene {
    /// UTC at [`SimInstant::START`].
    pub start: UtcTime,
    /// Simulation steps per second of the truth the frames are taken from.
    pub step_hz: f64,
    /// Simulation steps between shutter openings.
    pub period_steps: usize,
    /// How an exposure moves.
    pub exposure: StarExposure,
    /// What the sky behind the stars is.
    pub sky: StarSky,
    /// The camera noise seed.
    pub seed: u64,
}

/// What the navigator receives of one exposure: the detector's centroids,
/// with the instants the frame is stamped and processed at.
#[derive(Debug, Clone, PartialEq)]
pub struct StarImage {
    /// The step at the middle of the exposure, which sightings are stamped
    /// with.
    pub mid_step: usize,
    /// The step at which the shutter closes and the image can be processed.
    pub end_step: usize,
    /// The instant of `mid_step`.
    pub mid: SimInstant,
    /// Navigate's detections in the rendered counts.
    pub centroids: Vec<StarCentroid>,
}

/// What the scene knew while rendering an exposure. Scoring only.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StarFrameTruth {
    /// How far the camera turned during the exposure, in pixels at the
    /// focal length: the longest streak a star can draw.
    pub streak_px: f64,
    /// The sky behind the stars; `None` for [`StarSky::Fixed`].
    pub sky: Option<SkySample>,
}

/// One exposure.
#[derive(Debug, Clone, PartialEq)]
pub struct StarFrame {
    /// The navigator's part.
    pub image: StarImage,
    /// The scene's part.
    pub truth: Truth<StarFrameTruth>,
}

/// The tracker camera every frame is rendered with.
pub fn star_camera_config() -> StarCameraConfig {
    StarCameraConfig::star_tracker_1024()
}

/// Simulation steps the exposure lasts: a whole, even number, so the
/// opening, middle and closing all fall on truth samples.
pub(super) fn exposure_steps(config: &StarCameraConfig, step_hz: f64) -> Result<usize, TrialError> {
    let steps = config.exposure_s * step_hz;
    let whole = steps.round();
    if (steps - whole).abs() > 1e-9 || whole < 2.0 || !(whole as usize).is_multiple_of(2) {
        return Err(TrialError::StarCamera {
            what: format!("exposure of {steps} steps"),
        });
    }
    Ok(whole as usize)
}

/// Every frame along `truth` from `scene`, rendered from the true poses and
/// put through Navigate's detector.
pub fn render_star_frames(
    truth: &[Truth<(SimInstant, VehicleState)>],
    scene: &StarScene,
) -> Result<Vec<StarFrame>, TrialError> {
    let config = star_camera_config();
    let length = exposure_steps(&config, scene.step_hz)?;
    let opening_steps = 1..truth.len().saturating_sub(length);
    opening_steps
        .step_by(scene.period_steps.max(1))
        .map(|open| render_frame(truth, scene, &config, open, length))
        .collect()
}

pub(super) fn render_frame(
    truth: &(impl TruthAtStep + ?Sized),
    scene: &StarScene,
    config: &StarCameraConfig,
    open: usize,
    length: usize,
) -> Result<StarFrame, TrialError> {
    let (mid_step, end_step) = (open + length / 2, open + length);
    let state_at = |step: usize| {
        truth
            .truth_at(step)
            .map(|s| s.get())
            .ok_or(TrialError::StarCamera {
                what: format!("no truth at step {step}"),
            })
    };
    let (mount, cg_body_m) = roof_star_mount();
    let pose_at = |step: usize| {
        state_at(step).map(|(_, vehicle)| {
            let pose = mount.pose(vehicle, cg_body_m);
            CameraPose {
                position_ecef: pose.position_ecef,
                orientation: pose.orientation,
            }
        })
    };
    let (mid, vehicle) = *state_at(mid_step)?;
    let (opening, closing, middle) = (pose_at(open)?, pose_at(end_step)?, pose_at(mid_step)?);
    let sky = match scene.sky {
        StarSky::Fixed => None,
        StarSky::SunLit => Some(sky_at(scene.start, mid, &vehicle, &middle)?),
    };
    let camera = StarCamera::new(StarCameraConfig {
        seed: scene.seed,
        sky_v_mag_per_arcsec2: sky.map_or(config.sky_v_mag_per_arcsec2, |s| s.v_mag_per_arcsec2),
        ..*config
    })
    .map_err(|error| TrialError::StarCamera {
        what: format!("{error}"),
    })?;
    let stars = camera_stars(scene.start, mid, SKY_FAINTEST_V)?;
    let streak_px = match scene.exposure {
        StarExposure::Moving => {
            opening.orientation.angle_between(closing.orientation) * config.intrinsics.fx
        }
        StarExposure::Frozen => 0.0,
    };
    let image = match scene.exposure {
        StarExposure::Moving => camera.expose_moving(&opening, &closing, &stars, open as u64),
        StarExposure::Frozen => camera.expose(&middle, &stars, open as u64),
    }
    .image;
    Ok(StarFrame {
        image: StarImage {
            mid_step,
            end_step,
            mid,
            centroids: detect(&image, config),
        },
        truth: Truth::new(StarFrameTruth { streak_px, sky }),
    })
}

/// Navigate's detector on the rendered counts. A frame it cannot read, such
/// as one the daylight sky saturates, yields no centroids.
fn detect(image: &sim_render::SensorCounts, config: &StarCameraConfig) -> Vec<StarCentroid> {
    // The gain is a datasheet value of the sensor, not simulator truth.
    let detect = DetectConfig {
        electrons_per_count: config.gain_e_per_dn,
        ..DetectConfig::default()
    };
    let pixels = Pixels::Gray16 {
        data: &image.counts,
        stride: image.width as usize,
        significant_bits: image.significant_bits,
    };
    ImageView::new(image.width, image.height, pixels)
        .ok()
        .and_then(|view| detect_stars(&view, &detect).ok())
        .unwrap_or_default()
}

/// The sky behind the camera at `mid`, from the Sun's topocentric place.
fn sky_at(
    start: UtcTime,
    mid: SimInstant,
    vehicle: &VehicleState,
    pose: &CameraPose,
) -> Result<SkySample, TrialError> {
    let sun = sun_direction_ecef(start, mid, vehicle.position_ecef)?;
    let up = LocalFrame::at(&ecef_to_geodetic(vehicle.position_ecef)).up();
    let line_of_sight = pose.orientation * DVec3::NEG_Z;
    Ok(sky_brightness(up, sun, line_of_sight))
}
