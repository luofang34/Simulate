//! The scene side of the star + INS loop: star-camera frames rendered from
//! the true poses, and the navigator's catalogue tables.

use glam::DVec3;
use navigate_camera::{ImageView, Pixels};
use navigate_celestial::StarSight;
use navigate_startrack::{DetectConfig, PairCatalogue, StarCentroid, catalogue_ecef, detect_stars};
use sim_core::{CameraMount, SimInstant, VehicleState};
use sim_geo::UtcTime;
use sim_navigate::{Truth, camera_stars, star_camera_epoch};
use sim_render::{CameraPose, StarCamera, StarCameraConfig};

use super::{
    CATALOGUE_FAINTEST_V, LIS_FAINTEST_V, SKY_FAINTEST_V, STAR_PERIOD_STEPS, START_UNIX_S, STEP_HZ,
};

/// A star camera on the cabin roof, a little off the zenith toward the nose
/// and right wing, and the centre of gravity in the mount's axes.
pub(super) fn sim_mount() -> (CameraMount, DVec3) {
    let forward_body = DVec3::new(0.25, 0.12, -1.0).normalize();
    let mount = CameraMount::looking_along(DVec3::new(4.2, 0.6, -2.1), forward_body, DVec3::X);
    (mount, DVec3::new(1.1, 0.0, 0.3))
}

/// How the scene renders an exposure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Exposure {
    /// The camera moves from its true pose at the shutter's opening to its
    /// true pose at its closing, so stars streak.
    Moving,
    /// The camera holds its true mid-exposure pose, for comparison.
    Frozen,
}

/// What the navigator has of one exposure: centroids from the counts and the
/// catalogue at the mid-exposure epoch. The counts come from the true poses;
/// that is the scene, not an input to the navigator.
pub(super) struct StarFrame {
    /// The step at the middle of the exposure, which the sightings are
    /// stamped with.
    pub(super) mid_step: usize,
    /// The step at which the shutter closes and the image can be processed.
    pub(super) end_step: usize,
    pub(super) mid: SimInstant,
    /// How far the camera turned during the exposure, in pixels at the
    /// focal length: the longest streak a star can draw. Scoring only.
    pub(super) streak_px: Truth<f64>,
    pub(super) centroids: Vec<StarCentroid>,
    pub(super) catalogue: Vec<StarSight>,
}

/// Simulation steps the exposure lasts; it must be a whole, even number so
/// the opening, middle and closing all fall on truth samples.
fn exposure_steps(config: &StarCameraConfig) -> usize {
    let steps = config.exposure_s * STEP_HZ;
    let whole = steps.round();
    assert!(
        (steps - whole).abs() < 1e-9 && (whole as usize).is_multiple_of(2),
        "{steps}"
    );
    whole as usize
}

pub(super) fn star_frames(
    truth: &[Truth<(SimInstant, VehicleState)>],
    exposure: Exposure,
) -> Vec<StarFrame> {
    let config = StarCameraConfig::star_tracker_1024();
    let camera = StarCamera::new(StarCameraConfig { seed: 7, ..config }).unwrap();
    let length = exposure_steps(&config);
    let (mount, cg_body_m) = sim_mount();
    let start = UtcTime::from_unix_seconds(START_UNIX_S);
    let pose_at = |step: usize| {
        let pose = mount.pose(&truth[step].get().1, cg_body_m);
        CameraPose {
            position_ecef: pose.position_ecef,
            orientation: pose.orientation,
        }
    };
    (1..truth.len().saturating_sub(length))
        .step_by(STAR_PERIOD_STEPS)
        .map(|open| {
            let (mid_step, end_step) = (open + length / 2, open + length);
            let mid = truth[mid_step].get().0;
            let sky = camera_stars(start, mid, SKY_FAINTEST_V).unwrap();
            let turn_rad = pose_at(open)
                .orientation
                .angle_between(pose_at(end_step).orientation);
            let streak_px = match exposure {
                Exposure::Moving => turn_rad * config.intrinsics.fx,
                Exposure::Frozen => 0.0,
            };
            let image = match exposure {
                Exposure::Moving => {
                    camera.expose_moving(&pose_at(open), &pose_at(end_step), &sky, open as u64)
                }
                Exposure::Frozen => camera.expose(&pose_at(mid_step), &sky, open as u64),
            }
            .image;
            let pixels = Pixels::Gray16 {
                data: &image.counts,
                stride: image.width as usize,
                significant_bits: image.significant_bits,
            };
            let view = ImageView::new(image.width, image.height, pixels).unwrap();
            StarFrame {
                mid_step,
                end_step,
                mid,
                streak_px: Truth::new(streak_px),
                centroids: detect_stars(&view, &DetectConfig::default()).unwrap_or_default(),
                catalogue: catalogue_ecef(star_camera_epoch(start, mid), CATALOGUE_FAINTEST_V)
                    .unwrap(),
            }
        })
        .collect()
}

/// The lost-in-space pair table for the star camera's diagonal field of
/// view, made once from the catalogue at the scenario start; pair angles do
/// not change with the Earth's rotation.
pub(super) fn pair_table() -> PairCatalogue {
    let intrinsics = StarCameraConfig::star_tracker_1024().intrinsics;
    let half_diagonal_px = f64::from(intrinsics.width).hypot(f64::from(intrinsics.height)) / 2.0;
    let diagonal_rad = 2.0 * (half_diagonal_px / intrinsics.fx).atan();
    let start = UtcTime::from_unix_seconds(START_UNIX_S);
    let catalogue =
        catalogue_ecef(star_camera_epoch(start, SimInstant::START), LIS_FAINTEST_V).unwrap();
    PairCatalogue::new(&catalogue, diagonal_rad, LIS_FAINTEST_V).unwrap()
}
