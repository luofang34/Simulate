use glam::DVec3;

use super::{
    CatalogStar, StarCamera, StarCameraExposure, StarCameraTruth, StarTruth, geometry, psf,
};
use crate::CameraPose;

#[cfg(test)]
mod tests;

/// Largest sub-step spacing along a streak, in PSF standard deviations: at
/// half a sigma, the sum of the sub-step images is smooth to well under a
/// percent, so the streak is continuous.
const SUBSTEP_SPACING_SIGMA: f64 = 0.5;
/// Most sub-steps for one star, which bounds the work for a camera that
/// turns through a large part of the sky in one exposure.
const MAX_SUBSTEPS: usize = 1 << 14;
/// Sub-steps for a star whose path cannot be measured in pixels because part
/// of it is behind the camera.
const UNMEASURED_SUBSTEPS: usize = 256;

/// The camera pose a fraction `t` of the way through an exposure.
fn pose_at(start: &CameraPose, end: &CameraPose, t: f64) -> CameraPose {
    start.interpolated(end, t)
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

impl StarCamera {
    /// The mean electrons in every pixel and the truth of an exposure during
    /// which the camera moves from `start` to `end` at constant velocity and
    /// constant angular rate. A body rate `ω` over an exposure `t` from
    /// orientation `q` ends at `q · exp(ω t)`.
    ///
    /// Each star's light is split over sub-steps at the midpoints of equal
    /// time slices, spaced at most half a PSF sigma apart along its path, so
    /// a streak is continuous and holds the star's whole signal. The truth
    /// centroid and visibility are at mid-exposure, the instant a frame is
    /// stamped with; the streak ends are at the exposure's start and end.
    /// The Earth may hide part of a streak, which then carries only the
    /// unhidden part of the signal.
    pub fn expected_electrons_moving(
        &self,
        start: &CameraPose,
        end: &CameraPose,
        stars: &[CatalogStar],
    ) -> (Vec<f64>, StarCameraTruth) {
        let (mut image, background) = self.background_image();
        let middle = pose_at(start, end, 0.5);
        let truth = stars
            .iter()
            .map(|star| self.streak(&mut image, start, &middle, end, star))
            .collect();
        (
            image,
            StarCameraTruth {
                stars: truth,
                background_electrons_per_px: background,
            },
        )
    }

    /// Exposes frame `sequence` while the camera moves from `start` to `end`;
    /// see [`StarCamera::expected_electrons_moving`]. With `start == end` this
    /// is exactly [`StarCamera::expose`].
    pub fn expose_moving(
        &self,
        start: &CameraPose,
        end: &CameraPose,
        stars: &[CatalogStar],
        sequence: u64,
    ) -> StarCameraExposure {
        if start == end {
            return self.expose(start, stars, sequence);
        }
        let (electrons, truth) = self.expected_electrons_moving(start, end, stars);
        StarCameraExposure {
            image: self.counts(&electrons, sequence),
            truth,
        }
    }

    /// Adds one star's streak to `image` and gives its truth.
    fn streak(
        &self,
        image: &mut [f64],
        start: &CameraPose,
        middle: &CameraPose,
        end: &CameraPose,
        star: &CatalogStar,
    ) -> StarTruth {
        let pixel = |pose: &CameraPose| {
            self.projection
                .pixel_of(pose.orientation.inverse() * star.direction_ecef)
        };
        let mut truth = self.star_truth(
            middle,
            middle.orientation.inverse() * star.direction_ecef,
            star,
        );
        truth.streak_start_px = pixel(start);
        truth.streak_end_px = pixel(end);
        let path = [
            truth.streak_start_px,
            truth.centroid_px,
            truth.streak_end_px,
        ];
        if !star.v_magnitude.is_finite() || !self.path_may_touch_image(&path) {
            truth.signal_electrons = 0.0;
            return truth;
        }
        let steps = self.substeps(&path);
        let electrons = self.config.star_electrons(star.v_magnitude);
        let per_step = electrons / steps as f64;
        let lit = (0..steps)
            .filter(|&step| {
                let t = (step as f64 + 0.5) / steps as f64;
                let pose = pose_at(start, end, t);
                self.deposit(image, &pose, star.direction_ecef, per_step)
            })
            .count();
        truth.signal_electrons = electrons * lit as f64 / steps as f64;
        truth
    }

    /// Adds `electrons` of a star in `direction_ecef` seen from `pose`;
    /// whether the star was in front of the camera and not hidden.
    fn deposit(
        &self,
        image: &mut [f64],
        pose: &CameraPose,
        direction_ecef: DVec3,
        electrons: f64,
    ) -> bool {
        let Some(centre) = self
            .projection
            .pixel_of(pose.orientation.inverse() * direction_ecef)
        else {
            return false;
        };
        let atmosphere = self.config.atmosphere_height_m;
        if geometry::earth_blocks(pose.position_ecef, direction_ecef, atmosphere) {
            return false;
        }
        let intrinsics = self.config.intrinsics;
        let size = [intrinsics.width, intrinsics.height];
        psf::add_star(image, size, centre, self.config.psf_sigma_px, electrons);
        true
    }

    /// Sub-steps that keep a path through these pixels continuous.
    fn substeps(&self, path: &[Option<[f64; 2]>; 3]) -> usize {
        let [Some(first), Some(middle), Some(last)] = *path else {
            return UNMEASURED_SUBSTEPS;
        };
        let length = distance(first, middle) + distance(middle, last);
        let spacing = SUBSTEP_SPACING_SIGMA * self.config.psf_sigma_px;
        ((length / spacing).ceil() as usize).clamp(1, MAX_SUBSTEPS)
    }

    /// Whether a path through these pixels can put light on the detector:
    /// its bounding box, grown by the PSF's reach, meets the image. A path
    /// partly behind the camera is checked sub-step by sub-step instead.
    fn path_may_touch_image(&self, path: &[Option<[f64; 2]>; 3]) -> bool {
        let known: Vec<[f64; 2]> = path.iter().flatten().copied().collect();
        if known.is_empty() {
            return false;
        }
        if known.len() < path.len() {
            return true;
        }
        let reach = 5.0 * self.config.psf_sigma_px + 1.0;
        let intrinsics = self.config.intrinsics;
        let fits = |coordinates: Vec<f64>, extent: u32| {
            let low = coordinates.iter().copied().fold(f64::INFINITY, f64::min);
            let high = coordinates
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);
            high + reach >= -0.5 && low - reach <= f64::from(extent) - 0.5
        };
        fits(known.iter().map(|[x, _]| *x).collect(), intrinsics.width)
            && fits(known.iter().map(|[_, y]| *y).collect(), intrinsics.height)
    }
}
