use glam::DVec3;

use super::noise::{NoiseStream, NoiseStreamId};
use super::readout::{SensorCounts, digitise, full_scale};
use crate::{CameraPose, PinholeProjection, RenderError};

mod config;
mod geometry;
mod motion;
mod psf;

#[cfg(test)]
mod tests;

pub use config::{
    NIGHT_SKY_V_MAG_PER_ARCSEC2, STAR_ATMOSPHERE_HEIGHT_M, StarCameraConfig,
    V_ZERO_POINT_PHOTONS_PER_M2_S,
};

/// Near plane of the projection a star camera borrows; stars are at infinity,
/// so its value never matters.
const STAR_NEAR_M: f64 = 1.0;

/// One star as the camera's caller sees it at the exposure instant. The
/// caller applies proper motion, aberration and the Earth's orientation, so
/// this renderer needs no catalogue or ephemeris of its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CatalogStar {
    /// Catalogue number, such as the Harvard Revised (HR) number.
    pub id: u16,
    /// Unit vector toward the star's apparent place, ECEF axes.
    pub direction_ecef: DVec3,
    /// Johnson `V` magnitude.
    pub v_magnitude: f64,
}

/// Why a star does or does not appear in a frame. When more than one reason
/// applies, the field of view wins: a star off the detector is
/// [`StarVisibility::OutOfFov`] whether or not the Earth is also in the way.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StarVisibility {
    /// The star's centroid is on the detector and nothing blocks it.
    Visible,
    /// The star's centroid is on the detector but the Earth or its blocking
    /// atmosphere is in the way.
    Occluded,
    /// The star's centroid is off the detector or behind the camera.
    OutOfFov,
}

/// What the simulator knows about one star in a frame; never part of the
/// pixels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StarTruth {
    /// The star's [`CatalogStar::id`].
    pub id: u16,
    /// Where the star's light centres at mid-exposure, the instant a frame
    /// is stamped with, in the intrinsics' pixel coordinates (pixel centres
    /// on integers); `None` behind the camera.
    pub centroid_px: Option<[f64; 2]>,
    /// Where the star's light centres at the exposure start; the same as
    /// `centroid_px` for a camera that does not move during the exposure.
    pub streak_start_px: Option<[f64; 2]>,
    /// Where the star's light centres at the exposure end.
    pub streak_end_px: Option<[f64; 2]>,
    /// Mean signal electrons the star adds to this frame, summed over its
    /// whole point-spread function including any part beyond the image edge;
    /// zero when the Earth blocks it or it is behind the camera.
    pub signal_electrons: f64,
    /// Whether the star is imaged.
    pub visibility: StarVisibility,
}

/// The truth of one star-camera frame, for evaluation only.
#[derive(Debug, Clone, PartialEq)]
pub struct StarCameraTruth {
    /// One entry per input star, in input order.
    pub stars: Vec<StarTruth>,
    /// Mean sky plus dark electrons in every pixel.
    pub background_electrons_per_px: f64,
}

/// One star-camera frame: raw counts for the navigator and, separately, the
/// truth to score it against.
#[derive(Debug, Clone, PartialEq)]
pub struct StarCameraExposure {
    /// What the detector reads out.
    pub image: SensorCounts,
    /// What the simulator knows.
    pub truth: StarCameraTruth,
}

/// A radiometric star camera on the CPU: deterministic, headless, and
/// independent of any GPU.
///
/// [`StarCamera::expose`] holds the camera still for the exposure;
/// [`StarCamera::expose_moving`] smears each star along the path the camera
/// turns through.
///
/// Not modelled: atmospheric extinction, scintillation and refraction along
/// the line of sight; daylight and twilight sky gradients (the sky is one
/// configurable brightness); stray light from the Sun, Moon, or Earth limb;
/// rolling-shutter skew (the whole detector integrates over the same
/// interval); PSF variation across the field; hot pixels and dark-current
/// non-uniformity.
#[derive(Debug, Clone, PartialEq)]
pub struct StarCamera {
    config: StarCameraConfig,
    projection: PinholeProjection,
}

impl StarCamera {
    /// A camera of `config`. Fails on unusable optics or detector values.
    pub fn new(config: StarCameraConfig) -> Result<Self, RenderError> {
        config.validate()?;
        let projection = PinholeProjection::new(config.intrinsics, STAR_NEAR_M)?;
        Ok(Self { config, projection })
    }

    /// The camera's configuration.
    pub fn config(&self) -> &StarCameraConfig {
        &self.config
    }

    /// The mean electrons in every pixel, row-major, and the frame's truth.
    /// No noise: this is the expectation [`StarCamera::expose`] samples.
    pub fn expected_electrons(
        &self,
        pose: &CameraPose,
        stars: &[CatalogStar],
    ) -> (Vec<f64>, StarCameraTruth) {
        let intrinsics = self.config.intrinsics;
        let size = [intrinsics.width, intrinsics.height];
        let (mut image, background) = self.background_image();
        let to_camera = pose.orientation.inverse();
        let truth = stars
            .iter()
            .map(|star| {
                let truth = self.star_truth(pose, to_camera * star.direction_ecef, star);
                if let (Some(centre), true) = (truth.centroid_px, truth.signal_electrons > 0.0) {
                    psf::add_star(
                        &mut image,
                        size,
                        centre,
                        self.config.psf_sigma_px,
                        truth.signal_electrons,
                    );
                }
                truth
            })
            .collect();
        (
            image,
            StarCameraTruth {
                stars: truth,
                background_electrons_per_px: background,
            },
        )
    }

    /// Exposes frame `sequence` at `pose`: shot noise on signal, sky and dark
    /// current, then full-well clipping, read noise, gain, bias and the ADC.
    /// The same configuration, seed, pose, stars and sequence give the same
    /// counts.
    pub fn expose(
        &self,
        pose: &CameraPose,
        stars: &[CatalogStar],
        sequence: u64,
    ) -> StarCameraExposure {
        let (electrons, truth) = self.expected_electrons(pose, stars);
        StarCameraExposure {
            image: self.counts(&electrons, sequence),
            truth,
        }
    }

    fn star_truth(
        &self,
        pose: &CameraPose,
        camera_direction: DVec3,
        star: &CatalogStar,
    ) -> StarTruth {
        let intrinsics = self.config.intrinsics;
        let centroid_px = self.projection.pixel_of(camera_direction);
        let on_detector = centroid_px.is_some_and(|[x, y]| {
            geometry::on_detector(x, intrinsics.width)
                && geometry::on_detector(y, intrinsics.height)
        });
        let blocked = geometry::earth_blocks(
            pose.position_ecef,
            star.direction_ecef,
            self.config.atmosphere_height_m,
        );
        let visibility = match (on_detector, blocked) {
            (false, _) => StarVisibility::OutOfFov,
            (true, true) => StarVisibility::Occluded,
            (true, false) => StarVisibility::Visible,
        };
        let lit = centroid_px.is_some() && !blocked && star.v_magnitude.is_finite();
        StarTruth {
            id: star.id,
            centroid_px,
            streak_start_px: centroid_px,
            streak_end_px: centroid_px,
            signal_electrons: if lit {
                self.config.star_electrons(star.v_magnitude)
            } else {
                0.0
            },
            visibility,
        }
    }

    fn counts(&self, electrons: &[f64], sequence: u64) -> SensorCounts {
        let intrinsics = self.config.intrinsics;
        SensorCounts {
            width: intrinsics.width,
            height: intrinsics.height,
            significant_bits: self.config.significant_bits,
            counts: self.read_out(electrons, sequence),
        }
    }

    fn background_image(&self) -> (Vec<f64>, f64) {
        let intrinsics = self.config.intrinsics;
        let pixels = intrinsics.width as usize * intrinsics.height as usize;
        let background = self.config.sky_electrons_per_px() + self.config.dark_electrons_per_px();
        (vec![background; pixels], background)
    }

    fn read_out(&self, electrons: &[f64], sequence: u64) -> Vec<u16> {
        let config = &self.config;
        let ceiling = full_scale(config.significant_bits);
        let mut noise = NoiseStream::new(config.seed, sequence, NoiseStreamId::Detector);
        electrons
            .iter()
            .map(|&mean| {
                let collected = noise.poisson(mean).min(config.full_well_e);
                let read = config.read_noise_e * noise.gaussian();
                digitise(
                    config.bias_dn + (collected + read) / config.gain_e_per_dn,
                    ceiling,
                )
            })
            .collect()
    }
}
