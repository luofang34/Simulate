//! A deterministic CPU pinhole camera over a [`CatalogScenery`]: each pixel's
//! ray is intersected with the DEM and the orthoimagery there is sampled, so
//! a headless run sees the same ground a GPU frame would, without a device.

use glam::{DQuat, DVec3};
use sim_geo::{LocalFrame, SensorIntrinsics, WGS84_A, WGS84_E2, ecef_to_geodetic};

use crate::{CatalogScenery, SceneryError};

#[cfg(test)]
mod tests;

/// Fixed-point passes refining each ray's ground height; relief of a few
/// tens of metres converges to well under a metre in this many.
const GROUND_PASSES: u32 = 3;
/// Rays climbing above this slope below the horizon see no ground.
const MIN_DOWNWARD: f64 = 1e-6;

/// An 8-bit single-channel image, row-major, top row first. Pixel `(x, y)`
/// is centred on the image coordinate `(x, y)`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LumaImage {
    width: u32,
    height: u32,
    pixels: Vec<u8>,
}

impl LumaImage {
    /// An image over `pixels`, which must exactly fill the dimensions.
    pub fn new(width: u32, height: u32, pixels: Vec<u8>) -> Result<Self, SceneryError> {
        let expected = (width as usize).checked_mul(height as usize);
        if expected != Some(pixels.len()) || pixels.is_empty() {
            return Err(SceneryError::InvalidRaster {
                width,
                height,
                expected: expected.unwrap_or(usize::MAX),
                actual: pixels.len(),
            });
        }
        Ok(Self {
            width,
            height,
            pixels,
        })
    }

    /// Width in pixels.
    #[must_use]
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Height in pixels.
    #[must_use]
    pub fn height(&self) -> u32 {
        self.height
    }

    /// The pixels, row-major, top row first.
    #[must_use]
    pub fn pixels(&self) -> &[u8] {
        &self.pixels
    }

    /// The pixel at `(x, y)`, or `None` outside the image.
    #[must_use]
    pub fn at(&self, x: u32, y: u32) -> Option<u8> {
        if x >= self.width || y >= self.height {
            return None;
        }
        self.pixels
            .get(y as usize * self.width as usize + x as usize)
            .copied()
    }

    /// Bilinear value at the image coordinate `(x, y)`; `None` outside the
    /// pixel centres' hull.
    #[must_use]
    pub fn sample_bilinear(&self, x: f64, y: f64) -> Option<f64> {
        let max_x = f64::from(self.width - 1);
        let max_y = f64::from(self.height - 1);
        if !(x >= 0.0 && y >= 0.0 && x <= max_x && y <= max_y) {
            return None;
        }
        let (x0, y0) = (
            x.floor().min(max_x - 1.0).max(0.0),
            y.floor().min(max_y - 1.0).max(0.0),
        );
        let (tx, ty) = (x - x0, y - y0);
        let (ix, iy) = (x0 as u32, y0 as u32);
        let pixel = |dx: u32, dy: u32| {
            self.at(
                (ix + dx).min(self.width - 1),
                (iy + dy).min(self.height - 1),
            )
            .map(f64::from)
        };
        let top = pixel(0, 0)? * (1.0 - tx) + pixel(1, 0)? * tx;
        let bottom = pixel(0, 1)? * (1.0 - tx) + pixel(1, 1)? * tx;
        Some(top * (1.0 - ty) + bottom * ty)
    }
}

/// The ground seen by a pinhole camera at `eye_ecef`, where `camera_to_ecef`
/// rotates camera components (`+X` right, `+Y` up, looking along `-Z`, the
/// renderer's convention) into WGS84 body-fixed ones.
///
/// Each ray is intersected with the catalog DEM by fixed-point refinement in
/// the eye's tangent frame, corrected for the ellipsoid's curvature, and the
/// deepest resident imagery there is sampled as luma. Sky, and ground with
/// no imagery, are black.
pub fn render_luma_view(
    scenery: &CatalogScenery,
    eye_ecef: DVec3,
    camera_to_ecef: DQuat,
    intrinsics: &SensorIntrinsics,
) -> Result<LumaImage, SceneryError> {
    if !intrinsics.is_usable() || !eye_ecef.is_finite() || !camera_to_ecef.is_finite() {
        return Err(SceneryError::InvalidRaster {
            width: intrinsics.width,
            height: intrinsics.height,
            expected: 0,
            actual: 0,
        });
    }
    let eye = ecef_to_geodetic(eye_ecef);
    let frame = LocalFrame::at(&eye);
    let w2 = 1.0 - WGS84_E2 * eye.lat_rad.sin().powi(2);
    let prime_vertical = WGS84_A / w2.sqrt();
    let meridional = prime_vertical * (1.0 - WGS84_E2) / w2;
    let east_radius = prime_vertical * eye.lat_rad.cos().max(1e-9);
    let mean_radius = (prime_vertical * meridional).sqrt();
    let below_eye = scenery
        .ellipsoidal_ground_height_m(&eye)?
        .map_or(0.0, |height| height.0);
    let ground_luma = |ray: DVec3| -> Result<u8, SceneryError> {
        let (east, north, up) = (
            ray.dot(frame.east()),
            ray.dot(frame.north()),
            ray.dot(frame.up()),
        );
        if up > -MIN_DOWNWARD {
            return Ok(0);
        }
        let mut ground = below_eye;
        let mut spot = eye;
        for _ in 0..GROUND_PASSES {
            let reach = (eye.height_m - ground) / -up;
            let (d_east, d_north) = (east * reach, north * reach);
            spot.lat_rad = eye.lat_rad + d_north / meridional;
            spot.lon_rad = eye.lon_rad + d_east / east_radius;
            let drop = (d_east * d_east + d_north * d_north) / (2.0 * mean_radius);
            if let Some(height) = scenery.ellipsoidal_ground_height_m(&spot)? {
                ground = height.0 - drop;
            }
        }
        Ok(scenery
            .imagery_luma_at(&spot)
            .map_or(0, |luma| luma.round().clamp(0.0, 255.0) as u8))
    };
    let mut pixels = Vec::with_capacity(intrinsics.width as usize * intrinsics.height as usize);
    for y in 0..intrinsics.height {
        for x in 0..intrinsics.width {
            let ray = DVec3::new(
                (f64::from(x) - intrinsics.cx) / intrinsics.fx,
                (intrinsics.cy - f64::from(y)) / intrinsics.fy,
                -1.0,
            );
            pixels.push(ground_luma(camera_to_ecef * ray)?);
        }
    }
    LumaImage::new(intrinsics.width, intrinsics.height, pixels)
}
