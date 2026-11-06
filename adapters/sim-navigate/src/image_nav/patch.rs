//! Square ground-aligned rasters: east along columns, north up the rows,
//! one value per cell, `NaN` where nothing was seen.

use glam::{DMat3, DVec2, DVec3};
use sim_geo::{Geodetic, SensorIntrinsics, WGS84_A, WGS84_E2};
use sim_scenery::{CatalogScenery, LumaImage};

/// A square raster on level ground: column `i` runs east, row `j` south, and
/// the centre of the middle cell is the patch's reference point.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct GroundPatch {
    size: usize,
    gsd_m: f64,
    values: Vec<f32>,
}

impl GroundPatch {
    /// A patch whose cell `(i, j)` holds `value(east_m, north_m)` of its
    /// centre's offset from the patch centre.
    pub(crate) fn sample(
        size: usize,
        gsd_m: f64,
        mut value: impl FnMut(f64, f64) -> Option<f64>,
    ) -> Self {
        let half = (size as f64 - 1.0) / 2.0;
        let mut values = Vec::with_capacity(size * size);
        for j in 0..size {
            for i in 0..size {
                let east = (i as f64 - half) * gsd_m;
                let north = (half - j as f64) * gsd_m;
                values.push(value(east, north).map_or(f32::NAN, |v| v as f32));
            }
        }
        Self {
            size,
            gsd_m,
            values,
        }
    }

    pub(crate) fn size(&self) -> usize {
        self.size
    }

    pub(crate) fn gsd_m(&self) -> f64 {
        self.gsd_m
    }

    /// The cell at `(i, j)`, `NaN` where unseen; `None` off the patch.
    pub(crate) fn get(&self, i: usize, j: usize) -> Option<f32> {
        if i >= self.size || j >= self.size {
            return None;
        }
        self.values.get(j * self.size + i).copied()
    }

    /// The fraction of cells holding a value.
    pub(crate) fn coverage(&self) -> f64 {
        let seen = self.values.iter().filter(|v| v.is_finite()).count();
        seen as f64 / self.values.len().max(1) as f64
    }

    /// Half the size at twice the cell width; a cell is the mean of the
    /// seen cells it covers, unseen if fewer than three were.
    pub(crate) fn halved(&self) -> Self {
        let size = self.size / 2;
        let mut values = Vec::with_capacity(size * size);
        for j in 0..size {
            for i in 0..size {
                let block = [(0, 0), (1, 0), (0, 1), (1, 1)]
                    .iter()
                    .filter_map(|&(di, dj)| self.get(2 * i + di, 2 * j + dj))
                    .filter(|v| v.is_finite());
                let (sum, count) = block.fold((0.0, 0), |(s, c), v| (s + v, c + 1));
                values.push(if count >= 3 {
                    sum / count as f32
                } else {
                    f32::NAN
                });
            }
        }
        Self {
            size,
            gsd_m: self.gsd_m * 2.0,
            values,
        }
    }

    /// Shi-Tomasi corner strength at `(i, j)` over a 5 × 5 window: the
    /// smaller eigenvalue of the gradient structure tensor.
    pub(crate) fn corner_strength(&self, i: usize, j: usize) -> Option<f64> {
        let (mut gxx, mut gyy, mut gxy) = (0.0, 0.0, 0.0);
        for y in j.checked_sub(2)?..=j + 2 {
            for x in i.checked_sub(2)?..=i + 2 {
                let gx =
                    f64::from(self.get(x + 1, y)?) - f64::from(self.get(x.checked_sub(1)?, y)?);
                let gy =
                    f64::from(self.get(x, y + 1)?) - f64::from(self.get(x, y.checked_sub(1)?)?);
                gxx += gx * gx;
                gyy += gy * gy;
                gxy += gx * gy;
            }
        }
        let trace = gxx + gyy;
        let root = ((gxx - gyy).powi(2) + 4.0 * gxy * gxy).sqrt();
        let strength = 0.5 * (trace - root);
        strength.is_finite().then_some(strength)
    }
}

/// Ellipsoid radii at a latitude: metres per radian north and east.
pub(crate) fn metres_per_radian(lat_rad: f64) -> DVec2 {
    let w2 = 1.0 - WGS84_E2 * lat_rad.sin().powi(2);
    let prime_vertical = WGS84_A / w2.sqrt();
    DVec2::new(
        prime_vertical * (1.0 - WGS84_E2) / w2,
        prime_vertical * lat_rad.cos().max(1e-9),
    )
}

/// The reference orthoimagery around `centre` as a patch, from the deepest
/// imagery the catalog holds.
pub(crate) fn reference_patch(
    catalog: &CatalogScenery,
    centre: &Geodetic,
    size: usize,
    gsd_m: f64,
) -> GroundPatch {
    let radii = metres_per_radian(centre.lat_rad);
    GroundPatch::sample(size, gsd_m, |east, north| {
        catalog.imagery_luma_at(&Geodetic {
            lat_rad: centre.lat_rad + north / radii.x,
            lon_rad: centre.lon_rad + east / radii.y,
            height_m: 0.0,
        })
    })
}

/// The camera image laid onto the ground around `centre_m`, the east and
/// north offset from the point straight below the camera. `ground_up_m`
/// gives the ground's height relative to the eye at such an offset.
/// `camera_to_level` rotates camera components (`+X` right, `+Y` up,
/// looking along `-Z`) into the local east-north-up frame.
pub(crate) fn rectified_patch(
    image: &LumaImage,
    intrinsics: &SensorIntrinsics,
    camera_to_level: DMat3,
    centre_m: DVec2,
    ground_up_m: impl Fn(DVec2) -> f64,
    size: usize,
    gsd_m: f64,
) -> GroundPatch {
    let level_to_camera = camera_to_level.transpose();
    GroundPatch::sample(size, gsd_m, |east, north| {
        let offset = centre_m + DVec2::new(east, north);
        let ray = level_to_camera * offset.extend(ground_up_m(offset));
        if ray.z >= -1e-6 {
            return None;
        }
        let x = intrinsics.cx + intrinsics.fx * (ray.x / -ray.z);
        let y = intrinsics.cy - intrinsics.fy * (ray.y / -ray.z);
        image.sample_bilinear(x, y)
    })
}

/// Where the optical axis meets level ground `agl_m` below the camera, as
/// an east and north offset from the point straight below it; `None` when
/// it looks too near the horizon for the ground there to be usable.
pub(crate) fn boresight_offset(camera_to_level: DMat3, agl_m: f64) -> Option<DVec2> {
    let axis = camera_to_level * DVec3::NEG_Z;
    // Beyond about 63° off the vertical the ground is too oblique to match.
    (axis.z < -0.45).then(|| axis.truncate() * (agl_m / -axis.z))
}
