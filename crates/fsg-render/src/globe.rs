use std::f64::consts::{PI, TAU};

use fsg_geo::{Geodetic, geodetic_to_ecef};
use glam::DVec3;

#[cfg(test)]
mod tests;

/// Capacity of the shader's per-patch offset table. 512 `vec4<f32>` entries
/// plus the frame header stay inside WebGL2's 16 KiB uniform-buffer limit.
pub const MAX_PATCHES: usize = 512;

/// One globe vertex, packed for a single `f32` vertex buffer:
/// `[x, y, z]` relative to its patch centre (metres), `[nx, ny, nz]` the
/// ellipsoid normal, `[lat, lon]` in radians, and the patch index.
pub type GlobeVertex = [f32; 9];

/// A WGS84 ellipsoid split into latitude/longitude patches. Each patch stores
/// its vertices relative to its own `f64` centre so no vertex is ever further
/// than one patch radius from the value it is added to.
#[derive(Debug, Clone)]
pub struct GlobeMesh {
    /// Vertex data for every patch, patch after patch.
    pub vertices: Vec<GlobeVertex>,
    /// Triangle list indices into `vertices`, counter-clockwise seen from space.
    pub indices: Vec<u32>,
    /// Patch centres in ECEF, indexed by the vertex patch index.
    pub patch_centres: Vec<DVec3>,
}

impl GlobeMesh {
    /// Builds a mesh of `lat_patches × lon_patches` patches, each a
    /// `grid × grid` quad grid. Returns `None` if that exceeds [`MAX_PATCHES`]
    /// or any dimension is zero.
    pub fn build(lat_patches: u32, lon_patches: u32, grid: u32) -> Option<Self> {
        let patch_count = usize::try_from(lat_patches.checked_mul(lon_patches)?).ok()?;
        if patch_count == 0 || patch_count > MAX_PATCHES || grid == 0 {
            return None;
        }
        let mut mesh = Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            patch_centres: Vec::with_capacity(patch_count),
        };
        let lat_step = PI / f64::from(lat_patches);
        let lon_step = TAU / f64::from(lon_patches);
        for lat_index in 0..lat_patches {
            for lon_index in 0..lon_patches {
                let south = -PI / 2.0 + f64::from(lat_index) * lat_step;
                let west = -PI + f64::from(lon_index) * lon_step;
                mesh.push_patch(south, west, lat_step, lon_step, grid)?;
            }
        }
        Some(mesh)
    }

    fn push_patch(
        &mut self,
        south: f64,
        west: f64,
        lat_span: f64,
        lon_span: f64,
        grid: u32,
    ) -> Option<()> {
        let patch_index = self.patch_centres.len();
        let centre = geodetic_to_ecef(&Geodetic {
            lat_rad: south + lat_span / 2.0,
            lon_rad: west + lon_span / 2.0,
            height_m: 0.0,
        });
        self.patch_centres.push(centre);
        let first = u32::try_from(self.vertices.len()).ok()?;
        let row = grid + 1;
        for j in 0..=grid {
            for i in 0..=grid {
                let position = Geodetic {
                    lat_rad: south + lat_span * f64::from(j) / f64::from(grid),
                    lon_rad: west + lon_span * f64::from(i) / f64::from(grid),
                    height_m: 0.0,
                };
                self.vertices.push(vertex(&position, centre, patch_index));
            }
        }
        for j in 0..grid {
            for i in 0..grid {
                let v00 = first + j * row + i;
                let v10 = v00 + 1;
                let v01 = v00 + row;
                let v11 = v01 + 1;
                self.indices
                    .extend_from_slice(&[v00, v10, v11, v00, v11, v01]);
            }
        }
        Some(())
    }
}

fn vertex(position: &Geodetic, centre: DVec3, patch_index: usize) -> GlobeVertex {
    let relative = (geodetic_to_ecef(position) - centre).as_vec3();
    let (sin_lat, cos_lat) = position.lat_rad.sin_cos();
    let (sin_lon, cos_lon) = position.lon_rad.sin_cos();
    let normal = DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat).as_vec3();
    [
        relative.x,
        relative.y,
        relative.z,
        normal.x,
        normal.y,
        normal.z,
        position.lat_rad as f32,
        position.lon_rad as f32,
        patch_index as f32,
    ]
}
