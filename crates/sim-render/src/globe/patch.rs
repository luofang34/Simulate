use std::f64::consts::FRAC_PI_2;

use glam::DVec3;
use sim_geo::Geodetic;

use super::{GlobeMesh, GlobeVertex, TerrainRegion, TerrainSources};
use crate::RenderError;

/// Marks a vertex with no radiometry or outside the ortho atlas.
const NO_DATA: f32 = -1.0;
/// A cross product shorter than this means the grid has collapsed, as it does
/// at a pole, and has no direction worth trusting.
const DEGENERATE_AREA_M2: f64 = 1e-6;

/// Builds one patch. Heights are sampled one row beyond the patch edge, so
/// neighbouring patches compute identical normals along their shared edge
/// and the seam does not show in the shading.
pub(super) struct PatchBuilder<'a> {
    region: &'a TerrainRegion,
    sources: TerrainSources<'a>,
    south: f64,
    west: f64,
    lat_span: f64,
    lon_span: f64,
}

impl<'a> PatchBuilder<'a> {
    pub(super) fn new(
        region: &'a TerrainRegion,
        sources: TerrainSources<'a>,
        south: f64,
        west: f64,
        lat_span: f64,
        lon_span: f64,
    ) -> Self {
        Self {
            region,
            sources,
            south,
            west,
            lat_span,
            lon_span,
        }
    }

    /// The ellipsoid point at grid position `(i, j)`; `-1` and `grid + 1`
    /// reach one row outside the patch.
    fn geodetic(&self, i: i64, j: i64) -> Geodetic {
        let grid = f64::from(self.region.grid);
        Geodetic {
            lat_rad: (self.south + self.lat_span * j as f64 / grid).clamp(-FRAC_PI_2, FRAC_PI_2),
            lon_rad: self.west + self.lon_span * i as f64 / grid,
            height_m: 0.0,
        }
    }

    /// The surface height under `position`; the bare ellipsoid has height
    /// zero by definition, not for want of data.
    fn height(&self, position: &Geodetic) -> Result<Option<f64>, RenderError> {
        match self.sources.elevation {
            Some(source) => Ok(source.ground_height_m(position)?),
            None => Ok(Some(0.0)),
        }
    }

    /// Displaced positions over the patch and its one-row halo, row by row.
    /// Vertices must be covered; the halo may fall outside the data.
    fn surface(&self) -> Result<Vec<Option<DVec3>>, RenderError> {
        let grid = i64::from(self.region.grid);
        let mut surface = Vec::with_capacity(((grid + 3) * (grid + 3)) as usize);
        for j in -1..=grid + 1 {
            for i in -1..=grid + 1 {
                let mut position = self.geodetic(i, j);
                let height = self.height(&position)?;
                let on_patch = (0..=grid).contains(&i) && (0..=grid).contains(&j);
                if on_patch && height.is_none() {
                    return Err(RenderError::UncoveredTerrain {
                        lat_deg: position.lat_deg(),
                        lon_deg: position.lon_deg(),
                    });
                }
                surface.push(height.map(|height_m| {
                    position.height_m = height_m;
                    self.region.body.geodetic_to_fixed(&position)
                }));
            }
        }
        Ok(surface)
    }

    /// The entry of [`Self::surface`] for grid position `(i, j)`.
    fn surface_at(&self, surface: &[Option<DVec3>], i: i64, j: i64) -> Option<DVec3> {
        let row = i64::from(self.region.grid) + 3;
        let index = (j + 1) * row + i + 1;
        usize::try_from(index)
            .ok()
            .and_then(|index| surface.get(index).copied().flatten())
    }

    fn normal(&self, surface: &[Option<DVec3>], i: i64, j: i64, position: &Geodetic) -> DVec3 {
        let (sin_lat, cos_lat) = position.lat_rad.sin_cos();
        let (sin_lon, cos_lon) = position.lon_rad.sin_cos();
        let ellipsoid = DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
        if self.sources.elevation.is_none() {
            return ellipsoid;
        }
        let at = |i: i64, j: i64| self.surface_at(surface, i, j);
        let Some(centre) = at(i, j) else {
            return ellipsoid;
        };
        let east = at(i + 1, j).unwrap_or(centre) - at(i - 1, j).unwrap_or(centre);
        let north = at(i, j + 1).unwrap_or(centre) - at(i, j - 1).unwrap_or(centre);
        let across = east.cross(north);
        if across.length() < DEGENERATE_AREA_M2 {
            return ellipsoid;
        }
        across.normalize()
    }

    fn vertex(
        &self,
        surface: &[Option<DVec3>],
        i: i64,
        j: i64,
        centre: DVec3,
        patch_index: usize,
    ) -> Result<GlobeVertex, RenderError> {
        let position = self.geodetic(i, j);
        let fixed = self.surface_at(surface, i, j).unwrap_or(centre);
        let relative = (fixed - centre).as_vec3();
        let normal = self.normal(surface, i, j, &position).as_vec3();
        let [u, v] = self
            .sources
            .ortho
            .and_then(|ortho| ortho.uv_at(&position))
            .unwrap_or([NO_DATA, NO_DATA]);
        let (albedo, exitance) = match self.sources.radiometry {
            Some(source) => source
                .surface_radiometry(&position)?
                .map_or((NO_DATA, NO_DATA), |surface| {
                    (surface.albedo, surface.exitance_w_m2())
                }),
            None => (NO_DATA, NO_DATA),
        };
        Ok([
            relative.x,
            relative.y,
            relative.z,
            normal.x,
            normal.y,
            normal.z,
            position.lat_rad as f32,
            position.lon_rad as f32,
            patch_index as f32,
            u,
            v,
            albedo,
            exitance,
        ])
    }

    pub(super) fn append_to(&self, mesh: &mut GlobeMesh) -> Result<(), RenderError> {
        let patch_index = mesh.patch_centres.len();
        let centre = self.region.body.geodetic_to_fixed(&Geodetic {
            lat_rad: self.south + self.lat_span / 2.0,
            lon_rad: self.west + self.lon_span / 2.0,
            height_m: 0.0,
        });
        mesh.patch_centres.push(centre);
        let surface = self.surface()?;
        let grid = self.region.grid;
        let first =
            u32::try_from(mesh.vertices.len()).map_err(|_| RenderError::TooManyPatches {
                requested: patch_index,
                max: patch_index,
            })?;
        for j in 0..=i64::from(grid) {
            for i in 0..=i64::from(grid) {
                mesh.vertices
                    .push(self.vertex(&surface, i, j, centre, patch_index)?);
            }
        }
        let row = grid + 1;
        for j in 0..grid {
            for i in 0..grid {
                let v00 = first + j * row + i;
                let v10 = v00 + 1;
                let v01 = v00 + row;
                let v11 = v01 + 1;
                mesh.indices
                    .extend_from_slice(&[v00, v10, v11, v00, v11, v01]);
            }
        }
        Ok(())
    }
}
