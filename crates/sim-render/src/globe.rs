use std::f64::consts::{PI, TAU};

use glam::DVec3;
use sim_geo::BodyEllipsoid;
use sim_scenery::{ElevationSource, RadiometricSource};

use crate::{OrthoFootprint, RenderError};

mod patch;

use patch::PatchBuilder;

#[cfg(test)]
mod tests;

/// Capacity of the shader's per-patch offset table. 512 `vec4<f32>` entries
/// plus the frame header stay inside WebGL2's 16 KiB uniform-buffer limit.
pub const MAX_PATCHES: usize = 512;

/// Floats in a [`GlobeVertex`].
pub(crate) const GLOBE_VERTEX_FLOATS: usize = 13;

/// Where a [`GlobeVertex`] stores its patch index.
const PATCH_INDEX_FLOAT: usize = 8;

/// One globe vertex, packed for a single `f32` vertex buffer: `[x, y, z]`
/// relative to its patch centre (metres), `[nx, ny, nz]` the surface normal,
/// `[lat, lon]` in radians, the patch index, `[u, v]` into the ortho atlas,
/// the albedo, and the thermal exitance (W/m²). Coordinates outside `0..=1`
/// and negative radiometry mean no data.
pub type GlobeVertex = [f32; GLOBE_VERTEX_FLOATS];

/// A latitude/longitude block of a body's surface and how to tessellate it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainRegion {
    /// The body whose ellipsoid carries the terrain.
    pub body: BodyEllipsoid,
    /// Southern edge, radians.
    pub south_rad: f64,
    /// Western edge, radians.
    pub west_rad: f64,
    /// Extent north of `south_rad`, radians.
    pub lat_span_rad: f64,
    /// Extent east of `west_rad`, radians.
    pub lon_span_rad: f64,
    /// Patches from south to north.
    pub lat_patches: u32,
    /// Patches from west to east.
    pub lon_patches: u32,
    /// Quads along each patch edge.
    pub grid: u32,
}

impl TerrainRegion {
    /// The whole of `body`.
    pub fn whole_body(body: BodyEllipsoid, lat_patches: u32, lon_patches: u32, grid: u32) -> Self {
        Self {
            body,
            south_rad: -PI / 2.0,
            west_rad: -PI,
            lat_span_rad: PI,
            lon_span_rad: TAU,
            lat_patches,
            lon_patches,
            grid,
        }
    }

    fn validate(&self) -> Result<(), RenderError> {
        let empty = |detail: String| RenderError::EmptyRegion { detail };
        let positive = self.lat_span_rad > 0.0 && self.lon_span_rad > 0.0;
        if !(positive && self.south_rad.is_finite() && self.west_rad.is_finite()) {
            return Err(empty(format!(
                "{} × {} rad from {}, {}",
                self.lat_span_rad, self.lon_span_rad, self.south_rad, self.west_rad
            )));
        }
        if self.lat_patches == 0 || self.lon_patches == 0 || self.grid == 0 {
            return Err(empty(format!(
                "{} × {} patches of {} quads",
                self.lat_patches, self.lon_patches, self.grid
            )));
        }
        let requested = self.lat_patches as usize * self.lon_patches as usize;
        if requested > MAX_PATCHES {
            return Err(RenderError::TooManyPatches {
                requested,
                max: MAX_PATCHES,
            });
        }
        let row = u64::from(self.grid) + 1;
        let vertices = row * row * requested as u64;
        if vertices > u64::from(u32::MAX) {
            return Err(RenderError::TooManyVertices { vertices });
        }
        Ok(())
    }
}

/// What a mesh drapes over its ellipsoid. Everything is optional: with no
/// elevation the surface is the bare ellipsoid, and without imagery or
/// radiometry the shader falls back to its procedural surface.
#[derive(Clone, Copy, Default)]
pub struct TerrainSources<'a> {
    /// Displaces vertices along the ellipsoid normal.
    pub elevation: Option<&'a dyn ElevationSource>,
    /// Supplies per-vertex albedo and thermal exitance.
    pub radiometry: Option<&'a dyn RadiometricSource>,
    /// Maps vertices into the ortho atlas.
    pub ortho: Option<&'a OrthoFootprint>,
}

/// A body's surface split into latitude/longitude patches. Each patch stores
/// its vertices relative to its own `f64` centre so no vertex is ever further
/// than one patch radius from the value it is added to.
#[derive(Debug, Clone)]
pub struct GlobeMesh {
    /// Vertex data for every patch, patch after patch.
    pub vertices: Vec<GlobeVertex>,
    /// Triangle list indices into `vertices`, counter-clockwise seen from space.
    pub indices: Vec<u32>,
    /// The patch table: centres in body-fixed metres, indexed by the vertex
    /// patch index. Offsets from the eye are computed from these in `f64`
    /// every frame, which is what keeps the terrain free of `f32` jitter.
    pub patch_centres: Vec<DVec3>,
}

impl GlobeMesh {
    /// Builds a WGS84 mesh of `lat_patches × lon_patches` patches, each a
    /// `grid × grid` quad grid. Returns `None` if that exceeds [`MAX_PATCHES`]
    /// or any dimension is zero.
    pub fn build(lat_patches: u32, lon_patches: u32, grid: u32) -> Option<Self> {
        let region =
            TerrainRegion::whole_body(BodyEllipsoid::WGS84, lat_patches, lon_patches, grid);
        Self::build_region(&region, TerrainSources::default()).ok()
    }

    /// Builds a mesh of `region`, displaced and draped by `sources`. Fails
    /// with [`RenderError::UncoveredTerrain`] rather than guess the height of
    /// a vertex the elevation source has no data for.
    pub fn build_region(
        region: &TerrainRegion,
        sources: TerrainSources<'_>,
    ) -> Result<Self, RenderError> {
        region.validate()?;
        let mut mesh = Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            patch_centres: Vec::new(),
        };
        let lat_step = region.lat_span_rad / f64::from(region.lat_patches);
        let lon_step = region.lon_span_rad / f64::from(region.lon_patches);
        for lat_index in 0..region.lat_patches {
            for lon_index in 0..region.lon_patches {
                let south = region.south_rad + f64::from(lat_index) * lat_step;
                let west = region.west_rad + f64::from(lon_index) * lon_step;
                PatchBuilder::new(region, sources, south, west, lat_step, lon_step)
                    .append_to(&mut mesh)?;
            }
        }
        Ok(mesh)
    }

    /// Adds `other`'s patches to this mesh, so a detailed region can be drawn
    /// together with the coarse globe around it. Refuses to pass
    /// [`MAX_PATCHES`] and leaves the mesh unchanged.
    pub fn append(&mut self, other: &Self) -> Result<(), RenderError> {
        let requested = self.patch_centres.len() + other.patch_centres.len();
        if requested > MAX_PATCHES {
            return Err(RenderError::TooManyPatches {
                requested,
                max: MAX_PATCHES,
            });
        }
        let first_vertex = u32::try_from(self.vertices.len() + other.vertices.len())
            .map(|_| self.vertices.len() as u32)
            .map_err(|_| RenderError::TooManyVertices {
                vertices: (self.vertices.len() + other.vertices.len()) as u64,
            })?;
        let first_patch = self.patch_centres.len() as f32;
        self.vertices.extend(other.vertices.iter().map(|vertex| {
            let mut vertex = *vertex;
            if let Some(patch) = vertex.get_mut(PATCH_INDEX_FLOAT) {
                *patch += first_patch;
            }
            vertex
        }));
        self.indices
            .extend(other.indices.iter().map(|index| index + first_vertex));
        self.patch_centres.extend_from_slice(&other.patch_centres);
        Ok(())
    }
}
