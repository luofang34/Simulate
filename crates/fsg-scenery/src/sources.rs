use fsg_geo::Geodetic;

/// A Web-Mercator-style tile address (zoom, column, row).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TileId {
    /// Zoom level; 0 covers the whole globe.
    pub zoom: u8,
    /// Column, west to east.
    pub x: u32,
    /// Row, north to south.
    pub y: u32,
}

/// Why scenery could not be supplied.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SceneryError {
    /// The capability is intentionally not built yet.
    #[error("{capability} is deferred and has no source yet")]
    Deferred {
        /// Which capability.
        capability: &'static str,
    },
    /// The source has no data for this tile.
    #[error("no data for tile z{} x{} y{}", .tile.zoom, .tile.x, .tile.y)]
    Missing {
        /// The requested tile.
        tile: TileId,
    },
}

/// Terrain height, queried by physics (ground contact, radar altimeters) and
/// by the renderer (camera clearance).
pub trait ElevationSource {
    /// Height of the ground above the WGS84 ellipsoid at `position`, metres.
    /// `Ok(None)` means the data is not resident yet; callers must treat that
    /// as unknown, never as sea level.
    fn ground_height_m(&self, position: &Geodetic) -> Result<Option<f64>, SceneryError>;
}

/// Bare ellipsoid: the ground is at ellipsoid height zero everywhere.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EllipsoidSurface;

impl ElevationSource for EllipsoidSurface {
    fn ground_height_m(&self, _position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        Ok(Some(0.0))
    }
}

/// An RGBA8 imagery tile.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImageryRaster {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Row-major RGBA8 pixels.
    pub rgba: Vec<u8>,
}

/// Orthoimagery draped on terrain.
pub trait ImagerySource {
    /// The imagery for `tile`.
    fn imagery(&self, tile: TileId) -> Result<ImageryRaster, SceneryError>;
}

/// One node of a hierarchical photogrammetry mesh (a 3D Tiles style tileset).
#[derive(Debug, Clone, PartialEq)]
pub struct MeshNode {
    /// Node centre, ECEF metres; vertices are relative to it.
    pub centre_ecef: [f64; 3],
    /// Geometric error of this level of detail, metres.
    pub geometric_error_m: f64,
    /// Vertex positions relative to `centre_ecef`.
    pub positions: Vec<[f32; 3]>,
    /// Triangle indices.
    pub indices: Vec<u32>,
}

/// Photogrammetry meshes (city and terrain reconstructions).
pub trait PhotogrammetrySource {
    /// The mesh node at `path` in the tileset hierarchy (child indices from
    /// the root).
    fn node(&self, path: &[u8]) -> Result<MeshNode, SceneryError>;
}
