use sim_geo::Geodetic;

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
    /// The address does not exist at its zoom level.
    #[error("tile z{} x{} y{} is outside the tile pyramid", .tile.zoom, .tile.x, .tile.y)]
    TileOutOfRange {
        /// The rejected address.
        tile: TileId,
    },
    /// A raster's pixel data does not fill its dimensions.
    #[error("{width}x{height} raster needs {expected} bytes but has {actual}")]
    InvalidRaster {
        /// Width in pixels.
        width: u32,
        /// Height in pixels.
        height: u32,
        /// Bytes the dimensions require.
        expected: usize,
        /// Bytes supplied.
        actual: usize,
    },
    /// A catalog manifest is malformed or describes unusable data.
    #[error("catalog manifest rejected: {detail}")]
    InvalidCatalog {
        /// What was wrong.
        detail: String,
    },
    /// A catalog tile failed its integrity check or would not decode.
    #[error("catalog tile {path} rejected: {detail}")]
    CorruptTile {
        /// The tile's path in the manifest.
        path: String,
        /// What was wrong.
        detail: String,
    },
    /// Reading from the catalog's storage failed.
    #[error("reading {path} failed: {detail}")]
    Read {
        /// The file that could not be read.
        path: String,
        /// The underlying error.
        detail: String,
    },
}

/// Terrain height, queried by physics (ground contact, radar altimeters) and
/// by the renderer (displacement, camera clearance).
pub trait ElevationSource {
    /// Height of the ground at `position`, metres, in the source's native
    /// vertical datum: the body's reference ellipsoid unless the source says
    /// otherwise, as a [`crate::CatalogScenery`] does through its
    /// [`crate::CatalogVerticalDatum`]. Callers that need ellipsoidal heights
    /// from any catalog use [`crate::CatalogScenery::ellipsoidal_ground_height_m`]
    /// or [`crate::WorldSurfaceQuery`]. `Ok(None)` means the data is not
    /// resident; callers must treat that as unknown, never as sea level.
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
    /// Row-major RGBA8 pixels, `width * height * 4` bytes.
    pub rgba8: Vec<u8>,
}

impl ImageryRaster {
    /// A raster over `rgba8`, which must exactly fill the dimensions.
    pub fn new(width: u32, height: u32, rgba8: Vec<u8>) -> Result<Self, SceneryError> {
        let raster = Self {
            width,
            height,
            rgba8,
        };
        raster.validate()?;
        Ok(raster)
    }

    /// Checks that the pixel data exactly fills the dimensions.
    pub fn validate(&self) -> Result<(), SceneryError> {
        let expected = (self.width as usize)
            .checked_mul(self.height as usize)
            .and_then(|pixels| pixels.checked_mul(4));
        if expected == Some(self.rgba8.len()) {
            return Ok(());
        }
        Err(SceneryError::InvalidRaster {
            width: self.width,
            height: self.height,
            expected: expected.unwrap_or(usize::MAX),
            actual: self.rgba8.len(),
        })
    }
}

impl ImageryRaster {
    /// Bilinear Rec. 601 luma, `0..=255`, at the tile fraction `(u, v)`.
    /// Beyond the outermost pixel centres the edge pixels hold their value.
    pub fn luma_bilinear(&self, u: f64, v: f64) -> Option<f64> {
        if self.width == 0 || self.height == 0 {
            return None;
        }
        crate::terrarium::bilinear(self.width, self.height, u, v, |x, y| {
            let clamped_x = x.clamp(0, i64::from(self.width) - 1) as usize;
            let clamped_y = y.clamp(0, i64::from(self.height) - 1) as usize;
            let start = (clamped_y * self.width as usize + clamped_x) * 4;
            let [r, g, b] = self.rgba8.get(start..start + 3)?.first_chunk::<3>()?;
            Some(0.299 * f64::from(*r) + 0.587 * f64::from(*g) + 0.114 * f64::from(*b))
        })
    }
}

/// Orthoimagery draped on terrain.
pub trait ImagerySource {
    /// The imagery for `tile`. `Ok(None)` means the source has no coverage
    /// there.
    fn imagery(&self, tile: TileId) -> Result<Option<ImageryRaster>, SceneryError>;
}

/// How a patch of ground reflects sunlight and glows in the thermal infrared.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceRadiometry {
    /// Bond albedo, `0..=1`.
    pub albedo: f32,
    /// Broadband thermal emissivity, `0..=1`.
    pub emissivity: f32,
    /// Surface temperature, kelvin.
    pub temperature_k: f32,
}

impl SurfaceRadiometry {
    /// Stefan-Boltzmann constant, W/m²/K⁴.
    pub const STEFAN_BOLTZMANN: f32 = 5.670_374e-8;

    /// Grey-body exitance `σ ε T⁴`, W/m².
    pub fn exitance_w_m2(&self) -> f32 {
        Self::STEFAN_BOLTZMANN * self.emissivity * self.temperature_k.powi(4)
    }
}

/// Surface properties a thermal camera sees.
pub trait RadiometricSource {
    /// The surface at `position`. `Ok(None)` means there is no coverage.
    fn surface_radiometry(
        &self,
        position: &Geodetic,
    ) -> Result<Option<SurfaceRadiometry>, SceneryError>;
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
