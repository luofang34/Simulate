/// Failures that stop the renderer from being created or from drawing.
#[derive(Debug, thiserror::Error)]
pub enum RenderError {
    /// The window or canvas could not back a wgpu surface.
    #[error("creating the render surface failed")]
    CreateSurface(#[source] wgpu::CreateSurfaceError),
    /// No GPU adapter can present to the surface.
    #[error("no GPU adapter can present to this surface")]
    RequestAdapter(#[source] wgpu::RequestAdapterError),
    /// The adapter refused the device the renderer needs.
    #[error("requesting a GPU device from adapter {adapter} failed")]
    RequestDevice {
        /// Adapter name and backend, for diagnosing driver or browser issues.
        adapter: String,
        /// The underlying wgpu error.
        #[source]
        source: wgpu::RequestDeviceError,
    },
    /// The surface reports no configuration usable with the adapter.
    #[error("the surface offers no configuration for adapter {adapter}")]
    UnsupportedSurface {
        /// Adapter name and backend.
        adapter: String,
    },
    /// The globe mesh would need more patches than the shader's offset table.
    #[error("globe mesh needs {requested} patches; the shader table holds {max}")]
    TooManyPatches {
        /// Patches requested by the mesh layout.
        requested: usize,
        /// Capacity of the shader's per-patch offset table.
        max: usize,
    },
    /// The mesh would need more vertices than a `u32` index can address.
    #[error("mesh needs {vertices} vertices; indices address {max}", max = u32::MAX)]
    TooManyVertices {
        /// Vertices requested by the mesh layout.
        vertices: u64,
    },
    /// A terrain region has no area or no tessellation.
    #[error("terrain region is empty: {detail}")]
    EmptyRegion {
        /// Which dimension was unusable.
        detail: String,
    },
    /// The elevation source has no data under a mesh vertex. A mesh must not
    /// put unknown ground at sea level, so the region has to shrink to the
    /// source's coverage.
    #[error("no elevation at {lat_deg}°, {lon_deg}°")]
    UncoveredTerrain {
        /// Latitude of the vertex, degrees.
        lat_deg: f64,
        /// Longitude of the vertex, degrees.
        lon_deg: f64,
    },
    /// A scenery source failed.
    #[error("scenery source failed")]
    Scenery(#[from] sim_scenery::SceneryError),
    /// Imagery tiles do not share the atlas tile size.
    #[error("imagery tile is {width}x{height}; the atlas needs {tile_px}x{tile_px}")]
    AtlasTileSize {
        /// Tile width in pixels.
        width: u32,
        /// Tile height in pixels.
        height: u32,
        /// The atlas tile size.
        tile_px: u32,
    },
    /// A texture would not fit the device.
    #[error("{what} is {width}x{height}; the device limit is {max}")]
    TextureTooLarge {
        /// Which texture.
        what: &'static str,
        /// Requested width.
        width: u32,
        /// Requested height.
        height: u32,
        /// Largest supported edge.
        max: u32,
    },
    /// A texture was requested with a zero-sized edge.
    #[error("{what} is {width}x{height}; both edges must be at least one pixel")]
    EmptyTexture {
        /// Which texture.
        what: &'static str,
        /// Requested width.
        width: u32,
        /// Requested height.
        height: u32,
    },
    /// Vector geometry would not fit one device buffer.
    #[error("{vertices} vector vertices need {bytes} bytes; the device buffer limit is {max}")]
    VectorGeometryTooLarge {
        /// Vertices submitted.
        vertices: usize,
        /// Bytes they occupy.
        bytes: u64,
        /// Largest buffer the device allows.
        max: u64,
    },
    /// Uploaded RGBA8 byte slice did not match the panel texture's tight dimensions.
    #[error("RGBA upload supplied {actual} bytes; expected {expected}")]
    InvalidUploadBytes {
        /// Expected byte length (`width * height * 4`).
        expected: usize,
        /// Provided slice length.
        actual: usize,
    },
    /// The camera's optics cannot form an image.
    #[error("unusable camera: {detail}")]
    InvalidCamera {
        /// What was wrong.
        detail: String,
    },
    /// A simulated detector's configuration or input cannot produce a frame.
    #[error("unusable {sensor} sensor: {detail}")]
    InvalidSensor {
        /// Which sensor model.
        sensor: &'static str,
        /// What was wrong.
        detail: String,
    },
    /// Copying a rendered frame back from the GPU failed.
    #[error("reading the sensor frame back failed: {detail}")]
    Readback {
        /// wgpu's description of the failure.
        detail: String,
    },
    /// Building the render pipeline failed validation (for example a shader
    /// error).
    #[error("creating the render pipeline failed validation")]
    PipelineValidation(#[source] wgpu::Error),
    /// The GPU reported an error outside any error scope.
    #[error("GPU error: {message}")]
    Device {
        /// wgpu's description of the error.
        message: String,
    },
    /// wgpu rejected the frame's surface texture as invalid.
    #[error("acquiring the surface texture failed validation")]
    SurfaceValidation,
}

/// What happened to one requested frame when no error occurred.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameOutcome {
    /// The frame was drawn and presented.
    Presented,
    /// The surface was unavailable (minimised, occluded, timed out, or just
    /// reconfigured); the caller should simply request another frame.
    Skipped,
}
