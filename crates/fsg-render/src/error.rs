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
