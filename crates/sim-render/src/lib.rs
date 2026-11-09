//! World renderer on wgpu, targeting native (Metal/Vulkan/DX12) and the
//! browser (WebGPU, with WebGL2 as fallback) from one code path.
//!
//! Precision model: geometry is stored in `f32` relative to per-patch `f64`
//! centres, and every frame the patch-to-camera offsets are computed in
//! `f64` before narrowing, so the camera always sits at the shader's origin.
//! Depth is reversed-Z with an infinite far plane.
//!
//! [`Renderer`] presents to a window; [`SensorAperturePass`] renders the same
//! terrain offscreen into an [`OffscreenAperture`] for simulated cameras.
//! [`VectorScenePass`] draws anti-aliased 2D symbology into a [`VectorTarget`]
//! that [`CompositorPass`] layers over the world as a premultiplied overlay.

mod aperture;
mod camera;
mod compositor;
mod error;
mod frame;
mod globe;
mod gpu;
mod ortho;
mod pipeline;
mod renderer;
mod scene;
mod vector_pass;

pub use aperture::{
    APERTURE_COLOUR_FORMAT, APERTURE_DEPTH_FORMAT, ApertureFrame, ApertureImage, ApertureWaveband,
    CatalogStar, MICROBOLOMETER_BITS, Microbolometer, MicrobolometerConfig, MicrobolometerFrame,
    NIGHT_SKY_V_MAG_PER_ARCSEC2, OcclusionOverlayConfig, OffscreenAperture, OpticalReticle,
    PinholeProjection, PresentedViewport, PropellerDisk, STAR_ATMOSPHERE_HEIGHT_M,
    SceneTemperature, SensorAperturePass, SensorCounts, StarCamera, StarCameraConfig,
    StarCameraExposure, StarCameraTruth, StarTruth, StarVisibility, ThermalScale,
    V_ZERO_POINT_PHOTONS_PER_M2_S, WindowBezel, padded_bytes_per_row, reversed_z_range_m,
};
pub use camera::{Camera, CameraPose};
pub use compositor::{
    ColorTransfer, CompositorPass, CompositorTarget, MAX_COMPOSITE_QUADS, OverlayQuad, QuadFilter,
    RgbaPanelTexture, ViewportRect,
};
pub use error::{FrameOutcome, RenderError};
pub use globe::{GlobeMesh, GlobeVertex, MAX_PATCHES, TerrainRegion, TerrainSources};
pub use gpu::HeadlessGpu;
pub use ortho::{OrthoAtlas, OrthoFootprint};
pub use renderer::{Renderer, SceneLighting};
pub use vector_pass::{
    VECTOR_COLOUR_FORMAT, VECTOR_SAMPLE_COUNT, VectorScenePass, VectorTarget, VectorVertex,
};
