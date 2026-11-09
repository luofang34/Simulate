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
mod multiscale;
mod ortho;
mod pipeline;
mod renderer;
mod scene;
mod vector_pass;

pub use aperture::{
    APERTURE_COLOUR_FORMAT, APERTURE_DEPTH_FORMAT, ApertureFrame, ApertureImage, ApertureWaveband,
    CatalogStar, MICROBOLOMETER_BITS, Microbolometer, MicrobolometerConfig, MicrobolometerFrame,
    NIGHT_SKY_V_MAG_PER_ARCSEC2, OcclusionOverlayConfig, OffscreenAperture, OpticalReticle,
    PinholeProjection, PresentedViewport, PropellerDisk, STAR_ATMOSPHERE_HEIGHT_M, SceneInterval,
    SceneTemperature, SensorAperturePass, SensorCounts, StarCamera, StarCameraConfig,
    StarCameraExposure, StarCameraTruth, StarTruth, StarVisibility, ThermalLag, ThermalScale,
    V_ZERO_POINT_PHOTONS_PER_M2_S, WindowBezel, padded_bytes_per_row, reversed_z_range_m,
};
#[cfg(not(target_arch = "wasm32"))]
pub use aperture::{THERMAL_KELVIN_FORMAT, ThermalSceneTarget};
pub use camera::{Camera, CameraPose};
pub use compositor::{
    ColorTransfer, CompositorPass, CompositorTarget, MAX_COMPOSITE_QUADS, OverlayQuad, QuadFilter,
    RgbaPanelTexture, ViewportRect,
};
pub use error::{FrameOutcome, RenderError};
pub use globe::{GlobeMesh, GlobeVertex, MAX_PATCHES, TerrainRegion, TerrainSources};
pub use gpu::HeadlessGpu;
pub use multiscale::{
    AerospikePlumeVisual, CelestialBodySample, DynamicPlanetaryFrustum, FdaiMarkerDir,
    FdaiNavballState, NEAR_FIELD_USD_Z_FAR_M, NEAR_FIELD_USD_Z_NEAR_M, OrreryBodyNode,
    ProjectedOrreryNode, SortedCelestialLayer, WEB_MERCATOR_MAX_LAT_DEG,
    compute_dynamic_planetary_frustum, encode_finite_reversed_z, evaluate_aerospike_plume_visual,
    evaluate_eclipse_shadow_factor, evaluate_fdai_navball, evaluate_planet_impostor_to_terrain_lod,
    near_field_occluded_by_planet, polar_cap_blend_weight, project_orrery_scene,
    reconstruct_planetary_eye_depth_m, sort_celestial_bodies_back_to_front,
};
pub use ortho::{OrthoAtlas, OrthoFootprint};
pub use renderer::{Renderer, SceneLighting};
pub use vector_pass::{
    VECTOR_COLOUR_FORMAT, VECTOR_SAMPLE_COUNT, VectorScenePass, VectorTarget, VectorVertex,
};
