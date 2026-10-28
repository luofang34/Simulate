//! World renderer on wgpu, targeting native (Metal/Vulkan/DX12) and the
//! browser (WebGPU, with WebGL2 as fallback) from one code path.
//!
//! Precision model: geometry is stored in `f32` relative to per-patch `f64`
//! ECEF centres, and every frame the patch-to-camera offsets are computed in
//! `f64` before narrowing, so the camera always sits at the shader's origin.
//! Depth is reversed-Z with an infinite far plane.

mod camera;
mod error;
mod frame;
mod globe;
mod gpu;
mod pipeline;
mod renderer;

pub use camera::{Camera, CameraPose};
pub use error::{FrameOutcome, RenderError};
pub use globe::{GlobeMesh, GlobeVertex, MAX_PATCHES};
pub use renderer::{Renderer, SceneLighting};
