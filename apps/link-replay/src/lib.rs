//! Native headless flight runner and training-grade telemetry recorder.
//!
//! `link-replay` executes shareable `#v=2` links, structured JSON scenario
//! specs, and built-in test presets natively with a persistent local disk tile
//! cache (`SIMULATE_TILE_CACHE`), optional headless `wgpu` offscreen terrain
//! camera rendering, multi-seed Monte Carlo batching, and synchronized NDJSON +
//! summary JSON telemetry (`NEES`, spoofing ground truth, and ICAO RNP
//! `NSE`/`FTE`/`TSE`).

pub mod cache;
pub mod cli;
mod error;
pub mod presets;
#[cfg(not(target_arch = "wasm32"))]
pub mod runner;
#[cfg(not(target_arch = "wasm32"))]
pub mod seal;
#[cfg(not(target_arch = "wasm32"))]
pub mod wgpu_camera;

pub use error::ReplayError;
