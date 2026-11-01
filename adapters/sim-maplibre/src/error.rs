//! Typed error vocabulary for `sim-maplibre`.

use thiserror::Error;

/// Failures raised while constructing, driving, or streaming `maplibre` scenes
/// and avionics underlays.
#[derive(Debug, Error)]
pub enum MapAdapterError {
    /// Pixel dimensions were zero or exceeded device limits.
    #[error("viewport size {width}x{height} is invalid")]
    InvalidViewportSize {
        /// Requested width in physical pixels.
        width: u32,
        /// Requested height in physical pixels.
        height: u32,
    },
    /// Native Tokio worker pool could not be initialized.
    #[cfg(not(target_arch = "wasm32"))]
    #[error("shared map tile runtime failed to start")]
    RuntimeInit {
        /// Underlying OS I/O error.
        #[source]
        source: std::io::Error,
    },
    /// Map style JSON failed to parse.
    #[error("map style JSON is invalid")]
    StyleParse {
        /// Underlying JSON error.
        #[source]
        source: serde_json::Error,
    },
    /// Offscreen headless renderer initialization failed.
    #[error("headless map renderer initialization failed")]
    RendererInit {
        /// Underlying headless renderer error.
        #[source]
        source: maplibre::headless::HeadlessRendererError,
    },
    /// Assembling the `HeadlessMap` schedule and plugins failed.
    #[error("headless map assembly failed")]
    Assemble {
        /// Underlying map error.
        #[source]
        source: maplibre::map::MapError,
    },
    /// Synchronous tile loading or single-view frame execution failed.
    #[error("headless map operation failed")]
    MapOperation {
        /// Underlying operation error.
        #[source]
        source: maplibre::headless::map::HeadlessMapOperationError,
    },
    /// Multi-viewport frame validation or execution failed.
    #[error("multi-view frame failed")]
    MultiView {
        /// Underlying multi-view error.
        #[source]
        source: maplibre::headless::map::multi_view::MultiViewError,
    },
    /// Camera pose or frustum parameters were non-finite or degenerate.
    #[error("camera pose or frustum is invalid: {detail}")]
    InvalidPose {
        /// Diagnostic description.
        detail: &'static str,
    },
    /// Celestial lighting vector or twilight band was rejected.
    #[error("celestial lighting configuration failed")]
    Celestial {
        /// Underlying celestial error.
        #[source]
        source: maplibre::celestial::CelestialError,
    },
    /// `Indicate` panel or volumetric scene emission failed.
    #[error("indicate instrument emission failed")]
    Indicate {
        /// Underlying `sim-indicate` error.
        #[source]
        source: sim_indicate::PanelError,
    },
    /// Remote companion display wire packet was truncated or malformed.
    #[error("companion stream packet error: {detail}")]
    StreamWire {
        /// Diagnostic description.
        detail: &'static str,
    },
    /// Spatial interaction frame decoding failed.
    #[error("spatial interaction wire frame rejected")]
    InteractionWire {
        /// Underlying interaction error.
        #[source]
        source: indicate_interaction::InteractionError,
    },
    /// On-demand offscreen texture readback failed.
    #[error("virtual camera GPU readback failed: {detail}")]
    Readback {
        /// Diagnostic description.
        detail: String,
    },
}
