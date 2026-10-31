use indicate_instrument_descriptor::InstrumentDrawError;
use indicate_instrument_raster::RasterError;
use indicate_instrument_scene::{LayerError, SceneError};
use indicate_instrument_state::abi::v8::AbiError;
use indicate_interaction::InteractionError;
use thiserror::Error;

/// Why flight state could not be turned into Indicate's input state.
#[derive(Debug, Error, PartialEq)]
pub enum FeedError {
    /// A value that frame conversion depends on is NaN or infinite.
    #[error("{field} is not finite at {tick} ticks")]
    NonFinite {
        /// The value at fault.
        field: &'static str,
        /// The instant of the sample, master-clock ticks.
        tick: u128,
    },
    /// An Apollo reading is body to J2000, and the sample does not say how the
    /// central body stood.
    #[error("the sample at {tick} ticks has J2000 attitudes but no central-body frame")]
    MissingBodyFrame {
        /// The instant of the sample, master-clock ticks.
        tick: u128,
    },
    /// The frame did not fit its buffer.
    #[error("state frame does not encode into {capacity} bytes")]
    Encode {
        /// The buffer's size, bytes.
        capacity: usize,
        /// Indicate's reason.
        #[source]
        source: AbiError,
    },
    /// Indicate rejected the frame.
    #[error("state frame of {length} bytes does not decode")]
    Decode {
        /// The frame's size, bytes.
        length: usize,
        /// Indicate's reason.
        #[source]
        source: AbiError,
    },
}

/// Why a panel did not produce a frame.
#[derive(Debug, Error, PartialEq)]
pub enum PanelError {
    /// The panel could not be encoded. `SceneError` implements no
    /// `std::error::Error`, so it is carried as a value rather than a source.
    #[error("{panel} scene does not encode into {capacity} bytes: {cause:?}")]
    Scene {
        /// The panel.
        panel: &'static str,
        /// The scene buffer's size, bytes.
        capacity: usize,
        /// Indicate's reason.
        cause: SceneError,
    },
    /// The framebuffer would clip the panel, whose drawing is not scaled.
    #[error("{width}x{height} framebuffer cannot hold the {min_width}x{min_height} panel")]
    FramebufferClipsPanel {
        /// Framebuffer width, pixels.
        width: u32,
        /// Framebuffer height, pixels.
        height: u32,
        /// Panel width, pixels.
        min_width: u32,
        /// Panel height, pixels.
        min_height: u32,
    },
    /// The rasterizer refused the scene. Past a framebuffer-shape failure the
    /// pixels hold Indicate's red-cross spoil pattern, never a stale frame.
    #[error("{panel} rasterization failed")]
    Raster {
        /// The panel.
        panel: &'static str,
        /// Indicate's reason.
        #[source]
        source: RasterError,
    },
    /// A 3D volumetric instrument refused to draw or overflowed its buffer.
    #[error("{instrument} volume draw failed: {source}")]
    VolumeDraw {
        /// Volumetric instrument id.
        instrument: &'static str,
        /// Indicate's reason.
        #[source]
        source: InstrumentDrawError,
    },
    /// An emitted 3D volumetric scene failed band validation.
    #[error("{instrument} volume band validation failed: {cause:?}")]
    VolumeValidation {
        /// Volumetric instrument id.
        instrument: &'static str,
        /// Band validator failure.
        cause: LayerError,
    },
    /// A spatial gesture or view pose failed validation.
    #[error("spatial interaction failed: {source}")]
    Interaction {
        /// Indicate's interaction error.
        #[source]
        source: InteractionError,
    },
}
