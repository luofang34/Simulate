use indicate_instrument_glyphs::GlyphError;
use indicate_instrument_scene::{DecodeError, LayerError};
use thiserror::Error;

/// Why a scene could not be tessellated into vector geometry. Every variant
/// fails the whole frame, as the CPU rasterizer spoils one rather than
/// drawing part of it.
#[derive(Debug, Error, Clone, Copy, PartialEq)]
pub enum VectorSceneError {
    /// The scene letters text along an arc, which this tessellator, bound to
    /// the reference face alone, does not place.
    #[error("command {command} letters text along an arc")]
    ArcText {
        /// Zero-based index of the command in the scene.
        command: usize,
    },
    /// The scene failed Indicate's layer and stack validation.
    #[error("scene failed layer validation")]
    Layers(#[from] LayerError),
    /// The controlled glyph pack is corrupt or lacks a character.
    #[error("glyph pack refused text")]
    Glyphs(#[from] GlyphError),
    /// The scene bytes do not decode.
    #[error("scene does not decode")]
    Decode(#[from] DecodeError),
    /// A command carried a NaN or infinite value.
    #[error("command {command} has a non-finite value")]
    NonFinite {
        /// Zero-based index of the command in the scene.
        command: usize,
    },
    /// A transformed coordinate left the rasterizer's accepted range.
    #[error("command {command} maps outside ±{limit_px} design pixels")]
    CoordinateOutOfRange {
        /// Zero-based index of the command in the scene.
        command: usize,
        /// Largest accepted absolute design coordinate.
        limit_px: f32,
    },
    /// More nested saves than the graphics-state stack holds.
    #[error("command {command} saves past the {limit}-deep state stack")]
    StackOverflow {
        /// Zero-based index of the command in the scene.
        command: usize,
        /// Stack depth limit.
        limit: usize,
    },
    /// A restore without a matching save.
    #[error("command {command} restores with no saved state")]
    UnbalancedRestore {
        /// Zero-based index of the command in the scene.
        command: usize,
    },
    /// A polyline or polygon has more vertices than the rasterizer accepts.
    #[error("command {command} has more than {limit} vertices")]
    TooManyVertices {
        /// Zero-based index of the command in the scene.
        command: usize,
        /// Vertex limit per shape.
        limit: usize,
    },
    /// The design or target frame has no usable area.
    #[error("design {design_w}x{design_h} onto target {target_w}x{target_h} has no area")]
    EmptyFrame {
        /// Design frame width, logical units.
        design_w: f32,
        /// Design frame height, logical units.
        design_h: f32,
        /// Target width, physical pixels.
        target_w: u32,
        /// Target height, physical pixels.
        target_h: u32,
    },
}
