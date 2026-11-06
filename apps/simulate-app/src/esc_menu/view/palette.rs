//! Hub colours and the glyph-pack character mapping.

use indicate_instrument_glyphs::PANEL_GLYPHS;
use indicate_instrument_scene::Rgba8;

/// Hub colour palette.
#[derive(Debug, Clone, Copy)]
pub struct Palette;

impl Palette {
    /// Dimmed window behind the card.
    pub const BACKDROP: Rgba8 = Rgba8::rgba(0, 4, 10, 160);
    /// Card fill.
    pub const CARD: Rgba8 = Rgba8::rgba(10, 16, 26, 242);
    /// Card and panel edges.
    pub const EDGE: Rgba8 = Rgba8::rgb(74, 118, 160);
    /// Inner panels.
    pub const PANEL: Rgba8 = Rgba8::rgba(18, 28, 44, 235);
    /// Dark studio card behind the 3D preview.
    pub const STUDIO: Rgba8 = Rgba8::rgba(6, 9, 14, 255);
    /// Ordinary widget fill.
    pub const BUTTON: Rgba8 = Rgba8::rgba(28, 44, 66, 230);
    /// Active / chosen widget fill.
    pub const ACTIVE: Rgba8 = Rgba8::rgba(36, 92, 146, 235);
    /// Primary action fill.
    pub const PRIMARY: Rgba8 = Rgba8::rgba(30, 128, 86, 240);
    /// Attention action fill.
    pub const ATTENTION: Rgba8 = Rgba8::rgba(196, 112, 24, 245);
    /// Focus / hover ring.
    pub const FOCUS: Rgba8 = Rgba8::rgb(140, 220, 255);
    /// Hover fill lift.
    pub const HOVER: Rgba8 = Rgba8::rgba(255, 255, 255, 28);
    /// Title text.
    pub const TITLE: Rgba8 = Rgba8::rgb(236, 244, 252);
    /// Label text.
    pub const LABEL: Rgba8 = Rgba8::rgb(214, 226, 238);
    /// Value text.
    pub const VALUE: Rgba8 = Rgba8::rgb(118, 220, 255);
    /// Pacing / telemetry text.
    pub const READOUT: Rgba8 = Rgba8::rgb(140, 232, 150);
    /// Hints and secondary text.
    pub const HINT: Rgba8 = Rgba8::rgb(130, 146, 164);
    /// Warning text.
    pub const WARN: Rgba8 = Rgba8::rgb(255, 180, 90);
    /// Bar track.
    pub const TRACK: Rgba8 = Rgba8::rgba(255, 255, 255, 30);
}

/// `ch` as the panel glyph pack can draw it.
#[must_use]
pub fn glyph_char(ch: char) -> char {
    let ch = match ch {
        '&' => '/',
        '\u{2014}' | '\u{2013}' | '_' => '-',
        other => other.to_ascii_uppercase(),
    };
    if PANEL_GLYPHS.glyph(ch).is_ok() {
        ch
    } else {
        ' '
    }
}
