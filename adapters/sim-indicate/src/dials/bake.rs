//! Images a 3D backend maps onto a gauge, each rasterised once by Indicate's
//! reference rasteriser: a dial's face, its drum window's digit strip, and
//! the failure flag. Each carries the printed colour in `rgb` (sRGB) and in
//! `a` how strongly luminous paint there glows.

use indicate_instrument_analog::{Lighting, Paint, draw_face_scene};
use indicate_instrument_descriptor::{DesignFrame, PanelDrawError};
use indicate_instrument_glyphs::FaceTable;
use indicate_instrument_raster::{FrameId, FramebufferDims, render_with_faces};
use indicate_instrument_scene::{
    Anchor, LayerId, MAX_SCENE_BYTES, PaintMode, SceneError, SceneWriter,
};
use indicate_instrument_symbology::safety;

use super::DialSource;
use super::source::FacePaint;
use crate::PanelError;

/// A baked image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FaceBake {
    /// Width, pixels.
    pub width: u32,
    /// Height, pixels.
    pub height: u32,
    /// Tightly packed RGBA8 rows, top row first.
    pub rgba: Vec<u8>,
}

/// Darkest channel the glow pass draws for paint that does not glow:
/// Indicate dims unlit paint, such as a red limit line, rather than
/// blacking it out. Anything at or below it is not luminous.
const UNLIT_CEILING: u8 = 48;
/// The text the failure flag reads.
pub const OFF_FLAG_TEXT: &str = "OFF";

fn scene_error(cause: SceneError) -> PanelError {
    PanelError::Scene {
        panel: "analog dial",
        capacity: MAX_SCENE_BYTES,
        cause,
    }
}

/// `draw` rasterised onto `width` x `height` pixels, lettered in `faces`.
fn raster(
    width: u32,
    height: u32,
    faces: &FaceTable<'_>,
    draw: impl FnOnce(&mut SceneWriter<'_>) -> Result<(), PanelDrawError>,
) -> Result<Vec<u8>, PanelError> {
    let mut scene = vec![0_u8; MAX_SCENE_BYTES];
    let mut writer = SceneWriter::new(&mut scene).map_err(scene_error)?;
    draw(&mut writer).map_err(|source| PanelError::DialDraw { source })?;
    let used = writer.finish();
    let mut pixels = vec![0_u8; (width as usize * height as usize).saturating_mul(4)];
    render_with_faces(
        scene.get(..used).unwrap_or(&[]),
        &mut pixels,
        FramebufferDims::tight(width, height),
        FrameId {
            frame_generation: 1,
            render_generation: 1,
        },
        faces,
    )
    .map_err(|source| PanelError::Raster {
        panel: "analog dial",
        source,
    })?;
    Ok(pixels)
}

pub(super) fn lighting(mode: FacePaint) -> Lighting {
    match mode {
        FacePaint::Printed => Lighting::Day,
        FacePaint::GlowMask => Lighting::Night,
    }
}

/// `draw` baked twice, printed and as a glow mask, into one image.
pub(super) fn bake(
    width: u32,
    height: u32,
    faces: &FaceTable<'_>,
    draw: impl Fn(FacePaint, &mut SceneWriter<'_>) -> Result<(), PanelDrawError>,
) -> Result<FaceBake, PanelError> {
    let mut rgba = raster(width, height, faces, |w| draw(FacePaint::Printed, w))?;
    let glow = raster(width, height, faces, |w| draw(FacePaint::GlowMask, w))?;
    let span = f32::from(u8::MAX - UNLIT_CEILING);
    for (printed, lit) in rgba
        .as_chunks_mut::<4>()
        .0
        .iter_mut()
        .zip(glow.as_chunks::<4>().0)
    {
        let [r, g, b, _] = *lit;
        let strength = f32::from(r.max(g).max(b).saturating_sub(UNLIT_CEILING)) / span;
        printed[3] = (strength * 255.0).round() as u8;
    }
    Ok(FaceBake {
        width,
        height,
        rgba,
    })
}

/// The face of `source`, `side` pixels square across its case, lettered in
/// the faces its pack binds: printed under daylight, with the luminous
/// paint's glow strength in alpha. Drawn twice and never again: the glow is
/// lit by the cockpit's lamps, not redrawn per lighting.
pub fn bake_face(
    source: &DialSource,
    side: u32,
    faces: &FaceTable<'_>,
) -> Result<FaceBake, PanelError> {
    let frame = DesignFrame {
        width: side as f32,
        height: side as f32,
    };
    bake(side, side, faces, |mode, writer| {
        source.with_spec(mode, |spec| {
            draw_face_scene(spec, lighting(mode), frame, writer)
        })
    })
}

/// The digit strip of `source`'s drum window, `height` pixels tall: the
/// digits 0 to 9 top to bottom, each centred in a tenth of the height, on
/// the drum's colour. Its width keeps the digits' proportions on a wheel
/// `wheel_width_mm` wide whose digits sit `digit_pitch_mm` apart.
pub fn bake_drum_strip(
    source: &DialSource,
    height: u32,
    faces: &FaceTable<'_>,
) -> Result<Option<FaceBake>, PanelError> {
    let Some(build) = source
        .physical_build()
        .ok()
        .and_then(|b| b.windows.first().copied().flatten())
    else {
        return Ok(None);
    };
    let Some((window, _)) = source.window else {
        return Ok(None);
    };
    let pitch_px = height as f32 / 10.0;
    let width = ((pitch_px * build.wheel_width_mm / build.digit_pitch_mm).round() as u32).max(1);
    let size = pitch_px * build.digit_height_mm / build.digit_pitch_mm;
    let digits = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"];
    let paint = window.paint;
    let drum = window.drum;
    let slot = source.numerals_slot();
    bake(width, height, faces, |mode, w| {
        let ink = mode.paint(paint).under(lighting(mode));
        let ground = mode.paint(Paint::plain(drum)).under(lighting(mode));
        let draw = |w: &mut SceneWriter<'_>| -> Result<(), SceneError> {
            w.begin_layer(LayerId::Background)?;
            w.fill_color(ground)?;
            w.rect(PaintMode::Fill, 0.0, 0.0, width as f32, height as f32)?;
            w.end_layer(LayerId::Background)?;
            w.begin_layer(LayerId::Tapes)?;
            w.font(slot)?;
            w.fill_color(ink)?;
            for (k, digit) in (0_u8..).zip(digits) {
                let y = (f32::from(k) + 0.5) * pitch_px;
                w.text(width as f32 * 0.5, y, size, Anchor::CENTER, digit)?;
            }
            w.end_layer(LayerId::Tapes)
        };
        draw(w).map_err(PanelDrawError::from)
    })
    .map(Some)
}

/// A placard reading `text`, `width` x `height` pixels: white letters a
/// half the plate's height tall, centred on black, lettered in `slot` of
/// `faces`. Placards do not glow.
pub fn bake_placard(
    text: &str,
    width: u32,
    height: u32,
    faces: &FaceTable<'_>,
    slot: indicate_instrument_scene::FaceSlot,
) -> Result<FaceBake, PanelError> {
    bake(width, height, faces, |mode, w| {
        let draw = |w: &mut SceneWriter<'_>| -> Result<(), SceneError> {
            let (ground, ink) = match mode {
                FacePaint::Printed => (
                    indicate_instrument_scene::Rgba8::rgb(14, 14, 14),
                    indicate_instrument_scene::Rgba8::rgb(222, 220, 210),
                ),
                FacePaint::GlowMask => (
                    indicate_instrument_scene::Rgba8::rgb(0, 0, 0),
                    indicate_instrument_scene::Rgba8::rgb(0, 0, 0),
                ),
            };
            w.begin_layer(LayerId::Background)?;
            w.fill_color(ground)?;
            w.rect(PaintMode::Fill, 0.0, 0.0, width as f32, height as f32)?;
            w.font(slot)?;
            w.fill_color(ink)?;
            let size = height as f32 * 0.5;
            w.text(
                width as f32 * 0.5,
                height as f32 * 0.5,
                size,
                Anchor::CENTER,
                text,
            )?;
            w.end_layer(LayerId::Background)
        };
        draw(w).map_err(PanelDrawError::from)
    })
}

/// The red failure flag reading [`OFF_FLAG_TEXT`], `width` x `height` pixels,
/// in Indicate's proportions: the text's height a 1.6th of the flag's.
pub fn bake_off_flag(width: u32, height: u32) -> Result<FaceBake, PanelError> {
    bake(width, height, &FaceTable::DEFAULT, |mode, w| {
        let draw = |w: &mut SceneWriter<'_>| -> Result<(), SceneError> {
            let (red, white) = match mode {
                FacePaint::Printed => (safety::FAILURE_RED, safety::ANNUNCIATION_WHITE),
                FacePaint::GlowMask => (
                    indicate_instrument_scene::Rgba8::rgb(0, 0, 0),
                    indicate_instrument_scene::Rgba8::rgb(0, 0, 0),
                ),
            };
            w.begin_layer(LayerId::Annunciation)?;
            w.fill_color(red)?;
            w.rect(PaintMode::Fill, 0.0, 0.0, width as f32, height as f32)?;
            w.fill_color(white)?;
            let size = height as f32 / 1.6;
            w.text(
                width as f32 * 0.5,
                height as f32 * 0.5,
                size,
                Anchor::CENTER,
                OFF_FLAG_TEXT,
            )?;
            w.end_layer(LayerId::Annunciation)
        };
        draw(w).map_err(PanelDrawError::from)
    })
}
