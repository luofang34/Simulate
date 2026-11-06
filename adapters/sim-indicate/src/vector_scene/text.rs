use indicate_instrument_glyphs::{ADVANCE, CELL_H, CELL_W, PANEL_GLYPHS};
use indicate_instrument_scene::{Anchor, HAlign, VAlign};

use super::VectorSceneError;
use super::mesh::Mesh;
use super::state::Affine;

/// A text run in logical units.
#[derive(Debug, Clone, Copy)]
pub(super) struct Run<'a> {
    pub(super) x: f32,
    pub(super) y: f32,
    pub(super) size: f32,
    pub(super) anchor: Anchor,
    pub(super) text: &'a str,
}

/// Tessellates a run from the controlled glyph pack with the rasterizer's
/// metrics: `size` is the cell height, the pen advances by the manifest
/// advance, and the bitmap sits above its baseline, so `Bottom` coincides
/// with `Baseline`. Each horizontal run of set pixels in a glyph row becomes
/// one quad mapped through the transform, covering exactly the pixel quads
/// the rasterizer fills.
pub(super) fn draw_run(
    mesh: &mut Mesh,
    ctm: &Affine,
    run: Run<'_>,
    command: usize,
) -> Result<(), VectorSceneError> {
    if run.text.is_empty() || run.size <= 0.0 {
        return Ok(());
    }
    let scale = run.size / CELL_H as f32;
    let advance = f32::from(ADVANCE) * scale;
    let width = run.text.chars().count() as f32 * advance;
    let left = match run.anchor.h {
        HAlign::Left => run.x,
        HAlign::Center => run.x - width / 2.0,
        HAlign::Right => run.x - width,
    };
    let top = match run.anchor.v {
        VAlign::Baseline | VAlign::Bottom => run.y - run.size,
        VAlign::Top => run.y,
        VAlign::Middle => run.y - run.size / 2.0,
    };
    let mut pen = left;
    for ch in run.text.chars() {
        let glyph = PANEL_GLYPHS.glyph(ch)?.glyph;
        for row in 0..CELL_H {
            let y0 = top + row as f32 * scale;
            let mut col = 0;
            while col < CELL_W {
                if !glyph.pixel(col, row) {
                    col = col.wrapping_add(1);
                    continue;
                }
                let start = col;
                while col < CELL_W && glyph.pixel(col, row) {
                    col = col.wrapping_add(1);
                }
                let x0 = pen + start as f32 * scale;
                let x1 = pen + col as f32 * scale;
                mesh.quad([
                    ctm.map(x0, y0, command)?,
                    ctm.map(x1, y0, command)?,
                    ctm.map(x1, y0 + scale, command)?,
                    ctm.map(x0, y0 + scale, command)?,
                ]);
            }
        }
        pen += advance;
    }
    Ok(())
}
