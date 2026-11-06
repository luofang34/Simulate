//! Tessellates Indicate scenes into anti-aliased GPU triangles.
//!
//! The CPU rasterizer samples each pixel once, so its lines and glyph edges
//! alias, and its straight-alpha output fringes when scaled. This path walks
//! the same command stream with the same fail-closed checks, transform,
//! clip, fill, stroke, and text semantics, and emits physical-pixel
//! [`VectorVertex`] triangles for [`sim_render::VectorScenePass`] to draw
//! with multisampling at the display's own resolution.

use indicate_instrument_glyphs::PANEL_GLYPHS;
use indicate_instrument_raster::MAX_POLYGON_VERTICES;
use indicate_instrument_scene::{Cmd, PaintMode, PointsRef, SceneCmds, validate_layers};
use sim_render::VectorVertex;

use crate::PanelError;

mod error;
mod hud;
mod mesh;
mod shapes;
mod state;
#[cfg(test)]
mod tests;
mod text;

pub use error::VectorSceneError;
pub use hud::{VECTOR_HUD_DESIGN_HEIGHT, VectorHudPanel};

use mesh::Mesh;
use state::{StateStack, finite, rect_corners};

/// Tessellates an encoded scene laid out in a `design_w × design_h` logical
/// frame into triangles covering a `target_w × target_h` physical target.
///
/// The scene is validated exactly as the rasterizer validates it, and the
/// glyph pack is verified before any text could draw from it; any failure
/// returns an error and no geometry.
pub fn tessellate_scene_cmds(
    scene: &[u8],
    design_w: f32,
    design_h: f32,
    target_w: u32,
    target_h: u32,
) -> Result<Vec<VectorVertex>, PanelError> {
    tessellate(scene, [design_w, design_h], [target_w, target_h])
        .map_err(|source| PanelError::VectorScene { source })
}

fn tessellate(
    scene: &[u8],
    design: [f32; 2],
    target: [u32; 2],
) -> Result<Vec<VectorVertex>, VectorSceneError> {
    let [design_w, design_h] = design;
    let usable = |d: f32| d.is_finite() && d > 0.0;
    if !usable(design_w) || !usable(design_h) || target[0] == 0 || target[1] == 0 {
        return Err(VectorSceneError::EmptyFrame {
            design_w,
            design_h,
            target_w: target[0],
            target_h: target[1],
        });
    }
    validate_layers(scene)?;
    PANEL_GLYPHS.verify()?;
    let scale = [target[0] as f32 / design_w, target[1] as f32 / design_h];
    let mut state = StateStack::new(target, scale);
    let mut mesh = Mesh::new(scale);
    for (command, item) in SceneCmds::new(scene)?.enumerate() {
        run(&item?, command, &mut state, &mut mesh)?;
    }
    Ok(mesh.into_vertices())
}

fn run(
    cmd: &Cmd<'_>,
    command: usize,
    state: &mut StateStack,
    mesh: &mut Mesh,
) -> Result<(), VectorSceneError> {
    match cmd {
        Cmd::Save => state.save(command),
        Cmd::Restore => state.restore(command),
        Cmd::Translate { x, y } => state.translate(*x, *y, command),
        Cmd::Rotate { radians } => state.rotate(*radians, command),
        Cmd::FillColor { color } => {
            state.set_fill(*color);
            Ok(())
        }
        Cmd::Stroke { color, width } => state.set_stroke(*color, *width, command),
        Cmd::ClipRect { x, y, w, h } => state.clip_rect([*x, *y, *w, *h], command),
        // Provenance metadata and layer markers paint nothing on any backend,
        // and the rasterizer counts unknown opcodes rather than failing.
        Cmd::Attribute { .. }
        | Cmd::BeginLayer { .. }
        | Cmd::EndLayer { .. }
        | Cmd::Unknown { .. } => Ok(()),
        Cmd::Line { x1, y1, x2, y2 } => {
            let gs = state.current();
            let verts = [
                gs.ctm.map(*x1, *y1, command)?,
                gs.ctm.map(*x2, *y2, command)?,
            ];
            stroke(mesh, state, &verts, false);
            Ok(())
        }
        Cmd::Polyline { points } => {
            let verts = map_points(state, points, command)?;
            stroke(mesh, state, &verts, false);
            Ok(())
        }
        Cmd::Polygon { mode, points } => {
            let verts = map_points(state, points, command)?;
            paint_outline(mesh, state, *mode, &verts);
            Ok(())
        }
        Cmd::Rect { mode, x, y, w, h } => {
            let corners = rect_corners(&state.current().ctm, [*x, *y, *w, *h], command)?;
            paint_outline(mesh, state, *mode, &corners);
            Ok(())
        }
        Cmd::Circle { mode, cx, cy, r } => circle(mesh, state, *mode, [*cx, *cy, *r], command),
        Cmd::Arc {
            cx,
            cy,
            r,
            start,
            sweep,
        } => arc(mesh, state, [*cx, *cy, *r], [*start, *sweep], command),
        Cmd::Text {
            x,
            y,
            size,
            anchor,
            text: run_text,
        } => {
            let gs = state.current();
            let run = text::Run {
                x: *x,
                y: *y,
                size: finite(*size, command)?,
                anchor: *anchor,
                text: run_text,
            };
            mesh.begin(gs.fill, gs.clip);
            text::draw_run(mesh, &gs.ctm, run, command)
        }
    }
}

fn fills(mode: PaintMode) -> bool {
    matches!(mode, PaintMode::Fill | PaintMode::FillStroke)
}

fn strokes(mode: PaintMode) -> bool {
    matches!(mode, PaintMode::Stroke | PaintMode::FillStroke)
}

/// Strokes `verts` (design units) with the current stroke paint.
fn stroke(mesh: &mut Mesh, state: &StateStack, verts: &[[f32; 2]], closed: bool) {
    let gs = state.current();
    if mesh.begin(gs.stroke_color, gs.clip) {
        shapes::stroke_path(mesh, verts, closed, gs.stroke_width / 2.0);
    }
}

/// Fills then strokes a closed outline as `mode` selects, as two paint
/// operations so the stroke composites over the fill.
fn paint_outline(mesh: &mut Mesh, state: &StateStack, mode: PaintMode, verts: &[[f32; 2]]) {
    let gs = state.current();
    if fills(mode) && mesh.begin(gs.fill, gs.clip) {
        shapes::fill_polygon(mesh, verts);
    }
    if strokes(mode) {
        stroke(mesh, state, verts, true);
    }
}

/// A non-positive radius paints as radius zero, as the rasterizer clamps it.
fn disc(
    state: &StateStack,
    [cx, cy, r]: [f32; 3],
    command: usize,
) -> Result<([f32; 2], f32), VectorSceneError> {
    let centre = state.current().ctm.map(cx, cy, command)?;
    Ok((centre, finite(r, command)?.max(0.0)))
}

fn circle(
    mesh: &mut Mesh,
    state: &StateStack,
    mode: PaintMode,
    v: [f32; 3],
    command: usize,
) -> Result<(), VectorSceneError> {
    let (centre, r) = disc(state, v, command)?;
    let gs = state.current();
    if fills(mode) && mesh.begin(gs.fill, gs.clip) {
        shapes::fill_circle(mesh, centre, r);
    }
    if strokes(mode) && mesh.begin(gs.stroke_color, gs.clip) {
        shapes::stroke_circle(mesh, centre, r, gs.stroke_width / 2.0);
    }
    Ok(())
}

fn arc(
    mesh: &mut Mesh,
    state: &StateStack,
    v: [f32; 3],
    [start, sweep]: [f32; 2],
    command: usize,
) -> Result<(), VectorSceneError> {
    let (centre, r) = disc(state, v, command)?;
    let gs = state.current();
    let device_start = finite(start, command)? + gs.ctm.rotation();
    let sweep = finite(sweep, command)?;
    if mesh.begin(gs.stroke_color, gs.clip) {
        shapes::stroke_arc(mesh, centre, r, device_start, sweep, gs.stroke_width / 2.0);
    }
    Ok(())
}

fn map_points(
    state: &StateStack,
    points: &PointsRef<'_>,
    command: usize,
) -> Result<Vec<[f32; 2]>, VectorSceneError> {
    if points.len() > MAX_POLYGON_VERTICES {
        return Err(VectorSceneError::TooManyVertices {
            command,
            limit: MAX_POLYGON_VERTICES,
        });
    }
    let ctm = state.current().ctm;
    points.iter().map(|[x, y]| ctm.map(x, y, command)).collect()
}
