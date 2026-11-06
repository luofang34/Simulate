use indicate_instrument_scene::{MAX_STACK_DEPTH, Rgba8};

use super::VectorSceneError;

/// Largest absolute design coordinate the CPU rasterizer accepts; mirrored
/// so a scene fails on both paths alike.
pub(super) const COORD_LIMIT_PX: f32 = 32767.0;

/// A rigid 2x3 map `x' = a*x + c*y + e`, `y' = b*x + d*y + f` from logical
/// to design units, composed in Canvas2D order like the rasterizer's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Affine {
    a: f32,
    b: f32,
    c: f32,
    d: f32,
    e: f32,
    f: f32,
}

impl Affine {
    const IDENTITY: Self = Self {
        a: 1.0,
        b: 0.0,
        c: 0.0,
        d: 1.0,
        e: 0.0,
        f: 0.0,
    };

    fn translate(&mut self, tx: f32, ty: f32) {
        self.e += self.a * tx + self.c * ty;
        self.f += self.b * tx + self.d * ty;
    }

    fn rotate(&mut self, radians: f32) {
        let (s, k) = radians.sin_cos();
        let (a, b, c, d) = (self.a, self.b, self.c, self.d);
        self.a = a * k + c * s;
        self.b = b * k + d * s;
        self.c = -a * s + c * k;
        self.d = -b * s + d * k;
    }

    /// Maps a logical point to design units, failing where the rasterizer's
    /// fixed-point snap would.
    pub(super) fn map(&self, x: f32, y: f32, command: usize) -> Result<[f32; 2], VectorSceneError> {
        let dx = self.a * x + self.c * y + self.e;
        let dy = self.b * x + self.d * y + self.f;
        if !dx.is_finite() || !dy.is_finite() {
            return Err(VectorSceneError::NonFinite { command });
        }
        if dx.abs() > COORD_LIMIT_PX || dy.abs() > COORD_LIMIT_PX {
            return Err(VectorSceneError::CoordinateOutOfRange {
                command,
                limit_px: COORD_LIMIT_PX,
            });
        }
        Ok([dx, dy])
    }

    /// Accumulated rotation, exact for this rigid family.
    pub(super) fn rotation(&self) -> f32 {
        self.b.atan2(self.a)
    }
}

/// Transform, clip, and paints under save/restore.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct GraphicsState {
    /// Logical-to-design transform.
    pub(super) ctm: Affine,
    /// Clip `[min_x, min_y, max_x, max_y]` in physical pixels.
    pub(super) clip: [f32; 4],
    /// Fill colour.
    pub(super) fill: Rgba8,
    /// Stroke colour.
    pub(super) stroke_color: Rgba8,
    /// Stroke width in logical units, never negative.
    pub(super) stroke_width: f32,
}

/// The graphics-state stack, bounded like the rasterizer's.
pub(super) struct StateStack {
    current: GraphicsState,
    saved: Vec<GraphicsState>,
    scale: [f32; 2],
}

impl StateStack {
    /// Identity transform, full-target clip, opaque black paints, unit width.
    pub(super) fn new(target: [u32; 2], scale: [f32; 2]) -> Self {
        let black = Rgba8::rgba(0, 0, 0, 255);
        Self {
            current: GraphicsState {
                ctm: Affine::IDENTITY,
                clip: [0.0, 0.0, target[0] as f32, target[1] as f32],
                fill: black,
                stroke_color: black,
                stroke_width: 1.0,
            },
            saved: Vec::with_capacity(MAX_STACK_DEPTH),
            scale,
        }
    }

    pub(super) const fn current(&self) -> &GraphicsState {
        &self.current
    }

    pub(super) fn save(&mut self, command: usize) -> Result<(), VectorSceneError> {
        if self.saved.len() >= MAX_STACK_DEPTH {
            return Err(VectorSceneError::StackOverflow {
                command,
                limit: MAX_STACK_DEPTH,
            });
        }
        self.saved.push(self.current);
        Ok(())
    }

    pub(super) fn restore(&mut self, command: usize) -> Result<(), VectorSceneError> {
        self.current = self
            .saved
            .pop()
            .ok_or(VectorSceneError::UnbalancedRestore { command })?;
        Ok(())
    }

    pub(super) fn translate(
        &mut self,
        x: f32,
        y: f32,
        command: usize,
    ) -> Result<(), VectorSceneError> {
        self.current
            .ctm
            .translate(finite(x, command)?, finite(y, command)?);
        Ok(())
    }

    pub(super) fn rotate(&mut self, radians: f32, command: usize) -> Result<(), VectorSceneError> {
        self.current.ctm.rotate(finite(radians, command)?);
        Ok(())
    }

    pub(super) fn set_fill(&mut self, color: Rgba8) {
        self.current.fill = color;
    }

    pub(super) fn set_stroke(
        &mut self,
        color: Rgba8,
        width: f32,
        command: usize,
    ) -> Result<(), VectorSceneError> {
        self.current.stroke_color = color;
        self.current.stroke_width = finite(width, command)?.max(0.0);
        Ok(())
    }

    /// Intersects the clip with the axis-aligned bound of a transformed
    /// logical rectangle, rounded to whole physical pixels.
    pub(super) fn clip_rect(
        &mut self,
        rect: [f32; 4],
        command: usize,
    ) -> Result<(), VectorSceneError> {
        let corners = rect_corners(&self.current.ctm, rect, command)?;
        let [sx, sy] = self.scale;
        let (mut lo, mut hi) = ([f32::INFINITY; 2], [f32::NEG_INFINITY; 2]);
        for [x, y] in corners {
            lo = [lo[0].min(x * sx), lo[1].min(y * sy)];
            hi = [hi[0].max(x * sx), hi[1].max(y * sy)];
        }
        let clip = &mut self.current.clip;
        clip[0] = clip[0].max(lo[0].round());
        clip[1] = clip[1].max(lo[1].round());
        clip[2] = clip[2].min(hi[0].round());
        clip[3] = clip[3].min(hi[1].round());
        Ok(())
    }
}

/// Fails a non-finite command operand.
pub(super) fn finite(v: f32, command: usize) -> Result<f32, VectorSceneError> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(VectorSceneError::NonFinite { command })
    }
}

/// The four transformed corners of a logical rectangle, clockwise.
pub(super) fn rect_corners(
    ctm: &Affine,
    [x, y, w, h]: [f32; 4],
    command: usize,
) -> Result<[[f32; 2]; 4], VectorSceneError> {
    Ok([
        ctm.map(x, y, command)?,
        ctm.map(x + w, y, command)?,
        ctm.map(x + w, y + h, command)?,
        ctm.map(x, y + h, command)?,
    ])
}
