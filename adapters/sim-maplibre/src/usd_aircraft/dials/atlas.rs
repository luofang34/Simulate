//! Dial faces packed into one texture with its whole mip chain, built once on
//! the CPU so the GPU only ever samples it.

/// Faces per atlas row.
const COLUMNS: u32 = 4;

/// An sRGB RGBA8 texture of dial faces: printed colour in `rgb`, the glow
/// strength of luminous paint in `a`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DisplayAtlas {
    width: u32,
    height: u32,
    /// Mip levels, finest first, each tightly packed rows.
    levels: Vec<Vec<u8>>,
}

/// Linear light of each sRGB byte.
fn linear_table() -> [f32; 256] {
    let mut table = [0.0; 256];
    for (byte, slot) in table.iter_mut().enumerate() {
        let c = byte as f32 / 255.0;
        *slot = if c <= 0.04045 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        };
    }
    table
}

fn to_srgb_byte(linear: f32) -> u8 {
    let c = linear.clamp(0.0, 1.0);
    let s = if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    (s * 255.0).round() as u8
}

/// The next coarser level of `level` (`width` x `height`): each texel the
/// mean of four, colour averaged in linear light so faces do not darken as
/// they shrink.
fn downsample(level: &[u8], width: u32, height: u32, linear: &[f32; 256]) -> Vec<u8> {
    let (w, h) = ((width / 2).max(1), (height / 2).max(1));
    let texel = |x: u32, y: u32| -> [u8; 4] {
        let at = ((y.min(height - 1) * width + x.min(width - 1)) * 4) as usize;
        level
            .get(at..at + 4)
            .and_then(|px| <[u8; 4]>::try_from(px).ok())
            .unwrap_or([0; 4])
    };
    let mut out = vec![0_u8; (w * h * 4) as usize];
    for (index, px) in out.as_chunks_mut::<4>().0.iter_mut().enumerate() {
        let (x, y) = ((index as u32 % w) * 2, (index as u32 / w) * 2);
        let quad = [
            texel(x, y),
            texel(x + 1, y),
            texel(x, y + 1),
            texel(x + 1, y + 1),
        ];
        let mean_linear = |channel: usize| {
            quad.iter()
                .map(|t| {
                    let byte = usize::from(t.get(channel).copied().unwrap_or(0));
                    linear.get(byte).copied().unwrap_or(0.0)
                })
                .sum::<f32>()
                * 0.25
        };
        let alpha: u32 = quad
            .iter()
            .map(|t| u32::from(t.get(3).copied().unwrap_or(0)))
            .sum();
        *px = [
            to_srgb_byte(mean_linear(0)),
            to_srgb_byte(mean_linear(1)),
            to_srgb_byte(mean_linear(2)),
            ((alpha + 2) / 4) as u8,
        ];
    }
    out
}

/// Builds an atlas of `count` square cells, `cell` pixels a side.
pub(super) struct AtlasBuilder {
    cell: u32,
    width: u32,
    height: u32,
    pixels: Vec<u8>,
}

impl AtlasBuilder {
    pub(super) fn new(count: u32, cell: u32) -> Self {
        let columns = count.clamp(1, COLUMNS);
        let rows = count.div_ceil(COLUMNS).max(1);
        let (width, height) = (columns * cell, rows * cell);
        Self {
            cell,
            width,
            height,
            pixels: vec![0; (width * height * 4) as usize],
        }
    }

    /// Copies `rgba`, `width` pixels wide and at most a cell in either
    /// direction, into the top left of cell `index`, and returns where it
    /// landed, `[u0, v0, du, dv]` in the atlas.
    pub(super) fn place(&mut self, index: u32, [width, height]: [u32; 2], rgba: &[u8]) -> [f32; 4] {
        let columns = self.width / self.cell;
        let (cx, cy) = ((index % columns) * self.cell, (index / columns) * self.cell);
        let (width, height) = (width.min(self.cell), height.min(self.cell));
        let row_bytes = (width * 4) as usize;
        for (row, src) in rgba
            .chunks_exact(row_bytes)
            .take(height as usize)
            .enumerate()
        {
            let start = (((cy + row as u32) * self.width + cx) * 4) as usize;
            if let Some(dst) = self.pixels.get_mut(start..start + row_bytes) {
                dst.copy_from_slice(src);
            }
        }
        [
            cx as f32 / self.width as f32,
            cy as f32 / self.height as f32,
            width as f32 / self.width as f32,
            height as f32 / self.height as f32,
        ]
    }

    /// The atlas with its full mip chain down to one texel.
    pub(super) fn finish(self) -> DisplayAtlas {
        let linear = linear_table();
        let (mut w, mut h) = (self.width, self.height);
        let mut levels = vec![self.pixels];
        while w > 1 || h > 1 {
            let Some(finer) = levels.last() else {
                break;
            };
            let next = downsample(finer, w, h, &linear);
            (w, h) = ((w / 2).max(1), (h / 2).max(1));
            levels.push(next);
        }
        DisplayAtlas {
            width: self.width,
            height: self.height,
            levels,
        }
    }
}

impl DisplayAtlas {
    /// Width and height of the finest level, pixels.
    #[must_use]
    pub const fn size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// Mip levels, finest first.
    #[must_use]
    pub fn levels(&self) -> &[Vec<u8>] {
        &self.levels
    }
}
