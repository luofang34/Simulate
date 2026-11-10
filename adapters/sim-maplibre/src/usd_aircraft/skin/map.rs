//! One texture of a baked skin with its whole mip chain, built on the CPU so
//! the GPU only ever samples it.

/// Texel encoding of a [`SkinMap`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SkinFormat {
    /// sRGB colour and linear alpha, four bytes a texel; colour is averaged
    /// in linear light down the mips.
    Srgba8,
    /// Tangent-space normal `x`, `y` mapped from `-1..1` to `0..255`, two
    /// bytes a texel; `z` is rebuilt as `sqrt(1 - x² - y²)`.
    Normal8,
    /// Four linear channels, four bytes a texel.
    Rgba8,
}

impl SkinFormat {
    /// Bytes per texel.
    #[must_use]
    pub const fn texel_bytes(self) -> usize {
        match self {
            Self::Normal8 => 2,
            Self::Srgba8 | Self::Rgba8 => 4,
        }
    }
}

/// A square-or-rectangular texture and its mip levels, finest first.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SkinMap {
    format: SkinFormat,
    width: u32,
    height: u32,
    levels: Vec<Vec<u8>>,
}

impl SkinMap {
    /// The map whose finest level is `pixels` (`width` x `height`, tightly
    /// packed rows of `format`), with every coarser level down to one texel.
    #[must_use]
    pub fn with_mips(format: SkinFormat, width: u32, height: u32, pixels: Vec<u8>) -> Self {
        let (mut w, mut h) = (width.max(1), height.max(1));
        let mut levels = vec![pixels];
        let srgb = linear_table();
        while w > 1 || h > 1 {
            let Some(finer) = levels.last() else {
                break;
            };
            let next = downsample(format, finer, w, h, &srgb);
            (w, h) = ((w / 2).max(1), (h / 2).max(1));
            levels.push(next);
        }
        Self {
            format,
            width: width.max(1),
            height: height.max(1),
            levels,
        }
    }

    /// The same map from its second level down: half the resolution, a
    /// quarter of the memory.
    #[must_use]
    pub fn without_finest(mut self) -> Self {
        if self.levels.len() > 1 {
            self.levels.remove(0);
            (self.width, self.height) = ((self.width / 2).max(1), (self.height / 2).max(1));
        }
        self
    }

    /// Texel encoding.
    #[must_use]
    pub const fn format(&self) -> SkinFormat {
        self.format
    }

    /// Width and height of the finest level, texels.
    #[must_use]
    pub const fn size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// Mip levels, finest first.
    #[must_use]
    pub fn levels(&self) -> &[Vec<u8>] {
        &self.levels
    }

    /// Bytes of every level together: what the map costs on the GPU.
    #[must_use]
    pub fn byte_len(&self) -> usize {
        self.levels.iter().map(Vec::len).sum()
    }
}

/// Linear light of each sRGB byte.
fn linear_table() -> [f32; 256] {
    let mut table = [0.0; 256];
    for (byte, slot) in table.iter_mut().enumerate() {
        *slot = srgb_to_linear(byte as f32 / 255.0);
    }
    table
}

/// Linear light of an sRGB-encoded value in `0..1`.
pub(crate) fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// The sRGB byte of linear light `linear`.
pub(crate) fn linear_to_srgb_byte(linear: f32) -> u8 {
    let c = linear.clamp(0.0, 1.0);
    let s = if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    (s * 255.0).round() as u8
}

/// The byte of a value in `0..1`.
pub(crate) fn unorm_byte(value: f32) -> u8 {
    (value.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// The next coarser level of `level` (`width` x `height`), each texel the
/// mean of the two-by-two block above it.
fn downsample(
    format: SkinFormat,
    level: &[u8],
    width: u32,
    height: u32,
    srgb: &[f32; 256],
) -> Vec<u8> {
    let bytes = format.texel_bytes();
    let (w, h) = ((width / 2).max(1), (height / 2).max(1));
    let texel = |x: u32, y: u32| -> &[u8] {
        let at = (y.min(height - 1) as usize * width as usize + x.min(width - 1) as usize) * bytes;
        level.get(at..at + bytes).unwrap_or(&[])
    };
    let mut out = vec![0_u8; w as usize * h as usize * bytes];
    for (index, px) in out.chunks_exact_mut(bytes).enumerate() {
        let (x, y) = ((index as u32 % w) * 2, (index as u32 / w) * 2);
        let quad = [
            texel(x, y),
            texel(x + 1, y),
            texel(x, y + 1),
            texel(x + 1, y + 1),
        ];
        let mean = |channel: usize, decode: &dyn Fn(u8) -> f32| {
            quad.iter()
                .map(|t| decode(t.get(channel).copied().unwrap_or(0)))
                .sum::<f32>()
                * 0.25
        };
        let unorm = |b: u8| f32::from(b) / 255.0;
        match format {
            SkinFormat::Srgba8 => {
                let lin = |b: u8| srgb.get(usize::from(b)).copied().unwrap_or(0.0);
                let rgba = [
                    linear_to_srgb_byte(mean(0, &lin)),
                    linear_to_srgb_byte(mean(1, &lin)),
                    linear_to_srgb_byte(mean(2, &lin)),
                    unorm_byte(mean(3, &unorm)),
                ];
                px.copy_from_slice(&rgba);
            }
            SkinFormat::Rgba8 => {
                for (channel, slot) in px.iter_mut().enumerate() {
                    *slot = unorm_byte(mean(channel, &unorm));
                }
            }
            SkinFormat::Normal8 => {
                let snorm = |b: u8| f32::from(b) / 127.5 - 1.0;
                let (nx, ny) = (mean(0, &snorm), mean(1, &snorm));
                // Averaging shortens the normal; a bumpy patch seen from afar
                // reads flatter, which is what the coarser level should show.
                px.copy_from_slice(&[unorm_byte(nx * 0.5 + 0.5), unorm_byte(ny * 0.5 + 0.5)]);
            }
        }
    }
    out
}
