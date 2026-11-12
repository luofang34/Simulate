use crate::SceneryError;

#[cfg(test)]
mod tests;

const TERRARIUM_OFFSET_M: f64 = 32_768.0;

/// Decodes one Mapzen Terrarium pixel to metres.
pub fn decode_terrarium([r, g, b]: [u8; 3]) -> f64 {
    (f64::from(r) * 256.0 + f64::from(g) + f64::from(b) / 256.0) - TERRARIUM_OFFSET_M
}

/// Encodes a height as a Terrarium pixel, to the format's `1/256` m step.
/// Heights outside its range saturate.
pub fn encode_terrarium(height_m: f64) -> [u8; 3] {
    let scaled = ((height_m + TERRARIUM_OFFSET_M) * 256.0)
        .round()
        .clamp(0.0, 16_777_215.0) as u32;
    let [_, r, g, b] = scaled.to_be_bytes();
    [r, g, b]
}

/// A decoded elevation raster: one height per pixel, row-major, north row
/// first. A pixel's height applies at its centre.
///
/// Terrarium pixels are signed 24-bit fixed-point numbers with a `1/256` m
/// step (`[-32_768, 32_767 + 255/256]` m), so IEEE-754 `f32` (24-bit
/// significand) stores every possible Terrarium height bit-for-bit without
/// rounding while halving decoded tile memory.
#[derive(Debug, Clone, PartialEq)]
pub struct TerrariumTile {
    width: u32,
    height: u32,
    heights_m: Vec<f32>,
}

impl TerrariumTile {
    /// Decodes tightly packed RGB Terrarium pixels.
    pub fn from_rgb(width: u32, height: u32, rgb: &[u8]) -> Result<Self, SceneryError> {
        Self::decode(width, height, rgb, 3)
    }

    /// Decodes tightly packed RGBA Terrarium pixels, ignoring alpha.
    pub fn from_rgba(width: u32, height: u32, rgba: &[u8]) -> Result<Self, SceneryError> {
        Self::decode(width, height, rgba, 4)
    }

    /// A raster of already decoded heights.
    pub fn from_heights(
        width: u32,
        height: u32,
        heights_m: Vec<f64>,
    ) -> Result<Self, SceneryError> {
        let expected = pixel_count(width, height);
        if expected != Some(heights_m.len()) || heights_m.is_empty() {
            return Err(SceneryError::InvalidRaster {
                width,
                height,
                expected: expected.unwrap_or(usize::MAX),
                actual: heights_m.len(),
            });
        }
        Ok(Self {
            width,
            height,
            heights_m: heights_m.into_iter().map(|height| height as f32).collect(),
        })
    }

    /// Decodes an encoded Terrarium image (PNG, as the public Terrarium
    /// tile servers deliver it), ignoring any alpha channel.
    pub fn from_png_bytes(bytes: &[u8]) -> Result<Self, SceneryError> {
        let corrupt = |detail: String| SceneryError::CorruptTile {
            path: "in-memory Terrarium tile".to_owned(),
            detail,
        };
        let decoded = image::ImageReader::new(std::io::Cursor::new(bytes))
            .with_guessed_format()
            .map_err(|err| corrupt(err.to_string()))?
            .decode()
            .map_err(|err| corrupt(err.to_string()))?;
        match decoded {
            image::DynamicImage::ImageRgb8(rgb) => {
                Self::from_rgb(rgb.width(), rgb.height(), rgb.as_raw())
            }
            image::DynamicImage::ImageRgba8(rgba) => {
                Self::from_rgba(rgba.width(), rgba.height(), rgba.as_raw())
            }
            other => {
                let rgb = other.into_rgb8();
                Self::from_rgb(rgb.width(), rgb.height(), rgb.as_raw())
            }
        }
    }

    fn decode(width: u32, height: u32, pixels: &[u8], stride: usize) -> Result<Self, SceneryError> {
        let expected = pixel_count(width, height).and_then(|count| count.checked_mul(stride));
        if expected != Some(pixels.len()) {
            return Err(SceneryError::InvalidRaster {
                width,
                height,
                expected: expected.unwrap_or(usize::MAX),
                actual: pixels.len(),
            });
        }
        let heights_m: Vec<f32> = pixels
            .chunks_exact(stride)
            .filter_map(|pixel| pixel.first_chunk::<3>())
            .map(|rgb| decode_terrarium(*rgb) as f32)
            .collect();
        if heights_m.is_empty() {
            return Err(SceneryError::InvalidRaster {
                width,
                height,
                expected: expected.unwrap_or(usize::MAX),
                actual: pixels.len(),
            });
        }
        Ok(Self {
            width,
            height,
            heights_m,
        })
    }

    /// Width in pixels.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Height in pixels.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// The height at pixel `(x, y)`, or `None` outside the raster.
    pub fn texel(&self, x: u32, y: u32) -> Option<f64> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let index = y as usize * self.width as usize + x as usize;
        self.heights_m.get(index).copied().map(f64::from)
    }

    /// Replaces every pixel's height with `f(x, y, height)`, row by row.
    pub(crate) fn map_heights(&mut self, mut f: impl FnMut(u32, u32, f64) -> f64) {
        let width = self.width.max(1) as usize;
        for (index, height) in self.heights_m.iter_mut().enumerate() {
            let (x, y) = (index % width, index / width);
            *height = f(x as u32, y as u32, f64::from(*height)) as f32;
        }
    }

    /// Bilinear height at the tile fraction `(u, v)`. Beyond the outermost
    /// pixel centres the edge pixels hold their value.
    pub fn sample_bilinear(&self, u: f64, v: f64) -> Option<f64> {
        bilinear(self.width, self.height, u, v, |x, y| {
            let clamped_x = x.clamp(0, i64::from(self.width) - 1) as u32;
            let clamped_y = y.clamp(0, i64::from(self.height) - 1) as u32;
            self.texel(clamped_x, clamped_y)
        })
    }
}

fn pixel_count(width: u32, height: u32) -> Option<usize> {
    (width as usize).checked_mul(height as usize)
}

/// Blends the four pixels around the tile fraction `(u, v)` of a
/// `width × height` raster. `texel` is asked for pixels one step beyond the
/// raster, so a caller can reach across a tile edge.
pub(crate) fn bilinear(
    width: u32,
    height: u32,
    u: f64,
    v: f64,
    texel: impl Fn(i64, i64) -> Option<f64>,
) -> Option<f64> {
    // Pixel centres lie half a pixel in from the fraction's origin.
    let px = u * f64::from(width) - 0.5;
    let py = v * f64::from(height) - 0.5;
    let (fx, fy) = (px.floor(), py.floor());
    let (tx, ty) = (px - fx, py - fy);
    let (ix, iy) = (fx as i64, fy as i64);
    let north = texel(ix, iy)? * (1.0 - tx) + texel(ix + 1, iy)? * tx;
    let south = texel(ix, iy + 1)? * (1.0 - tx) + texel(ix + 1, iy + 1)? * tx;
    Some(north * (1.0 - ty) + south * ty)
}
