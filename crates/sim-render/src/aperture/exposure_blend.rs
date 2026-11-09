use super::ApertureImage;
use crate::RenderError;

#[cfg(test)]
mod tests;

/// sRGB transfer function, encoded to linear light.
fn srgb_to_linear(encoded: f64) -> f64 {
    if encoded <= 0.040_45 {
        encoded / 12.92
    } else {
        ((encoded + 0.055) / 1.055).powf(2.4)
    }
}

/// sRGB transfer function, linear light to encoded.
fn linear_to_srgb(linear: f64) -> f64 {
    if linear <= 0.003_130_8 {
        linear * 12.92
    } else {
        1.055 * linear.powf(1.0 / 2.4) - 0.055
    }
}

impl ApertureImage {
    /// One motion-blurred frame from sub-frames rendered across an exposure,
    /// such as at [`CameraPose::exposure_sub_poses`](crate::CameraPose::exposure_sub_poses):
    /// the colour is their mean in linear light, as a sensor integrates it,
    /// and the depth is the middle sub-frame's, the mid-exposure instant a
    /// frame is stamped with.
    ///
    /// Each sub-frame is already eight-bit, so the mean carries their
    /// rounding; with sub-frames that move the image by no more than about a
    /// pixel each, the blur is continuous. The whole frame integrates over
    /// the same interval: there is no rolling-shutter skew.
    ///
    /// Fails without sub-frames or when their sizes differ.
    pub fn blend_exposure(sub_frames: &[ApertureImage]) -> Result<Self, RenderError> {
        let (Some(first), Some(middle)) =
            (sub_frames.first(), sub_frames.get(sub_frames.len() / 2))
        else {
            return Err(RenderError::InvalidSensor {
                sensor: "visible exposure",
                detail: "no sub-frames".to_owned(),
            });
        };
        if let Some(odd) = sub_frames.iter().find(|frame| {
            (
                frame.width,
                frame.height,
                frame.rgba8.len(),
                frame.depth.len(),
            ) != (
                first.width,
                first.height,
                first.rgba8.len(),
                first.depth.len(),
            )
        }) {
            return Err(RenderError::InvalidSensor {
                sensor: "visible exposure",
                detail: format!(
                    "sub-frame of {}x{} among {}x{}",
                    odd.width, odd.height, first.width, first.height
                ),
            });
        }
        let decode: Vec<f64> = (0..=255_u8)
            .map(|level| srgb_to_linear(f64::from(level) / 255.0))
            .collect();
        let mut sums = vec![0.0_f64; first.rgba8.len()];
        for frame in sub_frames {
            for (sum, (channel, &level)) in sums.iter_mut().zip((0..4).cycle().zip(&frame.rgba8)) {
                // Alpha is linear already.
                *sum += if channel == 3 {
                    f64::from(level) / 255.0
                } else {
                    decode.get(usize::from(level)).copied().unwrap_or(0.0)
                };
            }
        }
        let count = sub_frames.len() as f64;
        let rgba8 = sums
            .iter()
            .zip((0..4).cycle())
            .map(|(sum, channel)| {
                let mean = sum / count;
                let encoded = if channel == 3 {
                    mean
                } else {
                    linear_to_srgb(mean)
                };
                (encoded * 255.0).round().clamp(0.0, 255.0) as u8
            })
            .collect();
        Ok(Self {
            width: first.width,
            height: first.height,
            rgba8,
            depth: middle.depth.clone(),
        })
    }
}
