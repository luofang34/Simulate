use super::noise::{NoiseStream, NoiseStreamId};
use super::readout::{SensorCounts, digitise, full_scale};
use crate::RenderError;

mod config;
mod scene;

#[cfg(test)]
mod tests;

pub use config::{MICROBOLOMETER_BITS, MicrobolometerConfig};
pub use scene::SceneTemperature;

/// One microbolometer frame: raw counts for the navigator and, separately,
/// the scene temperature it saw, for evaluation only.
#[derive(Debug, Clone, PartialEq)]
pub struct MicrobolometerFrame {
    /// What the core reads out.
    pub image: SensorCounts,
    /// Brightness temperature at each pixel before the sensor.
    pub truth: SceneTemperature,
}

/// An uncooled long-wave infrared microbolometer core on the CPU,
/// deterministic and headless.
///
/// The fixed pattern (per-pixel gain and offset, per-column offset) and the
/// vignetting are drawn once from the seed and stay constant for the
/// instance, as between two shutter corrections of a real core. Temporal
/// noise (per-pixel NETD, per-column) is drawn from the seed and the frame
/// sequence.
///
/// Not modelled: 1/f offset drift between shutter non-uniformity
/// corrections, the ~10 ms thermal time constant's motion blur, narcissus,
/// halo around hot objects, optics blur, atmospheric path transmission and
/// emission, the non-linearity of in-band radiance in temperature (the
/// output is linear in brightness temperature), and dead pixels.
#[derive(Debug, Clone, PartialEq)]
pub struct Microbolometer {
    config: MicrobolometerConfig,
    /// Per-pixel gain times vignetting, row-major.
    gain: Vec<f64>,
    /// Per-pixel plus per-column fixed offset, kelvin, row-major.
    offset_k: Vec<f64>,
}

impl Microbolometer {
    /// A core of `config` with its fixed pattern drawn from `config.seed`.
    pub fn new(config: MicrobolometerConfig) -> Result<Self, RenderError> {
        config.validate()?;
        let (gain, offset_k) = fixed_pattern(&config);
        Ok(Self {
            config,
            gain,
            offset_k,
        })
    }

    /// The core's configuration.
    pub fn config(&self) -> &MicrobolometerConfig {
        &self.config
    }

    /// Reads out frame `sequence` of `scene`. Fails if the scene is not the
    /// detector's size or holds a temperature that is not finite.
    pub fn capture(
        &self,
        scene: &SceneTemperature,
        sequence: u64,
    ) -> Result<MicrobolometerFrame, RenderError> {
        let config = &self.config;
        if scene.width != config.width
            || scene.height != config.height
            || scene.kelvin.len() != self.gain.len()
            || !scene.kelvin.iter().all(|kelvin| kelvin.is_finite())
        {
            return Err(RenderError::InvalidSensor {
                sensor: "microbolometer",
                detail: format!(
                    "scene of {}x{} ({} values, finite: {}) for a {}x{} detector",
                    scene.width,
                    scene.height,
                    scene.kelvin.len(),
                    scene.kelvin.iter().all(|kelvin| kelvin.is_finite()),
                    config.width,
                    config.height
                ),
            });
        }
        let ceiling = full_scale(MICROBOLOMETER_BITS);
        let mut column_stream = NoiseStream::new(config.seed, sequence, NoiseStreamId::Column);
        let column_noise: Vec<f64> = (0..config.width)
            .map(|_| config.column_noise_k * column_stream.gaussian())
            .collect();
        let mut pixel_stream = NoiseStream::new(config.seed, sequence, NoiseStreamId::Temporal);
        let width = config.width as usize;
        let mut counts = Vec::with_capacity(self.gain.len());
        let rows = scene
            .kelvin
            .chunks_exact(width)
            .zip(self.gain.chunks_exact(width))
            .zip(self.offset_k.chunks_exact(width));
        for ((temperatures, gains), offsets) in rows {
            let row = temperatures
                .iter()
                .zip(gains)
                .zip(offsets)
                .zip(&column_noise)
                .map(|(((&kelvin, gain), offset), column)| {
                    let contrast = f64::from(kelvin) - config.reference_temperature_k;
                    let temporal = config.netd_k * pixel_stream.gaussian();
                    let kelvin_equivalent = gain * contrast + offset + column + temporal;
                    digitise(
                        config.reference_counts
                            + config.responsivity_counts_per_k * kelvin_equivalent,
                        ceiling,
                    )
                });
            counts.extend(row);
        }
        Ok(MicrobolometerFrame {
            image: SensorCounts {
                width: config.width,
                height: config.height,
                significant_bits: MICROBOLOMETER_BITS,
                counts,
            },
            truth: scene.clone(),
        })
    }
}

/// The instance's fixed gain (with vignetting) and offset maps.
fn fixed_pattern(config: &MicrobolometerConfig) -> (Vec<f64>, Vec<f64>) {
    let mut gain_stream = NoiseStream::fixed(config.seed, NoiseStreamId::FixedGain);
    let mut offset_stream = NoiseStream::fixed(config.seed, NoiseStreamId::FixedOffset);
    let mut column_stream = NoiseStream::fixed(config.seed, NoiseStreamId::FixedColumn);
    let column_offset: Vec<f64> = (0..config.width)
        .map(|_| config.column_offset_k * column_stream.gaussian())
        .collect();
    let intrinsics = config.intrinsics();
    let corner_squared = intrinsics.cx * intrinsics.cx + intrinsics.cy * intrinsics.cy;
    let pixels = config.width as usize * config.height as usize;
    let mut gain = Vec::with_capacity(pixels);
    let mut offset = Vec::with_capacity(pixels);
    for y in 0..config.height {
        let dy = f64::from(y) - intrinsics.cy;
        for (x, column) in (0..config.width).zip(&column_offset) {
            let dx = f64::from(x) - intrinsics.cx;
            let radial = if corner_squared > 0.0 {
                (dx * dx + dy * dy) / corner_squared
            } else {
                0.0
            };
            let vignetting = 1.0 - config.vignetting * radial;
            gain.push((1.0 + config.gain_nonuniformity * gain_stream.gaussian()) * vignetting);
            offset.push(config.offset_nonuniformity_k * offset_stream.gaussian() + column);
        }
    }
    (gain, offset)
}
