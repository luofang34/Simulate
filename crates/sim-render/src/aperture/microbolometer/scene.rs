use sim_scenery::SurfaceRadiometry;

use crate::{ApertureImage, RenderError, ThermalScale};

/// The brightness temperature reaching each pixel before the sensor: the
/// input of a [`Microbolometer`](super::Microbolometer) and the truth of its
/// frames.
#[derive(Debug, Clone, PartialEq)]
pub struct SceneTemperature {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Row-major kelvin, top row first.
    pub kelvin: Vec<f32>,
}

impl SceneTemperature {
    /// A scene at one temperature.
    pub fn uniform(width: u32, height: u32, kelvin: f32) -> Self {
        Self {
            width,
            height,
            kelvin: vec![kelvin; width as usize * height as usize],
        }
    }

    /// The temperature at pixel `(x, y)`, or `None` outside the image.
    pub fn kelvin_at(&self, x: u32, y: u32) -> Option<f32> {
        if x >= self.width || y >= self.height {
            return None;
        }
        self.kelvin
            .get(y as usize * self.width as usize + x as usize)
            .copied()
    }

    /// Recovers the scene from a [`ThermalLwir`](crate::ApertureWaveband::ThermalLwir)
    /// frame rendered with `scale` and no noise (`nedt_k` zero).
    ///
    /// That frame stores black-body exitance in eight bits, so each kelvin
    /// here is quantised to one of 256 levels: a step of
    /// `(σ white⁴ − σ black⁴) / (255 · 4σT³)`, about 0.4 K at 300 K for a
    /// 250–350 K scale, which is coarser than a microbolometer's NETD. A
    /// native build reads the scene unquantised from a
    /// `ThermalSceneTarget` instead.
    /// Ground below `black_k`, open sky and ground without radiometry all
    /// read `black_k`, and ground above `white_k` reads `white_k`.
    pub fn from_thermal_image(
        image: &ApertureImage,
        scale: &ThermalScale,
    ) -> Result<Self, RenderError> {
        let pixels = image.width as usize * image.height as usize;
        let usable = scale.black_k.is_finite()
            && scale.white_k.is_finite()
            && scale.black_k >= 0.0
            && scale.white_k > scale.black_k;
        if !usable || image.rgba8.len() != pixels * 4 {
            return Err(RenderError::InvalidSensor {
                sensor: "thermal scene",
                detail: format!(
                    "{} bytes for {}x{} under {scale:?}",
                    image.rgba8.len(),
                    image.width,
                    image.height
                ),
            });
        }
        let sigma = f64::from(SurfaceRadiometry::STEFAN_BOLTZMANN);
        let exitance = |kelvin: f32| sigma * f64::from(kelvin).powi(4);
        let black = exitance(scale.black_k);
        let span = exitance(scale.white_k) - black;
        let kelvin = image
            .rgba8
            .as_chunks::<4>()
            .0
            .iter()
            .map(|[grey, ..]| {
                let level = f64::from(*grey) / 255.0;
                ((black + level * span) / sigma).powf(0.25) as f32
            })
            .collect();
        Ok(Self {
            width: image.width,
            height: image.height,
            kelvin,
        })
    }
}
