use sim_geo::SensorIntrinsics;

use crate::RenderError;

/// Optics, response and non-uniformity of a [`Microbolometer`](super::Microbolometer).
///
/// The output is linear in the scene's brightness temperature about
/// `reference_temperature_k`, as a radiometric core's linear-temperature mode
/// writes it:
///
/// `counts = reference_counts + responsivity · (g·v·(T − T_ref) + o + c + n)`
///
/// with `g` the pixel's fixed gain (`1 ± gain_nonuniformity`), `v` the lens
/// vignetting, `o` the pixel's fixed offset and `c` its column's fixed and
/// temporal offsets, all in kelvin, and `n` the temporal noise of standard
/// deviation `netd_k`. Counts are rounded and clipped to 14 bits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MicrobolometerConfig {
    /// Image width, pixels.
    pub width: u32,
    /// Image height, pixels.
    pub height: u32,
    /// Detector pixel pitch, metres.
    pub pixel_pitch_m: f64,
    /// Lens focal length, metres.
    pub focal_length_m: f64,
    /// Noise-equivalent temperature difference: per-pixel temporal noise,
    /// kelvin RMS, at the reference temperature.
    pub netd_k: f64,
    /// Counts per kelvin of scene temperature.
    pub responsivity_counts_per_k: f64,
    /// Scene temperature that reads `reference_counts`, kelvin.
    pub reference_temperature_k: f64,
    /// Count of a scene at `reference_temperature_k`.
    pub reference_counts: f64,
    /// Fixed pixel-to-pixel responsivity spread, fraction RMS.
    pub gain_nonuniformity: f64,
    /// Fixed pixel-to-pixel offset, kelvin RMS.
    pub offset_nonuniformity_k: f64,
    /// Fixed column-to-column offset of the readout, kelvin RMS.
    pub column_offset_k: f64,
    /// Column offset that changes every frame, kelvin RMS.
    pub column_noise_k: f64,
    /// Responsivity lost at the image corners, fraction; it falls off with
    /// the square of the distance from the principal point.
    pub vignetting: f64,
    /// First-order thermal time constant of a pixel, seconds: the detector
    /// temperature relaxes toward the scene as `exp(−t/τ)`. Used by
    /// [`Microbolometer::capture_lagged`](super::Microbolometer::capture_lagged).
    pub time_constant_s: f64,
    /// Selects the fixed pattern of this sensor instance and its noise.
    pub seed: u64,
}

/// Counts of a 14-bit microbolometer readout.
pub const MICROBOLOMETER_BITS: u8 = 14;

/// Responsivity of the presets: 0.02 K per count keeps a 50 mK NETD at 2.5
/// counts, well above the 0.29-count rounding noise, and spans 180 K to
/// 507 K over the 14 bits with 300 K at count 6000.
/// Thermal time constant of a 12 µm VOx pixel. Uncooled data sheets give
/// 8–12 ms at 12 µm pitch; the 10 ms here is mid-range, an assumption rather
/// than one product's figure.
const VOX_TIME_CONSTANT_S: f64 = 0.010;
/// Thermal time constant of a 17 µm a-Si pixel, typically a little slower
/// than VOx at its pitch; 12 ms is an assumption of the same kind.
const ASI_TIME_CONSTANT_S: f64 = 0.012;
const PRESET_COUNTS_PER_K: f64 = 50.0;
const PRESET_REFERENCE_K: f64 = 300.0;
const PRESET_REFERENCE_COUNTS: f64 = 6000.0;

impl MicrobolometerConfig {
    /// A vanadium-oxide 640 × 512 core with 12 µm pixels and a 13 mm lens:
    /// 32.9° by 26.6° field, NETD 50 mK, 10 ms time constant.
    pub fn vox_640x512() -> Self {
        Self {
            width: 640,
            height: 512,
            pixel_pitch_m: 12e-6,
            focal_length_m: 13e-3,
            netd_k: 0.050,
            time_constant_s: VOX_TIME_CONSTANT_S,
            ..Self::preset_response()
        }
    }

    /// An amorphous-silicon 640 × 512 core with 17 µm pixels and a 19 mm
    /// lens: 32.0° by 25.8° field, NETD 60 mK, 12 ms time constant.
    pub fn asi_640x512() -> Self {
        Self {
            width: 640,
            height: 512,
            pixel_pitch_m: 17e-6,
            focal_length_m: 19e-3,
            netd_k: 0.060,
            time_constant_s: ASI_TIME_CONSTANT_S,
            ..Self::preset_response()
        }
    }

    /// Response and non-uniformity shared by the presets: residuals after a
    /// factory two-point correction, of the order data sheets quote.
    fn preset_response() -> Self {
        Self {
            width: 0,
            height: 0,
            pixel_pitch_m: 0.0,
            focal_length_m: 0.0,
            netd_k: 0.0,
            responsivity_counts_per_k: PRESET_COUNTS_PER_K,
            reference_temperature_k: PRESET_REFERENCE_K,
            reference_counts: PRESET_REFERENCE_COUNTS,
            gain_nonuniformity: 0.01,
            offset_nonuniformity_k: 0.2,
            column_offset_k: 0.05,
            column_noise_k: 0.01,
            vignetting: 0.05,
            time_constant_s: VOX_TIME_CONSTANT_S,
            seed: 0,
        }
    }

    /// The pinhole intrinsics the lens and detector give, principal point at
    /// the image centre.
    pub fn intrinsics(&self) -> SensorIntrinsics {
        let focal_px = self.focal_length_m / self.pixel_pitch_m;
        SensorIntrinsics {
            width: self.width,
            height: self.height,
            fx: focal_px,
            fy: focal_px,
            cx: 0.5 * f64::from(self.width) - 0.5,
            cy: 0.5 * f64::from(self.height) - 0.5,
        }
    }

    /// Full horizontal field of view, degrees.
    pub fn horizontal_fov_deg(&self) -> f64 {
        let half_width_m = 0.5 * f64::from(self.width) * self.pixel_pitch_m;
        2.0 * (half_width_m / self.focal_length_m).atan().to_degrees()
    }

    /// Rejects a configuration that cannot form a meaningful frame.
    pub(super) fn validate(&self) -> Result<(), RenderError> {
        let positive = |value: f64| value.is_finite() && value > 0.0;
        let non_negative = |value: f64| value.is_finite() && value >= 0.0;
        let checks = [
            (self.width > 0 && self.height > 0, "image size"),
            (positive(self.pixel_pitch_m), "pixel pitch"),
            (positive(self.focal_length_m), "focal length"),
            (non_negative(self.netd_k), "NETD"),
            (positive(self.responsivity_counts_per_k), "responsivity"),
            (
                positive(self.reference_temperature_k),
                "reference temperature",
            ),
            (self.reference_counts.is_finite(), "reference counts"),
            (non_negative(self.gain_nonuniformity), "gain non-uniformity"),
            (
                non_negative(self.offset_nonuniformity_k),
                "offset non-uniformity",
            ),
            (non_negative(self.column_offset_k), "column offset"),
            (non_negative(self.column_noise_k), "column noise"),
            (
                non_negative(self.vignetting) && self.vignetting < 1.0,
                "vignetting",
            ),
        ];
        match checks.iter().find(|(ok, _)| !ok) {
            Some((_, what)) => Err(RenderError::InvalidSensor {
                sensor: "microbolometer",
                detail: format!("{what} out of range in {self:?}"),
            }),
            None => Ok(()),
        }
    }
}
