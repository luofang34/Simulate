use std::f64::consts::PI;

use sim_geo::SensorIntrinsics;

use crate::RenderError;

/// In-band photon flux of a star of `V = 0` above the atmosphere, photons per
/// square metre per second. From the Bessell (1998) Johnson `V` zero point,
/// `f_λ = 3.631e-9 erg cm⁻² s⁻¹ Å⁻¹` at `λ = 545 nm` (996 photons cm⁻² s⁻¹
/// Å⁻¹), times the `V` passband's 88 nm width. The detector's response is
/// treated as the `V` band; a wider unfiltered silicon response, and the
/// colour dependence it brings, belong in the quantum efficiency.
pub const V_ZERO_POINT_PHOTONS_PER_M2_S: f64 = 8.8e9;

/// Moonless sky at a dark site, Johnson `V` magnitudes per square arcsecond.
pub const NIGHT_SKY_V_MAG_PER_ARCSEC2: f64 = 21.5;

/// Height above WGS84 below which a grazing ray counts as blocked by the
/// Earth: under about 50 km tangent height, refraction and extinction spoil a
/// star sighting, so star trackers exclude that layer around the limb.
pub const STAR_ATMOSPHERE_HEIGHT_M: f64 = 50_000.0;

const ARCSEC_PER_RAD: f64 = 206_264.806_247;

/// Optics and detector of a [`StarCamera`](super::StarCamera).
///
/// Signal electrons from a star of magnitude `V`:
/// `V_ZERO_POINT_PHOTONS_PER_M2_S · 10^(−0.4 V) · π (D/2)² · t · τ · QE`.
/// The sky adds the same per square arcsecond of `sky_brightness`, times each
/// pixel's solid angle at the principal point. Counts are
/// `bias_dn + min(electrons, full_well_e) / gain_e_per_dn`, rounded and
/// clipped to `significant_bits`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StarCameraConfig {
    /// Image size and pinhole projection.
    pub intrinsics: SensorIntrinsics,
    /// Entrance pupil diameter `D`, metres.
    pub aperture_diameter_m: f64,
    /// Exposure time `t`, seconds.
    pub exposure_s: f64,
    /// Fraction of light the optics pass, `τ`, in `(0, 1]`.
    pub optics_transmission: f64,
    /// Electrons per incident photon, `QE`, in `(0, 1]`.
    pub quantum_efficiency: f64,
    /// Standard deviation of the Gaussian point-spread function, pixels.
    pub psf_sigma_px: f64,
    /// Sky surface brightness, Johnson `V` magnitudes per square arcsecond.
    /// [`NIGHT_SKY_V_MAG_PER_ARCSEC2`] for a dark night; twilight and daylight
    /// skies are the caller's to set, since their brightness depends on Sun
    /// geometry and aerosol this model does not carry.
    pub sky_v_mag_per_arcsec2: f64,
    /// Dark current, electrons per pixel per second.
    pub dark_current_e_per_s: f64,
    /// Read noise, electrons RMS.
    pub read_noise_e: f64,
    /// Electrons a pixel holds before it saturates.
    pub full_well_e: f64,
    /// Electrons per count.
    pub gain_e_per_dn: f64,
    /// Count of a pixel with no charge, so read noise is not clipped at zero.
    pub bias_dn: f64,
    /// ADC resolution, bits; one to sixteen.
    pub significant_bits: u8,
    /// Height of the Earth's blocking layer; see [`STAR_ATMOSPHERE_HEIGHT_M`].
    pub atmosphere_height_m: f64,
    /// Selects the noise; frames of the same seed and sequence are identical.
    pub seed: u64,
}

impl StarCameraConfig {
    /// A narrow-field star tracker: 1024 × 1024 pixels over 20°, 5.5 µm
    /// pixels behind a 16 mm f/1.33 lens (12 mm pupil), 100 ms exposure.
    /// A `V = 6` star gives about 190 electrons against about 5 electrons of
    /// noise per pixel; `V = 0` peaks at about a third of the full well.
    pub fn star_tracker_1024() -> Self {
        Self {
            intrinsics: square_intrinsics(1024, 20.0),
            aperture_diameter_m: 0.012,
            exposure_s: 0.1,
            optics_transmission: 0.8,
            quantum_efficiency: 0.6,
            psf_sigma_px: 1.0,
            sky_v_mag_per_arcsec2: NIGHT_SKY_V_MAG_PER_ARCSEC2,
            dark_current_e_per_s: 20.0,
            read_noise_e: 5.0,
            full_well_e: 20_000.0,
            gain_e_per_dn: 5.0,
            bias_dn: 64.0,
            significant_bits: 12,
            atmosphere_height_m: STAR_ATMOSPHERE_HEIGHT_M,
            seed: 0,
        }
    }

    /// A wide-field star camera: 2048 × 2048 pixels over 40°, 20 mm pupil,
    /// 200 ms exposure, for lost-in-space identification and sky survey.
    pub fn wide_field_2048() -> Self {
        Self {
            intrinsics: square_intrinsics(2048, 40.0),
            aperture_diameter_m: 0.020,
            exposure_s: 0.2,
            optics_transmission: 0.8,
            quantum_efficiency: 0.6,
            psf_sigma_px: 1.2,
            sky_v_mag_per_arcsec2: NIGHT_SKY_V_MAG_PER_ARCSEC2,
            dark_current_e_per_s: 20.0,
            read_noise_e: 3.0,
            full_well_e: 30_000.0,
            gain_e_per_dn: 7.5,
            bias_dn: 64.0,
            significant_bits: 12,
            atmosphere_height_m: STAR_ATMOSPHERE_HEIGHT_M,
            seed: 0,
        }
    }

    /// Signal electrons from a star of `v_magnitude` over one exposure,
    /// summed over its whole point-spread function.
    pub fn star_electrons(&self, v_magnitude: f64) -> f64 {
        self.electrons_per_unit_flux() * 10.0_f64.powf(-0.4 * v_magnitude)
    }

    /// Sky electrons per pixel over one exposure.
    pub fn sky_electrons_per_px(&self) -> f64 {
        let pixel_arcsec2 =
            (ARCSEC_PER_RAD / self.intrinsics.fx) * (ARCSEC_PER_RAD / self.intrinsics.fy);
        self.electrons_per_unit_flux()
            * 10.0_f64.powf(-0.4 * self.sky_v_mag_per_arcsec2)
            * pixel_arcsec2
    }

    /// Dark-current electrons per pixel over one exposure.
    pub fn dark_electrons_per_px(&self) -> f64 {
        self.dark_current_e_per_s * self.exposure_s
    }

    /// Electrons from a `V = 0` source over one exposure.
    fn electrons_per_unit_flux(&self) -> f64 {
        let radius = 0.5 * self.aperture_diameter_m;
        V_ZERO_POINT_PHOTONS_PER_M2_S
            * PI
            * radius
            * radius
            * self.exposure_s
            * self.optics_transmission
            * self.quantum_efficiency
    }

    /// Rejects a configuration that cannot form a meaningful frame.
    pub(super) fn validate(&self) -> Result<(), RenderError> {
        let positive = |value: f64| value.is_finite() && value > 0.0;
        let non_negative = |value: f64| value.is_finite() && value >= 0.0;
        let fraction = |value: f64| positive(value) && value <= 1.0;
        let checks = [
            (positive(self.aperture_diameter_m), "aperture diameter"),
            (positive(self.exposure_s), "exposure"),
            (fraction(self.optics_transmission), "optics transmission"),
            (fraction(self.quantum_efficiency), "quantum efficiency"),
            (positive(self.psf_sigma_px), "PSF sigma"),
            (self.sky_v_mag_per_arcsec2.is_finite(), "sky brightness"),
            (non_negative(self.dark_current_e_per_s), "dark current"),
            (non_negative(self.read_noise_e), "read noise"),
            (positive(self.full_well_e), "full well"),
            (positive(self.gain_e_per_dn), "gain"),
            (non_negative(self.bias_dn), "bias"),
            ((1..=16).contains(&self.significant_bits), "ADC bits"),
            (non_negative(self.atmosphere_height_m), "atmosphere height"),
        ];
        match checks.iter().find(|(ok, _)| !ok) {
            Some((_, what)) => Err(RenderError::InvalidSensor {
                sensor: "star camera",
                detail: format!("{what} out of range in {self:?}"),
            }),
            None => Ok(()),
        }
    }
}

/// Square pixels on a square detector with `fov_deg` across its width, and
/// the principal point at the image centre.
fn square_intrinsics(size_px: u32, fov_deg: f64) -> SensorIntrinsics {
    let half_width = 0.5 * f64::from(size_px);
    let focal_px = half_width / (0.5 * fov_deg).to_radians().tan();
    let centre = half_width - 0.5;
    SensorIntrinsics {
        width: size_px,
        height: size_px,
        fx: focal_px,
        fy: focal_px,
        cx: centre,
        cy: centre,
    }
}
