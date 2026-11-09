use sim_geo::InertialTransform;

use crate::{CameraPose, SceneLighting};

mod microbolometer;
mod noise;
mod overlay;
mod pass;
mod pinhole;
mod readout;
mod star_camera;
mod stars;
mod target;
#[cfg(not(target_arch = "wasm32"))]
mod thermal_target;

pub use microbolometer::{
    MICROBOLOMETER_BITS, Microbolometer, MicrobolometerConfig, MicrobolometerFrame,
    SceneTemperature,
};
pub use overlay::{OcclusionOverlayConfig, OpticalReticle, PropellerDisk, WindowBezel};
pub use pass::SensorAperturePass;
pub use pinhole::{PinholeProjection, reversed_z_range_m};
pub use readout::SensorCounts;
pub use star_camera::{
    CatalogStar, NIGHT_SKY_V_MAG_PER_ARCSEC2, STAR_ATMOSPHERE_HEIGHT_M, StarCamera,
    StarCameraConfig, StarCameraExposure, StarCameraTruth, StarTruth, StarVisibility,
    V_ZERO_POINT_PHOTONS_PER_M2_S,
};
pub use target::{
    APERTURE_COLOUR_FORMAT, APERTURE_DEPTH_FORMAT, ApertureImage, OffscreenAperture,
    padded_bytes_per_row,
};
#[cfg(not(target_arch = "wasm32"))]
pub use thermal_target::{THERMAL_KELVIN_FORMAT, ThermalSceneTarget};

/// How a thermal camera maps ground temperature to image level.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalScale {
    /// Grey-body temperature displayed as black, kelvin.
    pub black_k: f32,
    /// Grey-body temperature displayed as white, kelvin.
    pub white_k: f32,
    /// Noise-equivalent temperature difference, kelvin; zero for a clean image.
    pub nedt_k: f32,
    /// Selects the noise pattern; change it per frame for temporal noise.
    /// The same seed gives the same image on a given backend. Seeds travel
    /// through an `f32`, so only `0..2^24` are all distinct.
    pub noise_seed: u32,
}

/// What part of the spectrum a frame is rendered in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ApertureWaveband {
    /// Sunlit terrain with orthoimagery under a uniform sky.
    VisibleRgb {
        /// Sky colour, linear RGB.
        sky_linear_rgb: [f32; 3],
    },
    /// Stars on black space with the sunlit body's limb in front of them, as
    /// seen through a navigation optic.
    StarfieldOptical {
        /// Orients the star catalog relative to the body-fixed frame.
        inertial: InertialTransform,
        /// The optic's reticle, if any.
        reticle: Option<OpticalReticle>,
    },
    /// Long-wave infrared: grey level from surface exitance `σ ε T⁴`. Ground
    /// with no radiometry and open sky read as black.
    ThermalLwir(ThermalScale),
}

/// Frame-exact camera metadata paired with an [`OffscreenAperture`]'s bound
/// [`wgpu::TextureView`] so a single-copy shell (`sokoly.app`, `simulate-app`,
/// Swift Metal or WebXR) aligns conformal HUD/traffic/TFR symbology with the
/// exact pose and intrinsics rendered into that texture.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PresentedViewport {
    /// Camera pose rendered into the bound texture.
    pub pose: CameraPose,
    /// Exact pinhole projection rendered into the bound texture.
    pub projection: PinholeProjection,
}

/// One exposure of a [`SensorAperturePass`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ApertureFrame {
    /// Where the camera is.
    pub pose: CameraPose,
    /// Where the Sun is.
    pub lighting: SceneLighting,
    /// What the camera senses.
    pub waveband: ApertureWaveband,
    /// What obstructs the view, drawn in every waveband.
    pub occlusion: OcclusionOverlayConfig,
}
