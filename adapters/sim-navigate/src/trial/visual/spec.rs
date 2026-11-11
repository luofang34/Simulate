use glam::{DMat3, DQuat};
use navigate_visual::CameraModel;
use serde::{Deserialize, Serialize};
use sim_geo::SensorIntrinsics;

/// Visual terrain navigation: the simulated vehicle camera, the onboard map
/// package it is matched against, and the declared error budget.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualSpec {
    /// Directory of the onboard map package, written before departure.
    pub package_dir: String,
    /// SHA-256 the package manifest must have, when pinned.
    #[serde(default)]
    pub package_sha256: Option<String>,
    /// Directory of the simulated world's imagery, which only the vehicle
    /// camera sees.
    pub world_dir: String,
    /// The vehicle camera.
    #[serde(default)]
    pub camera: CameraSpec,
    /// What the camera does to the world's light.
    #[serde(default)]
    pub radiometry: RadiometrySpec,
    /// Errors the fix covariance declares beyond image geometry.
    #[serde(default)]
    pub budget: VisualBudgetSpec,
    /// The search around the navigator's estimate.
    #[serde(default)]
    pub prior: PriorSpec,
    /// Navigate's reference renderer settings.
    #[serde(default)]
    pub render: RenderSpec,
    /// What `prepare` puts in the onboard package and the world imagery.
    #[serde(default)]
    pub maps: MapsSpec,
}

/// The tiles `prepare` fetches.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapsSpec {
    /// Half width of the planned corridor, metres. The package also covers
    /// the camera footprint and the widest search beyond it.
    pub corridor_half_width_m: f64,
    /// Onboard imagery zoom.
    pub onboard_imagery_zoom: u8,
    /// Onboard elevation zoom.
    pub onboard_terrain_zoom: u8,
    /// World imagery zoom: finer than the onboard's, so the camera's view is
    /// a different resampling.
    pub world_imagery_zoom: u8,
    /// World elevation zoom.
    pub world_terrain_zoom: u8,
    /// Most tiles of one map.
    pub max_tiles: usize,
}

impl Default for MapsSpec {
    /// A 1.5 km corridor, onboard z15 imagery on z12 elevation, world z16 on
    /// z13, at most 20 000 tiles a map.
    fn default() -> Self {
        Self {
            corridor_half_width_m: 1_500.0,
            onboard_imagery_zoom: 15,
            onboard_terrain_zoom: 12,
            world_imagery_zoom: 16,
            world_terrain_zoom: 13,
            max_tiles: 20_000,
        }
    }
}

/// A nadir camera on the vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraSpec {
    /// Width, pixels.
    pub width: u32,
    /// Height, pixels.
    pub height: u32,
    /// Horizontal field of view, degrees.
    pub horizontal_fov_deg: f64,
    /// Time between frames, seconds; a whole number of simulation steps.
    pub period_s: f64,
    /// Camera position in body axes (forward, right, down) from the centre
    /// of gravity, metres.
    pub mount_offset_body_m: [f64; 3],
    /// Lowest Sun elevation at which the scene is lit, degrees; below it the
    /// camera sees only its own noise.
    pub min_sun_elevation_deg: f64,
}

impl Default for CameraSpec {
    /// A 320 × 240 nadir camera with a 50° field, a frame every five
    /// seconds, under the belly 1 m behind the centre of gravity.
    fn default() -> Self {
        Self {
            width: 320,
            height: 240,
            horizontal_fov_deg: 50.0,
            period_s: 5.0,
            mount_offset_body_m: [-1.0, 0.0, 0.5],
            min_sun_elevation_deg: 0.0,
        }
    }
}

impl CameraSpec {
    /// Focal length in pixels.
    pub fn focal_px(&self) -> f64 {
        f64::from(self.width) / 2.0 / (self.horizontal_fov_deg.to_radians() / 2.0).tan()
    }

    /// The pinhole intrinsics, principal point at the centre.
    pub fn intrinsics(&self) -> SensorIntrinsics {
        let f = self.focal_px();
        SensorIntrinsics {
            width: self.width,
            height: self.height,
            fx: f,
            fy: f,
            cx: (f64::from(self.width) - 1.0) / 2.0,
            cy: (f64::from(self.height) - 1.0) / 2.0,
        }
    }

    /// The same intrinsics in Navigate's type.
    pub fn camera_model(&self) -> CameraModel {
        let k = self.intrinsics();
        CameraModel {
            width: k.width,
            height: k.height,
            fx: k.fx,
            fy: k.fy,
            cx: k.cx,
            cy: k.cy,
        }
    }

    /// Half of the image diagonal's angle, radians.
    pub fn half_diagonal_rad(&self) -> f64 {
        let half = f64::from(self.width).hypot(f64::from(self.height)) / 2.0;
        (half / self.focal_px()).atan()
    }

    /// Camera axes (right, up, back) into body axes (forward, right, down)
    /// for a nadir camera whose image top points forward.
    pub fn camera_to_body() -> DQuat {
        DQuat::from_mat3(&DMat3::from_cols(
            glam::DVec3::Y,
            glam::DVec3::X,
            glam::DVec3::NEG_Z,
        ))
    }
}

/// The camera's response: `gain · 255 · (L/255)^gamma + offset`, plus white
/// noise, in counts.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RadiometrySpec {
    /// Gain.
    pub gain: f64,
    /// Offset, counts.
    pub offset_dn: f64,
    /// Gamma.
    pub gamma: f64,
    /// White noise, counts RMS.
    pub noise_dn: f64,
}

impl Default for RadiometrySpec {
    /// A slightly darker, flatter and noisier rendering than the imagery.
    fn default() -> Self {
        Self {
            gain: 0.85,
            offset_dn: 10.0,
            gamma: 1.15,
            noise_dn: 3.0,
        }
    }
}

/// The error terms Navigate's fusion bridge adds to the image-geometry
/// covariance, metres.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualBudgetSpec {
    /// Horizontal registration error of the package imagery.
    pub map_horizontal_m: f64,
    /// Vertical error of the package elevation.
    pub map_vertical_m: f64,
    /// Camera calibration and mounting.
    pub calibration_m: f64,
    /// Geometry model: elevation resampling and the frame's model error.
    pub geometry_model_m: f64,
    /// Geoid conversion of the Terrarium heights.
    pub vertical_datum_m: f64,
}

impl Default for VisualBudgetSpec {
    /// Metre-level orthoimagery registration, a 10 m elevation model,
    /// decimetre calibration, and a metre each for geometry and geoid.
    fn default() -> Self {
        Self {
            map_horizontal_m: 3.0,
            map_vertical_m: 10.0,
            calibration_m: 0.5,
            geometry_model_m: 1.0,
            vertical_datum_m: 1.0,
        }
    }
}

/// The search around the navigator's estimate.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PriorSpec {
    /// Horizontal 1-sigma multiples the search radius spans.
    pub sigma_multiple: f64,
    /// Smallest search radius, metres.
    pub min_radius_m: f64,
    /// Largest search radius, metres.
    pub max_radius_m: f64,
    /// Attitude search bound, radians.
    pub attitude_radius_rad: f64,
}

impl Default for PriorSpec {
    /// Three sigma between 30 m and 300 m, and 0.05 rad.
    fn default() -> Self {
        Self {
            sigma_multiple: 3.0,
            min_radius_m: 30.0,
            max_radius_m: 300.0,
            attitude_radius_rad: 0.05,
        }
    }
}

/// Settings of Navigate's CPU reference renderer.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RenderSpec {
    /// Farthest terrain distance, metres.
    pub far_m: f64,
    /// Ray-march steps.
    pub steps: u32,
    /// Bisection refinements of a bracketed hit.
    pub refinements: u32,
    /// Grid stride of the adaptive renderer, pixels.
    pub stride: u32,
}

impl Default for RenderSpec {
    /// Settings that resolve a 1–3 km altitude view to well under a metre.
    fn default() -> Self {
        Self {
            far_m: 6_000.0,
            steps: 32,
            refinements: 10,
            stride: 8,
        }
    }
}
