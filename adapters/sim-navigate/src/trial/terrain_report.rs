//! What a visual set's navigator reports of its terrain means, as the result
//! states it: the map it loaded, the camera it matched at, and what became
//! of each frame.

use std::collections::BTreeMap;

use navigate_onboard::{AccuracySource, LayerReport, MapReport, WorkingCamera, WorkingLimit};
use serde::{Deserialize, Serialize};

/// A visual set's terrain-camera record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerrainSetReport {
    /// What the navigator did with each frame's measurement: `accepted`,
    /// `rejected:<filter reason>`, `unused:<navigator reason>:matched` or
    /// `:unmatched`, and `context:<keyframe|render|none>`.
    pub fixes: BTreeMap<String, u64>,
    /// The map the navigator loaded.
    pub map: Option<MapLoadReport>,
    /// The camera the navigator matched the last matched frame at.
    pub working_camera: Option<WorkingCameraReport>,
}

/// The navigator's own account of the map it loaded.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapLoadReport {
    /// The package identity, hex.
    pub package: String,
    /// The imagery layer.
    pub imagery: MapLayerReport,
    /// The elevation layer.
    pub elevation: MapLayerReport,
    /// Bound on the height error from an unknown geoid undulation, metres;
    /// zero when the heights are ellipsoidal or the host's geoid applies.
    pub undulation_bound_m: f64,
    /// Route keyframes in use.
    pub keyframes: u64,
}

/// One loaded layer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapLayerReport {
    /// Kind and source, as the navigator names them.
    pub key: String,
    /// The zoom levels holding tiles.
    pub zooms: Vec<u32>,
    /// Ground size of a pixel at the finest zoom, metres.
    pub ground_sample_m: f64,
    /// Tiles held.
    pub tiles: u64,
    /// Declared gaps.
    pub gaps: u64,
    /// The one-sigma error the navigator takes, metres.
    pub sigma_m: f64,
    /// `package` when the package states it, `assumed` when the request's
    /// assumption stands in for it.
    pub sigma_from: String,
}

/// The camera a frame is matched at.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorkingCameraReport {
    /// Width, pixels.
    pub width: u32,
    /// Height, pixels.
    pub height: u32,
    /// Focal length, pixels.
    pub fx: f64,
    /// Ground size of a pixel, metres.
    pub ground_pixel_m: f64,
    /// What set the resolution: `native`, `map_texel`, `pixel_bound`,
    /// `pixel_floor` or `keyframe`.
    pub limit: String,
}

impl MapLoadReport {
    /// The result's form of the navigator's report.
    pub fn of(report: &MapReport) -> Self {
        Self {
            package: report.package.to_string(),
            imagery: MapLayerReport::of(&report.imagery),
            elevation: MapLayerReport::of(&report.elevation),
            undulation_bound_m: report.undulation_bound_m,
            keyframes: report.keyframes as u64,
        }
    }
}

impl MapLayerReport {
    fn of(layer: &LayerReport) -> Self {
        Self {
            key: format!("{:?}/{:?}", layer.key.kind, layer.key.source),
            zooms: layer.zooms.clone(),
            ground_sample_m: layer.ground_sample_m,
            tiles: layer.tiles,
            gaps: layer.gaps,
            sigma_m: layer.sigma_m,
            sigma_from: match layer.sigma_from {
                AccuracySource::Package => "package",
                AccuracySource::Assumed => "assumed",
            }
            .into(),
        }
    }
}

impl WorkingCameraReport {
    /// The result's form of the navigator's working camera.
    pub fn of(camera: &WorkingCamera) -> Self {
        Self {
            width: camera.intrinsics.width,
            height: camera.intrinsics.height,
            fx: camera.intrinsics.fx,
            ground_pixel_m: camera.ground_pixel_m,
            limit: match camera.limit {
                WorkingLimit::Native => "native",
                WorkingLimit::MapTexel => "map_texel",
                WorkingLimit::PixelBound => "pixel_bound",
                WorkingLimit::PixelFloor => "pixel_floor",
                WorkingLimit::Keyframe => "keyframe",
            }
            .into(),
        }
    }
}
