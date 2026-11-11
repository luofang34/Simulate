//! Wall-clock timing of a trial. It depends on the machine and its load, so
//! it never enters the result, which is a function of the request alone; a
//! caller that wants it writes it beside the result.

use serde::{Deserialize, Serialize};

/// Where a trial's wall-clock time went.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrialTiming {
    /// Terrain-camera frames rendered from the world.
    pub terrain_frames_rendered: u64,
    /// Mean wall-clock seconds to render one of them.
    pub terrain_render_s_per_frame: f64,
    /// Each visual set's navigator, in request order.
    pub sets: Vec<SetTiming>,
}

/// One set's navigator time on terrain frames.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SetTiming {
    /// The set.
    pub set: String,
    /// Terrain frames handed to its navigator.
    pub terrain_frames: u64,
    /// Mean wall-clock seconds the navigator spent on one, matching
    /// included.
    pub navigator_s_per_frame: f64,
    /// The slowest frame's seconds.
    pub navigator_s_slowest: f64,
}

impl TrialTiming {
    /// The timing as pretty JSON.
    pub fn to_json(&self) -> Result<String, super::TrialError> {
        serde_json::to_string_pretty(self).map_err(|source| super::TrialError::Json {
            what: "timing",
            source,
        })
    }
}
