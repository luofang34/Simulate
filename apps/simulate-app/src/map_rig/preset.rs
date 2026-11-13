//! Avionics presets and the instrument maps each one owns.

use std::time::Duration;

use serde_json::Value;
use sim_indicate::{PANEL_HEIGHT_PX, PANEL_WIDTH_PX};
use sim_maplibre::{
    InstrumentMapMode, InstrumentMapSpec, SceneryEndpoints, belief_aviation_chart_style,
    belief_real_svs_style,
};

use crate::error::AppSimError;

/// PFD synthetic-vision relief behind the attitude indicator.
pub const PFD_SVS_MAP: &str = "pfd_svs";
/// MFD moving map in the instrument dock.
pub const MFD_CHART_MAP: &str = "mfd_nav_chart";
/// Full-window airport-surface diagram.
pub const AIRPORT_SURFACE_MAP: &str = "airport_surface";

const CHART_MIN_INTERVAL: Duration = Duration::from_millis(66);

/// A cockpit instrument fit and the `maplibre-rs` maps it carries.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AvionicsPreset {
    /// Attitude and HSI panels with no map at all.
    NoMapGauges,
    /// Conformal synthetic-vision PFD and the shared EFB map only, with no docked MFD
    /// moving-map or full-window airport-surface chart (used when panels stand apart on a
    /// link's page).
    SyntheticVisionOnly,
    /// One moving map beside the panels.
    SingleMovingMap,
    /// Synthetic-vision PFD, MFD moving map, and a full-window airport-surface chart.
    #[default]
    GlassCockpit,
}

impl AvionicsPreset {
    /// The next preset, cycling.
    #[must_use]
    pub const fn next(self) -> Self {
        match self {
            Self::NoMapGauges => Self::SingleMovingMap,
            Self::SyntheticVisionOnly | Self::SingleMovingMap => Self::GlassCockpit,
            Self::GlassCockpit => Self::NoMapGauges,
        }
    }

    /// Display name.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::NoMapGauges => "Gauges (no maps)",
            Self::SyntheticVisionOnly => "Synthetic vision PFD",
            Self::SingleMovingMap => "Single moving map",
            Self::GlassCockpit => "Glass cockpit",
        }
    }

    /// The instrument maps this preset owns, each with its own style.
    pub fn map_specs(
        self,
        endpoints: &SceneryEndpoints,
        aerodrome_geojson: &Value,
        window_px: [u32; 2],
    ) -> Result<Vec<InstrumentMapSpec>, AppSimError> {
        let panel = [PANEL_WIDTH_PX, PANEL_HEIGHT_PX];
        let svs = || -> Result<InstrumentMapSpec, AppSimError> {
            Ok(InstrumentMapSpec {
                id: PFD_SVS_MAP.to_owned(),
                style: belief_real_svs_style(endpoints)?,
                mode: InstrumentMapMode::Conformal3d {
                    initial_reference_altitude_m: Some(450.0),
                },
                size: panel,
            })
        };
        let chart = |id: &str, size| -> Result<InstrumentMapSpec, AppSimError> {
            Ok(InstrumentMapSpec {
                id: id.to_owned(),
                style: belief_aviation_chart_style(endpoints, aerodrome_geojson.clone())?,
                mode: InstrumentMapMode::Chart2d {
                    min_interval: CHART_MIN_INTERVAL,
                },
                size,
            })
        };
        Ok(match self {
            Self::NoMapGauges => Vec::new(),
            Self::SyntheticVisionOnly => vec![svs()?],
            Self::SingleMovingMap => vec![chart(MFD_CHART_MAP, panel)?],
            Self::GlassCockpit => vec![
                svs()?,
                chart(MFD_CHART_MAP, panel)?,
                chart(AIRPORT_SURFACE_MAP, window_px)?,
            ],
        })
    }
}
