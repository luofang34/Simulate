//! GPU pool, drape, and memory budgets tailored per device tier and scene role.
//!
//! `TruthScene` (OTW + optical sensors) and instrument maps (`ConformalSvs3D`
//! + `Map2D`) never use `maplibre`'s `1.0–1.66 GiB` GIS defaults:
//! - OTW and SVS disable `SymbolBufferPool` (`BufferPoolSizes::EMPTY`) so zero
//!   VRAM is spent on road/city SDF text pools and `CollisionSystem` costs `0 ms`.
//! - PFD `ConformalSvs3D` uses `TerrainDrapeMode::ProceduralRelief`, skipping
//!   `DrapeScratch` and `DrapeCache` completely.

use std::time::Duration;

use maplibre::{
    render::settings::{
        BufferPoolSizes, Msaa, ProceduralReliefStyle, RendererSettings, TerrainDrapeMode,
    },
    terrain::DrapeBudget,
};

/// Hardware memory and bandwidth profile of the host device.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum CockpitDeviceTier {
    /// Native desktop workstation or discrete/unified GPU (`>= 8 GiB`).
    #[default]
    Desktop,
    /// Browser tab sharing a WebGPU / WebGL2 GPU process.
    Browser,
    /// Shared-memory mobile / headset target (iPad, iPhone, Apple Vision Pro).
    Handheld,
}

/// Handheld available-memory ceiling (`700 MiB`), matching `visionOS` reserve rules.
pub const HANDHELD_AVAILABLE_MEMORY_BYTES: u64 = 700 << 20;

const OTW_DESKTOP_VECTORS: BufferPoolSizes = BufferPoolSizes {
    vertices: 2_000_000,
    indices: 6_000_000,
    feature_metadata: 2_000_000,
    layer_metadata: 4 * 1024,
};

const OTW_COMPACT_VECTORS: BufferPoolSizes = BufferPoolSizes {
    vertices: 1_000_000,
    indices: 3_000_000,
    feature_metadata: 1_000_000,
    layer_metadata: 4 * 1024,
};

const AVIONICS_DESKTOP_VECTORS: BufferPoolSizes = BufferPoolSizes {
    vertices: 1_000_000,
    indices: 3_000_000,
    feature_metadata: 1_000_000,
    layer_metadata: 4 * 1024,
};

const AVIONICS_COMPACT_VECTORS: BufferPoolSizes = BufferPoolSizes {
    vertices: 600_000,
    indices: 1_800_000,
    feature_metadata: 600_000,
    layer_metadata: 4 * 1024,
};

const AVIONICS_MAP_SYMBOLS: BufferPoolSizes = BufferPoolSizes {
    vertices: 150_000,
    indices: 300_000,
    feature_metadata: 150_000,
    layer_metadata: 2 * 1024,
};

impl CockpitDeviceTier {
    /// Available-memory cap reported to `HeadlessMap::set_available_memory`.
    #[must_use]
    pub const fn available_memory_bytes(self) -> Option<u64> {
        match self {
            Self::Desktop | Self::Browser => None,
            Self::Handheld => Some(HANDHELD_AVAILABLE_MEMORY_BYTES),
        }
    }

    /// Renderer settings for the Simulator `TruthScene` (OTW + optical sensors).
    ///
    /// Always sets `symbol_pools = BufferPoolSizes::EMPTY` (`0` SDF buffers,
    /// `0 ms` label collision).
    #[must_use]
    pub fn truth_scene_settings(self, format: wgpu::TextureFormat) -> RendererSettings {
        let (buffer_pools, edge_px, msaa) = match self {
            Self::Desktop => (OTW_DESKTOP_VECTORS, 1024, Msaa { samples: 4 }),
            Self::Browser => (OTW_COMPACT_VECTORS, 1024, Msaa { samples: 4 }),
            Self::Handheld => (OTW_COMPACT_VECTORS, 1024, Msaa { samples: 1 }),
        };
        RendererSettings {
            msaa,
            buffer_pools,
            symbol_pools: BufferPoolSizes::EMPTY,
            texture_format: Some(format),
            terrain_drape_size: edge_px,
            terrain_drape_mode: TerrainDrapeMode::Draped { edge_px },
            ..RendererSettings::default()
        }
    }

    /// Frame-time-bounded drape bake budget for 60–120 Hz OTW flight.
    #[must_use]
    pub const fn otw_drape_budget(self) -> DrapeBudget {
        match self {
            Self::Desktop => DrapeBudget {
                per_frame: 6,
                per_eye_frame: 5,
                moving_time: Duration::from_micros(3_000),
            },
            Self::Browser | Self::Handheld => DrapeBudget {
                per_frame: 4,
                per_eye_frame: 3,
                moving_time: Duration::from_micros(2_000),
            },
        }
    }

    /// Shortest interval between conformal synthetic-vision redraws (30 Hz). Relief behind
    /// a PFD changes slowly, so drawing it every display frame costs GPU time the OTW needs.
    /// The interval sits just under 1/30 s so 60/120 Hz frame timestamps converted through
    /// `f64` seconds still land on every second/fourth frame.
    #[must_use]
    pub const fn conformal_min_interval(self) -> Duration {
        match self {
            Self::Desktop | Self::Browser | Self::Handheld => Duration::from_micros(33_000),
        }
    }

    /// Renderer settings for a dedicated PFD `ConformalSvs3D` procedural DEM view.
    ///
    /// Disables both `SymbolBufferPool` and `VectorBufferPool` (`EMPTY`) and
    /// activates [`TerrainDrapeMode::ProceduralRelief`] so zero `DrapeScratch` or
    /// `DrapeCache` textures are allocated.
    #[must_use]
    pub fn belief_svs_settings(
        self,
        format: wgpu::TextureFormat,
        reference_altitude_m: Option<f32>,
    ) -> RendererSettings {
        let msaa = match self {
            Self::Desktop | Self::Browser => Msaa { samples: 4 },
            Self::Handheld => Msaa { samples: 1 },
        };
        let relief = ProceduralReliefStyle {
            reference_altitude_meters: reference_altitude_m,
            ..ProceduralReliefStyle::default()
        };
        RendererSettings {
            msaa,
            buffer_pools: BufferPoolSizes::EMPTY,
            symbol_pools: BufferPoolSizes::EMPTY,
            texture_format: Some(format),
            terrain_drape_mode: TerrainDrapeMode::ProceduralRelief(relief),
            ..RendererSettings::default()
        }
    }

    /// Renderer settings for a 2D instrument chart map.
    #[must_use]
    pub fn belief_chart_settings(self, format: wgpu::TextureFormat) -> RendererSettings {
        let (buffer_pools, edge_px, msaa) = match self {
            Self::Desktop => (AVIONICS_DESKTOP_VECTORS, 512, Msaa { samples: 4 }),
            Self::Browser | Self::Handheld => (AVIONICS_COMPACT_VECTORS, 512, Msaa { samples: 1 }),
        };
        RendererSettings {
            msaa,
            buffer_pools,
            symbol_pools: AVIONICS_MAP_SYMBOLS,
            texture_format: Some(format),
            terrain_drape_size: edge_px,
            terrain_drape_mode: TerrainDrapeMode::Draped { edge_px },
            ..RendererSettings::default()
        }
    }
}

#[cfg(test)]
mod tests;
