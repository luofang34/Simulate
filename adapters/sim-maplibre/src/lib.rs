//! Shared-GPU `maplibre-rs` adapter for `Simulate` and `Indicate`.
//!
//! Enforces strict physical/operational isolation between Simulator `TruthScene`
//! (`OTW` + `30 Hz` optical/EVS sensors via [`TruthSceneMap`]) and `BeliefAvionics`
//! (`ConformalSvs3D` procedural DEM relief + dirty-gated `Map2D` charts via
//! [`BeliefAvionicsMap`]), while sharing one `wgpu::Device`, one compiled shader
//! pipeline cache, and one decoded tile cache through [`SharedMapRuntime`].
//!
//! Also provides deterministic `mbd-core` / `sim_core::Block` conformal framing
//! ([`XvsSvsMbdBlock`]), 3D volumetric terrain patch extraction
//! ([`encode_terrain_patch_aux`], [`extract_volumetric_mesh_patch`]), and
//! fail-closed companion display streaming ([`CompanionStreamSession`]).

mod budget;
mod entities;
mod error;
mod pose;
mod runtime;
mod scenery_loader;
mod streaming;
mod volume_patch;
mod xvs_block;

#[cfg(test)]
mod tests;

pub use budget::{CockpitDeviceTier, HANDHELD_AVAILABLE_MEMORY_BYTES};
pub use entities::{
    DynamicEntity3D, DynamicEntityKind, build_conformal_overlay, entities_from_surveillance_belief,
    orientation_from_ned_body_quat,
};
pub use error::MapAdapterError;
pub use maplibre::{
    headless::{
        AltitudeReference, Orientation3d, Placement3d, TerrainDrapeMode, TerrainPatchSnapshot,
        ViewSlotId, ViewSlotMask,
    },
    render::host_gpu::HostGpu,
};
pub use pose::{
    ConformalFrameStamp, DEFAULT_FAR_CLIP_M, DEFAULT_NEAR_CLIP_M, GeodeticAnchor,
    conformal_stamp_from_vehicle, external_view_from_pose, frustum_from_intrinsics,
    project_geodetic_to_screen, symmetric_frustum,
};
pub use runtime::{
    BeliefAvionicsConfig, BeliefAvionicsMap, ChartCameraSpec, ChartFrameOutcome, SharedMapRuntime,
    TruthSceneConfig, TruthSceneFrame, TruthSceneMap,
};
pub use scenery_loader::{
    CATALOG_DEM_URL_TEMPLATE, CATALOG_ORTHO_URL_TEMPLATE, CatalogTileLoader, belief_chart_style,
    belief_svs_relief_style, supply_catalog_dem_synchronously, terrarium_tile_to_rgba_image,
    tile_id_to_world_coords, truth_otw_style,
};
pub use streaming::{
    CompanionStreamSession, InboundInteractionResult, OutboundDisplayPacket, STREAM_HEADER_BYTES,
    decode_outbound_packet, encode_inbound_interaction_bytes,
};
pub use volume_patch::{
    TerrainVolumeRequest, VolumetricMeshPatch, emit_terrain_volume_with_patch,
    encode_terrain_patch_aux, extract_volumetric_mesh_patch,
};
pub use xvs_block::{
    XVS_DEFAULT_PERIOD, XvsSvsConfig, XvsSvsInput, XvsSvsMbdBlock, XvsSvsOutput, XvsSvsSnapshot,
    xvs_svs_component_spec,
};
