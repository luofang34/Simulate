//! Shared-GPU `maplibre-rs` adapter for `Simulate` and `Indicate`: [`TruthSceneMap`],
//! [`AvionicsMapSet`], [`EfbMapSurface`], [`GlobePickerSurface`], and [`XvsSvsMbdBlock`].

mod budget;
mod chart_assets;
mod chart_layers;
mod chart_symbology;
mod efb_map;
mod entities;
mod error;
mod geodesy;
mod globe_picker;
mod hangar_preview;
mod hybrid_loader;
mod pose;
mod real_styles;
mod runtime;
mod scenery_loader;
mod streaming;
mod usd_aircraft;
mod usd_pass;
mod volume_patch;
mod xvs_block;

#[cfg(test)]
mod tests;

pub use budget::{CockpitDeviceTier, HANDHELD_AVAILABLE_MEMORY_BYTES};
pub use chart_assets::{CHART_URL_PREFIX, ChartAssetStore, chart_url};
pub use chart_symbology::{
    ChartOwnship, ChartSymbologyInput, build_chart_symbology, build_chart_symbology_scaled,
};
pub use efb_map::{
    ApproachPlateOverlay, DEFAULT_COMPACT_ZOOM, DEFAULT_EXPANDED_ZOOM, EFB_ZOOM_RANGE,
    EfbDisplayMode, EfbFollowMode, EfbLayerFlags, EfbMapConfig, EfbMapSurface, EfbPacer,
    EfbRefreshPolicy, ProceduralPlateSpec, generate_procedural_approach_plate,
};
pub use entities::{
    DynamicEntity3D, DynamicEntityKind, build_conformal_overlay, entities_from_surveillance_belief,
    entity_depth_probes, orientation_from_ned_body_quat,
};
pub use error::MapAdapterError;
pub use globe_picker::{GlobeAirportMarker, GlobePickerSurface, GlobeRunwayEndOverlay};
pub use hangar_preview::{
    HANGAR_AUTO_ROTATE_RAD_PER_S, HANGAR_DEFAULT_SIZE, HangarOrbit, HangarPreviewPass,
};
pub use hybrid_loader::HybridTileLoader;
pub use maplibre::{
    coords::LatLon,
    headless::{
        AltitudeReference, Orientation3d, Placement3d, TerrainDrapeMode, TerrainPatchSnapshot,
        ViewSlotId, ViewSlotMask,
    },
    io::resource_loader::SharedLoader,
    overlay::OverlayMesh,
    projection::body::Body as RenderBody,
    render::host_gpu::HostGpu,
};
pub use pose::{
    ConformalFrameStamp, DEFAULT_FAR_CLIP_M, DEFAULT_NEAR_CLIP_M, GeodeticAnchor,
    LUNAR_RENDER_BODY, MARS_RENDER_BODY, anchor_enu_from_local_enu, clip_from_anchor_enu,
    conformal_stamp_from_pose, conformal_stamp_from_pose_on, conformal_stamp_from_vehicle,
    conformal_stamp_from_vehicle_on, default_render_body, external_view_from_pose,
    external_view_from_pose_on, frustum_from_intrinsics, globe_ned_between_geodetic,
    local_enu_from_geodetic, project_geodetic_to_screen, symmetric_frustum,
};
pub use real_styles::{
    AWS_TERRARIUM_DEM_URL_TEMPLATE, ESRI_WORLD_IMAGERY_URL_TEMPLATE,
    OPENFREEMAP_PLANET_TILEJSON_URL, SceneryEndpoints, belief_aviation_chart_style,
    belief_real_svs_style, truth_real_world_style,
};
pub use runtime::{
    AircraftInstance, AvionicsMapSet, CelestialDisc, CelestialEarthShadow, CelestialSkyInput,
    CelestialStar, CelestialTwilight, ChartCameraSpec, ChartFrameOutcome, ConformalMapFrame,
    InstrumentMapMode, InstrumentMapSpec, InstrumentMapSurface, SharedMapRuntime, TruthSceneConfig,
    TruthSceneFrame, TruthSceneMap, TruthSlotMask,
};
pub use scenery_loader::{
    CATALOG_DEM_URL_TEMPLATE, CATALOG_ORTHO_URL_TEMPLATE, CatalogTileLoader,
    supply_catalog_dem_synchronously, supply_catalog_ortho_synchronously,
    terrarium_tile_to_rgba_image, tile_id_to_world_coords,
};
pub use streaming::{
    CompanionStreamSession, InboundInteractionResult, OutboundDisplayPacket, STREAM_HEADER_BYTES,
    decode_outbound_packet, encode_inbound_interaction_bytes,
};
pub use usd_aircraft::{
    PartRole, PbrMaterial, SlotCameraRole, UsdAircraftAsset, UsdCameraSample, UsdMeshPart,
    UsdMeshVertex, UsdPartDraw, UsdRunwaySample, UsdSceneExportInput, export_scene_usda,
};
pub use usd_pass::{OwnshipVisualKind, USD_AIRCRAFT_PASS};
pub use volume_patch::{
    TerrainVolumeRequest, VolumetricMeshPatch, emit_terrain_volume_with_patch,
    encode_terrain_patch_aux, extract_volumetric_mesh_patch,
};
pub use xvs_block::{
    XVS_DEFAULT_PERIOD, XvsSvsConfig, XvsSvsInput, XvsSvsMbdBlock, XvsSvsOutput, XvsSvsSnapshot,
    xvs_svs_component_spec,
};
