//! Articulated OpenUSD aircraft asset composition (`openusd` + `openusd-schemas`),
//! per-camera-role kinematic evaluation, per-tail livery composition, and `.usda` scene export.

use std::sync::Arc;

use glam::{DMat4, DVec3};
use openusd::ar::AssetBuffer;
use sim_core::AirframeArticulation;

use crate::MapAdapterError;

mod controls;
mod dials;
mod export;
mod finish;
mod joints;
mod livery_compose;
mod livery_loader;
mod loader;
mod occlusion;
mod procedural;
mod resolver;
mod roles;
mod skin;
#[cfg(test)]
mod tests;

pub use controls::{ControlChannel, ControlJoint};
pub use dials::DisplayAtlas;
pub use export::{UsdCameraSample, UsdRunwaySample, UsdSceneExportInput, export_scene_usda};
pub use finish::{SootStreak, SurfaceDetail};
pub(crate) use resolver::EmbeddedUsdResolver;
use roles::role_visible_to_camera;
pub use roles::{PartRole, SlotCameraRole};
pub use skin::{SkinFormat, SkinMap, SkinTextures};

/// Decoded `UsdPreviewSurface` PBR material parameters for GPU shading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PbrMaterial {
    /// Linear base/diffuse reflectance RGB in `[0, 1]`.
    pub diffuse_rgb: [f32; 3],
    /// Linear emissive radiance RGB.
    pub emissive_rgb: [f32; 3],
    /// Metallic factor in `[0, 1]`.
    pub metallic: f32,
    /// Perceptual roughness in `[0, 1]`.
    pub roughness: f32,
    /// Rest opacity in `[0, 1]` (`< 1.0` selects the translucent pass).
    pub opacity: f32,
}

impl Default for PbrMaterial {
    fn default() -> Self {
        Self {
            diffuse_rgb: [0.85, 0.87, 0.90],
            emissive_rgb: [0.0, 0.0, 0.0],
            metallic: 0.1,
            roughness: 0.35,
            opacity: 1.0,
        }
    }
}

/// How a part's surface is coloured beyond its material.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum PartSurface {
    /// Its material alone.
    #[default]
    Material,
    /// Painted by the asset's baked [`SkinTextures`], sampled through texture coordinates.
    Skin,
    /// Printed from the asset's [`DisplayAtlas`]: `uv` `0..1` maps onto `uv_rect`.
    Display {
        /// Where the part's texture sits in the atlas.
        uv_rect: [f32; 4],
        /// Linear colour of the luminous paint's glow.
        glow_rgb: [f32; 3],
    },
    /// Painted all over in luminous paint glowing `glow_rgb` under the cockpit's lamps.
    Luminous {
        /// Linear colour of the glow.
        glow_rgb: [f32; 3],
    },
}

/// Interleaved GPU-ready vertex extracted from a composed OpenUSD `UsdGeomMesh`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UsdMeshVertex {
    /// Prim-local position `(x_right, y_fwd, z_up)` in metres.
    pub position: [f32; 3],
    /// Prim-local unit normal `(nx, ny, nz)`.
    pub normal: [f32; 3],
    /// Texture / polar coordinate `(u, v)` from `primvars:st`.
    pub uv: [f32; 2],
    /// Baked ambient occlusion (`1.0` fully open), scaling sky and environment light.
    pub occlusion: f32,
    /// Prim-local unit tangent along increasing `u`, `w` bitangent handedness (`±1`).
    pub tangent: [f32; 4],
}

impl UsdMeshVertex {
    /// The tangent of a vertex whose part samples no tangent-space map.
    pub const NO_TANGENT: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
}

/// One articulated mesh part decoded from a composed OpenUSD aircraft stage.
#[derive(Clone, Debug)]
pub struct UsdMeshPart {
    /// Full composed USD prim path (e.g. `/C172/ControlSurfaces/FlapLeft/Surface`).
    pub prim_path: String,
    /// Semantic articulation role.
    pub role: PartRole,
    /// Resolved `UsdPreviewSurface` material.
    pub material: PbrMaterial,
    /// Whether backfaces should be rendered (`UsdGeomGprim.doubleSided`).
    pub double_sided: bool,
    /// Rest transform from mesh-local space to aircraft root (`+X` right, `+Y` fwd, `+Z` up).
    pub rest_to_aircraft: DMat4,
    /// Optional custom hinge axis (`sim:hingeAxis`, e.g. swept rudder hinge).
    pub custom_hinge_axis: Option<DVec3>,
    /// The [`sim_core::InstrumentNeedles`] channel an instrument part follows.
    pub needle_channel: Option<usize>,
    /// The joints a [`PartRole::CockpitControl`] moves on, outermost first.
    pub control_joints: Vec<ControlJoint>,
    /// Procedural paint finish: panel seams, weathering and soot.
    pub detail: SurfaceDetail,
    /// What colours the surface beyond the material.
    pub surface: PartSurface,
    /// Triangulated vertex buffer.
    pub vertices: Vec<UsdMeshVertex>,
    /// Triangle index buffer (`0..vertices.len()`).
    pub indices: Vec<u32>,
}

/// Evaluated per-frame transform and shading state for one [`UsdMeshPart`].
#[derive(Clone, Copy, Debug)]
pub struct UsdPartDraw {
    /// Index into [`UsdAircraftAsset::parts`].
    pub part_index: usize,
    /// Transform from part-local coordinates to aircraft root (`+X` right, `+Y` fwd, `+Z` up).
    pub part_to_aircraft: DMat4,
    /// Effective opacity after RPM blade/disc crossfade and camera-role filtering.
    pub opacity: f32,
    /// Dynamic emissive multiplier (e.g. flashing anti-collision fin beacon).
    pub emissive_scale: f32,
    /// Propeller phase angle in radians (`[0, TAU)`) for procedural shutter-blur wedges.
    pub prop_phase_rad: f32,
    /// Azimuthal shutter-exposure blur angle in radians for the spinning propeller disc.
    pub shutter_blur_rad: f32,
    /// Whether this draw is the procedural motion-blurred propeller disc.
    pub is_prop_disc: bool,
}

/// Embedded BSD-3-Clause Cessna 172 Skyhawk binary OpenUSD Crate (`c172.usdc`).
pub const EMBEDDED_C172_USDC: &[u8] = include_bytes!("../../../vehicles/c172s/usd/c172.usdc");
/// Shared Cessna 172S Skyhawk SP equipment & bare-gear airframe layer (`c172s_airframe.usda`).
pub const EMBEDDED_C172S_AIRFRAME_USDA: &[u8] =
    include_bytes!("../../../vehicles/c172s/usd/c172s_airframe.usda");
/// Default C-172S tail `N642SP` root layer (`n642sp.usda`).
pub const EMBEDDED_N642SP_USDA: &[u8] = include_bytes!("../../../vehicles/c172s/usd/n642sp.usda");
/// Default C-172S tail `N642SP` livery layer (`n642sp_livery.usda`).
pub const EMBEDDED_N642SP_LIVERY_USDA: &[u8] =
    include_bytes!("../../../vehicles/c172s/usd/n642sp_livery.usda");
/// Selectable C-172S tail `N172SP` root layer (`n172sp.usda`).
pub const EMBEDDED_N172SP_USDA: &[u8] = include_bytes!("../../../vehicles/c172s/usd/n172sp.usda");
/// Selectable C-172S tail `N172SP` livery layer (`n172sp_livery.usda`).
pub const EMBEDDED_N172SP_LIVERY_USDA: &[u8] =
    include_bytes!("../../../vehicles/c172s/usd/n172sp_livery.usda");
/// Shared Cessna 172P Skyhawk II / Reims F172P equipment & bare-gear layer (`c172p_airframe.usda`).
pub const EMBEDDED_C172P_AIRFRAME_USDA: &[u8] =
    include_bytes!("../../../vehicles/c172p/usd/c172p_airframe.usda");
/// Default C-172P tail `D-ECJB` root layer (`d_ecjb.usda`).
pub const EMBEDDED_D_ECJB_USDA: &[u8] = include_bytes!("../../../vehicles/c172p/usd/d_ecjb.usda");
/// Default C-172P tail `D-ECJB` livery layer (`d_ecjb_livery.usda`).
pub const EMBEDDED_D_ECJB_LIVERY_USDA: &[u8] =
    include_bytes!("../../../vehicles/c172p/usd/d_ecjb_livery.usda");
/// Selectable C-172P tail `N53295` root layer (`n53295.usda`).
pub const EMBEDDED_N53295_USDA: &[u8] = include_bytes!("../../../vehicles/c172p/usd/n53295.usda");
/// Selectable C-172P tail `N53295` livery layer (`n53295_livery.usda`).
pub const EMBEDDED_N53295_LIVERY_USDA: &[u8] =
    include_bytes!("../../../vehicles/c172p/usd/n53295_livery.usda");
/// Embedded `Raketa` single-stage test rocket OpenUSD stage (`retro_rocket_v4.usd`).
pub const EMBEDDED_RAKETA_USD: &[u8] =
    include_bytes!("../../../vehicles/raketa/assets/retro_rocket_v4.usd");

/// Composed OpenUSD aircraft model ready for GPU upload and kinematic evaluation.
#[derive(Clone, Debug)]
pub struct UsdAircraftAsset {
    parts: Vec<UsdMeshPart>,
    radius_m: f32,
    lowest_z_m: f32,
    skin: Option<Arc<SkinTextures>>,
    display_atlas: Option<Arc<DisplayAtlas>>,
    uv_flood_lamp_m: Option<DVec3>,
}

impl UsdAircraftAsset {
    /// Decodes and composes the generic BSD-3-Clause Cessna 172 (`c172.usdc`) traffic stage.
    pub fn embedded_c172() -> Result<Self, MapAdapterError> {
        Self::from_usd_bytes("c172.usdc", EMBEDDED_C172_USDC)
    }

    /// Composes `N642SP`, the default Cessna 172S Skyhawk SP ownship stage.
    pub fn embedded_n642sp() -> Result<Self, MapAdapterError> {
        Self::from_usd_layers(
            "n642sp.usda",
            [
                ("c172.usdc", EMBEDDED_C172_USDC.into()),
                ("c172s_airframe.usda", EMBEDDED_C172S_AIRFRAME_USDA.into()),
                ("n642sp.usda", EMBEDDED_N642SP_USDA.into()),
                ("n642sp_livery.usda", EMBEDDED_N642SP_LIVERY_USDA.into()),
            ],
        )
    }

    /// Composes `N172SP`, the selectable Cessna 172S Skyhawk SP factory demonstrator stage.
    pub fn embedded_c172s_n172sp() -> Result<Self, MapAdapterError> {
        Self::from_usd_layers(
            "n172sp.usda",
            [
                ("c172.usdc", EMBEDDED_C172_USDC.into()),
                ("c172s_airframe.usda", EMBEDDED_C172S_AIRFRAME_USDA.into()),
                ("n172sp.usda", EMBEDDED_N172SP_USDA.into()),
                ("n172sp_livery.usda", EMBEDDED_N172SP_LIVERY_USDA.into()),
            ],
        )
    }

    /// Composes `D-ECJB`, Mathias Rust's 1987 Reims F172P default Cessna 172P ownship stage.
    pub fn embedded_c172p_d_ecjb() -> Result<Self, MapAdapterError> {
        Self::from_usd_layers(
            "d_ecjb.usda",
            [
                ("c172.usdc", EMBEDDED_C172_USDC.into()),
                ("c172p_airframe.usda", EMBEDDED_C172P_AIRFRAME_USDA.into()),
                ("d_ecjb.usda", EMBEDDED_D_ECJB_USDA.into()),
                ("d_ecjb_livery.usda", EMBEDDED_D_ECJB_LIVERY_USDA.into()),
            ],
        )
    }

    /// Composes `N53295`, the selectable 1982 Cessna 172P Skyhawk II per-tail stage.
    pub fn embedded_c172p_n53295() -> Result<Self, MapAdapterError> {
        Self::from_usd_layers(
            "n53295.usda",
            [
                ("c172.usdc", EMBEDDED_C172_USDC.into()),
                ("c172p_airframe.usda", EMBEDDED_C172P_AIRFRAME_USDA.into()),
                ("n53295.usda", EMBEDDED_N53295_USDA.into()),
                ("n53295_livery.usda", EMBEDDED_N53295_LIVERY_USDA.into()),
            ],
        )
    }

    /// Decodes and composes the embedded `Raketa` (`retro_rocket_v4.usd`) OpenUSD stage.
    pub fn embedded_raketa() -> Result<Self, MapAdapterError> {
        Self::from_usd_bytes("retro_rocket_v4.usd", EMBEDDED_RAKETA_USD)
    }

    /// Decodes and composes an OpenUSD aircraft stage from in-memory `.usdc` or `.usda` bytes.
    pub fn from_usd_bytes(
        identifier: &str,
        bytes: impl Into<AssetBuffer>,
    ) -> Result<Self, MapAdapterError> {
        loader::load_aircraft(
            identifier,
            EmbeddedUsdResolver::single(identifier, bytes.into()),
        )
    }

    /// Composes the stage rooted at `root` from static-keyed in-memory layers.
    pub fn from_usd_layers(
        root: &str,
        layers: impl IntoIterator<Item = (&'static str, AssetBuffer)>,
    ) -> Result<Self, MapAdapterError> {
        Self::from_usd_layers_owned(root, layers.into_iter().map(|(k, v)| (k.to_owned(), v)))
    }

    /// Composes the stage rooted at `root` from runtime-named in-memory layers.
    pub fn from_usd_layers_owned(
        root: &str,
        layers: impl IntoIterator<Item = (String, AssetBuffer)>,
    ) -> Result<Self, MapAdapterError> {
        let mut resolver = EmbeddedUsdResolver::default();
        for (name, bytes) in layers {
            resolver.insert(&name, bytes);
        }
        loader::load_aircraft(&resolver::normalize_asset_key(root), resolver)
    }

    pub(crate) fn from_parts(parts: Vec<UsdMeshPart>) -> Self {
        let radius_m = occlusion::bounding_radius(&parts);
        let lowest_z_m = occlusion::lowest_point_z(&parts);
        Self {
            parts,
            radius_m,
            lowest_z_m,
            skin: None,
            display_atlas: None,
            uv_flood_lamp_m: None,
        }
    }

    /// The baked skin its [`PartSurface::Skin`] parts sample.
    #[must_use]
    pub fn skin(&self) -> Option<&Arc<SkinTextures>> {
        self.skin.as_ref()
    }

    /// Where the cockpit's ultraviolet flood lamp sits, aircraft frame, if present.
    #[must_use]
    pub fn uv_flood_lamp_m(&self) -> Option<DVec3> {
        self.uv_flood_lamp_m
    }

    /// The printed faces its [`PartSurface::Display`] parts sample.
    #[must_use]
    pub fn display_atlas(&self) -> Option<&Arc<DisplayAtlas>> {
        self.display_atlas.as_ref()
    }

    /// An airframe whose vertices carry ambient occlusion baked from its own rest pose.
    pub(crate) fn airframe(name: &str, mut parts: Vec<UsdMeshPart>) -> Self {
        let started = web_time::Instant::now();
        occlusion::bake(&mut parts);
        let vertices: usize = parts.iter().map(|p| p.vertices.len()).sum();
        tracing::info!(
            name,
            vertices,
            elapsed_ms = started.elapsed().as_secs_f64() * 1e3,
            "baked airframe ambient occlusion"
        );
        Self::from_parts(parts)
    }

    /// Height of the lowest rest-pose vertex on the aircraft `+Z` axis, in metres.
    #[must_use]
    pub fn lowest_point_z_m(&self) -> f32 {
        self.lowest_z_m
    }

    /// Radius about the aircraft origin enclosing every rest-pose vertex, in metres.
    #[must_use]
    pub fn bounding_radius_m(&self) -> f32 {
        self.radius_m
    }

    /// Ordered mesh parts in the composed aircraft stage.
    #[must_use]
    pub fn parts(&self) -> &[UsdMeshPart] {
        &self.parts
    }

    /// Total triangle count across all composed parts.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.parts.iter().map(|p| p.indices.len() / 3).sum()
    }

    /// Evaluates articulated part transforms and camera-role visibility for one frame.
    #[must_use]
    pub fn evaluate_draws(
        &self,
        articulation: &AirframeArticulation,
        camera_role: SlotCameraRole,
    ) -> Vec<UsdPartDraw> {
        self.evaluate_draws_exposed(articulation, camera_role, Self::DISPLAY_SHUTTER_S)
    }

    /// Exposure over which a turning propeller smears for the eye or a display camera.
    pub const DISPLAY_SHUTTER_S: f64 = 1.0 / 60.0;

    /// [`Self::evaluate_draws`] for a camera whose shutter stays open `shutter_s` seconds.
    #[must_use]
    pub fn evaluate_draws_exposed(
        &self,
        articulation: &AirframeArticulation,
        camera_role: SlotCameraRole,
        shutter_s: f64,
    ) -> Vec<UsdPartDraw> {
        let mount_vib = joints::mount_vibration_matrix(articulation.engine_vibration_rad);
        let mut draws = Vec::with_capacity(self.parts.len());

        for (part_index, part) in self.parts.iter().enumerate() {
            if !role_visible_to_camera(part.role, camera_role) {
                continue;
            }
            if !dials::instrument_part_shown(part, &articulation.instrument_needles) {
                continue;
            }
            if articulation.gear_pos_norm < 0.03
                && matches!(
                    part.role,
                    PartRole::GearStruts
                        | PartRole::NoseWheel
                        | PartRole::WheelLeft
                        | PartRole::WheelRight
                        | PartRole::LandingLight
                )
            {
                continue;
            }
            let engine_idx = usize::from(part.prim_path.contains("PropRight"));
            let prop_vis = articulation.propeller_visual_for_engine(engine_idx, shutter_s);
            let beacon_emissive = (0.15 + 2.55 * articulation.lights.beacon_flash_norm) as f32;
            let land_emissive =
                (0.08 + 3.20 * articulation.lights.landing_light_norm.clamp(0.0, 1.0)) as f32;
            let (opacity, emissive_scale, is_prop_disc) = match part.role {
                PartRole::PropellerBlades => (prop_vis.blade_opacity, 1.0, false),
                PartRole::PropellerDisc => (prop_vis.disc_opacity, 1.0, true),
                PartRole::BeaconLight => (part.material.opacity, beacon_emissive, false),
                PartRole::LandingLight => (part.material.opacity, land_emissive, false),
                _ => (part.material.opacity, 1.0, false),
            };
            if opacity <= 0.005 {
                continue;
            }
            let part_to_aircraft =
                mount_vib * part.rest_to_aircraft * joints::joint_delta_matrix(part, articulation);
            draws.push(UsdPartDraw {
                part_index,
                part_to_aircraft,
                opacity,
                emissive_scale,
                prop_phase_rad: prop_vis.phase_rad,
                shutter_blur_rad: prop_vis.shutter_blur_angle_rad,
                is_prop_disc,
            });
        }
        draws
    }
}
