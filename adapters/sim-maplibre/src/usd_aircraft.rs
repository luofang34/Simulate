//! Articulated OpenUSD aircraft asset composition (`openusd` + `openusd-schemas`),
//! per-camera-role kinematic evaluation, and live `.usda` scene stage export.

use std::sync::Arc;

use glam::{DMat4, DQuat, DVec3};
use openusd::ar::AssetBuffer;
use sim_core::AirframeArticulation;

use crate::MapAdapterError;

mod export;
mod finish;
mod loader;
mod occlusion;
mod procedural;
mod resolver;
mod roles;
mod skin;
#[cfg(test)]
mod tests;

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
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PartSurface {
    /// Its material alone.
    #[default]
    Material,
    /// Painted by the asset's baked [`SkinTextures`], sampled through the
    /// part's texture coordinates and tangents.
    Skin,
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
    /// Baked ambient occlusion: the cosine-weighted share of the vertex's sky hemisphere
    /// the airframe leaves open (`1.0` fully open), scaling only sky and environment light.
    pub occlusion: f32,
    /// Prim-local unit tangent along increasing `u` of the part's texture
    /// coordinates, `w` the bitangent's handedness (`±1`); only parts with a
    /// textured skin read it.
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
    /// The [`sim_core::InstrumentNeedles`] channel a [`PartRole::InstrumentNeedle`]
    /// follows (`sim:needleChannel`).
    pub needle_channel: Option<usize>,
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
pub const EMBEDDED_C172_USDC: &[u8] = include_bytes!("../../../vehicles/c172/usd/c172.usdc");

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
}

impl UsdAircraftAsset {
    /// Decodes and composes the embedded BSD-3-Clause Cessna 172 (`c172.usdc`) OpenUSD stage.
    pub fn embedded_c172() -> Result<Self, MapAdapterError> {
        Self::from_usd_bytes("c172.usdc", EMBEDDED_C172_USDC)
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
        loader::load_aircraft_from_bytes(identifier, bytes.into())
    }

    pub(crate) fn from_parts(parts: Vec<UsdMeshPart>) -> Self {
        let radius_m = occlusion::bounding_radius(&parts);
        let lowest_z_m = occlusion::lowest_point_z(&parts);
        Self {
            parts,
            radius_m,
            lowest_z_m,
            skin: None,
        }
    }

    /// The baked skin its [`PartSurface::Skin`] parts sample.
    #[must_use]
    pub fn skin(&self) -> Option<&Arc<SkinTextures>> {
        self.skin.as_ref()
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

    /// Height of the lowest rest-pose vertex (wheel bottoms on a landplane) on the aircraft
    /// `+Z` axis, in metres; ground contact on level gear.
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

    /// Evaluates articulated part transforms and camera-role visibility for one frame, as
    /// seen by the eye or a display camera ([`Self::DISPLAY_SHUTTER_S`]).
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

    /// [`Self::evaluate_draws`] for a camera whose shutter stays open `shutter_s` seconds:
    /// a short exposure leaves a turning propeller as separate blurred blades, a long one
    /// as an even disc.
    #[must_use]
    pub fn evaluate_draws_exposed(
        &self,
        articulation: &AirframeArticulation,
        camera_role: SlotCameraRole,
        shutter_s: f64,
    ) -> Vec<UsdPartDraw> {
        let mount_vib = mount_vibration_matrix(articulation.engine_vibration_rad);
        let mut draws = Vec::with_capacity(self.parts.len());

        for (part_index, part) in self.parts.iter().enumerate() {
            if !role_visible_to_camera(part.role, camera_role) {
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
            let beacon_emissive = (0.15 + 2.55 * articulation.beacon_flash_norm) as f32;
            let land_emissive =
                (0.08 + 3.20 * articulation.landing_light_norm.clamp(0.0, 1.0)) as f32;
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
                mount_vib * part.rest_to_aircraft * joint_delta_matrix(part, articulation);
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

fn mount_vibration_matrix([rx, ry, rz]: [f64; 3]) -> DMat4 {
    DMat4::from_quat(
        DQuat::from_rotation_y(rx) * DQuat::from_rotation_x(ry) * DQuat::from_rotation_z(-rz),
    )
}

fn joint_delta_matrix(part: &UsdMeshPart, art: &AirframeArticulation) -> DMat4 {
    let [comp_nose, comp_left, comp_right] = art.gear_compression_m;
    let [spin_nose, spin_left, spin_right] = art.wheel_phase_rad;
    let retract = (1.0 - art.gear_pos_norm.clamp(0.0, 1.0)).max(0.0);
    match part.role {
        PartRole::PropellerSpinner | PartRole::PropellerBlades => propeller_joint_matrix(part, art),
        PartRole::FlapLeft | PartRole::FlapRight => {
            let rot = DMat4::from_rotation_x(art.flap_deg.to_radians());
            let f = (art.flap_deg / 40.0).clamp(0.0, 1.0);
            if part.prim_path.contains("P38") {
                DMat4::from_translation(DVec3::new(0.0, -0.22 * f, -0.045 * f)) * rot
            } else {
                rot
            }
        }
        PartRole::AileronLeft => DMat4::from_rotation_x(art.left_aileron_rad),
        PartRole::AileronRight => DMat4::from_rotation_x(art.right_aileron_rad),
        PartRole::Elevator => DMat4::from_rotation_x(art.elevator_rad),
        PartRole::RuddervatorLeft => DMat4::from_rotation_x(art.elevator_rad - art.rudder_rad),
        PartRole::RuddervatorRight => DMat4::from_rotation_x(art.elevator_rad + art.rudder_rad),
        PartRole::Rudder => {
            let default_axis = DVec3::new(0.0, -0.261_309, 0.965_255);
            let axis = part
                .custom_hinge_axis
                .unwrap_or(default_axis)
                .normalize_or_zero();
            DMat4::from_axis_angle(axis, -art.rudder_rad)
        }
        PartRole::GearStruts if retract > 1e-4 => {
            DMat4::from_translation(DVec3::new(0.0, -retract * 0.45, retract * 1.15))
        }
        PartRole::LandingLight if retract > 1e-4 => {
            DMat4::from_translation(DVec3::new(0.0, 0.0, retract * 0.08))
                * DMat4::from_rotation_x(-retract * 1.35)
        }
        PartRole::NoseWheel => {
            DMat4::from_translation(DVec3::new(0.0, -retract * 0.72, comp_nose + retract * 1.16))
                * DMat4::from_rotation_x(-retract * 1.35)
                * DMat4::from_rotation_z(-art.nosewheel_steer_rad)
                * DMat4::from_rotation_x(-spin_nose)
        }
        PartRole::WheelLeft | PartRole::WheelRight => {
            let (comp, spin) = if part.role == PartRole::WheelLeft {
                (comp_left, spin_left)
            } else {
                (comp_right, spin_right)
            };
            let t = DVec3::new(0.0, -retract * 0.78, comp + retract * 1.22);
            DMat4::from_translation(t) * DMat4::from_rotation_x(-retract * 1.35 - spin)
        }
        PartRole::InstrumentNeedle => DMat4::from_rotation_y(
            part.needle_channel
                .map_or(0.0, |ch| art.instrument_needles.angle_rad(ch)),
        ),
        _ => articulate_auxiliary_part(part, art),
    }
}

fn propeller_joint_matrix(part: &UsdMeshPart, art: &AirframeArticulation) -> DMat4 {
    let r = part.prim_path.contains("PropRight");
    let s = if r {
        art.propeller_sense_right
    } else {
        art.propeller_sense
    };
    let ph = if r {
        art.propeller_phase_right_rad
    } else {
        art.propeller_phase_rad
    };
    let deg = if r {
        art.propeller_blade_angle_right_deg
    } else {
        art.propeller_blade_angle_deg
    };
    let spin = DMat4::from_rotation_y(-s * ph);
    if part.role == PartRole::PropellerBlades
        && part.custom_hinge_axis.is_some_and(|a| a.x.abs() > 0.9)
    {
        spin * DMat4::from_rotation_x((deg - 24.0).to_radians())
    } else {
        spin
    }
}

fn articulate_auxiliary_part(part: &UsdMeshPart, art: &AirframeArticulation) -> DMat4 {
    let [cl, cr] = art.coolant_flap_norm;
    let [ol, or] = art.oil_shutter_norm;
    let [tr, tp, ty] = art.trim_tab_rad;
    let path = part.prim_path.as_str();
    let sign = part.custom_hinge_axis.map_or(1.0, |a| a.z.signum());
    if path.contains("CoolantFlap") {
        let v = if path.contains("Left") { cl } else { cr };
        DMat4::from_rotation_z(sign * v.clamp(0.0, 1.0) * 0.48)
    } else if path.contains("OilShutter") {
        let v = if path.contains("Left") { ol } else { or };
        DMat4::from_rotation_x(-v.clamp(0.0, 1.0) * 0.45)
    } else if path.contains("ElevatorTrimTab") {
        DMat4::from_rotation_x(art.elevator_rad - tp)
    } else if path.contains("AileronTrimTab") {
        DMat4::from_rotation_x(art.left_aileron_rad - tr)
    } else if path.contains("GearDoor") {
        // Doors are authored closed, flush with the skin, hinged along `y`; they hang
        // open while the gear is down. The hinge axis's length is the signed angle a
        // door opens through.
        let open = part.custom_hinge_axis.map_or(1.45, |a| a.y);
        DMat4::from_rotation_y(open * art.gear_pos_norm.clamp(0.0, 1.0))
    } else if path.contains("RudderTrimTab") {
        DMat4::from_rotation_z(-(art.rudder_rad - ty))
    } else {
        DMat4::IDENTITY
    }
}
