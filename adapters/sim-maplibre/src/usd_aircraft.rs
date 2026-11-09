//! Articulated OpenUSD aircraft asset composition (`openusd` + `openusd-schemas`),
//! per-camera-role kinematic evaluation, and live `.usda` scene stage export.

use std::{
    borrow::Cow,
    collections::BTreeMap,
    io::{self, Cursor},
    time::SystemTime,
};

use glam::{DMat4, DQuat, DVec3};
use openusd::ar::{Asset, AssetBuffer, ResolvedPath, Resolver};
use sim_core::AirframeArticulation;

use crate::MapAdapterError;

mod export;
mod loader;
mod procedural;
#[cfg(test)]
mod tests;

pub use export::{UsdCameraSample, UsdRunwaySample, UsdSceneExportInput, export_scene_usda};

/// Camera mounting role governing interior/glass/wing self-occlusion culling on ownship.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SlotCameraRole {
    /// Unobstructed forward HUD / boresight view hiding all ownship airframe and propeller parts.
    HudForward,
    /// Pilot eye inside the cabin looking forward through the windshield and propeller arc.
    CockpitInterior,
    /// Left cabin window camera looking abeam under the high wing and strut.
    LeftCabinWindow,
    /// Belly nadir optical navigation camera with unobstructed downward view.
    BellyNadir,
    /// External camera (chase, tower PTZ, wingtip, flyby) viewing the complete aircraft.
    #[default]
    External,
}

/// Semantic articulation role authored via `custom token sim:partRole` on each USD `Mesh`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartRole {
    /// Main fuselage, empennage, and wheel-pant exterior skin.
    CabinExterior,
    /// Cabin interior liner and instrument glare shield.
    CabinInterior,
    /// Forward engine cowling visible ahead of the windshield from the pilot seat.
    NoseCowling,
    /// High-wing structure visible from external and cabin-window cameras.
    WingStructure,
    /// Diagonal wing lift struts and pitot hardware.
    WingStrut,
    /// Fixed or retractable landing-gear struts and open wheel-bay doors.
    GearStruts,
    /// Flashing red anti-collision fin beacon lens.
    BeaconLight,
    /// Retractable under-wing or leading-edge landing light assembly.
    LandingLight,
    /// Translucent cabin windshield and side/rear windows.
    CabinGlass,
    /// Polished nose spinner cone rotating with the propeller hub.
    PropellerSpinner,
    /// Solid twisted propeller blades and safety tips (visible at low RPM / stopped).
    PropellerBlades,
    /// Translucent motion-blurred propeller disc annulus (visible at running RPM).
    PropellerDisc,
    /// Left trailing-edge electric slotted flap.
    FlapLeft,
    /// Right trailing-edge electric slotted flap.
    FlapRight,
    /// Left outboard differential aileron.
    AileronLeft,
    /// Right outboard differential aileron.
    AileronRight,
    /// Horizontal-tail elevator.
    Elevator,
    /// Swept vertical-fin rudder.
    Rudder,
    /// Left V-tail ruddervator hinged about local `+X`, deflecting by
    /// `elevator_rad - rudder_rad` so it mixes pitch and yaw.
    RuddervatorLeft,
    /// Right V-tail ruddervator hinged about local `+X`, deflecting by
    /// `elevator_rad + rudder_rad` so it mixes pitch and yaw.
    RuddervatorRight,
    /// Steerable nosewheel tire and hubcap.
    NoseWheel,
    /// Left main-gear tire and hubcap.
    WheelLeft,
    /// Right main-gear tire and hubcap.
    WheelRight,
}

impl PartRole {
    pub(crate) fn from_token(token: &str) -> Self {
        match token {
            "cabin_interior" => Self::CabinInterior,
            "nose_cowling" => Self::NoseCowling,
            "wing_structure" => Self::WingStructure,
            "wing_strut" => Self::WingStrut,
            "gear_struts" => Self::GearStruts,
            "beacon_light" => Self::BeaconLight,
            "landing_light" => Self::LandingLight,
            "cabin_glass" => Self::CabinGlass,
            "propeller_spinner" => Self::PropellerSpinner,
            "propeller_blades" => Self::PropellerBlades,
            "propeller_disc" => Self::PropellerDisc,
            "flap_left" => Self::FlapLeft,
            "flap_right" => Self::FlapRight,
            "aileron_left" => Self::AileronLeft,
            "aileron_right" => Self::AileronRight,
            "elevator" => Self::Elevator,
            "rudder" => Self::Rudder,
            "ruddervator_left" => Self::RuddervatorLeft,
            "ruddervator_right" => Self::RuddervatorRight,
            "nose_wheel" => Self::NoseWheel,
            "wheel_left" => Self::WheelLeft,
            "wheel_right" => Self::WheelRight,
            _ => Self::CabinExterior,
        }
    }
}

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

/// Interleaved GPU-ready vertex extracted from a composed OpenUSD `UsdGeomMesh`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UsdMeshVertex {
    /// Prim-local position `(x_right, y_fwd, z_up)` in metres.
    pub position: [f32; 3],
    /// Prim-local unit normal `(nx, ny, nz)`.
    pub normal: [f32; 3],
    /// Texture / polar coordinate `(u, v)` from `primvars:st`.
    pub uv: [f32; 2],
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
        Self { parts }
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
            let prop_vis = articulation.propeller_visual_for_engine(engine_idx, 1.0 / 60.0);
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

/// In-memory `openusd::ar::Resolver` serving embedded `.usdc` / `.usda` buffers on both
/// native and `wasm32` targets without filesystem access.
#[derive(Clone, Debug, Default)]
pub(crate) struct EmbeddedUsdResolver {
    assets: BTreeMap<String, AssetBuffer>,
}

impl EmbeddedUsdResolver {
    pub(crate) fn single(name: &str, bytes: AssetBuffer) -> Self {
        Self {
            assets: BTreeMap::from([(normalize_asset_key(name), bytes)]),
        }
    }

    #[cfg(test)]
    pub(crate) fn insert(&mut self, name: &str, bytes: AssetBuffer) {
        self.assets.insert(normalize_asset_key(name), bytes);
    }
}

impl Resolver for EmbeddedUsdResolver {
    fn create_identifier(&self, asset_path: &str, _anchor: Option<&ResolvedPath>) -> String {
        normalize_asset_key(asset_path)
    }

    fn resolve(&self, asset_path: &str) -> Option<ResolvedPath> {
        let key = normalize_asset_key(asset_path);
        self.assets
            .contains_key(&key)
            .then(|| ResolvedPath::new(key))
    }

    fn resolve_for_new_asset(&self, asset_path: &str) -> Option<ResolvedPath> {
        Some(ResolvedPath::new(normalize_asset_key(asset_path)))
    }

    fn open_asset(&self, resolved_path: &ResolvedPath) -> io::Result<Box<dyn Asset>> {
        let key = normalize_asset_key(&resolved_path.to_string_lossy());
        let buffer = self
            .assets
            .get(&key)
            .cloned()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, key))?;
        Ok(Box::new(Cursor::new(buffer)))
    }

    fn in_memory_identifier<'a>(&self, asset_path: &'a str) -> Cow<'a, str> {
        Cow::Owned(normalize_asset_key(asset_path))
    }

    fn get_modification_timestamp(
        &self,
        _path: &str,
        _resolved: &ResolvedPath,
    ) -> Option<SystemTime> {
        None
    }

    fn identity(&self) -> String {
        "sim-embedded-usd".to_owned()
    }
}

fn normalize_asset_key(path: &str) -> String {
    path.trim_start_matches("./")
        .trim_start_matches('/')
        .to_owned()
}

fn role_visible_to_camera(role: PartRole, camera: SlotCameraRole) -> bool {
    use PartRole::{
        CabinExterior, CabinGlass, CabinInterior, PropellerBlades, PropellerDisc, PropellerSpinner,
    };
    match camera {
        SlotCameraRole::External => true,
        SlotCameraRole::HudForward | SlotCameraRole::BellyNadir => false,
        SlotCameraRole::CockpitInterior => {
            matches!(role, PropellerSpinner | PropellerBlades | PropellerDisc)
        }
        SlotCameraRole::LeftCabinWindow => {
            !matches!(role, CabinExterior | CabinInterior | CabinGlass)
        }
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
    } else if path.contains("RudderTrimTab") {
        DMat4::from_rotation_z(-(art.rudder_rad - ty))
    } else {
        DMat4::IDENTITY
    }
}
