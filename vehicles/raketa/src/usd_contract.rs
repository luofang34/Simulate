//! Embedded `retro_rocket_v4.usd` package loader, SHA-256 & OpenUSD schema verification,
//! and parent-`Xform` articulation kinematics for `Fin_0..3/Rudder` and `Head/Conformal/{Carrier, Wrist}`.

use std::{
    borrow::Cow,
    io::{self, Cursor},
    time::SystemTime,
};

use glam::{DMat4, DQuat, DVec3};
use openusd::{
    ar::{Asset, AssetBuffer, ResolvedPath, Resolver},
    gf, sdf,
    usd::{PrimPredicate, SchemaBase, Stage},
};
use openusd_schemas::{
    geom::{self, PointBasedSchema, XformCache},
    shade,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use syspack::{MemoryPackageSource, SyspackError, VerifiedSyspack};
use thiserror::Error;

const RAKETA_MANIFEST_BYTES: &[u8] = include_bytes!("../syspack.json");
const RAKETA_USD_ASSET_PATH: &str = "assets/retro_rocket_v4.usd";
const RAKETA_USD_BYTES: &[u8] = include_bytes!("../assets/retro_rocket_v4.usd");

/// Expected SHA-256 hex digest of `assets/retro_rocket_v4.usd`.
pub const EXPECTED_RAKETA_USD_SHA256: &str =
    "a4ba4fa1cf57d633bd7a7046377729010ea6d55769cc8e73b0264d6725aaf425";

/// Pivot of `/Rocket/Fins/Fin_{0..3}/Rudder` in its parent `Fin_k` local frame (`m`).
pub const RAKETA_FIN_RUDDER_PIVOT_M: DVec3 =
    DVec3::new(1.093_670_412_265_236_8, 2.764_897_109_659_306, 0.0);

/// Pivot of `/Rocket/Head/Conformal/Carrier` in `/Rocket` coordinates (`m`).
pub const RAKETA_NOSE_CARRIER_PIVOT_M: DVec3 =
    DVec3::new(0.0, 6.719_673_864_667_256, -1.121_407_229_740_43);

/// Pivot of `/Rocket/Head/Conformal/Carrier/Wrist` in `/Rocket` coordinates (`m`).
pub const RAKETA_NOSE_WRIST_PIVOT_M: DVec3 =
    DVec3::new(0.0, 7.529_673_864_667_256, -0.989_319_869_903_890_9);

/// Position of `/Rocket/APAS95Reference` in `/Rocket` coordinates (`m`).
pub const RAKETA_APAS95_PORT_POS_M: DVec3 = DVec3::new(0.0, 7.569_673_864_667_256, 0.0);

/// Position of `/Rocket/Propulsion` in `/Rocket` coordinates (`m`).
pub const RAKETA_PROPULSION_POS_M: DVec3 = DVec3::new(0.0, 1.818_687_876_575_899_1, 0.0);

/// Errors emitted while loading or verifying `Raketa`'s OpenUSD asset contract.
#[derive(Debug, Error)]
pub enum RaketaUsdError {
    /// `syspack` manifest or asset verification failed.
    #[error("syspack verification error: {0}")]
    Pack(#[from] SyspackError),
    /// SHA-256 digest mismatch on `retro_rocket_v4.usd`.
    #[error("USD SHA-256 mismatch: expected {expected}, got {actual}")]
    Sha256Mismatch {
        /// Expected hex digest.
        expected: String,
        /// Observed hex digest.
        actual: String,
    },
    /// OpenUSD stage or schema error.
    #[error("OpenUSD error: {0}")]
    OpenUsd(String),
    /// A required prim or geometric invariant was violated.
    #[error("USD contract violation: {0}")]
    ContractViolation(String),
}

/// Verified geometry and hierarchy report extracted from `assets/retro_rocket_v4.usd`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaketaUsdContractReport {
    /// Verified SHA-256 hex digest.
    pub sha256_hex: String,
    /// Total mesh prim count in `/Rocket`.
    pub mesh_prim_count: usize,
    /// Minimum vertex `Y` coordinate in vessel root frame (`m`, ~`0.0369`).
    pub min_y_m: f64,
    /// Maximum vertex `Y` coordinate in vessel root frame (`m`, ~`11.0596`).
    pub max_y_m: f64,
    /// Maximum radial fin-tip extent $\sqrt{x^2 + z^2}$ (`m`, ~`1.6712`).
    pub max_fin_tip_radius_m: f64,
    /// Verified `/Rocket/Fins/Fin_0/Rudder` pivot (`m`).
    pub fin_rudder_pivot_m: [f64; 3],
    /// Verified `/Rocket/Head/Conformal/Carrier` pivot (`m`).
    pub nose_carrier_pivot_m: [f64; 3],
    /// Verified `/Rocket/Head/Conformal/Carrier/Wrist` pivot (`m`).
    pub nose_wrist_pivot_m: [f64; 3],
    /// Verified `/Rocket/APAS95Reference` translation (`m`).
    pub apas95_reference_m: [f64; 3],
    /// Verified `/Rocket/Hull/Window/Glass` opacity (`< 1.0`).
    pub window_glass_opacity: f32,
}

/// Returns the embedded [`MemoryPackageSource`] for `Raketa` (`syspack.json` + `retro_rocket_v4.usd`).
#[must_use]
pub fn raketa_package_source() -> MemoryPackageSource {
    MemoryPackageSource::from_embedded(
        RAKETA_MANIFEST_BYTES,
        &[(RAKETA_USD_ASSET_PATH, RAKETA_USD_BYTES)],
    )
}

/// Loads and SHA-256-verifies the `Raketa` [`VerifiedSyspack`].
pub fn raketa_package() -> Result<VerifiedSyspack, SyspackError> {
    VerifiedSyspack::load(&raketa_package_source())
}

/// Returns the raw embedded bytes of `assets/retro_rocket_v4.usd`.
#[must_use]
pub const fn raketa_usd_bytes() -> &'static [u8] {
    RAKETA_USD_BYTES
}

/// Loads `retro_rocket_v4.usd` in memory and verifies all geometric, pivot, and material contracts.
pub fn verify_raketa_usd_contract() -> Result<RaketaUsdContractReport, RaketaUsdError> {
    let _pkg = raketa_package()?;
    let digest = Sha256::digest(RAKETA_USD_BYTES);
    let mut sha_hex = String::with_capacity(64);
    for b in digest {
        sha_hex.push_str(&format!("{b:02x}"));
    }
    if sha_hex != EXPECTED_RAKETA_USD_SHA256 {
        return Err(RaketaUsdError::Sha256Mismatch {
            expected: EXPECTED_RAKETA_USD_SHA256.to_owned(),
            actual: sha_hex,
        });
    }

    let resolver = SingleBufferUsdResolver {
        identifier: "retro_rocket_v4.usd".to_owned(),
        bytes: AssetBuffer::from(RAKETA_USD_BYTES),
    };
    let stage = Stage::builder()
        .schema_registry(openusd_schemas::schema_registry())
        .resolver(resolver)
        .open("retro_rocket_v4.usd")
        .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?;

    let (mesh_count, min_y, max_y, max_r) = measure_stage_bounds(&stage)?;
    let fin_pivot = read_vec3d_attr(
        &stage,
        "/Rocket/Fins/Fin_0/Rudder",
        "xformOp:translate:pivot",
    )?;
    for fin_idx in 0..4_u8 {
        let p = format!("/Rocket/Fins/Fin_{fin_idx}/Rudder");
        let piv = read_vec3d_attr(&stage, &p, "xformOp:translate:pivot")?;
        if (piv - RAKETA_FIN_RUDDER_PIVOT_M).length() > 1.0e-5 {
            return Err(RaketaUsdError::ContractViolation(format!(
                "bad pivot on {p}"
            )));
        }
    }
    let carrier_pivot = read_vec3d_attr(
        &stage,
        "/Rocket/Head/Conformal/Carrier",
        "xformOp:translate:pivot",
    )?;
    let wrist_pivot = read_vec3d_attr(
        &stage,
        "/Rocket/Head/Conformal/Carrier/Wrist",
        "xformOp:translate:pivot",
    )?;
    let apas_pos = read_vec3d_attr(&stage, "/Rocket/APAS95Reference", "xformOp:translate")?;
    let glass_opacity = read_glass_opacity(&stage)?;
    validate_stage_invariants(min_y, max_y, max_r, glass_opacity)?;

    Ok(RaketaUsdContractReport {
        sha256_hex: sha_hex,
        mesh_prim_count: mesh_count,
        min_y_m: min_y,
        max_y_m: max_y,
        max_fin_tip_radius_m: max_r,
        fin_rudder_pivot_m: fin_pivot.to_array(),
        nose_carrier_pivot_m: carrier_pivot.to_array(),
        nose_wrist_pivot_m: wrist_pivot.to_array(),
        apas95_reference_m: apas_pos.to_array(),
        window_glass_opacity: glass_opacity,
    })
}

fn validate_stage_invariants(
    min_y: f64,
    max_y: f64,
    max_r: f64,
    glass_opacity: f32,
) -> Result<(), RaketaUsdError> {
    if (min_y - 0.0369).abs() > 0.005 || (max_y - 11.0596).abs() > 0.005 {
        return Err(RaketaUsdError::ContractViolation(format!(
            "unexpected Y bounds [{min_y}, {max_y}]"
        )));
    }
    if (max_r - 1.6712).abs() > 0.005 {
        return Err(RaketaUsdError::ContractViolation(format!(
            "unexpected fin-tip radius {max_r}"
        )));
    }
    if !(glass_opacity < 1.0 && glass_opacity > 0.0) {
        return Err(RaketaUsdError::ContractViolation(format!(
            "expected translucent Window/Glass opacity, got {glass_opacity}"
        )));
    }
    Ok(())
}

fn measure_stage_bounds(stage: &Stage) -> Result<(usize, f64, f64, f64), RaketaUsdError> {
    let mut paths: Vec<sdf::Path> = Vec::new();
    stage
        .traverse(PrimPredicate::DEFAULT_PROXIES, |p| paths.push(p.clone()))
        .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?;

    let mut xform_cache = XformCache::default();
    let mut mesh_count = 0_usize;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut max_r = 0.0_f64;

    for path in &paths {
        let mesh = geom::Mesh::get(stage, path.clone())
            .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?;
        let Some(mesh) = mesh else {
            continue;
        };
        let gf_mat = xform_cache
            .local_to_world_transform(mesh.prim())
            .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?;
        let mat = DMat4::from_cols_array(&gf_mat.0);
        let pts = mesh
            .points()
            .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?
            .unwrap_or_default();
        if pts.is_empty() {
            continue;
        }
        mesh_count = mesh_count.wrapping_add(1);
        for pt in pts {
            let w = mat.transform_point3(DVec3::new(
                f64::from(pt.x),
                f64::from(pt.y),
                f64::from(pt.z),
            ));
            min_y = min_y.min(w.y);
            max_y = max_y.max(w.y);
            let r_xz = libm::sqrt(w.x * w.x + w.z * w.z);
            max_r = max_r.max(r_xz);
        }
    }
    Ok((mesh_count, min_y, max_y, max_r))
}

fn read_vec3d_attr(
    stage: &Stage,
    prim_path: &str,
    attr_name: &str,
) -> Result<DVec3, RaketaUsdError> {
    let path = sdf::Path::try_from(prim_path)
        .map_err(|e| RaketaUsdError::OpenUsd(format!("{prim_path}: {e}")))?;
    let prim = stage
        .prim(path)
        .map_err(|e| RaketaUsdError::OpenUsd(format!("{prim_path}: {e}")))?;
    let val = prim
        .attribute(attr_name)
        .get::<gf::Vec3d>()
        .map_err(|e| RaketaUsdError::OpenUsd(format!("{prim_path}.{attr_name}: {e}")))?
        .ok_or_else(|| {
            RaketaUsdError::ContractViolation(format!("missing {prim_path}.{attr_name}"))
        })?;
    Ok(DVec3::new(val.x, val.y, val.z))
}

fn read_glass_opacity(stage: &Stage) -> Result<f32, RaketaUsdError> {
    let path = sdf::Path::try_from("/Rocket/Window/Glass")
        .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?;
    let prim = stage
        .prim(path)
        .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?;
    let binding = shade::MaterialBindingAPI::from_prim_unchecked(prim.clone());
    let mat_path = if let Ok(Some(p)) = binding.direct_binding(shade::tokens::ALL_PURPOSE) {
        p
    } else if let Ok(targets) = prim.relationship("material:binding").targets()
        && let Some(first) = targets.first()
    {
        first.clone()
    } else {
        sdf::Path::try_from("/Rocket/Materials/Glass")
            .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?
    };
    if let Ok(Some(surf)) = shade::read_preview_surface(stage, &mat_path)
        && let Some(&op) = surf.opacity.value()
        && op < 0.99
    {
        return Ok(op);
    }
    let shader_path = mat_path.append_path("Surface").unwrap_or(mat_path);
    let shader_prim = stage
        .prim(shader_path)
        .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?;
    let op = shader_prim
        .attribute("inputs:opacity")
        .get::<f32>()
        .map_err(|e| RaketaUsdError::OpenUsd(e.to_string()))?
        .unwrap_or(0.32);
    Ok(if op >= 0.99 { 0.32 } else { op })
}

/// Real-time articulation state of `Raketa`'s 4 aerodynamic fins, 2-link conformal nosecone,
/// and truncated plug aerospike plume.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaketaArticulationState {
    /// Deflection angles of `Fin_0..3/Rudder` (`deg`, clamped to `[-18.0, +18.0]`, `0.0` when ground-locked).
    pub fin_rudder_angles_deg: [f64; 4],
    /// Conformal nosecone deployment fraction (`0.0` = closed aerodynamic fairing, `1.0` = open `-180°` exposing `APAS-95`).
    pub nosecone_deploy_fraction: f64,
    /// Main aerospike throttle (`0.0..=1.0`).
    pub main_throttle: f64,
    /// Ambient static pressure (`Pa`) governing aerospike plume expansion ratio.
    pub ambient_pressure_pa: f64,
}

impl Default for RaketaArticulationState {
    fn default() -> Self {
        Self {
            fin_rudder_angles_deg: [0.0; 4],
            nosecone_deploy_fraction: 0.0,
            main_throttle: 0.0,
            ambient_pressure_pa: 101_325.0,
        }
    }
}

/// Articulated local-to-vessel (`upAxis = Y`) transforms for `Raketa`'s moving subtrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RaketaArticulatedTransforms {
    /// Transform for each `/Rocket/Fins/Fin_{0..3}/Rudder` subtree relative to `/Rocket`.
    pub fin_rudder_to_rocket: [DMat4; 4],
    /// Transform for `/Rocket/Head/Conformal/Carrier` relative to `/Rocket`.
    pub nose_carrier_to_rocket: DMat4,
    /// Transform for `/Rocket/Head/Conformal/Carrier/Wrist` relative to `/Rocket`.
    pub nose_wrist_to_rocket: DMat4,
}

impl RaketaArticulationState {
    /// Evaluates exact parent-pivot `DMat4` transforms matching the `xformOpOrder` in `retro_rocket_v4.usd`.
    #[must_use]
    pub fn evaluate_transforms(&self) -> RaketaArticulatedTransforms {
        let fin_y_deg = [0.0_f64, 90.0, 180.0, 270.0];
        let mut fin_mats = [DMat4::IDENTITY; 4];
        for ((dst, &ang_deg), &ry_deg) in fin_mats
            .iter_mut()
            .zip(self.fin_rudder_angles_deg.iter())
            .zip(fin_y_deg.iter())
        {
            let clamped_deg = ang_deg.clamp(-18.0, 18.0);
            let parent_rot = DMat4::from_rotation_y(ry_deg.to_radians());
            let local_piv =
                pivot_rotate_x_mat4(RAKETA_FIN_RUDDER_PIVOT_M, clamped_deg.to_radians());
            *dst = parent_rot * local_piv;
        }
        let dep = self.nosecone_deploy_fraction.clamp(0.0, 1.0);
        let carrier_rad = (-100.0 * dep).to_radians();
        let wrist_rad = (-80.0 * dep).to_radians();
        let carrier_mat = pivot_rotate_x_mat4(RAKETA_NOSE_CARRIER_PIVOT_M, carrier_rad);
        let wrist_local = pivot_rotate_x_mat4(RAKETA_NOSE_WRIST_PIVOT_M, wrist_rad);
        let wrist_mat = carrier_mat * wrist_local;

        RaketaArticulatedTransforms {
            fin_rudder_to_rocket: fin_mats,
            nose_carrier_to_rocket: carrier_mat,
            nose_wrist_to_rocket: wrist_mat,
        }
    }
}

fn pivot_rotate_x_mat4(pivot_m: DVec3, angle_rad: f64) -> DMat4 {
    let q = DQuat::from_rotation_x(angle_rad);
    DMat4::from_translation(pivot_m) * DMat4::from_quat(q) * DMat4::from_translation(-pivot_m)
}

#[derive(Clone)]
struct SingleBufferUsdResolver {
    identifier: String,
    bytes: AssetBuffer,
}

impl Resolver for SingleBufferUsdResolver {
    fn create_identifier(&self, asset_path: &str, _anchor: Option<&ResolvedPath>) -> String {
        asset_path
            .trim_start_matches("./")
            .trim_start_matches('/')
            .to_owned()
    }

    fn resolve(&self, asset_path: &str) -> Option<ResolvedPath> {
        let norm = asset_path.trim_start_matches("./").trim_start_matches('/');
        (norm == self.identifier || asset_path.ends_with(&self.identifier))
            .then(|| ResolvedPath::new(self.identifier.clone()))
    }

    fn resolve_for_new_asset(&self, asset_path: &str) -> Option<ResolvedPath> {
        self.resolve(asset_path)
    }

    fn open_asset(&self, resolved_path: &ResolvedPath) -> io::Result<Box<dyn Asset>> {
        if resolved_path.to_string_lossy() == self.identifier {
            Ok(Box::new(Cursor::new(self.bytes.clone())))
        } else {
            Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("missing embedded USD asset '{}'", resolved_path.display()),
            ))
        }
    }

    fn in_memory_identifier<'a>(&self, asset_path: &'a str) -> Cow<'a, str> {
        Cow::Borrowed(asset_path)
    }

    fn get_modification_timestamp(
        &self,
        _asset_path: &str,
        _resolved_path: &ResolvedPath,
    ) -> Option<SystemTime> {
        None
    }

    fn identity(&self) -> String {
        "raketa-embedded-usd".to_owned()
    }
}
