//! 3D volumetric terrain patch extraction (`WS-E2`): bridges `maplibre` stitched
//! `129 × 129` [`TerrainPatchSnapshot`]s into `Indicate` [`TerrainPatchAux`] wire
//! frames and full-resolution metric tabletop meshes for `visionOS` `RealityKit`
//! `LowLevelMesh` and WebXR.

use indicate_alerts::AlertOutput;
use indicate_instrument_descriptor::{AuxChannelId, AuxEntry};
use indicate_instrument_scene::volume::Extent3D;
use indicate_instrument_state::PanelData;
use indicate_instrument_volumes::{TERRAIN_PATCH_HEADER_BYTES, TerrainPatchAux};
use indicate_interaction::ViewPose3D;
use maplibre::headless::TerrainPatchSnapshot;
use sim_indicate::{VolumeInstrument, VolumeRender, emit_volume_scene};

use crate::MapAdapterError;

/// Full-resolution metric 3D terrain surface extracted from a [`TerrainPatchSnapshot`]
/// for direct upload into `visionOS` `RealityKit` `LowLevelMesh` or WebXR `wgpu` buffers.
#[derive(Clone, Debug)]
pub struct VolumetricMeshPatch {
    /// Vertices per edge (`129` for `maplibre`'s `TERRAIN_MESH_SIZE + 1`).
    pub grid_side: u32,
    /// Bounding volume dimensions in local tabletop metres.
    pub extent_m: Extent3D,
    /// World metres represented by `1.0 m` in the local volume (`> 0.0`).
    pub meters_per_unit: f64,
    /// Row-major `[x, y, z]` positions in local volume metres (`x` East, `y` North, `z` Up).
    pub positions_local_m: Vec<[f32; 3]>,
    /// Row-major unit surface normals `[nx, ny, nz]` in local volume coordinates.
    pub normals_local: Vec<[f32; 3]>,
    /// Row-major `[u, v]` drape texture coordinates in `[0.0, 1.0]`.
    pub uvs: Vec<[f32; 2]>,
    /// Triangle index list (`(grid_side - 1)^2 * 6` indices).
    pub indices: Vec<u32>,
    /// Optional zero-copy GPU drape texture when extracted from a `Draped` world.
    pub drape_texture: Option<wgpu::Texture>,
}

/// Resamples a [`TerrainPatchSnapshot`] into `Indicate`'s fail-closed
/// `AuxChannelId::TERRAIN_PATCH_3D` binary wire payload (`cols × rows` in `2..=16`).
pub fn encode_terrain_patch_aux(
    snapshot: &TerrainPatchSnapshot,
    grid_side: u8,
    meters_per_unit: f64,
    reference_msl_m: f32,
) -> Result<Vec<u8>, MapAdapterError> {
    if !(2..=16).contains(&grid_side)
        || !meters_per_unit.is_finite()
        || meters_per_unit <= 0.0
        || !reference_msl_m.is_finite()
        || snapshot.grid_side < 2
    {
        return Err(MapAdapterError::InvalidPose {
            detail: "terrain patch aux grid_side (2..=16) or scale is out of range",
        });
    }
    let count = usize::from(grid_side).saturating_mul(usize::from(grid_side));
    let mut out = Vec::with_capacity(TERRAIN_PATCH_HEADER_BYTES.saturating_add(count * 4));
    out.push(grid_side);
    out.push(grid_side);
    out.extend_from_slice(&meters_per_unit.to_le_bytes());

    let exag = snapshot.exaggeration.abs().max(1e-6);
    let src_max = (snapshot.grid_side - 1) as f32;
    let dst_max = f32::from(grid_side - 1);
    let inv_mpu = (1.0 / meters_per_unit) as f32;

    for r in 0..grid_side {
        let v = f32::from(r) / dst_max;
        for c in 0..grid_side {
            let u = f32::from(c) / dst_max;
            let true_msl = sample_bilinear_msl(snapshot, u * src_max, v * src_max) / exag;
            let local_z = (true_msl - reference_msl_m) * inv_mpu;
            out.extend_from_slice(&local_z.to_le_bytes());
        }
    }
    // Fail-closed self-check through Indicate's own decoder.
    let _ = TerrainPatchAux::decode(&out).map_err(|_| MapAdapterError::InvalidPose {
        detail: "encoded TerrainPatchAux failed Indicate wire validation",
    })?;
    Ok(out)
}

/// Extracts a full-resolution (`129 × 129`) metric [`VolumetricMeshPatch`] bounded inside
/// `extent_m` for `visionOS` `RealityKit` `LowLevelMesh` or WebXR tabletop rendering.
pub fn extract_volumetric_mesh_patch(
    snapshot: &TerrainPatchSnapshot,
    extent_m: Extent3D,
    tile_span_world_m: f64,
    reference_msl_m: f32,
) -> Result<VolumetricMeshPatch, MapAdapterError> {
    let side = snapshot.grid_side;
    if side < 2
        || !tile_span_world_m.is_finite()
        || tile_span_world_m <= 0.0
        || extent_m.width_m <= 0.0
        || extent_m.depth_m <= 0.0
        || extent_m.height_m <= 0.0
    {
        return Err(MapAdapterError::InvalidPose {
            detail: "invalid snapshot or volumetric extent dimensions",
        });
    }
    let horizontal_extent = f64::from(extent_m.width_m.min(extent_m.depth_m));
    let meters_per_unit = tile_span_world_m / horizontal_extent;
    let inv_mpu = (1.0 / meters_per_unit) as f32;
    let exag = snapshot.exaggeration.abs().max(1e-6);
    let half_w = extent_m.width_m * 0.5;
    let half_d = extent_m.depth_m * 0.5;
    let half_h = extent_m.height_m * 0.5;
    let denom = (side - 1) as f32;

    let vtx_count = (side as usize).saturating_mul(side as usize);
    let mut positions = Vec::with_capacity(vtx_count);
    let mut uvs = Vec::with_capacity(vtx_count);

    for row in 0..side {
        let v = (row as f32) / denom;
        let y_local = (0.5 - v) * extent_m.depth_m;
        for col in 0..side {
            let u = (col as f32) / denom;
            let x_local = (u - 0.5) * extent_m.width_m;
            let msl = snapshot.height_at_grid(col, row).unwrap_or(0.0) / exag;
            let z_local = ((msl - reference_msl_m) * inv_mpu).clamp(-half_h, half_h);
            positions.push([
                x_local.clamp(-half_w, half_w),
                y_local.clamp(-half_d, half_d),
                z_local,
            ]);
            uvs.push([u, v]);
        }
    }
    let normals = compute_grid_normals(side, &positions);
    let indices = build_grid_indices(side);

    Ok(VolumetricMeshPatch {
        grid_side: side,
        extent_m,
        meters_per_unit,
        positions_local_m: positions,
        normals_local: normals,
        uvs,
        indices,
        drape_texture: snapshot.drape_texture.clone(),
    })
}

/// Parameters for emitting `Indicate`'s `terrain-volume` 3D scene from a resident DEM patch.
#[derive(Clone, Copy, Debug)]
pub struct TerrainVolumeRequest<'a> {
    /// Resident stitched DEM patch from a conformal instrument map.
    pub snapshot: &'a TerrainPatchSnapshot,
    /// Downsampled grid side (`2..=32`, e.g. `8`) encoded into `AuxChannelId::TERRAIN_PATCH_3D`.
    pub grid_side: u8,
    /// Metres of physical world terrain per 1.0 m of volumetric bounding box.
    pub meters_per_unit: f64,
    /// Reference altitude in metres MSL mapped to volumetric `Z = 0`.
    pub reference_msl_m: f32,
    /// Avionics belief `PanelData` (`ROLE_OPERATIONAL_ESTIMATE`).
    pub data: &'a PanelData,
    /// Local spatial manipulation pose (`ViewPose3D`).
    pub view: ViewPose3D,
    /// Optional alert output.
    pub alerts: Option<&'a AlertOutput>,
    /// Optional volume bounding box override (`CANONICAL_EXTENT` when `None`).
    pub extent_m: Option<Extent3D>,
}

/// Emits and validates `Indicate`'s `terrain-volume` 3D scene populated with a live
/// [`TerrainPatchSnapshot`] from a conformal instrument map.
pub fn emit_terrain_volume_with_patch(
    request: &TerrainVolumeRequest<'_>,
    out_scene: &mut [u8],
) -> Result<VolumeRender, MapAdapterError> {
    let aux_bytes = encode_terrain_patch_aux(
        request.snapshot,
        request.grid_side,
        request.meters_per_unit,
        request.reference_msl_m,
    )?;
    let entries = [AuxEntry {
        id: AuxChannelId::TERRAIN_PATCH_3D,
        payload: &aux_bytes,
    }];
    emit_volume_scene(
        VolumeInstrument::Terrain,
        request.data,
        request.view,
        request.alerts,
        &entries,
        request.extent_m,
        out_scene,
    )
    .map_err(|source| MapAdapterError::Indicate { source })
}

fn sample_bilinear_msl(snapshot: &TerrainPatchSnapshot, gx: f32, gy: f32) -> f32 {
    let max_idx = snapshot.grid_side.saturating_sub(1);
    let x0 = (gx.floor().max(0.0) as u32).min(max_idx);
    let y0 = (gy.floor().max(0.0) as u32).min(max_idx);
    let x1 = x0.saturating_add(1).min(max_idx);
    let y1 = y0.saturating_add(1).min(max_idx);
    let tx = (gx - (x0 as f32)).clamp(0.0, 1.0);
    let ty = (gy - (y0 as f32)).clamp(0.0, 1.0);

    let h00 = snapshot.height_at_grid(x0, y0).unwrap_or(0.0);
    let h10 = snapshot.height_at_grid(x1, y0).unwrap_or(0.0);
    let h01 = snapshot.height_at_grid(x0, y1).unwrap_or(0.0);
    let h11 = snapshot.height_at_grid(x1, y1).unwrap_or(0.0);
    let top = h00 + (h10 - h00) * tx;
    let bot = h01 + (h11 - h01) * tx;
    top + (bot - top) * ty
}

fn compute_grid_normals(side: u32, positions: &[[f32; 3]]) -> Vec<[f32; 3]> {
    let side_usize = side as usize;
    let max_i = side.saturating_sub(1);
    let mut normals = Vec::with_capacity(positions.len());
    for row in 0..side {
        for col in 0..side {
            let l = col.saturating_sub(1);
            let r = col.saturating_add(1).min(max_i);
            let u = row.saturating_sub(1);
            let d = row.saturating_add(1).min(max_i);
            let pl = positions
                .get((row as usize) * side_usize + (l as usize))
                .copied()
                .unwrap_or([0.0; 3]);
            let pr = positions
                .get((row as usize) * side_usize + (r as usize))
                .copied()
                .unwrap_or([0.0; 3]);
            let pu = positions
                .get((u as usize) * side_usize + (col as usize))
                .copied()
                .unwrap_or([0.0; 3]);
            let pd = positions
                .get((d as usize) * side_usize + (col as usize))
                .copied()
                .unwrap_or([0.0; 3]);
            let [lx, ly, lz] = pl;
            let [rx, ry, rz] = pr;
            let [ux, uy, uz] = pu;
            let [dx, dy, dz] = pd;
            let tx = glam::Vec3::new(rx - lx, ry - ly, rz - lz);
            let ty = glam::Vec3::new(ux - dx, uy - dy, uz - dz);
            let n = tx.cross(ty).try_normalize().unwrap_or(glam::Vec3::Z);
            normals.push([n.x, n.y, n.z]);
        }
    }
    normals
}

fn build_grid_indices(side: u32) -> Vec<u32> {
    let quads = side.saturating_sub(1);
    let cap = (quads as usize)
        .saturating_mul(quads as usize)
        .saturating_mul(6);
    let mut out = Vec::with_capacity(cap);
    for row in 0..quads {
        for col in 0..quads {
            let tl = row * side + col;
            let tr = tl + 1;
            let bl = tl + side;
            let br = bl + 1;
            out.extend_from_slice(&[tl, bl, tr, tr, bl, br]);
        }
    }
    out
}
