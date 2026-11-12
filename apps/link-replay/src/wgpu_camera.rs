//! Headless `wgpu` offscreen terrain camera renderer over `CatalogScenery`,
//! with deterministic software fallback (`CameraWorld`).

use std::num::NonZeroU32;
use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_chain_trial::FrameRequest;
use sim_geo::{
    BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics, ecef_to_geodetic, egm96_undulation_m,
};
use sim_link_host::CameraWorld;
use sim_render::{
    ApertureFrame, ApertureWaveband, CameraPose, EnvironmentFrame, GlobeMesh, HeadlessGpu,
    OcclusionOverlayConfig, OffscreenAperture, OrthoAtlas, OrthoFootprint, PinholeProjection,
    RenderError, SensorAperturePass, Sunlight, TerrainRegion, TerrainSources, TwilightRamp,
};
use sim_scenery::{
    CatalogScenery, ElevationSource, ImageryRaster, ImagerySource, SceneryError, TileId,
    project_to_tile, tile_point_to_geodetic,
};
use tracing::{info, warn};

use crate::ReplayError;
use crate::cli::CameraBackend;

const TILE_PX: u32 = 256;
const MAX_ATLAS_DIM_PX: u32 = 8_192;
const NEAR_PLANE_M: f64 = 2.0;

/// Offscreen terrain camera renderer supporting headless `wgpu` GPU rendering
/// and CPU software raycasting.
pub enum TerrainCameraRenderer {
    /// Hardware/software headless `wgpu` offscreen terrain aperture.
    Wgpu(Box<WgpuTerrainCamera>),
    /// Deterministic `CameraWorld` CPU raycaster.
    Cpu(CameraWorld),
}

impl TerrainCameraRenderer {
    /// Opens the requested camera backend over `world`, falling back cleanly to
    /// `CameraBackend::Cpu` if no headless `wgpu` adapter is available.
    #[must_use]
    pub fn open(world: CameraWorld, requested: CameraBackend) -> Self {
        if requested == CameraBackend::Cpu {
            return Self::Cpu(world);
        }
        match WgpuTerrainCamera::new(world.scenery()) {
            Ok(gpu_cam) => {
                info!("initialized headless wgpu offscreen terrain camera");
                Self::Wgpu(Box::new(gpu_cam))
            }
            Err(err) => {
                warn!(error = %err, "wgpu headless terrain camera unavailable; falling back to CPU raycaster");
                Self::Cpu(world)
            }
        }
    }

    /// Active backend name (`"wgpu"` or `"cpu"`).
    #[must_use]
    pub const fn backend_name(&self) -> &'static str {
        match self {
            Self::Wgpu(_) => "wgpu",
            Self::Cpu(_) => "cpu",
        }
    }

    /// Renders `request` into row-major top-down `u8` scene luma bytes.
    ///
    /// # Errors
    ///
    /// Returns [`ReplayError`] if GPU readback or CPU raycasting fails.
    pub fn render_frame(&mut self, request: &FrameRequest) -> Result<Vec<u8>, ReplayError> {
        let [w, x, y, z] = request.camera_to_ecef_wxyz;
        let [fx, fy, cx, cy] = request.pinhole;
        let eye_ecef = DVec3::from_array(request.eye_ecef);
        let orientation = DQuat::from_xyzw(x, y, z, w);
        let intrinsics = SensorIntrinsics {
            width: request.size.0,
            height: request.size.1,
            fx,
            fy,
            cx,
            cy,
        };
        match self {
            Self::Wgpu(gpu) => gpu.render(eye_ecef, orientation, intrinsics),
            Self::Cpu(world) => {
                let luma = world.render(eye_ecef, orientation, &intrinsics)?;
                Ok(luma.pixels().to_vec())
            }
        }
    }
}

/// Headless `wgpu` offscreen terrain camera backed by [`SensorAperturePass`].
pub struct WgpuTerrainCamera {
    gpu: HeadlessGpu,
    pass: SensorAperturePass,
    cached_target: Option<([u64; 6], OffscreenAperture)>,
}

impl WgpuTerrainCamera {
    fn new(scenery: Arc<CatalogScenery>) -> Result<Self, RenderError> {
        pollster::block_on(async {
            let gpu = HeadlessGpu::new().await?;
            let max_dim = gpu
                .device
                .limits()
                .max_texture_dimension_2d
                .min(MAX_ATLAS_DIM_PX);
            let footprint = choose_ortho_footprint(&scenery, max_dim);
            let atlas = match footprint {
                Some(fp) => Some(OrthoAtlas::from_source(
                    &AdaptiveSceneryImagery(&scenery),
                    fp,
                    TILE_PX,
                )?),
                None => None,
            };
            let region = corridor_region(&scenery)?;
            let padded_dem = EllipsoidalPaddedElevation(&scenery);
            let mesh = GlobeMesh::build_region(
                &region,
                TerrainSources {
                    elevation: Some(&padded_dem),
                    radiometry: None,
                    ortho: footprint.as_ref(),
                },
            )?;
            let pass =
                SensorAperturePass::new(&gpu.device, &gpu.queue, &mesh, atlas.as_ref()).await?;
            Ok(Self {
                gpu,
                pass,
                cached_target: None,
            })
        })
    }

    fn render(
        &mut self,
        eye_ecef: DVec3,
        orientation: DQuat,
        intrinsics: SensorIntrinsics,
    ) -> Result<Vec<u8>, ReplayError> {
        let key = [
            u64::from(intrinsics.width),
            u64::from(intrinsics.height),
            intrinsics.fx.to_bits(),
            intrinsics.fy.to_bits(),
            intrinsics.cx.to_bits(),
            intrinsics.cy.to_bits(),
        ];
        if self.cached_target.as_ref().is_none_or(|(k, _)| *k != key) {
            let proj = PinholeProjection::new(intrinsics, NEAR_PLANE_M)?;
            let target = OffscreenAperture::new(&self.gpu.device, proj)?;
            self.cached_target = Some((key, target));
        }
        let Some((_, target)) = self.cached_target.as_ref() else {
            return Err(RenderError::InvalidCamera {
                detail: "missing aperture target".to_owned(),
            }
            .into());
        };
        // Overhead sun direction yields unit shading (`0.35 + 0.65 * 1.0 = 1.0`)
        // in `globe.wgsl` because `ChainTrial`'s `TerrainExposer` applies true
        // solar ephemeris irradiance and sensor noise in `deliver_frame`.
        let zenith = LocalFrame::at(&ecef_to_geodetic(eye_ecef)).up();
        let environment = EnvironmentFrame::clear(
            BodyEllipsoid::WGS84,
            Sunlight::uneclipsed(zenith, None),
            TwilightRamp {
                night_end_deg: -6.0,
                day_start_deg: 6.0,
                night_brightness: 1.0,
            },
        );
        let frame = ApertureFrame {
            pose: CameraPose {
                position_ecef: eye_ecef,
                orientation,
            },
            environment,
            waveband: ApertureWaveband::VisibleRgb,
            occlusion: OcclusionOverlayConfig::default(),
        };
        self.pass
            .render(&self.gpu.device, &self.gpu.queue, target, &frame);
        let image = target.read_blocking(&self.gpu.device, &self.gpu.queue)?;
        Ok(rgba8_to_luma(&image.rgba8))
    }
}

fn rgba8_to_luma(rgba8: &[u8]) -> Vec<u8> {
    let (pixels, _) = rgba8.as_chunks::<4>();
    pixels
        .iter()
        .map(|&[r, g, b, _]| {
            (0.299 * f64::from(r) + 0.587 * f64::from(g) + 0.114 * f64::from(b))
                .round()
                .clamp(0.0, 255.0) as u8
        })
        .collect()
}

fn corridor_region(scenery: &CatalogScenery) -> Result<TerrainRegion, RenderError> {
    let mut south = f64::INFINITY;
    let mut north = f64::NEG_INFINITY;
    let mut west = f64::INFINITY;
    let mut east = f64::NEG_INFINITY;
    for tile in scenery.imagery_tiles().chain(scenery.elevation_tiles()) {
        if let (Some(nw), Some(se)) = (
            tile_point_to_geodetic(tile, 0.0, 0.0),
            tile_point_to_geodetic(tile, 1.0, 1.0),
        ) {
            south = south.min(se.lat_rad);
            north = north.max(nw.lat_rad);
            west = west.min(nw.lon_rad);
            east = east.max(se.lon_rad);
        }
    }
    if !south.is_finite()
        || !north.is_finite()
        || !west.is_finite()
        || !east.is_finite()
        || south >= north
        || west >= east
    {
        return Err(RenderError::EmptyRegion {
            detail: "catalog scenery contains no tiles".to_owned(),
        });
    }
    let pad = 0.004;
    Ok(TerrainRegion {
        body: BodyEllipsoid::WGS84,
        south_rad: south - pad,
        west_rad: west - pad,
        lat_span_rad: (north - south) + 2.0 * pad,
        lon_span_rad: (east - west) + 2.0 * pad,
        lat_patches: 12,
        lon_patches: 12,
        grid: 24,
    })
}

fn choose_ortho_footprint(scenery: &CatalogScenery, max_dim_px: u32) -> Option<OrthoFootprint> {
    let max_tiles = (max_dim_px / TILE_PX).max(1);
    let max_zoom = scenery.imagery_tiles().map(|t| t.zoom).max()?;
    let mut south = f64::INFINITY;
    let mut north = f64::NEG_INFINITY;
    let mut west = f64::INFINITY;
    let mut east = f64::NEG_INFINITY;
    for tile in scenery.imagery_tiles() {
        if let (Some(nw), Some(se)) = (
            tile_point_to_geodetic(tile, 0.0, 0.0),
            tile_point_to_geodetic(tile, 1.0, 1.0),
        ) {
            south = south.min(se.lat_rad);
            north = north.max(nw.lat_rad);
            west = west.min(nw.lon_rad);
            east = east.max(se.lon_rad);
        }
    }
    let nw_geo = Geodetic {
        lat_rad: north,
        lon_rad: west,
        height_m: 0.0,
    };
    let se_geo = Geodetic {
        lat_rad: south,
        lon_rad: east,
        height_m: 0.0,
    };
    for zoom in (6..=max_zoom).rev() {
        let nw_c = project_to_tile(&nw_geo, zoom)?;
        let se_c = project_to_tile(&se_geo, zoom)?;
        let wide = se_c.tile.x.saturating_sub(nw_c.tile.x).saturating_add(1);
        let high = se_c.tile.y.saturating_sub(nw_c.tile.y).saturating_add(1);
        if wide <= max_tiles && high <= max_tiles {
            return Some(OrthoFootprint {
                zoom,
                west: nw_c.tile.x,
                north: nw_c.tile.y,
                tiles_wide: NonZeroU32::new(wide)?,
                tiles_high: NonZeroU32::new(high)?,
            });
        }
    }
    None
}

struct EllipsoidalPaddedElevation<'a>(&'a CatalogScenery);

impl ElevationSource for EllipsoidalPaddedElevation<'_> {
    fn ground_height_m(&self, position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        if let Some(ellip) = self.0.ellipsoidal_ground_height_m(position)? {
            return Ok(Some(ellip.0));
        }
        Ok(Some(egm96_undulation_m(position.lat_rad, position.lon_rad)))
    }
}

struct AdaptiveSceneryImagery<'a>(&'a CatalogScenery);

impl ImagerySource for AdaptiveSceneryImagery<'_> {
    fn imagery(&self, tile: TileId) -> Result<Option<ImageryRaster>, SceneryError> {
        if let Some(exact) = self.0.imagery_tile(tile) {
            return Ok(Some(exact.clone()));
        }
        let total = (TILE_PX as usize) * (TILE_PX as usize) * 4;
        let mut rgba8 = vec![0_u8; total];
        let mut any_covered = false;
        let mut offset = 0_usize;
        let inv = 1.0 / f64::from(TILE_PX);
        for py in 0..TILE_PX {
            let v = (f64::from(py) + 0.5) * inv;
            for px in 0..TILE_PX {
                let u = (f64::from(px) + 0.5) * inv;
                if let Some(geo) = tile_point_to_geodetic(tile, u, v)
                    && let Some(luma) = self.0.imagery_luma_at(&geo)
                {
                    let byte = luma.round().clamp(0.0, 255.0) as u8;
                    if let Some(dst) = rgba8.get_mut(offset..offset.saturating_add(4)) {
                        dst.copy_from_slice(&[byte, byte, byte, 255]);
                    }
                    any_covered = true;
                }
                offset = offset.saturating_add(4);
            }
        }
        if !any_covered {
            return Ok(None);
        }
        ImageryRaster::new(TILE_PX, TILE_PX, rgba8).map(Some)
    }
}
