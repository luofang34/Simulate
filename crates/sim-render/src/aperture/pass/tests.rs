use std::io::Write;

use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics};
use sim_scenery::{
    CatalogScenery, ElevationSource, ImageryRaster, LunarTranquilityScenery, RadiometricSource,
    SceneryError, SurfaceRadiometry, TileId, WEST_CRATER, tile_point_to_geodetic,
};
use std::num::NonZeroU32;

mod optics;

use super::*;
use crate::aperture::{OcclusionOverlayConfig, PropellerDisk, WindowBezel};
use crate::{
    ApertureImage, CameraPose, HeadlessGpu, OrthoFootprint, PinholeProjection, SceneLighting,
    TerrainRegion, TerrainSources,
};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const NEAR_M: f64 = 0.5;

/// A machine with no GPU cannot run these; everything they do not render is
/// covered by the pure tests. A skip is reported on the raw stream, which the
/// harness does not capture, so it cannot pass unnoticed.
fn gpu() -> Option<HeadlessGpu> {
    match pollster::block_on(HeadlessGpu::new()) {
        Ok(gpu) => Some(gpu),
        Err(err) => {
            writeln!(std::io::stderr(), "GPU test skipped: {err}").ok();
            None
        }
    }
}

fn intrinsics(fx: f64) -> SensorIntrinsics {
    SensorIntrinsics {
        width: WIDTH,
        height: HEIGHT,
        fx,
        fy: fx,
        cx: 159.5,
        cy: 119.5,
    }
}

/// Looks straight down from `height_m` above `position`, north at the top.
fn nadir(body: &BodyEllipsoid, position: &Geodetic, height_m: f64) -> CameraPose {
    let frame = LocalFrame::at(position);
    let eye = body.geodetic_to_fixed(&Geodetic {
        height_m,
        ..*position
    });
    CameraPose::looking(eye, -frame.up(), frame.north())
}

fn sun_overhead(position: &Geodetic) -> SceneLighting {
    SceneLighting {
        sun_direction_ecef: LocalFrame::at(position).up(),
    }
}

fn render(
    gpu: &HeadlessGpu,
    mesh: &GlobeMesh,
    atlas: Option<&OrthoAtlas>,
    fx: f64,
    frame: &ApertureFrame,
) -> ApertureImage {
    let projection = PinholeProjection::new(intrinsics(fx), NEAR_M).unwrap();
    let target = OffscreenAperture::new(&gpu.device, projection).unwrap();
    let mut pass = pollster::block_on(SensorAperturePass::new(
        &gpu.device,
        &gpu.queue,
        mesh,
        atlas,
    ))
    .unwrap();
    pass.render(&gpu.device, &gpu.queue, &target, frame);
    let image = target.read_blocking(&gpu.device, &gpu.queue).unwrap();
    gpu.check_device().unwrap();
    image
}

fn visible(pose: CameraPose, lighting: SceneLighting) -> ApertureFrame {
    ApertureFrame {
        pose,
        lighting,
        waveband: ApertureWaveband::VisibleRgb {
            sky_linear_rgb: [0.0; 3],
        },
        occlusion: OcclusionOverlayConfig::default(),
    }
}

/// A block of ground `half_span_rad` around `centre`.
fn block(body: BodyEllipsoid, centre: &Geodetic, half_span_rad: f64, grid: u32) -> TerrainRegion {
    TerrainRegion {
        body,
        south_rad: centre.lat_rad - half_span_rad,
        west_rad: centre.lon_rad - half_span_rad,
        lat_span_rad: 2.0 * half_span_rad,
        lon_span_rad: 2.0 * half_span_rad,
        lat_patches: 2,
        lon_patches: 2,
        grid,
    }
}

struct Plateau(f64);

impl ElevationSource for Plateau {
    fn ground_height_m(&self, _position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        Ok(Some(self.0))
    }
}

struct UniformGround(SurfaceRadiometry);

impl RadiometricSource for UniformGround {
    fn surface_radiometry(
        &self,
        _position: &Geodetic,
    ) -> Result<Option<SurfaceRadiometry>, SceneryError> {
        Ok(Some(self.0))
    }
}

#[test]
fn the_depth_buffer_reads_back_the_range_to_displaced_terrain() {
    let Some(gpu) = gpu() else { return };
    let body = BodyEllipsoid::WGS84;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let sources = TerrainSources {
        elevation: Some(&Plateau(500.0)),
        ..Default::default()
    };
    let mesh = GlobeMesh::build_region(&block(body, &centre, 0.002, 8), sources).unwrap();
    let frame = visible(nadir(&body, &centre, 2_000.0), sun_overhead(&centre));
    let image = render(&gpu, &mesh, None, 200.0, &frame);
    let depth = image.depth_at(WIDTH / 2, HEIGHT / 2).unwrap();
    let projection = PinholeProjection::new(intrinsics(200.0), NEAR_M).unwrap();
    let range = projection.range_m(depth).unwrap();
    assert!((range - 1_500.0).abs() < 1.0, "range {range} m");
}

#[test]
fn open_sky_has_no_depth() {
    let Some(gpu) = gpu() else { return };
    let body = BodyEllipsoid::WGS84;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let mesh = GlobeMesh::build_region(&block(body, &centre, 0.0001, 2), TerrainSources::default())
        .unwrap();
    // Too small a block to fill the frame.
    let frame = visible(nadir(&body, &centre, 50_000.0), sun_overhead(&centre));
    let image = render(&gpu, &mesh, None, 200.0, &frame);
    assert_eq!(image.depth_at(2, 2), Some(0.0));
    assert!(image.depth_at(WIDTH / 2, HEIGHT / 2).unwrap() > 0.0);
}

/// Two tiles side by side, with a third position in the footprint left empty.
fn tile_scene() -> (TileId, TileId, TileId, CatalogScenery, OrthoFootprint) {
    let west = TileId {
        zoom: 12,
        x: 1188,
        y: 1563,
    };
    let east = TileId { x: 1189, ..west };
    let south = TileId { y: 1564, ..west };
    let flat = |rgb: [u8; 3]| {
        let [r, g, b] = rgb;
        ImageryRaster::new(4, 4, [r, g, b, 255].repeat(16)).unwrap()
    };
    let mut catalog = CatalogScenery::new();
    catalog.insert_imagery(west, flat([230, 20, 20])).unwrap();
    catalog.insert_imagery(east, flat([20, 20, 230])).unwrap();
    let footprint = OrthoFootprint {
        zoom: 12,
        west: west.x,
        north: west.y,
        tiles_wide: NonZeroU32::new(2).unwrap(),
        tiles_high: NonZeroU32::new(2).unwrap(),
    };
    (west, east, south, catalog, footprint)
}

fn draped_mesh(body: BodyEllipsoid, centre: &Geodetic, footprint: &OrthoFootprint) -> GlobeMesh {
    let sources = TerrainSources {
        ortho: Some(footprint),
        ..Default::default()
    };
    GlobeMesh::build_region(&block(body, centre, 0.04, 16), sources).unwrap()
}

#[test]
fn imagery_drapes_with_east_to_the_right() {
    let Some(gpu) = gpu() else { return };
    let (_, east, _, catalog, footprint) = tile_scene();
    let atlas = OrthoAtlas::from_source(&catalog, footprint, 4).unwrap();
    let body = BodyEllipsoid::WGS84;
    let edge = tile_point_to_geodetic(east, 0.0, 0.5).unwrap();
    let mesh = draped_mesh(body, &edge, &footprint);
    let frame = visible(nadir(&body, &edge, 20_000.0), sun_overhead(&edge));
    let image = render(&gpu, &mesh, Some(&atlas), 800.0, &frame);
    let left = image.pixel(WIDTH / 2 - 40, HEIGHT / 2).unwrap();
    let right = image.pixel(WIDTH / 2 + 40, HEIGHT / 2).unwrap();
    assert!(left[0] > 180 && left[2] < 90, "west tile {left:?}");
    assert!(right[2] > 180 && right[0] < 90, "east tile {right:?}");
}

#[test]
fn a_hole_in_the_atlas_falls_back_to_the_procedural_surface() {
    let Some(gpu) = gpu() else { return };
    let (_, _, south, catalog, footprint) = tile_scene();
    let atlas = OrthoAtlas::from_source(&catalog, footprint, 4).unwrap();
    let body = BodyEllipsoid::WGS84;
    let boundary = tile_point_to_geodetic(south, 0.5, 0.0).unwrap();
    let mesh = draped_mesh(body, &boundary, &footprint);
    let frame = visible(nadir(&body, &boundary, 20_000.0), sun_overhead(&boundary));
    let image = render(&gpu, &mesh, Some(&atlas), 800.0, &frame);
    let north = image.pixel(WIDTH / 2, HEIGHT / 2 - 40).unwrap();
    let hole = image.pixel(WIDTH / 2, HEIGHT / 2 + 40).unwrap();
    assert!(north[0] > 180 && north[2] < 90, "imagery north {north:?}");
    assert!(hole[2] > hole[0], "the graticule's blue ground {hole:?}");
    assert!(hole[0] < 180, "not the tile's red {hole:?}");
}

#[test]
fn with_no_atlas_the_surface_is_procedural() {
    let Some(gpu) = gpu() else { return };
    let body = BodyEllipsoid::WGS84;
    let centre = Geodetic::from_degrees(39.5, -75.5, 0.0);
    let mesh =
        GlobeMesh::build_region(&block(body, &centre, 0.02, 8), TerrainSources::default()).unwrap();
    let frame = visible(nadir(&body, &centre, 20_000.0), sun_overhead(&centre));
    let image = render(&gpu, &mesh, None, 400.0, &frame);
    let ground = image.pixel(WIDTH / 2, HEIGHT / 2).unwrap();
    assert!(ground[2] > ground[0], "{ground:?}");
}

fn thermal_frame(pose: CameraPose, scale: ThermalScale) -> ApertureFrame {
    ApertureFrame {
        pose,
        lighting: SceneLighting {
            sun_direction_ecef: DVec3::X,
        },
        waveband: ApertureWaveband::ThermalLwir(scale),
        occlusion: OcclusionOverlayConfig::default(),
    }
}

#[test]
fn thermal_level_is_linear_in_exitance_in_the_stored_bytes() {
    let Some(gpu) = gpu() else { return };
    let body = BodyEllipsoid::WGS84;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let ground = UniformGround(SurfaceRadiometry {
        albedo: 0.2,
        emissivity: 1.0,
        temperature_k: 300.0,
    });
    let sources = TerrainSources {
        radiometry: Some(&ground),
        ..Default::default()
    };
    let mesh = GlobeMesh::build_region(&block(body, &centre, 0.002, 4), sources).unwrap();
    let scale = ThermalScale {
        black_k: 200.0,
        white_k: 400.0,
        nedt_k: 0.0,
        noise_seed: 0,
    };
    let image = render(
        &gpu,
        &mesh,
        None,
        200.0,
        &thermal_frame(nadir(&body, &centre, 2_000.0), scale),
    );
    // (300⁴ − 200⁴) / (400⁴ − 200⁴) = 0.2708 of full scale.
    let [grey, g, b, _] = image.pixel(WIDTH / 2, HEIGHT / 2).unwrap();
    assert!((i32::from(grey) - 69).abs() <= 1, "grey {grey}");
    assert_eq!((grey, grey), (g, b), "monochrome");
}

fn west_crater_scene() -> (LunarTranquilityScenery, GlobeMesh, Geodetic) {
    let body = BodyEllipsoid::MOON_IAU;
    let site = LunarTranquilityScenery::apollo_11_landing();
    let centre = site.at_local(WEST_CRATER.east_m, WEST_CRATER.north_m);
    let sources = TerrainSources {
        elevation: Some(&site),
        radiometry: Some(&site),
        ..Default::default()
    };
    let half_span = 600.0 / body.semi_major_m;
    let mesh = GlobeMesh::build_region(&block(body, &centre, half_span, 48), sources).unwrap();
    (site, mesh, centre)
}

const LUNAR_SCALE: ThermalScale = ThermalScale {
    black_k: 90.0,
    white_k: 320.0,
    nedt_k: 0.0,
    noise_seed: 0,
};

#[test]
fn west_crater_shows_cold_on_warm_in_the_thermal_band() {
    let Some(gpu) = gpu() else { return };
    let (_, mesh, centre) = west_crater_scene();
    let pose = nadir(&BodyEllipsoid::MOON_IAU, &centre, 2_000.0);
    let image = render(&gpu, &mesh, None, 800.0, &thermal_frame(pose, LUNAR_SCALE));
    let floor = image.pixel(WIDTH / 2, HEIGHT / 2).unwrap()[0];
    // 350 m west of the crater, out on the sunlit plain.
    let plain = image.pixel(WIDTH / 2 - 140, HEIGHT / 2).unwrap()[0];
    assert!(floor < 5, "floor {floor}");
    assert!(plain > 40, "plain {plain}");
}

#[test]
fn sensor_noise_is_repeatable_and_varies_with_the_seed() {
    let Some(gpu) = gpu() else { return };
    let (_, mesh, centre) = west_crater_scene();
    let pose = nadir(&BodyEllipsoid::MOON_IAU, &centre, 2_000.0);
    let noisy = |noise_seed| ThermalScale {
        nedt_k: 3.0,
        noise_seed,
        ..LUNAR_SCALE
    };
    let shoot = |scale| render(&gpu, &mesh, None, 800.0, &thermal_frame(pose, scale));
    let clean = shoot(LUNAR_SCALE);
    let first = shoot(noisy(1));
    assert_eq!(first, shoot(noisy(1)), "the same seed gives the same frame");
    assert_ne!(first.rgba8, shoot(noisy(2)).rgba8);
    assert_ne!(first.rgba8, clean.rgba8);
    assert_eq!(
        first.depth, clean.depth,
        "noise is in the image, not the scene"
    );
}

#[test]
fn drawn_obstructions_match_the_ground_truth_mask() {
    let Some(gpu) = gpu() else { return };
    let body = BodyEllipsoid::WGS84;
    let centre = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let mesh = GlobeMesh::build_region(&block(body, &centre, 0.0001, 2), TerrainSources::default())
        .unwrap();
    let occlusion = OcclusionOverlayConfig {
        bezel: Some(WindowBezel {
            semi_axes_px: [150.0, 110.0],
            vignette_px: 0.0,
            colour: [1.0, 0.0, 0.0],
        }),
        propeller: Some(PropellerDisk {
            hub_px: [150.0, 130.0],
            tip_radius_px: 90.0,
            hub_radius_px: 8.0,
            blade_count: 3,
            blade_half_width_rad: 0.15,
            phase_rad: 0.4,
            colour: [0.0, 1.0, 0.0],
        }),
    };
    let frame = ApertureFrame {
        occlusion,
        ..visible(nadir(&body, &centre, 50_000.0), sun_overhead(&centre))
    };
    let image = render(&gpu, &mesh, None, 200.0, &frame);
    let mask = occlusion.ground_truth_mask(WIDTH, HEIGHT);
    let hidden = mask.iter().filter(|hidden| **hidden).count();
    assert!(hidden > 5_000, "{hidden}");
    let wrong = image
        .rgba8
        .as_chunks::<4>()
        .0
        .iter()
        .zip(&mask)
        .filter(|(pixel, hidden)| (pixel[0] > 200 || pixel[1] > 200) != **hidden)
        .count();
    assert!(wrong < mask.len() / 500, "{wrong} of {} pixels", mask.len());
}

#[test]
fn the_targets_have_the_formats_and_usages_cameras_need() {
    let Some(gpu) = gpu() else { return };
    let projection = PinholeProjection::new(intrinsics(200.0), NEAR_M).unwrap();
    let target = OffscreenAperture::new(&gpu.device, projection).unwrap();
    let usage = wgpu::TextureUsages::RENDER_ATTACHMENT
        | wgpu::TextureUsages::COPY_SRC
        | wgpu::TextureUsages::TEXTURE_BINDING;
    let colour = target.colour_texture();
    assert_eq!(colour.format(), wgpu::TextureFormat::Rgba8UnormSrgb);
    assert_eq!(colour.usage(), usage);
    let depth = target.depth_texture();
    assert_eq!(depth.format(), wgpu::TextureFormat::Depth32Float);
    assert_eq!(depth.usage(), usage);
    assert_eq!((colour.width(), colour.height()), (WIDTH, HEIGHT));
}

#[test]
fn a_sensor_larger_than_the_device_allows_is_refused() {
    let Some(gpu) = gpu() else { return };
    let huge = SensorIntrinsics {
        width: gpu.device.limits().max_texture_dimension_2d + 1,
        ..intrinsics(200.0)
    };
    let projection = PinholeProjection::new(huge, NEAR_M).unwrap();
    let err = OffscreenAperture::new(&gpu.device, projection).err();
    assert!(
        matches!(err, Some(RenderError::TextureTooLarge { .. })),
        "{err:?}"
    );
}
