//! A synthetic world for terrain-camera tests: navigate-data packages of a
//! textured and of a featureless ground along a short eastbound route, over
//! flat land, written once into a temporary store.

#![allow(
    dead_code,
    reason = "each test crate that includes this support uses part of it"
)]

use std::sync::OnceLock;

use navigate_data::{
    Accuracy, DEFAULT_CORRELATION_ZOOM, Descriptor, DirectoryStore, Encoding, GeoidModel,
    HorizontalDatum, LayerKey, LayerKind, LayerSpec, PackageWriter, Revision, SourceKey,
    TileAddress, VerticalDatum,
};
use sim_navigate::trial::{
    CameraBand, CameraMountSpec, MapsSpec, OnboardMapSpec, PackageRef, RadiometrySpec,
    TerrainCameraSpec,
};

/// The route's latitude and its first longitude, degrees.
pub const LAT_DEG: f64 = 39.0;
/// The route's first longitude, degrees.
pub const LON_DEG: f64 = -75.6;
/// The route's length in longitude, degrees: about 4.3 km.
pub const LENGTH_DEG: f64 = 0.05;
/// Imagery zoom of the packages.
pub const ZOOM: u32 = 16;
/// The ground's height above the geoid, metres.
pub const GROUND_M: f64 = 10.0;

/// The packages of the synthetic world, in one store.
pub struct World {
    /// The store's directory, kept alive with the packages.
    pub dir: tempfile::TempDir,
    /// The textured package.
    pub textured: PackageRef,
    /// The featureless package: the same tiles, one grey.
    pub featureless: PackageRef,
}

/// The world, written on first use.
pub fn world() -> &'static World {
    static WORLD: OnceLock<World> = OnceLock::new();
    WORLD.get_or_init(|| {
        let dir = tempfile::tempdir().unwrap();
        let textured = package(dir.path(), texture);
        let featureless = package(dir.path(), |_, _, _, _| 128);
        World {
            dir,
            textured,
            featureless,
        }
    })
}

fn layer(kind: LayerKind, encoding: Encoding) -> LayerSpec {
    LayerSpec {
        key: LayerKey {
            kind,
            source: SourceKey::Synthetic,
        },
        revision: Revision::Unversioned,
        horizontal_datum: HorizontalDatum::Wgs84,
        vertical_datum: (kind == LayerKind::Elevation).then_some(VerticalDatum::Orthometric {
            geoid: GeoidModel::Egm96,
        }),
        encoding,
        tile_pixels: 256,
        accuracy: Accuracy::unknown(),
        correlation_zoom: DEFAULT_CORRELATION_ZOOM,
    }
}

fn png(image: image::DynamicImage) -> Vec<u8> {
    let mut bytes = std::io::Cursor::new(Vec::new());
    image.write_to(&mut bytes, image::ImageFormat::Png).unwrap();
    bytes.into_inner()
}

/// A hash of an integer lattice point, 0 to 255.
fn hash(x: i64, y: i64, salt: u64) -> f64 {
    let mut h = (x as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ (y as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F)
        ^ salt;
    h ^= h >> 31;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 29;
    (h & 0xFF) as f64
}

/// Value noise at `cell` pixels, bilinear between lattice points.
fn noise(gx: f64, gy: f64, cell: f64, salt: u64) -> f64 {
    let (u, v) = (gx / cell, gy / cell);
    let (x0, y0) = (u.floor(), v.floor());
    let (fx, fy) = (u - x0, v - y0);
    let at = |dx: f64, dy: f64| hash((x0 + dx) as i64, (y0 + dy) as i64, salt);
    let top = at(0.0, 0.0) * (1.0 - fx) + at(1.0, 0.0) * fx;
    let bottom = at(0.0, 1.0) * (1.0 - fx) + at(1.0, 1.0) * fx;
    top * (1.0 - fy) + bottom * fy
}

/// Fields and blobs of tens of metres with finer detail: unique ground
/// texture a matcher can lock onto.
fn texture(tile_x: u32, tile_y: u32, px: u32, py: u32) -> u8 {
    let gx = f64::from(tile_x) * 256.0 + f64::from(px);
    let gy = f64::from(tile_y) * 256.0 + f64::from(py);
    let value =
        0.55 * noise(gx, gy, 24.0, 1) + 0.3 * noise(gx, gy, 8.0, 2) + 0.15 * noise(gx, gy, 3.0, 3);
    value.round().clamp(0.0, 255.0) as u8
}

/// A package of the route's corridor with imagery from `luma`.
fn package(store: &std::path::Path, luma: fn(u32, u32, u32, u32) -> u8) -> PackageRef {
    let mut writer = PackageWriter::new();
    let imagery = layer(LayerKind::Imagery, Encoding::Png);
    let elevation = layer(LayerKind::Elevation, Encoding::TerrariumPng);
    writer.add_layer(imagery).unwrap();
    writer.add_layer(elevation).unwrap();
    let mut directory = DirectoryStore::new(store.to_path_buf());
    let west = TileAddress::containing([LAT_DEG, LON_DEG], ZOOM).unwrap();
    let east = TileAddress::containing([LAT_DEG, LON_DEG + LENGTH_DEG], ZOOM).unwrap();
    // Terrarium encodes h + 32768 as R·256 + G + B/256.
    let encoded = GROUND_M + 32_768.0;
    let ground = image::RgbImage::from_pixel(
        256,
        256,
        image::Rgb([(encoded / 256.0).floor() as u8, (encoded % 256.0) as u8, 0]),
    );
    let ground = png(image::DynamicImage::ImageRgb8(ground));
    for x in west.x - 2..=east.x + 2 {
        for y in west.y - 2..=west.y + 2 {
            let address = TileAddress::new(ZOOM, x, y).unwrap();
            let tile =
                image::GrayImage::from_fn(256, 256, |px, py| image::Luma([luma(x, y, px, py)]));
            let bytes = png(image::DynamicImage::ImageLuma8(tile));
            pollster::block_on(writer.add_tile(&mut directory, imagery.key, address, &bytes))
                .unwrap();
            pollster::block_on(writer.add_tile(&mut directory, elevation.key, address, &ground))
                .unwrap();
        }
    }
    let descriptor = Descriptor {
        release_id: format!("synthetic-{}", luma(0, 0, 0, 0)),
        ..Descriptor::default()
    };
    let identity = pollster::block_on(writer.seal(&mut directory, &descriptor)).unwrap();
    PackageRef {
        store: store.display().to_string(),
        package: identity.to_string(),
    }
}

/// A nadir camera of 160 × 120 pixels and a 67° horizontal field, a frame
/// every second.
pub fn camera() -> TerrainCameraSpec {
    TerrainCameraSpec {
        band: CameraBand::Visible,
        width: 160,
        height: 120,
        fx_px: Some(120.0),
        horizontal_fov_deg: None,
        fy_px: None,
        cx_px: Some(79.5),
        cy_px: Some(59.5),
        mount: CameraMountSpec {
            offset_body_m: [0.0, 0.0, 0.0],
            roll_deg: 0.0,
            pitch_deg: -90.0,
            yaw_deg: 0.0,
        },
        period_s: 1.0,
        exposure_s: 0.004,
        radiometry: RadiometrySpec {
            gain: 1.0,
            offset_dn: 0.0,
            gamma: 1.0,
            noise_dn: 2.0,
        },
        min_sun_elevation_deg: 5.0,
    }
}

/// The maps of `package`, the world and the onboard map alike, with the
/// world's geoid and the assumed accuracy the package does not declare.
pub fn maps(package: &PackageRef) -> MapsSpec {
    MapsSpec {
        onboard: OnboardMapSpec {
            store: package.store.clone(),
            package: package.package.clone(),
            assumed_accuracy: Some(sim_navigate::trial::AssumedAccuracy {
                horizontal_m: 3.0,
                vertical_m: 10.0,
            }),
            max_match_pixels: None,
        },
        world: package.clone(),
        geoid: Some(sim_navigate::trial::GeoidChoice::World),
    }
}
