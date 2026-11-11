//! Visual trial fixtures: packages cut from the Delaware catalogs on disk
//! through the same writer `prepare` uses. The catalogs are not in the
//! repository, so the tests that read them are ignored by default.

#![allow(
    dead_code,
    reason = "each test crate that includes this support uses part of it"
)]

use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use sim_navigate::package::{
    CatalogDirSource, PackageRole, PackageSpec, TileFetch, prepare_package_blocking,
};
use sim_navigate::trial::{VisualSpec, WaypointSpec};

/// Where the Delaware catalogs live.
pub const CATALOG_ROOT: &str = "/Users/fangluo/Downloads/delaware-videos/claw";
/// The onboard map: Esri z15 over the Delaware Water Gap.
pub const ONBOARD_CATALOG: &str = "reference-watergap";
/// The simulated world: Esri z16 over an overlapping area.
pub const WORLD_CATALOG: &str = "v5730-gap";
/// Corridor half width of the planned route, metres.
pub const CORRIDOR_HALF_WIDTH_M: f64 = 1_500.0;
/// The corridor grown by the camera footprint and the widest search.
pub const PACKAGE_RADIUS_M: f64 = 3_000.0;

fn waypoint(ident: &str, lat_deg: f64, lon_deg: f64) -> WaypointSpec {
    WaypointSpec {
        ident: ident.into(),
        lat_deg,
        lon_deg,
        altitude_m: 1_800.0,
        speed_m_s: None,
        fly_over: false,
    }
}

/// About 20 km east, 13 km north and 13 km west inside both catalogs.
pub fn visual_route() -> Vec<WaypointSpec> {
    vec![
        waypoint("V0", 40.86, -75.18),
        waypoint("V1", 40.86, -74.94),
        waypoint("V2", 40.98, -74.94),
        waypoint("V3", 40.98, -75.10),
    ]
}

/// The first leg only, about 8 minutes at 40 m/s.
pub fn short_visual_route() -> Vec<WaypointSpec> {
    let mut route = visual_route();
    route.truncate(2);
    route
}

pub fn route_points(route: &[WaypointSpec]) -> Vec<[f64; 2]> {
    route.iter().map(|w| [w.lat_deg, w.lon_deg]).collect()
}

/// Writes a package of `role` for `route` from `source` into `dir`.
pub fn write_package(
    source: &mut dyn TileFetch,
    route: &[WaypointSpec],
    role: PackageRole,
    imagery_zoom: u8,
    terrain_zoom: u8,
    dir: &Path,
) -> String {
    let spec = PackageSpec {
        release_id: format!("delaware-{role:?}-z{imagery_zoom}").to_lowercase(),
        role,
        route: route_points(route),
        corridor_radius_m: PACKAGE_RADIUS_M,
        imagery_zoom,
        terrain_zoom,
        max_tiles: 20_000,
    };
    prepare_package_blocking(&spec, source, dir)
        .unwrap()
        .manifest_sha256
}

pub fn catalog(name: &str) -> CatalogDirSource {
    CatalogDirSource::open_blocking(&Path::new(CATALOG_ROOT).join(name)).unwrap()
}

/// The onboard package and world of the full visual route, written once per
/// test process.
pub fn packages() -> &'static (tempfile::TempDir, PathBuf, PathBuf) {
    static PACKAGES: OnceLock<(tempfile::TempDir, PathBuf, PathBuf)> = OnceLock::new();
    PACKAGES.get_or_init(|| {
        let dir = tempfile::tempdir().unwrap();
        let onboard = dir.path().join("onboard");
        let world = dir.path().join("world");
        let route = visual_route();
        write_package(
            &mut catalog(ONBOARD_CATALOG),
            &route,
            PackageRole::Onboard,
            15,
            12,
            &onboard,
        );
        write_package(
            &mut catalog(WORLD_CATALOG),
            &route,
            PackageRole::World,
            16,
            13,
            &world,
        );
        (dir, onboard, world)
    })
}

/// The visual configuration over `onboard` and `world`.
pub fn visual_spec(onboard: &Path, world: &Path) -> VisualSpec {
    VisualSpec {
        package_dir: onboard.display().to_string(),
        package_sha256: None,
        world_dir: world.display().to_string(),
        camera: Default::default(),
        radiometry: Default::default(),
        budget: Default::default(),
        prior: Default::default(),
        render: Default::default(),
        maps: Default::default(),
    }
}

use sim_navigate::package::{PackageError, TileAddress, TileLayer};

fn encode_png(image: image::RgbImage) -> Vec<u8> {
    let mut bytes = std::io::Cursor::new(Vec::new());
    image::DynamicImage::ImageRgb8(image)
        .write_to(&mut bytes, image::ImageFormat::Png)
        .unwrap();
    bytes.into_inner()
}

/// The catalog with its imagery moved `shift_px` pixels east: every feature
/// sits that much east of where it is. Elevation is not moved.
pub struct ShiftedImagery {
    pub inner: CatalogDirSource,
    pub shift_px: u32,
}

impl TileFetch for ShiftedImagery {
    fn fetch(
        &mut self,
        layer: TileLayer,
        address: TileAddress,
    ) -> Result<Option<Vec<u8>>, PackageError> {
        if layer == TileLayer::Dem {
            return self.inner.fetch(layer, address);
        }
        let west = TileAddress {
            x: address.x - 1,
            ..address
        };
        let (Some(here), Some(left)) = (
            self.inner.fetch(layer, address)?,
            self.inner.fetch(layer, west)?,
        ) else {
            return Ok(None);
        };
        let here = image::load_from_memory(&here).unwrap().to_rgb8();
        let left = image::load_from_memory(&left).unwrap().to_rgb8();
        let shift = self.shift_px;
        let shifted = image::RgbImage::from_fn(256, 256, |x, y| {
            if x < shift {
                *left.get_pixel(256 - shift + x, y)
            } else {
                *here.get_pixel(x - shift, y)
            }
        });
        Ok(Some(encode_png(shifted)))
    }

    fn attribution(&self, layer: TileLayer) -> String {
        self.inner.attribution(layer)
    }

    fn identity(&self) -> String {
        format!(
            "{} shifted {} px east",
            self.inner.identity(),
            self.shift_px
        )
    }
}

/// The catalog with every imagery tile replaced by flat grey.
pub struct FeaturelessImagery {
    pub inner: CatalogDirSource,
}

impl TileFetch for FeaturelessImagery {
    fn fetch(
        &mut self,
        layer: TileLayer,
        address: TileAddress,
    ) -> Result<Option<Vec<u8>>, PackageError> {
        let tile = self.inner.fetch(layer, address)?;
        Ok(match layer {
            TileLayer::Dem => tile,
            TileLayer::Imagery => tile.map(|_| {
                encode_png(image::RgbImage::from_pixel(
                    256,
                    256,
                    image::Rgb([128, 128, 128]),
                ))
            }),
        })
    }

    fn attribution(&self, layer: TileLayer) -> String {
        self.inner.attribution(layer)
    }

    fn identity(&self) -> String {
        format!("{} made featureless", self.inner.identity())
    }
}

/// Onboard packages of the short route: shifted east and featureless.
pub fn altered_packages() -> &'static (tempfile::TempDir, PathBuf, PathBuf) {
    static PACKAGES: OnceLock<(tempfile::TempDir, PathBuf, PathBuf)> = OnceLock::new();
    PACKAGES.get_or_init(|| {
        let dir = tempfile::tempdir().unwrap();
        let shifted = dir.path().join("shifted");
        let featureless = dir.path().join("featureless");
        let route = visual_route();
        let mut source = ShiftedImagery {
            inner: catalog(ONBOARD_CATALOG),
            shift_px: SHIFT_PX,
        };
        write_package(&mut source, &route, PackageRole::Onboard, 15, 12, &shifted);
        let mut source = FeaturelessImagery {
            inner: catalog(ONBOARD_CATALOG),
        };
        write_package(
            &mut source,
            &route,
            PackageRole::Onboard,
            15,
            12,
            &featureless,
        );
        (dir, shifted, featureless)
    })
}

/// Pixels the shifted package moves its imagery east: about 40 m at z15.
pub const SHIFT_PX: u32 = 11;
