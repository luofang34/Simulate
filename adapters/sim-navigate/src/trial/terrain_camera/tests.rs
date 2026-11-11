//! The terrain camera over a synthetic world package: the package opens
//! only under its own identity, the camera sees the world's pattern by day
//! and only its own noise by night, its frames are a function of pose and
//! seed, and moving the true pose moves the picture.

use std::sync::Arc;

use navigate_data::{
    Accuracy, DEFAULT_CORRELATION_ZOOM, Descriptor, DirectoryStore, Encoding, GeoidModel,
    HorizontalDatum, LayerKey, LayerKind, LayerSpec, PackageWriter, Revision, SourceKey,
    TileAddress, VerticalDatum,
};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{Geodetic, LocalFrame, UtcTime, geodetic_to_ecef};

use super::{CameraBand, CameraMountSpec, RadiometrySpec, TerrainCamera, TerrainCameraSpec};
use crate::trial::TrialError;
use crate::trial::maps::{OpenedPackage, PackageRef, PackageRefusal};

const LAT_DEG: f64 = 39.0;
const LON_DEG: f64 = -75.55;
const ZOOM: u32 = 15;
/// A night and a noon over the corridor.
const NIGHT_UNIX_S: f64 = 1_773_972_000.0;
const NOON_UNIX_S: f64 = 1_774_026_300.0;

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

/// A 3 × 3 block of tiles around the corridor: a checkerboard of 32-pixel
/// squares with a diagonal ramp, over flat ground at 10 m above the geoid.
fn world(dir: &std::path::Path) -> PackageRef {
    let store_root = dir.to_path_buf();
    let mut store = DirectoryStore::new(store_root.clone());
    let mut writer = PackageWriter::new();
    let imagery = layer(LayerKind::Imagery, Encoding::Png);
    let elevation = layer(LayerKind::Elevation, Encoding::TerrariumPng);
    writer.add_layer(imagery).unwrap();
    writer.add_layer(elevation).unwrap();
    let centre = TileAddress::containing([LAT_DEG, LON_DEG], ZOOM).unwrap();
    let ground = image::RgbImage::from_pixel(256, 256, image::Rgb([128, 10, 0]));
    let ground = png(image::DynamicImage::ImageRgb8(ground));
    for dy in 0..3_u32 {
        for dx in 0..3_u32 {
            let (x, y) = (centre.x + dx - 1, centre.y + dy - 1);
            let address = TileAddress::new(ZOOM, x, y).unwrap();
            let pattern = image::RgbaImage::from_fn(256, 256, |px, py| {
                let square = ((px / 32) + (py / 32) + x + y) % 2;
                let luma = (40 + 160 * square + (px + py) / 8) as u8;
                image::Rgba([luma, luma, luma, 255])
            });
            let bytes = png(image::DynamicImage::ImageRgba8(pattern));
            pollster::block_on(writer.add_tile(&mut store, imagery.key, address, &bytes)).unwrap();
            pollster::block_on(writer.add_tile(&mut store, elevation.key, address, &ground))
                .unwrap();
        }
    }
    let identity = pollster::block_on(writer.seal(&mut store, &Descriptor::default())).unwrap();
    PackageRef {
        store: store_root.display().to_string(),
        package: identity.to_string(),
    }
}

fn nadir() -> TerrainCameraSpec {
    TerrainCameraSpec {
        band: CameraBand::Visible,
        width: 64,
        height: 48,
        fx_px: 64.0,
        fy_px: 64.0,
        cx_px: 32.0,
        cy_px: 24.0,
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
            offset_dn: 4.0,
            gamma: 1.0,
            noise_dn: 1.0,
        },
        min_sun_elevation_deg: 5.0,
    }
}

/// Level flight east at 300 m, `east_m` east of the corridor point.
fn state(east_m: f64) -> VehicleState {
    let at = Geodetic {
        lat_rad: LAT_DEG.to_radians(),
        lon_rad: LON_DEG.to_radians(),
        height_m: 300.0,
    };
    let moved = geodetic_to_ecef(&at) + LocalFrame::at(&at).east() * east_m;
    let mut state = VehicleState::level_flight(&at, std::f64::consts::FRAC_PI_2, 60.0);
    state.position_ecef = moved;
    state
}

fn camera(package: &PackageRef, start_unix_s: f64) -> TerrainCamera {
    let opened = OpenedPackage::open_blocking(package, "maps.world").unwrap();
    let world = Arc::new(opened.scenery_blocking().unwrap());
    TerrainCamera::new(world, nadir(), 11, UtcTime::from_unix_seconds(start_unix_s)).unwrap()
}

fn spread(pixels: &[u8]) -> f64 {
    let mean = pixels.iter().map(|p| f64::from(*p)).sum::<f64>() / pixels.len() as f64;
    let variance = pixels
        .iter()
        .map(|p| (f64::from(*p) - mean).powi(2))
        .sum::<f64>()
        / pixels.len() as f64;
    variance.sqrt()
}

#[test]
fn a_package_opens_only_under_its_own_identity() {
    let dir = tempfile::tempdir().unwrap();
    let package = world(dir.path());
    let opened = OpenedPackage::open_blocking(&package, "maps.world").unwrap();
    let report = opened.report_blocking("world").unwrap();
    assert_eq!(report.manifest_sha256, package.package);
    assert_eq!((report.imagery_zoom, report.imagery_tiles), (ZOOM, 9));
    assert!(
        (3.0..4.0).contains(&report.imagery_ground_sample_m),
        "{report:?}"
    );
    let refusal =
        |reference: &PackageRef| match OpenedPackage::open_blocking(reference, "maps.world") {
            Err(TrialError::Package { reason, .. }) => reason,
            other => panic!("{other:?}"),
        };
    let mut wrong = package.clone();
    wrong.package = "0".repeat(64);
    assert_eq!(refusal(&wrong), PackageRefusal::Missing);
    wrong.package = "not a digest".into();
    assert_eq!(refusal(&wrong), PackageRefusal::MalformedIdentity);
    // A store whose manifest file under the named identity holds other
    // bytes is not that package.
    let manifest = dir.path().join(navigate_data::manifest_path(
        package.package.parse().unwrap(),
    ));
    let mut bytes = std::fs::read(&manifest).unwrap();
    bytes.push(b' ');
    std::fs::write(&manifest, bytes).unwrap();
    assert_eq!(refusal(&package), PackageRefusal::IdentityMismatch);
}

#[test]
fn the_camera_sees_the_world_by_day_and_only_noise_by_night() {
    let dir = tempfile::tempdir().unwrap();
    let package = world(dir.path());
    let at = SimInstant::START;
    let day = camera(&package, NOON_UNIX_S)
        .capture(1, (0, 1), at, &state(0.0))
        .unwrap();
    let night = camera(&package, NIGHT_UNIX_S)
        .capture(1, (0, 1), at, &state(0.0))
        .unwrap();
    assert_eq!(day.pixels.len(), 64 * 48);
    assert!(day.truth.get().sun_elevation_deg > 30.0);
    assert!(night.truth.get().sun_elevation_deg < -18.0);
    assert!(spread(&day.pixels) > 30.0, "{}", spread(&day.pixels));
    assert!(spread(&night.pixels) < 2.0, "{}", spread(&night.pixels));
}

#[test]
fn frames_follow_pose_and_seed_alone() {
    let dir = tempfile::tempdir().unwrap();
    let package = world(dir.path());
    let camera = camera(&package, NOON_UNIX_S);
    let at = SimInstant::START;
    let first = camera.capture(1, (0, 1), at, &state(0.0)).unwrap();
    let again = camera.capture(1, (0, 1), at, &state(0.0)).unwrap();
    assert_eq!(first, again);
    let next = camera.capture(2, (0, 1), at, &state(0.0)).unwrap();
    assert_ne!(first.pixels, next.pixels, "each frame draws its own noise");
    // Half a 32-pixel square at about 3.4 m per world pixel: the picture
    // must change where the vehicle truly is.
    let moved = camera.capture(1, (0, 1), at, &state(55.0)).unwrap();
    let changed = first
        .pixels
        .iter()
        .zip(&moved.pixels)
        .filter(|(a, b)| a.abs_diff(**b) > 40)
        .count();
    assert!(changed > first.pixels.len() / 10, "{changed}");
}

#[test]
fn an_lwir_or_unscheduled_camera_is_refused() {
    let mut spec = nadir();
    spec.band = CameraBand::Lwir;
    assert!(spec.validate().is_err());
    let mut spec = nadir();
    spec.period_s = 0.105;
    assert!(spec.validate().is_err());
    let mut spec = nadir();
    spec.exposure_s = 2.0;
    assert!(spec.validate().is_err());
}

/// Frames come a period apart from the first period, each read out within
/// the flight, and each at the true pose of its middle.
#[test]
fn frames_follow_the_schedule_along_a_flight() {
    let dir = tempfile::tempdir().unwrap();
    let package = world(dir.path());
    let camera = camera(&package, NOON_UNIX_S);
    let schedule = camera.schedule(250).unwrap();
    assert_eq!(schedule, vec![(120, 121), (240, 241)]);
    let truth: Vec<_> = (0..250_u64)
        .map(|step| {
            let at = SimInstant::START.advanced(sim_core::TickSpan::from_ticks(
                sim_core::TICKS_PER_SECOND / 120 * step,
            ));
            crate::Truth::new((at, state(step as f64 * 0.5)))
        })
        .collect();
    let frames = camera.render_along(&truth).unwrap();
    assert_eq!(frames.len(), 2);
    assert_eq!((frames[1].mid_step, frames[1].end_step), (240, 241));
    let expected = camera
        .capture(2, (240, 241), truth[240].get().0, &state(120.0))
        .unwrap();
    assert_eq!(frames[1], expected);
}

/// The maps section names both identities and says when the world is the
/// onboard package, and which geoid each side has.
#[test]
fn the_maps_report_states_both_packages() {
    let dir = tempfile::tempdir().unwrap();
    let package = world(dir.path());
    let maps = crate::trial::maps::MapsSpec {
        onboard: crate::trial::maps::OnboardMapSpec {
            store: package.store.clone(),
            package: package.package.clone(),
            assumed_accuracy: None,
            max_match_pixels: None,
        },
        world: package.clone(),
        geoid: None,
    };
    let report = crate::trial::maps::maps_report_blocking(&maps).unwrap();
    assert!(report.world_is_onboard);
    assert_eq!(report.onboard.manifest_sha256, report.world.manifest_sha256);
    assert_eq!(
        (report.onboard.role.as_str(), report.world.role.as_str()),
        ("onboard", "world")
    );
    assert_eq!(report.navigator_geoid, "none");
    assert!(report.world_geoid.contains("5 degree"));
}
