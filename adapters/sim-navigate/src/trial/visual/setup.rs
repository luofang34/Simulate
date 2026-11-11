//! Loading the maps of a run and building each set's visual chain.

use std::path::Path;
use std::sync::Arc;

use super::{CameraFrame, MapIdentity, VisualAid, VisualMapsReport};
use crate::NavMeans;
use crate::package::{MapPackage, PackageRole, PackageSpec, read_identity_blocking};
use crate::trial::{SensorSetSpec, TrialError, TrialRequest};

/// The onboard package of `request`, verified, when any set takes visual
/// fixes. The flight reads the map only through it.
pub fn load_onboard_package(request: &TrialRequest) -> Result<Option<Arc<MapPackage>>, TrialError> {
    let Some(visual) = request.visual.as_ref().filter(|_| request.uses_visual()) else {
        return Ok(None);
    };
    let package = MapPackage::open_blocking(
        Path::new(&visual.package_dir),
        visual.package_sha256.as_deref(),
    )?;
    Ok(Some(Arc::new(package)))
}

/// The visual chain of `set` over `package`, when the set takes visual
/// fixes.
pub fn visual_aid_for(
    request: &TrialRequest,
    set: &SensorSetSpec,
    package: Option<&Arc<MapPackage>>,
) -> Option<VisualAid> {
    let (Some(package), Some(visual)) = (package, request.visual.as_ref()) else {
        return None;
    };
    set.means
        .contains(&NavMeans::Visual)
        .then(|| VisualAid::new(Arc::clone(package), visual.clone()))
}

/// Mean distance from the camera to the ground over `frames`, metres.
pub fn mean_ground_distance_m(frames: &[CameraFrame]) -> Option<f64> {
    let distances: Vec<f64> = frames
        .iter()
        .filter_map(|frame| frame.truth.get().ground_distance_m)
        .collect();
    (!distances.is_empty()).then(|| distances.iter().sum::<f64>() / distances.len() as f64)
}

/// The maps of a visual run, and what the run does and does not test.
pub fn maps_report(
    request: &TrialRequest,
    mean_ground_distance_m: Option<f64>,
) -> Result<Option<VisualMapsReport>, TrialError> {
    let Some(visual) = request.visual.as_ref().filter(|_| request.uses_visual()) else {
        return Ok(None);
    };
    let onboard = MapIdentity::from(&read_identity_blocking(Path::new(&visual.package_dir))?);
    let world = MapIdentity::from(&read_identity_blocking(Path::new(&visual.world_dir))?);
    let footprint = mean_ground_distance_m.map(|d| d / visual.camera.focal_px());
    let r = visual.radiometry;
    let statement = format!(
        "The vehicle camera's frames are rendered by Simulate (sim_scenery::render_luma_view) from the world imagery ({world_source}, zoom {wz}, {wg:.2} m per pixel) at the vehicle's true pose, through gain {gain}, offset {offset} counts, gamma {gamma} and {noise} counts of white noise. References are rendered by Navigate (navigate_visual_render::SparseSceneSampler) from the onboard package ({onboard_source}, zoom {pz}, {pg:.2} m per pixel) at the navigator's estimated pose, matched by navigate_visual::Localizer with PyramidalMatcher, and fused through navigate_visual::fusion_bridge. Both imagery sets come from the same provider, so appearance change (season, lighting, cloud) is not tested: only geometry, resampling between zoom levels, viewpoint, sensor noise and the estimator chain.",
        world_source = world.source,
        wz = world.imagery_zoom,
        wg = world.imagery_gsd_m,
        gain = r.gain,
        offset = r.offset_dn,
        gamma = r.gamma,
        noise = r.noise_dn,
        onboard_source = onboard.source,
        pz = onboard.imagery_zoom,
        pg = onboard.imagery_gsd_m,
    );
    Ok(Some(VisualMapsReport {
        onboard,
        world,
        mean_ground_distance_m,
        camera_pixel_footprint_m: footprint,
        statement,
    }))
}

/// What `prepare` writes for `request`: the onboard package and the world
/// imagery, over the route's corridor grown by the camera footprint at the
/// highest waypoint (measured from the ellipsoid, so never short) and the
/// widest search.
pub fn package_specs(request: &TrialRequest) -> Result<(PackageSpec, PackageSpec), TrialError> {
    let visual = request.visual.as_ref().ok_or(TrialError::InvalidRequest {
        field: "visual",
        detail: "a package needs the visual section".into(),
    })?;
    let route: Vec<[f64; 2]> = request
        .waypoints
        .iter()
        .map(|w| [w.lat_deg, w.lon_deg])
        .collect();
    let highest = request
        .waypoints
        .iter()
        .map(|w| w.altitude_m)
        .fold(0.0, f64::max);
    let maps = visual.maps;
    let radius = maps.corridor_half_width_m
        + highest * visual.camera.half_diagonal_rad().tan()
        + visual.prior.max_radius_m;
    let spec = |role: PackageRole, imagery_zoom: u8, terrain_zoom: u8| PackageSpec {
        release_id: format!("{}-z{imagery_zoom}", role_name(role)),
        role,
        route: route.clone(),
        corridor_radius_m: radius,
        imagery_zoom,
        terrain_zoom,
        max_tiles: maps.max_tiles,
    };
    Ok((
        spec(
            PackageRole::Onboard,
            maps.onboard_imagery_zoom,
            maps.onboard_terrain_zoom,
        ),
        spec(
            PackageRole::World,
            maps.world_imagery_zoom,
            maps.world_terrain_zoom,
        ),
    ))
}

const fn role_name(role: PackageRole) -> &'static str {
    match role {
        PackageRole::Onboard => "onboard",
        PackageRole::World => "world",
    }
}
