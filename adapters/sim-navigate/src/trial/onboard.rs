//! The onboard link: the one place the trial names Navigate's composition.
//!
//! The trial declares the vehicle's sensors once as a
//! [`navigate_onboard::SensorSuite`] and links a navigator and a flight
//! director as trait objects. Everything else in the trial sees only
//! [`Navigator`] and [`FlightDirector`]: it hands sensor inputs over,
//! reads solutions, acceptance records and director outputs back, and
//! never reaches into the algorithms behind them.

use std::sync::Arc;

use navigate_camera::{CameraMount as NavigateMount, ImagerModel, Intrinsics};
use navigate_contract::{Ellipsoid, SolutionQuality, SourceId, Waypoint};
use navigate_data::{DayNumber, GeoidModel};
use navigate_geodesy::GeoidUndulation;
use navigate_guidance::GuidanceConfig;
use navigate_onboard::{
    AssumedAccuracy, ErrorBudget, FlightDirector, GuidanceLaw, HostGeoid, Navigator,
    ReferenceNavigator, RouteDirector, SensorKind, SensorSpec, SensorSuite, StarCatalogueConfig,
    TerrainMapRequest,
};

use super::epoch::parse_utc;
use super::map_store::{DirectoryStores, MapStores};
use super::route::execution_config;
use super::stars::{roof_star_mount, star_camera_config};
use super::terrain_camera::TerrainCameraSpec;
use super::{QualityFloor, SensorSetSpec, TrialError, TrialRequest, VehicleSpec};
use crate::{MASTER_CLOCK_DOMAIN, NavMeans, imager_model, inertial_config, navigate_mount};
pub(super) use readings::{altimeter_budget, baro_altitude_data, gnss_fix_data};

mod readings;

/// The trial's IMU.
pub const IMU_SOURCE: SourceId = SourceId::new(90);
/// The GNSS receiver.
pub const GNSS_SOURCE: SourceId = SourceId::new(91);
/// The barometric altimeter.
pub const BARO_SOURCE: SourceId = SourceId::new(93);
/// The roof star camera.
pub const STAR_SOURCE: SourceId = SourceId::new(94);
/// The terrain camera.
pub const TERRAIN_CAMERA_SOURCE: SourceId = SourceId::new(95);
/// The terrain camera's declared calibration residual, radians: the
/// simulated camera has the intrinsics it states exactly.
const SIMULATED_CALIBRATION_SIGMA_RAD: f64 = 1e-4;

/// The sensors of the request's vehicle as `set` carries them: the IMU,
/// the GNSS receiver and the altimeter always, the star camera when the set
/// takes stars. A sensor a set lacks or a schedule withholds sends no
/// inputs.
pub fn sensor_suite(
    request: &TrialRequest,
    set: &SensorSetSpec,
) -> Result<SensorSuite, TrialError> {
    let imu = inertial_config(&request.imu_grade.grade().spec());
    let mut sensors = vec![
        SensorSpec {
            id: IMU_SOURCE,
            kind: SensorKind::Imu,
            imager: None,
            mount: None,
            rate_hz: super::route::STEP_HZ as f64,
            budget: ErrorBudget::Imu {
                accel_noise_mps2_sqrt_hz: imu.accel_noise_sigma_mps2_sqrt_hz,
                gyro_noise_rad_s_sqrt_hz: imu.gyro_noise_sigma_rad_s_sqrt_hz,
                accel_bias: imu.accel_bias,
                gyro_bias: imu.gyro_bias,
            },
        },
        SensorSpec {
            id: GNSS_SOURCE,
            kind: SensorKind::Gnss,
            imager: None,
            mount: None,
            rate_hz: request.gnss.rate_hz,
            budget: ErrorBudget::Declared,
        },
        SensorSpec {
            id: BARO_SOURCE,
            kind: SensorKind::Barometer,
            imager: None,
            mount: None,
            rate_hz: request.baro.rate_hz,
            budget: altimeter_budget(&request.baro, planned_height_m(request))?,
        },
    ];
    if let (true, Some(camera)) = (
        set.means.contains(&NavMeans::Visual),
        request.terrain_camera.as_ref(),
    ) {
        sensors.push(terrain_camera_spec(camera));
    }
    if set.means.contains(&NavMeans::Celestial) {
        let (mount, cg_body_m) = roof_star_mount();
        sensors.push(SensorSpec {
            id: STAR_SOURCE,
            kind: SensorKind::StarCamera,
            imager: Some(imager_model(&star_camera_config().intrinsics)?),
            mount: Some(navigate_mount(&mount, cg_body_m)),
            rate_hz: 1.0 / request.stars.period_s,
            budget: ErrorBudget::StarCamera {
                mount_alignment_sigma_rad: request.stars.mount_alignment_sigma_rad,
            },
        });
    }
    Ok(SensorSuite::new(sensors)?)
}

/// The star catalogue of the request's epoch: the navigator clock reads
/// zero at the first route instant.
pub fn star_catalogue(request: &TrialRequest) -> Result<StarCatalogueConfig, TrialError> {
    let start = parse_utc(&request.epoch_utc)?;
    Ok(StarCatalogueConfig::with_utc_at_zero(start.unix_seconds()))
}

/// The navigator `set` flies with, unaligned; a visual set's reads its
/// onboard package from the store.
pub fn trial_navigator_blocking(
    request: &TrialRequest,
    set: &SensorSetSpec,
) -> Result<Box<dyn Navigator>, TrialError> {
    trial_navigator_with_stores(request, set, &DirectoryStores)
}

/// [`trial_navigator_blocking`], reading the onboard package from the
/// store `stores` resolves.
pub fn trial_navigator_with_stores(
    request: &TrialRequest,
    set: &SensorSetSpec,
    stores: &dyn MapStores,
) -> Result<Box<dyn Navigator>, TrialError> {
    trial_navigator_inspecting(request, set, stores, false)
}

/// [`trial_navigator_with_stores`], whose visual navigator keeps a record of
/// its latest terrain match when `inspect` is on. The navigator's outputs
/// are the same either way; the record costs a copy of the matched images.
pub fn trial_navigator_inspecting(
    request: &TrialRequest,
    set: &SensorSetSpec,
    stores: &dyn MapStores,
    inspect: bool,
) -> Result<Box<dyn Navigator>, TrialError> {
    let suite = sensor_suite(request, set)?;
    let stars = set
        .means
        .contains(&NavMeans::Celestial)
        .then(|| star_catalogue(request))
        .transpose()?;
    let mut navigator = ReferenceNavigator::new(suite, MASTER_CLOCK_DOMAIN, stars)?;
    if set.means.contains(&NavMeans::Visual) {
        load_onboard_map(request, &mut navigator, stores)?;
        navigator.set_terrain_match_inspection(inspect);
    }
    Ok(Box::new(navigator))
}

/// The height the altimeter's budget is declared at: the route's start.
fn planned_height_m(request: &TrialRequest) -> f64 {
    request.waypoints.first().map_or(0.0, |w| w.altitude_m)
}

/// The terrain camera as the navigator is told of it: pinhole intrinsics
/// with pixel centres on integer coordinates, as Simulate renders, and the
/// same mount angles, in Navigate's own constructor.
fn terrain_camera_spec(camera: &TerrainCameraSpec) -> SensorSpec {
    let mount = camera.mount;
    let intrinsics = camera.intrinsics();
    let [x, y, z] = mount.offset_body_m;
    SensorSpec {
        id: TERRAIN_CAMERA_SOURCE,
        kind: SensorKind::TerrainCamera,
        imager: Some(ImagerModel::pinhole(Intrinsics {
            width: camera.width,
            height: camera.height,
            fx: intrinsics.fx,
            fy: intrinsics.fy,
            cx: intrinsics.cx,
            cy: intrinsics.cy,
        })),
        mount: Some(
            NavigateMount::rotated_forward(
                mount.roll_deg.to_radians(),
                mount.pitch_deg.to_radians(),
                mount.yaw_deg.to_radians(),
            )
            .with_lever_arm(nalgebra::Vector3::new(x, y, z)),
        ),
        rate_hz: 1.0 / camera.period_s,
        budget: ErrorBudget::TerrainCamera {
            calibration_sigma_rad: SIMULATED_CALIBRATION_SIGMA_RAD,
        },
    }
}

/// Simulate's own geoid, handed to the navigator so that it and the world
/// convert orthometric heights identically.
struct WorldGeoid;

impl GeoidUndulation for WorldGeoid {
    fn ellipsoid(&self) -> Ellipsoid {
        Ellipsoid::WGS84
    }

    fn undulation_m(&self, latitude_rad: f64, longitude_rad: f64) -> Option<f64> {
        let n = sim_geo::egm96_undulation_m(latitude_rad, longitude_rad);
        (latitude_rad.is_finite() && longitude_rad.is_finite()).then_some(n)
    }
}

/// Hands the onboard package to the navigator, as the request states it.
fn load_onboard_map(
    request: &TrialRequest,
    navigator: &mut ReferenceNavigator,
    stores: &dyn MapStores,
) -> Result<(), TrialError> {
    let Some(maps) = request.maps.as_ref() else {
        return Err(TrialError::InvalidRequest {
            field: "maps",
            detail: "a set takes visual fixes".into(),
        });
    };
    let identity = maps
        .onboard
        .package
        .parse()
        .map_err(|_| TrialError::InvalidRequest {
            field: "maps.onboard.package",
            detail: format!("`{}` is not 64 hex digits", maps.onboard.package),
        })?;
    let start = parse_utc(&request.epoch_utc)?;
    let today = DayNumber((start.unix_seconds() / 86_400.0).floor() as i32);
    let mut map = TerrainMapRequest::new(identity, today);
    map.assumed_accuracy = maps.onboard.assumed_accuracy.map(|a| AssumedAccuracy {
        horizontal_sigma_m: a.horizontal_m,
        vertical_sigma_m: a.vertical_m,
    });
    if let Some(pixels) = maps.onboard.max_match_pixels {
        map.config.max_match_pixels = pixels;
    }
    let geoid: Option<HostGeoid> = maps.geoid.map(|_| {
        let model: Arc<dyn GeoidUndulation + Send + Sync> = Arc::new(WorldGeoid);
        (GeoidModel::Egm96, model)
    });
    let store = Arc::new(stores.store(&maps.onboard.store)?);
    let report = navigator.load_terrain_map(store, &map, geoid)?;
    tracing::info!(?report, "onboard map loaded");
    Ok(())
}

/// The flight director over the request's route: deviation guidance with
/// the vehicle's quality floor, Navigate's own default when it states none.
pub fn trial_director(
    waypoints: &[Waypoint],
    vehicle: &VehicleSpec,
) -> Result<Box<dyn FlightDirector>, TrialError> {
    let mut guidance = GuidanceConfig::default().with_expected_ellipsoid(Ellipsoid::WGS84);
    guidance.minimum_quality = match vehicle.guidance_min_quality {
        Some(QualityFloor::Good) => SolutionQuality::Good,
        Some(QualityFloor::Unusable) => SolutionQuality::Unusable,
        Some(QualityFloor::Degraded) | None => guidance.minimum_quality,
    };
    Ok(Box::new(RouteDirector::new(
        "trial-route",
        waypoints.to_vec(),
        execution_config(vehicle),
        GuidanceLaw::DeviationTracking(guidance),
        vehicle.cruise_speed_m_s,
        MASTER_CLOCK_DOMAIN,
    )?))
}
