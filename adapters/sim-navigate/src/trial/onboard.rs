//! The onboard link: the one place the trial names Navigate's composition.
//!
//! The trial declares the vehicle's sensors once as a
//! [`navigate_onboard::SensorSuite`] and links a navigator and a flight
//! director as trait objects. Everything else in the trial sees only
//! [`Navigator`] and [`FlightDirector`]: it hands sensor inputs over,
//! reads solutions, acceptance records and director outputs back, and
//! never reaches into the algorithms behind them.

use navigate_contract::{Ellipsoid, SolutionQuality, SourceId, Waypoint};
use navigate_fusion::FogmBias;
use navigate_guidance::GuidanceConfig;
use navigate_onboard::{
    ErrorBudget, FlightDirector, GuidanceLaw, Navigator, ReferenceNavigator, RouteDirector,
    SensorKind, SensorSpec, SensorSuite, StarCatalogueConfig,
};

use super::epoch::parse_utc;
use super::route::execution_config;
use super::stars::{roof_star_mount, star_camera_config};
use super::{QualityFloor, SensorSetSpec, TrialError, TrialRequest, VehicleSpec};
use crate::{MASTER_CLOCK_DOMAIN, NavMeans, imager_model, inertial_config, navigate_mount};

/// The trial's IMU.
pub const IMU_SOURCE: SourceId = SourceId::new(90);
/// The GNSS receiver.
pub const GNSS_SOURCE: SourceId = SourceId::new(91);
/// The barometric altimeter.
pub const BARO_SOURCE: SourceId = SourceId::new(93);
/// The roof star camera.
pub const STAR_SOURCE: SourceId = SourceId::new(94);

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
            budget: ErrorBudget::Barometer {
                white_sigma_m: request.baro.white_sigma_m,
                bias: FogmBias::new(
                    request.baro.bias_correlation_time_s,
                    request.baro.bias_sigma_m,
                ),
            },
        },
    ];
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

/// The navigator `set` flies with, unaligned.
pub fn trial_navigator(
    request: &TrialRequest,
    set: &SensorSetSpec,
) -> Result<Box<dyn Navigator>, TrialError> {
    let suite = sensor_suite(request, set)?;
    let stars = set
        .means
        .contains(&NavMeans::Celestial)
        .then(|| star_catalogue(request))
        .transpose()?;
    Ok(Box::new(ReferenceNavigator::new(
        suite,
        MASTER_CLOCK_DOMAIN,
        stars,
    )?))
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
