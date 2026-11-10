//! Separated domain models for spatiotemporal geometric and access analysis:
//! [`GroundStationModel`], [`VehicleSensorModel`], [`SensorFovModel`],
//! [`OccultationEnvironmentModel`], and [`AccessQuerySpec`].

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_geo::{BodyEllipsoid, Geodetic};

use crate::{
    NaifId, SensorMount, ValidEpochInterval, acos_clamped, atan2, body_ellipsoid_serde,
    geodetic_serde, serde_dvec3, sqrt,
};

/// Field-of-view (`FOV`) constraint in a sensor's mount frame (`+Z_mount` boresight).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SensorFovModel {
    /// Unobstructed forward hemisphere ($\theta \le \pi/2$ around `+Z_mount`).
    OmnidirectionalHemisphere,
    /// Circular cone around `+Z_mount` with maximum off-boresight half-angle `max_half_angle_rad` (`rad`).
    ConicalHalfAngle {
        /// Maximum off-boresight angle $\theta_{\max} \in (0, \pi]$ (`rad`).
        max_half_angle_rad: f64,
    },
    /// Rectangular pyramid around `+Z_mount` with horizontal (`+X_mount`) and vertical (`+Y_mount`) half-angles (`rad`).
    RectangularPyramid {
        /// Maximum horizontal half-angle $|\operatorname{atan2}(x, z)| \le \alpha_x$ (`rad`).
        half_angle_x_rad: f64,
        /// Maximum vertical half-angle $|\operatorname{atan2}(y, z)| \le \alpha_y$ (`rad`).
        half_angle_y_rad: f64,
    },
}

impl SensorFovModel {
    /// Computes the off-boresight angle $\theta \in [0, \pi]$ (`rad`) from `+Z_mount` to `los_in_mount`.
    #[must_use]
    pub fn off_boresight_angle_rad(los_in_mount: DVec3) -> f64 {
        let len = sqrt(los_in_mount.length_squared());
        if len <= 1.0e-15 {
            0.0
        } else {
            acos_clamped(los_in_mount.z / len)
        }
    }

    /// Evaluates whether a line-of-sight vector `los_in_mount` lies inside this `FOV`.
    #[must_use]
    pub fn contains_direction_in_mount(&self, los_in_mount: DVec3) -> bool {
        let len = sqrt(los_in_mount.length_squared());
        if len <= 1.0e-15 {
            return true;
        }
        let u = los_in_mount / len;
        match *self {
            Self::OmnidirectionalHemisphere => u.z >= 0.0,
            Self::ConicalHalfAngle { max_half_angle_rad } => {
                acos_clamped(u.z) <= max_half_angle_rad
            }
            Self::RectangularPyramid {
                half_angle_x_rad,
                half_angle_y_rad,
            } => {
                if u.z <= 0.0 {
                    return false;
                }
                let ax = atan2(u.x.abs(), u.z);
                let ay = atan2(u.y.abs(), u.z);
                ax <= half_angle_x_rad && ay <= half_angle_y_rad
            }
        }
    }
}

/// Topocentric planetary ground station model separated from vehicle sensor and query specifications.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundStationModel {
    /// Unique numeric station identifier.
    pub station_id: u32,
    /// Human-readable station name (e.g. `"DSS-14 Goldstone"`).
    pub name: String,
    /// Host celestial body (`NaifId::EARTH = 399`, `NaifId::MOON = 301`, `NaifId::MARS = 499`).
    pub body: NaifId,
    /// Geodetic `(lat_rad, lon_rad, height_m)` coordinates on `ellipsoid`.
    #[serde(with = "geodetic_serde")]
    pub geodetic: Geodetic,
    /// Reference ellipsoid of the host body (`semi_major_m`, `flattening`, `omega_rad_s`).
    #[serde(with = "body_ellipsoid_serde")]
    pub ellipsoid: BodyEllipsoid,
    /// Antenna/telescope mount relative to the local topocentric `ENU` frame (`+X` East, `+Y` North, `+Z` Up).
    pub mount: SensorMount,
    /// Station antenna/sensor field-of-view model in station mount coordinates.
    pub fov: SensorFovModel,
    /// Minimum topocentric horizon elevation mask $\text{el}_{\min} \in [-\pi/2, \pi/2]$ (`rad`).
    pub min_elevation_rad: f64,
}

impl GroundStationModel {
    /// Creates an Earth (`WGS84`) ground station with a zenith-pointing hemispherical antenna and horizon elevation mask.
    #[must_use]
    pub fn earth_wgs84(
        station_id: u32,
        name: impl Into<String>,
        geodetic: Geodetic,
        min_elevation_rad: f64,
    ) -> Self {
        Self {
            station_id,
            name: name.into(),
            body: NaifId::EARTH,
            geodetic,
            ellipsoid: BodyEllipsoid::WGS84,
            mount: SensorMount::IDENTITY,
            fov: SensorFovModel::OmnidirectionalHemisphere,
            min_elevation_rad,
        }
    }

    /// Creates a Lunar (`MOON_ME` sphere `R = 1,737.4 km`) ground station with horizon elevation mask.
    #[must_use]
    pub fn moon_me(
        station_id: u32,
        name: impl Into<String>,
        geodetic: Geodetic,
        min_elevation_rad: f64,
    ) -> Self {
        Self {
            station_id,
            name: name.into(),
            body: NaifId::MOON,
            geodetic,
            ellipsoid: BodyEllipsoid::MOON_IAU,
            mount: SensorMount::IDENTITY,
            fov: SensorFovModel::OmnidirectionalHemisphere,
            min_elevation_rad,
        }
    }
}

/// Vehicle-mounted sensor or antenna model separated from ground-station and propagation models.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VehicleSensorModel {
    /// Unique numeric sensor identifier on the vehicle.
    pub sensor_id: u32,
    /// Human-readable sensor label (e.g. `"High-Gain X-Band Dish"` or `"Nadir Optical Nav Camera"`).
    pub label: String,
    /// Rigid placement (`offset_body_m` and `body_to_mount` rotation) in vehicle body coordinates.
    pub mount: SensorMount,
    /// Center-of-gravity position in vehicle structural body coordinates (`m`, default `DVec3::ZERO`).
    #[serde(with = "serde_dvec3")]
    pub cg_body_m: DVec3,
    /// Sensor field-of-view constraint in sensor mount coordinates (`+Z_mount` boresight).
    pub fov: SensorFovModel,
    /// Whether an attitude trajectory source is required to evaluate this sensor.
    pub require_attitude: bool,
}

impl VehicleSensorModel {
    /// Creates an omnidirectional vehicle antenna at the vehicle CG requiring no attitude source.
    #[must_use]
    pub fn omnidirectional_at_cg(sensor_id: u32, label: impl Into<String>) -> Self {
        Self {
            sensor_id,
            label: label.into(),
            mount: SensorMount::IDENTITY,
            cg_body_m: DVec3::ZERO,
            fov: SensorFovModel::ConicalHalfAngle {
                max_half_angle_rad: core::f64::consts::PI,
            },
            require_attitude: false,
        }
    }

    /// Creates a directional vehicle sensor with explicit `mount`, `fov`, and mandatory attitude query.
    #[must_use]
    pub fn directional(
        sensor_id: u32,
        label: impl Into<String>,
        mount: SensorMount,
        cg_body_m: DVec3,
        fov: SensorFovModel,
    ) -> Self {
        Self {
            sensor_id,
            label: label.into(),
            mount,
            cg_body_m,
            fov,
            require_attitude: true,
        }
    }
}

/// Geometric shape used for planetary line-of-sight occultation testing.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum OccultingShape {
    /// Spherical occulting body of radius `radius_m` (`m`).
    Sphere {
        /// Occulting sphere radius (`m`).
        radius_m: f64,
    },
    /// Oblate/spheroidal occulting body evaluated in its body-fixed frame.
    Ellipsoid(#[serde(with = "body_ellipsoid_serde")] BodyEllipsoid),
}

/// One celestial body tested for line-of-sight occultation between station and vehicle sensor.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OccultingBodySpec {
    /// NAIF body identifier (`399`, `301`, `499`, `10`, etc.).
    pub body: NaifId,
    /// Geometric occulting shape.
    pub shape: OccultingShape,
}

/// Environment model listing celestial bodies that can occult a station-to-vehicle line of sight.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OccultationEnvironmentModel {
    /// Occulting bodies checked along every line-of-sight segment.
    pub bodies: Vec<OccultingBodySpec>,
}

impl OccultationEnvironmentModel {
    /// Standard Earth (`R = 6,378.137 km`) and Moon (`R = 1,737.4 km`) spherical occultation model.
    #[must_use]
    pub fn earth_and_moon_spherical() -> Self {
        Self {
            bodies: vec![
                OccultingBodySpec {
                    body: NaifId::EARTH,
                    shape: OccultingShape::Sphere {
                        radius_m: 6_378_137.0,
                    },
                },
                OccultingBodySpec {
                    body: NaifId::MOON,
                    shape: OccultingShape::Sphere {
                        radius_m: 1_737_400.0,
                    },
                },
            ],
        }
    }

    /// Earth `WGS84` oblate ellipsoid and Moon sphere occultation model.
    #[must_use]
    pub fn earth_wgs84_and_moon() -> Self {
        Self {
            bodies: vec![
                OccultingBodySpec {
                    body: NaifId::EARTH,
                    shape: OccultingShape::Ellipsoid(BodyEllipsoid::WGS84),
                },
                OccultingBodySpec {
                    body: NaifId::MOON,
                    shape: OccultingShape::Sphere {
                        radius_m: 1_737_400.0,
                    },
                },
            ],
        }
    }

    /// Single-body Earth `WGS84` oblate ellipsoid occultation model.
    #[must_use]
    pub fn earth_wgs84() -> Self {
        Self {
            bodies: vec![OccultingBodySpec {
                body: NaifId::EARTH,
                shape: OccultingShape::Ellipsoid(BodyEllipsoid::WGS84),
            }],
        }
    }
}

/// When the two ends of a link are compared.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignalTimeModel {
    /// Both ends at the sample epoch, as if light crossed in no time. Right for links a
    /// few light-milliseconds long.
    #[default]
    Geometric,
    /// The source where it was when the signal left it, the target where it is when the
    /// signal arrives at the sample epoch, and each occulting body where it is as the
    /// signal passes. Across interplanetary distances the two ends are minutes apart.
    LightTime,
}

/// Query specification for evaluating time-series link geometry and access intervals.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AccessQuerySpec {
    /// Closed TDB query interval `[start_epoch, end_epoch]`.
    pub query_interval: ValidEpochInterval,
    /// Uniform coarse grid sampling step (`s`, `> 0`).
    pub sample_step_s: f64,
    /// Bisection root-refinement tolerance on rise/set window transitions (`s`, `> 0`).
    /// If `>= sample_step_s`, no bracket refinement is performed (`WindowSearchCompleteness::SampledGridOnly`).
    pub boundary_time_tolerance_s: f64,
    /// Optional minimum slant range constraint (`m`).
    pub min_range_m: Option<f64>,
    /// Optional maximum slant range constraint (`m`).
    pub max_range_m: Option<f64>,
    /// When the two ends are compared; requests without it are geometric.
    #[serde(default)]
    pub signal_time: SignalTimeModel,
}
