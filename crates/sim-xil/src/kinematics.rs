use glam::{DMat3, DQuat, DVec3};
use sim_atmosphere::AtmosphereSample;
use sim_core::VehicleState;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, ecef_to_geodetic};

use crate::XilError;

/// Standard sea-level acceleration of gravity (`m/s²`), used by MAVLink
/// `HIL_STATE_QUATERNION` millig encoding.
pub const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// ICAO standard sea-level air density (`kg/m³`) for indicated airspeed.
pub const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;

/// Nominal mid-latitude geomagnetic field in NED (`Gauss`), used when no
/// site-specific geomagnetic model overrides the stimulus.
const DEFAULT_MAG_NED_GAUSS: DVec3 = DVec3::new(0.215, 0.020, 0.430);

/// Simulator truth and local environment presented to an external flight
/// controller at one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct XilStimulus {
    /// True centre-of-gravity vehicle state in ECEF and FRD body axes.
    pub vehicle: VehicleState,
    /// Specific force measured by an ideal accelerometer at the sensor site, in
    /// body FRD axes (`m/s²`). Stationary level flight on Earth is `[0, 0, -g]`.
    pub specific_force_body_m_s2: DVec3,
    /// Coordinate acceleration in body FRD axes (`m/s²`).
    pub linear_accel_body_m_s2: DVec3,
    /// Ambient static pressure (`Pa`).
    pub static_pressure_pa: f64,
    /// Ambient static air temperature (`K`).
    pub temperature_k: f64,
    /// Pitot impact (differential) pressure (`Pa`).
    pub differential_pressure_pa: f64,
    /// Local magnetic field expressed in body FRD axes (`Gauss`).
    pub mag_field_body_gauss: DVec3,
    /// True airspeed along the relative wind (`m/s`).
    pub airspeed_true_m_s: f64,
    /// Indicated airspeed (`m/s`).
    pub airspeed_indicated_m_s: f64,
}

/// WGS84 geodetic coordinates and local NED/FRD kinematics derived from an
/// [`XilStimulus`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NedKinematics {
    /// WGS84 latitude, longitude, and ellipsoidal height.
    pub geodetic: Geodetic,
    /// Rotation taking body FRD vectors into local NED vectors.
    pub attitude_frd_to_ned: DQuat,
    /// Velocity in local NED (`m/s`).
    pub velocity_ned_m_s: DVec3,
    /// Velocity in local ENU (`m/s`).
    pub velocity_enu_m_s: DVec3,
}

impl XilStimulus {
    /// Builds a stimulus from `vehicle`, body FRD `specific_force_body_m_s2`,
    /// and a local [`AtmosphereSample`], using WGS84 gravity for coordinate
    /// acceleration and a nominal geomagnetic vector in NED.
    pub fn from_state_and_atmosphere(
        vehicle: VehicleState,
        specific_force_body_m_s2: DVec3,
        atmosphere: &AtmosphereSample,
    ) -> Result<Self, XilError> {
        validate_vec3(vehicle.position_ecef, "vehicle.position_ecef")?;
        validate_vec3(vehicle.velocity_ecef, "vehicle.velocity_ecef")?;
        validate_vec3(vehicle.angular_rate_body, "vehicle.angular_rate_body")?;
        if !vehicle.attitude.is_finite() {
            return Err(XilError::InvalidStimulus {
                field: "vehicle.attitude",
            });
        }
        validate_vec3(specific_force_body_m_s2, "specific_force_body_m_s2")?;
        validate_scalar(atmosphere.pressure_pa, "atmosphere.pressure_pa")?;
        validate_scalar(atmosphere.temperature_k, "atmosphere.temperature_k")?;
        validate_scalar(atmosphere.density_kg_m3, "atmosphere.density_kg_m3")?;
        validate_vec3(atmosphere.wind_ned_m_s, "atmosphere.wind_ned_m_s")?;

        let geodetic = ecef_to_geodetic(vehicle.position_ecef);
        let local = LocalFrame::at(&geodetic);
        let ecef_to_body = DMat3::from_quat(vehicle.attitude.inverse());
        let gravity_ecef = BodyEllipsoid::WGS84.gravity_fixed(vehicle.position_ecef);
        let linear_accel_body_m_s2 = specific_force_body_m_s2 + ecef_to_body * gravity_ecef;

        let wind_ecef = local.ned_to_ecef() * atmosphere.wind_ned_m_s;
        let air_rel_ecef = vehicle.velocity_ecef - wind_ecef;
        let airspeed_true_m_s = air_rel_ecef.length();
        let rho = atmosphere.density_kg_m3.max(0.0);
        let differential_pressure_pa = 0.5 * rho * airspeed_true_m_s * airspeed_true_m_s;
        let airspeed_indicated_m_s =
            (2.0 * differential_pressure_pa / SEA_LEVEL_DENSITY_KG_M3).sqrt();

        let mag_ecef = local.ned_to_ecef() * DEFAULT_MAG_NED_GAUSS;
        let mag_field_body_gauss = ecef_to_body * mag_ecef;

        Ok(Self {
            vehicle,
            specific_force_body_m_s2,
            linear_accel_body_m_s2,
            static_pressure_pa: atmosphere.pressure_pa,
            temperature_k: atmosphere.temperature_k,
            differential_pressure_pa,
            mag_field_body_gauss,
            airspeed_true_m_s,
            airspeed_indicated_m_s,
        })
    }

    /// Replaces the body-frame magnetic vector (`Gauss`).
    #[must_use]
    pub fn with_mag_field_body_gauss(mut self, mag_field_body_gauss: DVec3) -> Self {
        self.mag_field_body_gauss = mag_field_body_gauss;
        self
    }

    /// Validates all fields and projects the vehicle into WGS84 geodetic and
    /// local NED/ENU frames.
    pub fn ned_kinematics(&self) -> Result<NedKinematics, XilError> {
        validate_vec3(self.vehicle.position_ecef, "vehicle.position_ecef")?;
        validate_vec3(self.vehicle.velocity_ecef, "vehicle.velocity_ecef")?;
        validate_vec3(self.vehicle.angular_rate_body, "vehicle.angular_rate_body")?;
        if !self.vehicle.attitude.is_finite() {
            return Err(XilError::InvalidStimulus {
                field: "vehicle.attitude",
            });
        }
        validate_vec3(self.specific_force_body_m_s2, "specific_force_body_m_s2")?;
        validate_vec3(self.linear_accel_body_m_s2, "linear_accel_body_m_s2")?;
        validate_vec3(self.mag_field_body_gauss, "mag_field_body_gauss")?;
        validate_scalar(self.static_pressure_pa, "static_pressure_pa")?;
        validate_scalar(self.temperature_k, "temperature_k")?;
        validate_scalar(self.differential_pressure_pa, "differential_pressure_pa")?;
        validate_scalar(self.airspeed_true_m_s, "airspeed_true_m_s")?;
        validate_scalar(self.airspeed_indicated_m_s, "airspeed_indicated_m_s")?;

        let geodetic = ecef_to_geodetic(self.vehicle.position_ecef);
        let local = LocalFrame::at(&geodetic);
        let ecef_to_ned = local.ned_to_ecef().transpose();
        let body_to_ecef = DMat3::from_quat(self.vehicle.attitude);
        let attitude_frd_to_ned = DQuat::from_mat3(&(ecef_to_ned * body_to_ecef)).normalize();
        let velocity_ned_m_s = local.ecef_to_ned_vector(self.vehicle.velocity_ecef);
        let velocity_enu_m_s = local.ecef_to_enu_vector(self.vehicle.velocity_ecef);

        Ok(NedKinematics {
            geodetic,
            attitude_frd_to_ned,
            velocity_ned_m_s,
            velocity_enu_m_s,
        })
    }
}

fn validate_vec3(v: DVec3, field: &'static str) -> Result<(), XilError> {
    if v.is_finite() {
        Ok(())
    } else {
        Err(XilError::InvalidStimulus { field })
    }
}

fn validate_scalar(v: f64, field: &'static str) -> Result<(), XilError> {
    if v.is_finite() {
        Ok(())
    } else {
        Err(XilError::InvalidStimulus { field })
    }
}
