use glam::{DMat3, DQuat, DVec3};
use navigate_contract::{
    AttitudeQuaternion, GeodeticPosition, NedVelocity, SensorClass, SourceComposition,
    SymmetricCov3,
};
use navigate_onboard::ImuBiasPrior;
use sim_core::VehicleState;
use sim_devices::WhiteNoise;
use sim_geo::{LocalFrame, ecef_to_geodetic};

/// Smallest declared 1-sigma of each part of the alignment. Navigate needs a
/// positive-definite prior even from an exact alignment; the attitude floor
/// matches the filter's own default floor.
const POSITION_FLOOR_M: f64 = 1e-3;
const VELOCITY_FLOOR_M_S: f64 = 1e-4;
const ATTITUDE_FLOOR_RAD: f64 = 1e-6;

/// The initial alignment of an inertial navigator, modelled as a sensor.
///
/// It is the one place a filter's state comes from simulator truth, so it
/// carries declared errors: each part of the truth is offset by a seeded
/// Gaussian draw of the stated 1-sigma, and the fix declares exactly that
/// sigma, never less than a small floor, as its covariance. Position and
/// velocity errors are in local north, east, down; the attitude error is a
/// rotation vector in body axes, the right perturbation Navigate's error
/// state uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitialAlignment {
    /// Position error 1-sigma, NED metres.
    pub position_sigma_m: DVec3,
    /// Velocity error 1-sigma, NED m/s.
    pub velocity_sigma_m_s: DVec3,
    /// Attitude error 1-sigma about each body axis, radians.
    pub attitude_sigma_rad: DVec3,
    /// Seed of the error draw.
    pub seed: u64,
}

impl InitialAlignment {
    /// An alignment with no error, declaring only the covariance floor.
    pub const EXACT: Self = Self {
        position_sigma_m: DVec3::ZERO,
        velocity_sigma_m_s: DVec3::ZERO,
        attitude_sigma_rad: DVec3::ZERO,
        seed: 0,
    };

    /// The fix this alignment gives for a vehicle in `truth`.
    pub fn measure(&self, truth: &VehicleState) -> AlignmentFix {
        let mut noise = WhiteNoise::from_seed(self.seed);
        let position_error = noise.next_vec3(self.position_sigma_m);
        let velocity_error = noise.next_vec3(self.velocity_sigma_m_s);
        let attitude_error = noise.next_vec3(self.attitude_sigma_rad);

        let frame = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef));
        let ned_to_ecef = frame.ned_to_ecef();
        let position = ecef_to_geodetic(truth.position_ecef + ned_to_ecef * position_error);
        let velocity = frame.ecef_to_ned_vector(truth.velocity_ecef) + velocity_error;
        let body_to_ned = body_to_ned(&frame, truth.attitude);
        let measured = (body_to_ned * DQuat::from_scaled_axis(attitude_error)).normalize();

        let attitude_variance = floored(self.attitude_sigma_rad, ATTITUDE_FLOOR_RAD);
        let rotation = DMat3::from_quat(body_to_ned);
        let attitude_ned =
            rotation * DMat3::from_diagonal(attitude_variance) * rotation.transpose();
        AlignmentFix {
            position: GeodeticPosition::new(position.lat_rad, position.lon_rad, position.height_m),
            velocity: NedVelocity::new(velocity.x, velocity.y, velocity.z),
            attitude: AttitudeQuaternion::new(measured.w, measured.x, measured.y, measured.z),
            position_cov: diagonal(floored(self.position_sigma_m, POSITION_FLOOR_M)),
            velocity_cov: diagonal(floored(self.velocity_sigma_m_s, VELOCITY_FLOOR_M_S)),
            attitude_cov: symmetric(&attitude_ned),
            composition: SourceComposition::of(SensorClass::Gnss).with(SensorClass::Inertial),
            imu_bias_prior: None,
        }
    }
}

/// The rotation from the vehicle's body axes to local NED, given its
/// body-to-ECEF `attitude`.
pub(crate) fn body_to_ned(frame: &LocalFrame, attitude: DQuat) -> DQuat {
    (DQuat::from_mat3(&frame.ned_to_ecef()).inverse() * attitude).normalize()
}

fn floored(sigma: DVec3, floor: f64) -> DVec3 {
    let sigma = sigma.max(DVec3::splat(floor));
    sigma * sigma
}

fn diagonal(variance: DVec3) -> SymmetricCov3 {
    SymmetricCov3::from_diagonal(variance.x, variance.y, variance.z)
}

fn symmetric(matrix: &DMat3) -> SymmetricCov3 {
    let [x, y, z] = [matrix.x_axis, matrix.y_axis, matrix.z_axis];
    SymmetricCov3::from_upper_triangle([x.x, y.x, z.x, y.y, z.y, z.z])
}

/// One initial alignment: the state Navigate's surface initialisation takes,
/// with its declared covariances.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlignmentFix {
    /// Position on WGS84.
    pub position: GeodeticPosition,
    /// Velocity, local NED.
    pub velocity: NedVelocity,
    /// Body-to-NED rotation, body axes forward, right, down.
    pub attitude: AttitudeQuaternion,
    /// Position covariance, NED m².
    pub position_cov: SymmetricCov3,
    /// Velocity covariance, NED (m/s)².
    pub velocity_cov: SymmetricCov3,
    /// Attitude covariance in local NED axes, rad².
    pub attitude_cov: SymmetricCov3,
    /// Declared provenance: a surveyed position and an inertial levelling.
    pub composition: SourceComposition,
    /// The IMU biases a calibration before the start measured, with their
    /// covariances; `None` starts the filter's biases at zero with their
    /// stationary variances.
    pub imu_bias_prior: Option<ImuBiasPrior>,
}
