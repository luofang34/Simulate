use glam::DVec3;
use navigate_contract::SymmetricCov3;
use navigate_onboard::ImuBiasPrior;
use sim_devices::{ImuErrorSpec, TriadErrorSpec, WhiteNoise};
use sim_geo::BodyEllipsoid;

use super::inertial_config;

/// Offset of the residual's stream from the alignment seed, so the residual
/// does not repeat the alignment's own draws.
const RESIDUAL_STREAM: u64 = 0x6772_6f75_6e64;

/// A gyroscope calibration at rest on the ground before an in-air start,
/// modelled as a sensor: what the unit's mean rate over the rest window,
/// less the Earth rate, tells the filter about its biases.
///
/// At rest the gyroscopes sense their bias plus the Earth's spin. Taking the
/// spin off needs the latitude, which the surveyed start gives, and the
/// attitude: the levelling from the accelerometers and a heading of
/// [`Self::heading_sigma_rad`], from the magnetometer at rest. The heading
/// error turns the spin's horizontal part into the estimate.
///
/// A flight that starts in the air never had that window, yet a real one
/// would have. Without it, a consumer part's turn-on bias of a fraction of
/// a degree per second is left to the filter's stationary spread, and an
/// unaided set turns its heading away by degrees a minute.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundCalibration {
    /// Length of the rest window, seconds.
    pub window_s: f64,
    /// Seed of the residual draw.
    pub seed: u64,
    /// 1-sigma of the heading the calibration takes the Earth's spin off
    /// with, radians.
    pub heading_sigma_rad: f64,
}

impl GroundCalibration {
    /// A minute at rest, the run-up a crew waits through anyway.
    pub const WINDOW_S: f64 = 60.0;

    /// A magnetometer heading at rest after a field calibration, 1-sigma, 2°:
    /// an assumed value for the class, with declination from a field model.
    pub const MAGNETIC_HEADING_SIGMA_RAD: f64 = 2.0 * core::f64::consts::PI / 180.0;

    /// A minute at rest with a magnetic heading, the residual drawn from
    /// `seed`.
    pub const fn at_rest(seed: u64) -> Self {
        Self {
            window_s: Self::WINDOW_S,
            seed,
            heading_sigma_rad: Self::MAGNETIC_HEADING_SIGMA_RAD,
        }
    }

    /// The prior a unit of statistics `spec` whose gyroscopes carry
    /// `gyro_bias_rad_s` at the start gives the filter.
    ///
    /// The gyroscope bias is the true one plus a seeded residual of
    /// variance, per axis,
    ///
    /// `σ_res² = N_g²/T + 2·σ_in²·(1 − e^(−T/τ_in)) + q²·T + (Ω·σ_ψ)²`,
    ///
    /// the white noise `N_g` averaged over the window `T`, the in-run
    /// wander `σ_in`, `τ_in` between the window and the start, the bias
    /// random walk `q` over the same span, and the Earth's spin `Ω` turned
    /// by the heading error `σ_ψ`, taken whole on every axis.
    ///
    /// The accelerometer bias is not separable from the tilt at rest, so
    /// its prior is zero with the filter's own declared variance, as
    /// without a calibration. `None` for a unit with no gyroscope error to
    /// state.
    pub fn prior(&self, spec: &ImuErrorSpec, gyro_bias_rad_s: DVec3) -> Option<ImuBiasPrior> {
        let residual_var = self.residual_variance(&spec.gyro);
        let accel_sigma = inertial_config(spec).accel_bias.sigma();
        let accel_var = accel_sigma * accel_sigma;
        let usable = |v: f64| v.is_finite() && v > 0.0;
        if !(usable(residual_var) && usable(accel_var)) {
            return None;
        }
        let mut noise = WhiteNoise::from_seed(self.seed.wrapping_add(RESIDUAL_STREAM));
        let residual = noise.next_vec3(DVec3::splat(residual_var.sqrt()));
        Some(ImuBiasPrior::new(
            [0.0; 3],
            SymmetricCov3::from_diagonal(accel_var, accel_var, accel_var),
            (gyro_bias_rad_s + residual).to_array(),
            SymmetricCov3::from_diagonal(residual_var, residual_var, residual_var),
        ))
    }

    /// `σ_res²` of [`Self::prior`] for the gyroscope triad `gyro`, (rad/s)².
    pub fn residual_variance(&self, gyro: &TriadErrorSpec) -> f64 {
        let t = self.window_s;
        let noise = gyro.noise_density * gyro.noise_density / t;
        let wander = match gyro.bias.tau_s.is_finite() && gyro.bias.tau_s > 0.0 {
            true => 2.0 * gyro.bias.sigma * gyro.bias.sigma * (1.0 - (-t / gyro.bias.tau_s).exp()),
            false => 0.0,
        };
        let walk = gyro.bias_random_walk_density * gyro.bias_random_walk_density * t;
        let spin = BodyEllipsoid::WGS84.omega_rad_s * self.heading_sigma_rad;
        noise + wander + walk + spin * spin
    }
}
