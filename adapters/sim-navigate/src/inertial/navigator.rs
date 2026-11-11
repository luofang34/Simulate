use navigate_contract::{
    NavigationSolution, SensorClass, SourceComposition, SourceEpoch, SourceId, WrappingSequence,
};
use navigate_fusion::{
    FogmBias, ImuIncrement, InertialConfig, InertialNavigationFilter, IngestOutcome, Observation,
    PlanetaryBody,
};
use sim_core::SimInstant;
use sim_devices::{GaussMarkov, ImuErrorSpec, InertialIncrement, TriadErrorSpec};

use super::AlignmentFix;
use crate::{MasterClockBridge, NavigateBridgeError};

/// Correlation time given to a bias that only random-walks, seconds. For
/// spans well under it, a Gauss–Markov bias with the walk's density grows
/// like the walk.
const RANDOM_WALK_TAU_S: f64 = 3_600.0;

/// Navigate's inertial configuration for an IMU with the statistics `spec`,
/// from the same numbers [`super::SimulatedImu`] draws its errors from.
///
/// White-noise densities carry over directly. Each Gauss–Markov bias keeps its
/// time constant and sigma, and a constant one becomes the filter's
/// non-correlating bias. A bias random walk becomes extra Gauss–Markov
/// variance with the walk's density, `q²·τ/2`, which declares more turn-on
/// variance than the unit has. Scale factor and misalignment have no filter
/// states and are left unmodelled.
pub fn inertial_config(spec: &ImuErrorSpec) -> InertialConfig {
    InertialConfig {
        accel_noise_sigma_mps2_sqrt_hz: spec.accel.noise_density,
        gyro_noise_sigma_rad_s_sqrt_hz: spec.gyro.noise_density,
        accel_bias: filter_bias(&spec.accel),
        gyro_bias: filter_bias(&spec.gyro),
        ..InertialConfig::default()
    }
}

/// The filter's first-order Gauss–Markov bias for `triad`: its spread is
/// the turn-on constant and the in-run wander together, and its correlation
/// time is stretched so that its process noise is the in-run wander's
/// alone, `2·σ_in²/τ`. The turn-on constant adds spread and no wander.
fn filter_bias(triad: &TriadErrorSpec) -> FogmBias {
    let GaussMarkov { tau_s, sigma } = triad.bias;
    let walk = triad.bias_random_walk_density;
    let turn_on = triad.turn_on_bias_sigma * triad.turn_on_bias_sigma;
    let tau = match (tau_s.is_finite() && tau_s > 0.0, walk > 0.0) {
        (true, _) => tau_s,
        (false, true) => RANDOM_WALK_TAU_S,
        (false, false) => return FogmBias::new(0.0, (turn_on + sigma * sigma).sqrt()),
    };
    let in_run = sigma * sigma + walk * walk * tau / 2.0;
    if in_run <= 0.0 {
        return FogmBias::new(0.0, turn_on.sqrt());
    }
    FogmBias::new(tau * (turn_on + in_run) / in_run, (turn_on + in_run).sqrt())
}

/// Navigate's 22-state [`InertialNavigationFilter`] on the WGS84 Earth surface,
/// fed simulated IMU increments stamped on the master clock.
///
/// Nothing here reads simulator truth: the state starts from an
/// [`AlignmentFix`] and moves only on [`InertialIncrement`]s.
#[derive(Debug, Clone)]
pub struct InertialNavigator {
    clock: MasterClockBridge,
    filter: InertialNavigationFilter,
    source: SourceId,
    epoch: SourceEpoch,
    next: WrappingSequence,
}

impl InertialNavigator {
    /// An unaligned navigator whose IMU stream is `source`.
    pub fn new(config: InertialConfig, clock: MasterClockBridge, source: SourceId) -> Self {
        Self {
            clock,
            filter: InertialNavigationFilter::new_surface(
                config,
                clock.domain(),
                PlanetaryBody::EARTH_WGS84,
            ),
            source,
            epoch: SourceEpoch::new(0),
            next: WrappingSequence::new(0),
        }
    }

    /// The filter, for its rejection counters and configuration.
    pub const fn filter(&self) -> &InertialNavigationFilter {
        &self.filter
    }

    /// Starts the state from `fix`, valid at `at`.
    pub fn align(&mut self, fix: &AlignmentFix, at: SimInstant) -> Result<(), NavigateBridgeError> {
        self.filter
            .initialize_surface(
                fix.position,
                fix.velocity,
                fix.attitude,
                fix.position_cov,
                fix.velocity_cov,
                fix.attitude_cov,
                self.clock.to_monotonic(at)?,
                fix.composition,
            )
            .map_err(|reason| NavigateBridgeError::InertialRefused {
                stage: "alignment",
                reason,
            })
    }

    /// Propagates over `increment`, which ends at `at`. A refusal is an error:
    /// a dead-reckoning stream with a gap is no longer the one being scored.
    pub fn ingest(
        &mut self,
        increment: &InertialIncrement,
        at: SimInstant,
    ) -> Result<(), NavigateBridgeError> {
        let stamp = self.clock.stamp(self.source, self.epoch, self.next, at)?;
        self.next = self.next.next();
        let sample = ImuIncrement::new(
            stamp,
            SourceComposition::of(SensorClass::Inertial),
            increment.dt_s,
            increment.delta_theta_rad.to_array(),
            increment.delta_velocity_m_s.to_array(),
        );
        match self.filter.ingest_imu_increment(&sample, stamp.acquired_at) {
            IngestOutcome::Accepted => Ok(()),
            IngestOutcome::Rejected(reason) => Err(NavigateBridgeError::InertialRefused {
                stage: "imu increment",
                reason,
            }),
        }
    }

    /// Offers an aiding observation, such as a star sighting, at `at`.
    ///
    /// The outcome is returned, not an error: the innovation gate refusing an
    /// aid is part of what an evaluation scores, while a gap in the IMU stream
    /// is not.
    pub fn observe(
        &mut self,
        observation: &Observation,
        at: SimInstant,
    ) -> Result<IngestOutcome, NavigateBridgeError> {
        Ok(self
            .filter
            .ingest(observation, self.clock.to_monotonic(at)?))
    }

    /// The solution at `at`, or `None` before the alignment and its first
    /// increment.
    pub fn solution(
        &mut self,
        at: SimInstant,
    ) -> Result<Option<NavigationSolution>, NavigateBridgeError> {
        Ok(self.filter.tick(self.clock.to_monotonic(at)?))
    }
}
