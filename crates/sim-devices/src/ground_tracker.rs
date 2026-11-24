//! Ground-based radiometric and optical tracking measurements (`OneWayDownlink` and
//! `TwoWayCoherent`) with causal light-time iteration, exact range-rate chain rule,
//! topocentric ENU pointing, planetary occultation gating, and deterministic `TrialRng` noise.
//!
//! # Measurement Conventions
//!
//! - **One-way downlink** ([`TrackingLinkMode::OneWayDownlink`]): target transmits at `tx_epoch`,
//!   ground station receives at `rx_epoch`. `light_time_s = rx_epoch - tx_epoch`,
//!   `range_m = c * light_time_s`, `range_rate_m_s = d(range_m) / d(rx_epoch)`, and
//!   `doppler_hz = -f * range_rate_m_s / c`.
//! - **Two-way coherent** ([`TrackingLinkMode::TwoWayCoherent`]): ground station transmits at
//!   `tx_epoch`, target turns around at `bounce_epoch`, and ground station receives at `rx_epoch`.
//!   `light_time_s = rx_epoch - tx_epoch` is the total round-trip light time,
//!   `range_m = 0.5 * c * light_time_s = 0.5 * (R_up + R_down)` is the equivalent one-way range,
//!   `range_rate_m_s = d(range_m) / d(rx_epoch)` differentiates both legs by the chain rule, and
//!   `doppler_hz = -2 * f * range_rate_m_s / c` is the coherent unit-turnaround sky-frequency shift.
//! - **Topocentric pointing**: `(azimuth_rad, elevation_rad)` at `rx_epoch` in the station's local
//!   East–North–Up frame with azimuth in `[0, 2*pi)` clockwise from North.

mod solver;
mod tdm;
#[cfg(test)]
mod tests;
mod types;

use serde::{Deserialize, Serialize};
use sim_spacetime::{
    BodyOrientationCatalog, EphemerisCatalog, TdbEpoch, TranslationalTrajectorySource, TrialRng,
    TrialStreamKey, mul_add_unfused,
};

use solver::{doppler_from_range_rate, evaluate_truth, wrap_azimuth_rad};

pub use tdm::{
    GroundTrackerTdm, GroundTrackerTdmError, GroundTrackerTdmMeta, TdmImportedObservation,
    TdmRangeExportUnit,
};
pub use types::{
    GroundTrackerError, GroundTrackerStationSpec, TrackingErrorSpec, TrackingLinkMode,
    TrackingMeasurement, TrackingTruthMeasurement,
};

/// Ground tracking station model producing causal truth and deterministic noisy observations.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundTracker {
    station: GroundTrackerStationSpec,
    error: TrackingErrorSpec,
    sample_counter: u64,
}

impl GroundTracker {
    /// Validates `station` and `error` specifications and constructs a tracker starting at sample `0`.
    pub fn new(
        station: GroundTrackerStationSpec,
        error: TrackingErrorSpec,
    ) -> Result<Self, GroundTrackerError> {
        station.validate()?;
        error.validate()?;
        Ok(Self {
            station,
            error,
            sample_counter: 0,
        })
    }

    /// Returns the station specification.
    #[must_use]
    pub const fn station(&self) -> &GroundTrackerStationSpec {
        &self.station
    }

    /// Returns the bias and noise specification.
    #[must_use]
    pub const fn error(&self) -> &TrackingErrorSpec {
        &self.error
    }

    /// Returns the sequential sample index to be used by the next [`Self::measure_noisy`] call.
    #[must_use]
    pub const fn sample_counter(&self) -> u64 {
        self.sample_counter
    }

    /// Sets the sequential sample index used by [`Self::measure_noisy`].
    pub const fn set_sample_counter(&mut self, sample_counter: u64) {
        self.sample_counter = sample_counter;
    }

    /// Evaluates an exact causal light-time ground tracking measurement at `rx_epoch`,
    /// enforcing the station elevation mask and configured planetary occultation checks.
    pub fn measure_truth(
        &self,
        target: &dyn TranslationalTrajectorySource,
        rx_epoch: TdbEpoch,
        mode: TrackingLinkMode,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<TrackingTruthMeasurement, GroundTrackerError> {
        evaluate_truth(
            &self.station,
            target,
            rx_epoch,
            mode,
            ephemeris,
            orientation,
            true,
        )
    }

    /// Evaluates exact causal light-time truth at `rx_epoch` without rejecting observations below
    /// `station.min_elevation_rad` (occultation checks still apply).
    pub fn measure_truth_unmasked(
        &self,
        target: &dyn TranslationalTrajectorySource,
        rx_epoch: TdbEpoch,
        mode: TrackingLinkMode,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<TrackingTruthMeasurement, GroundTrackerError> {
        evaluate_truth(
            &self.station,
            target,
            rx_epoch,
            mode,
            ephemeris,
            orientation,
            false,
        )
    }

    /// Evaluates truth and applies station bias and deterministic Gaussian noise using the
    /// tracker's sequential `sample_counter` (incremented via `wrapping_add(1)` on success).
    pub fn measure_noisy(
        &mut self,
        target: &dyn TranslationalTrajectorySource,
        rx_epoch: TdbEpoch,
        mode: TrackingLinkMode,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<TrackingMeasurement, GroundTrackerError> {
        let truth = self.measure_truth(target, rx_epoch, mode, ephemeris, orientation)?;
        let sample_index = self.sample_counter;
        self.sample_counter = self.sample_counter.wrapping_add(1);
        Ok(self.apply_noise(truth, mode, sample_index))
    }

    /// Evaluates unmasked truth and applies station bias and deterministic Gaussian noise using
    /// the sequential `sample_counter` (incremented via `wrapping_add(1)` on success).
    pub fn measure_noisy_unmasked(
        &mut self,
        target: &dyn TranslationalTrajectorySource,
        rx_epoch: TdbEpoch,
        mode: TrackingLinkMode,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<TrackingMeasurement, GroundTrackerError> {
        let truth = self.measure_truth_unmasked(target, rx_epoch, mode, ephemeris, orientation)?;
        let sample_index = self.sample_counter;
        self.sample_counter = self.sample_counter.wrapping_add(1);
        Ok(self.apply_noise(truth, mode, sample_index))
    }

    /// Evaluates truth and applies deterministic Gaussian noise keyed by an explicit `sample_index`
    /// via [`TrialStreamKey`], allowing order-independent random-access replay across native and `wasm32`.
    pub fn measure_noisy_at_sample(
        &self,
        sample_index: u64,
        target: &dyn TranslationalTrajectorySource,
        rx_epoch: TdbEpoch,
        mode: TrackingLinkMode,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<TrackingMeasurement, GroundTrackerError> {
        let truth = self.measure_truth(target, rx_epoch, mode, ephemeris, orientation)?;
        Ok(self.apply_noise(truth, mode, sample_index))
    }

    /// Evaluates unmasked truth and applies deterministic Gaussian noise keyed by `sample_index`.
    pub fn measure_noisy_unmasked_at_sample(
        &self,
        sample_index: u64,
        target: &dyn TranslationalTrajectorySource,
        rx_epoch: TdbEpoch,
        mode: TrackingLinkMode,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<TrackingMeasurement, GroundTrackerError> {
        let truth = self.measure_truth_unmasked(target, rx_epoch, mode, ephemeris, orientation)?;
        Ok(self.apply_noise(truth, mode, sample_index))
    }

    fn apply_noise(
        &self,
        truth: TrackingTruthMeasurement,
        mode: TrackingLinkMode,
        sample_index: u64,
    ) -> TrackingMeasurement {
        let mut rng = TrialRng::from_key(TrialStreamKey::new(
            self.error.rng_seed,
            sample_index,
            self.error.stream_tag,
        ));
        let z_range = rng.standard_normal();
        let z_rate = rng.standard_normal();
        let z_az = rng.standard_normal();
        let z_el = rng.standard_normal();

        let range_m = perturb(
            truth.range_m,
            self.error.range_bias_m,
            self.error.range_sigma_m,
            z_range,
        );
        let range_rate_m_s = perturb(
            truth.range_rate_m_s,
            self.error.range_rate_bias_m_s,
            self.error.range_rate_sigma_m_s,
            z_rate,
        );
        let doppler_hz =
            doppler_from_range_rate(self.station.carrier_frequency_hz, range_rate_m_s, mode);
        let azimuth_rad = wrap_azimuth_rad(perturb(
            truth.azimuth_rad,
            self.error.azimuth_bias_rad,
            self.error.azimuth_sigma_rad,
            z_az,
        ));
        let elevation_rad = perturb(
            truth.elevation_rad,
            self.error.elevation_bias_rad,
            self.error.elevation_sigma_rad,
            z_el,
        );
        let below_mask = elevation_rad < self.station.min_elevation_rad;

        TrackingMeasurement {
            truth,
            range_m,
            range_rate_m_s,
            doppler_hz,
            azimuth_rad,
            elevation_rad,
            below_mask,
        }
    }
}

fn perturb(truth_val: f64, bias: f64, sigma: f64, z: f64) -> f64 {
    if bias == 0.0 && sigma == 0.0 {
        truth_val
    } else if sigma == 0.0 {
        truth_val + bias
    } else {
        mul_add_unfused(sigma, z, truth_val + bias)
    }
}
