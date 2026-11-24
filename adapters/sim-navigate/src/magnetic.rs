//! Magnetic field sensing and crustal anomaly map linearization for Navigate.
//!
//! [`SimulatedMagnetometer`] reads the world's true [`MagneticField`] at the
//! vehicle's state through a seeded [`Magnetometer`]. [`MagneticAnomalyMap`]
//! holds the onboard main-field reference and surveyed crustal overlays,
//! extracts scalar magnetic anomaly observations against the navigator's own
//! position estimate, and labels each observation with
//! [`SensorClass::MagneticAnomaly`].

use std::sync::Arc;

use glam::DVec3;
use navigate_contract::{
    GeodeticPosition, SensorClass, SourceComposition, SourceEpoch, SourceId, WrappingSequence,
};
use navigate_fusion::{Observation, ObservationValue};
use sim_bodies::{AnomalyOverlay, MagnetismWithAnomaly};
use sim_core::{SimInstant, VehicleState};
use sim_devices::{Magnetometer, MagnetometerSpec};
use sim_geo::{Geodetic, LocalFrame, MagneticField, MagneticFieldError, geodetic_to_ecef};

use crate::{Estimate, MasterClockBridge, NavigateBridgeError};

#[cfg(test)]
mod tests;

const NANOTESLA_PER_MICROTESLA: f64 = 1_000.0;
const MICROTESLA_TO_TESLA: f64 = 1e-6;
const NANOTESLA_TO_TESLA: f64 = 1e-9;

/// A three-axis magnetometer sensing a simulated world's magnetic field.
#[derive(Debug, Clone)]
pub struct SimulatedMagnetometer {
    world_field: Arc<dyn MagneticField>,
    device: Magnetometer,
    decimal_year: f64,
}

impl SimulatedMagnetometer {
    /// Builds a magnetometer reading `world_field` on `decimal_year` with
    /// errors drawn from `spec` and `seed`.
    pub fn new(
        spec: MagnetometerSpec,
        seed: u64,
        decimal_year: f64,
        world_field: Arc<dyn MagneticField>,
    ) -> Result<Self, NavigateBridgeError> {
        if !decimal_year.is_finite() {
            return Err(NavigateBridgeError::InvalidMagneticConfig {
                field: "decimal_year",
                value: decimal_year,
            });
        }
        let device =
            Magnetometer::new(spec, seed).map_err(NavigateBridgeError::MagnetometerModel)?;
        Ok(Self {
            world_field,
            device,
            decimal_year,
        })
    }

    /// The world magnetic field the magnetometer senses.
    pub fn world_field(&self) -> &Arc<dyn MagneticField> {
        &self.world_field
    }

    /// The underlying error model, for inspecting hard-iron offsets.
    pub const fn device(&self) -> &Magnetometer {
        &self.device
    }

    /// Samples the magnetic field in body axes at `truth`, microtesla.
    pub fn sense_body_ut(&mut self, truth: &VehicleState) -> Result<DVec3, NavigateBridgeError> {
        let fixed_nt = self
            .world_field
            .field_fixed_nt(truth.position_ecef, self.decimal_year)
            .map_err(NavigateBridgeError::MagneticField)?;
        let body_ut = (truth.attitude.inverse() * fixed_nt) / NANOTESLA_PER_MICROTESLA;
        Ok(self.device.sense(body_ut))
    }

    /// Samples the total scalar magnetic intensity at `truth`, tesla.
    pub fn sense_total_field_t(
        &mut self,
        truth: &VehicleState,
    ) -> Result<f64, NavigateBridgeError> {
        Ok(self.sense_body_ut(truth)?.length() * MICROTESLA_TO_TESLA)
    }
}

/// Configuration for [`MagneticAnomalyMap`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagneticAnomalyConfig {
    /// Decimal year at which the geomagnetic main field is evaluated.
    pub decimal_year: f64,
    /// Radius around each linearization point where the tangent linear model
    /// holds, metres.
    pub valid_radius_m: f64,
    /// Declared observation variance (sensor plus map uncertainty), tesla squared.
    pub variance_t2: f64,
    /// Constant survey-sheet leveling offset on the onboard map, tesla.
    pub bias_offset_t: f64,
    /// Symmetric finite-difference step for the local NED gradient, metres.
    pub gradient_step_m: f64,
}

impl Default for MagneticAnomalyConfig {
    fn default() -> Self {
        Self {
            decimal_year: 2026.5,
            valid_radius_m: 600.0,
            variance_t2: 9e-18,
            bias_offset_t: 0.0,
            gradient_step_m: 25.0,
        }
    }
}

impl MagneticAnomalyConfig {
    fn validate(&self) -> Result<(), NavigateBridgeError> {
        let finite_check = [
            ("decimal_year", self.decimal_year),
            ("bias_offset_t", self.bias_offset_t),
        ];
        if let Some(&(field, value)) = finite_check.iter().find(|(_, v)| !v.is_finite()) {
            return Err(NavigateBridgeError::InvalidMagneticConfig { field, value });
        }
        let positive_check = [
            ("valid_radius_m", self.valid_radius_m),
            ("variance_t2", self.variance_t2),
            ("gradient_step_m", self.gradient_step_m),
        ];
        if let Some(&(field, value)) = positive_check
            .iter()
            .find(|(_, v)| !(v.is_finite() && *v > 0.0))
        {
            return Err(NavigateBridgeError::InvalidMagneticConfig { field, value });
        }
        Ok(())
    }
}

/// Onboard geomagnetic main-field reference and surveyed crustal anomaly map
/// producing [`ObservationValue::MagneticAnomaly`] observations for Navigate.
#[derive(Debug, Clone)]
pub struct MagneticAnomalyMap {
    main_field: Arc<dyn MagneticField>,
    survey_field: MagnetismWithAnomaly,
    config: MagneticAnomalyConfig,
    clock: MasterClockBridge,
    source: SourceId,
    epoch: SourceEpoch,
    next: WrappingSequence,
    map_revision: Option<u64>,
}

impl MagneticAnomalyMap {
    /// Builds an onboard map from a global `main_field` and surveyed crustal
    /// `overlays`.
    pub fn new(
        main_field: Arc<dyn MagneticField>,
        overlays: Vec<AnomalyOverlay>,
        config: MagneticAnomalyConfig,
        clock: MasterClockBridge,
        source: SourceId,
        epoch: SourceEpoch,
    ) -> Result<Self, NavigateBridgeError> {
        config.validate()?;
        let survey_field = MagnetismWithAnomaly {
            base: Arc::clone(&main_field),
            overlays,
        };
        Ok(Self {
            main_field,
            survey_field,
            config,
            clock,
            source,
            epoch,
            next: WrappingSequence::new(0),
            map_revision: None,
        })
    }

    /// Sets the active survey sheet revision and its constant leveling offset,
    /// tesla.
    pub fn set_map_revision(
        &mut self,
        revision: Option<u64>,
        bias_offset_t: f64,
    ) -> Result<(), NavigateBridgeError> {
        if !bias_offset_t.is_finite() {
            return Err(NavigateBridgeError::InvalidMagneticConfig {
                field: "bias_offset_t",
                value: bias_offset_t,
            });
        }
        self.map_revision = revision;
        self.config.bias_offset_t = bias_offset_t;
        Ok(())
    }

    /// Evaluates the scalar crustal anomaly at ECEF `fixed` in tesla, or
    /// `Ok(None)` when outside a refusing overlay grid's coverage.
    pub fn crustal_scalar_t(&self, fixed: DVec3) -> Result<Option<f64>, NavigateBridgeError> {
        let main_nt = match self
            .main_field
            .field_fixed_nt(fixed, self.config.decimal_year)
        {
            Ok(field) => field,
            Err(MagneticFieldError::OutsideDomain { .. }) => return Ok(None),
            Err(error) => return Err(NavigateBridgeError::MagneticField(error)),
        };
        let survey_nt = match self
            .survey_field
            .field_fixed_nt(fixed, self.config.decimal_year)
        {
            Ok(field) => field,
            Err(MagneticFieldError::OutsideDomain { .. }) => return Ok(None),
            Err(error) => return Err(NavigateBridgeError::MagneticField(error)),
        };
        Ok(Some(
            (survey_nt.length() - main_nt.length()) * NANOTESLA_TO_TESLA
                + self.config.bias_offset_t,
        ))
    }

    /// Linearizes the anomaly map at `estimated_position` and turns a scalar
    /// total-field magnetometer measurement `measured_total_field_t` into a
    /// Navigate [`Observation`], or `Ok(None)` when outside survey coverage.
    pub fn observe_total_field(
        &mut self,
        at: SimInstant,
        estimated_position: &Estimate<GeodeticPosition>,
        measured_total_field_t: f64,
    ) -> Result<Option<Observation>, NavigateBridgeError> {
        let lin = *estimated_position.get();
        let geo = Geodetic {
            lat_rad: lin.latitude_rad,
            lon_rad: lin.longitude_rad,
            height_m: lin.altitude_m,
        };
        let ecef = geodetic_to_ecef(&geo);
        let main_lin_nt = match self
            .main_field
            .field_fixed_nt(ecef, self.config.decimal_year)
        {
            Ok(field) => field,
            Err(MagneticFieldError::OutsideDomain { .. }) => return Ok(None),
            Err(error) => return Err(NavigateBridgeError::MagneticField(error)),
        };
        let Some(map_t_at_lin) = self.crustal_scalar_t(ecef)? else {
            return Ok(None);
        };
        let Some(gradient_ned_t_per_m) = self.gradient_ned(&geo, ecef)? else {
            return Ok(None);
        };
        let measured_t = measured_total_field_t - main_lin_nt.length() * NANOTESLA_TO_TESLA;
        let stamp = self.clock.stamp(self.source, self.epoch, self.next, at)?;
        self.next = self.next.next();
        let value = ObservationValue::MagneticAnomaly {
            measured_t,
            map_t_at_lin,
            gradient_ned_t_per_m,
            linearization_point: lin,
            valid_radius_m: self.config.valid_radius_m,
            variance_t2: self.config.variance_t2,
        };
        let mut observation = Observation::new(
            stamp,
            value,
            SourceComposition::of(SensorClass::MagneticAnomaly),
        );
        if let Some(revision) = self.map_revision {
            observation = observation.with_map_revision(revision);
        }
        Ok(Some(observation))
    }

    fn gradient_ned(
        &self,
        geo: &Geodetic,
        ecef: DVec3,
    ) -> Result<Option<[f64; 3]>, NavigateBridgeError> {
        let frame = LocalFrame::at(geo);
        let step = self.config.gradient_step_m;
        let mut out = [0.0; 3];
        let axes = [frame.north(), frame.east(), -frame.up()];
        for (slot, axis) in out.iter_mut().zip(axes) {
            let Some(plus) = self.crustal_scalar_t(ecef + axis * step)? else {
                return Ok(None);
            };
            let Some(minus) = self.crustal_scalar_t(ecef - axis * step)? else {
                return Ok(None);
            };
            *slot = (plus - minus) / (2.0 * step);
        }
        Ok(Some(out))
    }
}
