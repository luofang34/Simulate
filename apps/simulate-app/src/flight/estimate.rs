//! What the aircraft itself believes: the onboard sensor bus's navigation solution and
//! sensed instruments, which every controller and display reads in place of truth.
//!
//! The Apollo scenarios do not use the bus: those spacecraft carry their own
//! guidance-computer instruments (attitude to J2000, not ECEF), so they read truth here.

use glam::DVec3;
use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::VehicleState;
use sim_geo::BodyEllipsoid;
use sim_indicate::AirDataReading;
use sim_scenario::ScenarioProfile;

use super::Flight;
use crate::chain_avionics;
use crate::sensor_bus::{
    OnboardSensorBus, SensorErrors, SensorRealism, sensed_cockpit, sensed_specific_force,
};
use crate::world_surface::WorldSurfaceHandle;

#[cfg(test)]
mod tests;

/// Every onboard reading of one display frame.
#[derive(Debug, Clone, Copy)]
pub struct OnboardSensorSnapshot {
    /// Navigation solution (GNSS/INS position and velocity, AHRS attitude and rates).
    pub estimated_vehicle: VehicleState,
    /// Pitot-static air data, `None` above the static port's floor or off Earth.
    pub air_data: Option<AirDataReading>,
    /// Cockpit instruments with attitude and rate gyros reading the navigation solution.
    pub cockpit: CockpitInstrumentSnapshot,
    /// Accelerometer specific force, body FRD, m/s², when the plant models it.
    pub specific_force_frd_m_s2: Option<DVec3>,
}

impl Flight {
    /// The world surface this flight's ground contact and spawns read.
    #[must_use]
    pub const fn world_surface(&self) -> &WorldSurfaceHandle {
        &self.surface
    }

    /// Active onboard sensor realism.
    #[must_use]
    pub const fn sensor_realism(&self) -> SensorRealism {
        self.sensors.realism()
    }

    /// Switches to `realism`, restarting the sensor errors from their seed.
    pub fn set_sensor_realism(&mut self, realism: SensorRealism) {
        self.sensors = OnboardSensorBus::new(realism);
    }

    /// Cycles to the next [`SensorRealism`] and returns it.
    pub fn cycle_sensor_realism(&mut self) -> SensorRealism {
        let next = self.sensors.realism().next();
        self.set_sensor_realism(next);
        next
    }

    /// Sensor errors the vehicle flies with, or `None` where the bus does not apply.
    fn sensor_errors(&self, display: bool) -> Option<SensorErrors> {
        let apollo = matches!(
            self.profile(),
            Some(ScenarioProfile::Apollo11LunarLanding | ScenarioProfile::Apollo11CapsuleReentry)
        );
        if apollo || self.world().body() != BodyEllipsoid::WGS84 {
            return None;
        }
        Some(if display {
            self.sensors.errors_at(self.display_phase())
        } else {
            self.sensors.current()
        })
    }

    /// Navigation solution at the latest physics step, never raw truth unless
    /// [`SensorRealism::Ideal`].
    #[must_use]
    pub fn estimated_vehicle_state(&self) -> VehicleState {
        if let Some((_, held)) = self.chain_avionics() {
            return held;
        }
        let truth = self.vehicle_state();
        let body = self.world().body();
        self.sensor_errors(false)
            .map_or(truth, |errors| errors.estimate(body, &truth))
    }

    /// Navigation solution as displays draw it: [`Self::display_vehicle_state`] with the
    /// sensor errors interpolated by the same sub-step phase, so it is as smooth as the
    /// window yet keeps the estimator's offset from truth.
    #[must_use]
    pub fn display_estimated_vehicle_state(&self) -> VehicleState {
        if let Some((_, held)) = self.chain_avionics() {
            return held;
        }
        let truth = self.display_vehicle_state();
        let body = self.world().body();
        self.sensor_errors(true)
            .map_or(truth, |errors| errors.estimate(body, &truth))
    }

    /// Every onboard reading for the current display frame.
    #[must_use]
    pub fn onboard_sensors(&self) -> OnboardSensorSnapshot {
        let cockpit = self.cockpit();
        // On the real chain the flight controller's own account is the onboard reading.
        if let Some((avionics, held)) = self.chain_avionics() {
            return OnboardSensorSnapshot {
                estimated_vehicle: held,
                air_data: chain_avionics::air_data_reading(&avionics),
                cockpit: sensed_cockpit(cockpit, &held),
                specific_force_frd_m_s2: avionics.specific_force_body_m_s2,
            };
        }
        let truth = self.display_vehicle_state();
        let force = self.specific_force_frd_m_s2();
        let Some(errors) = self.sensor_errors(true) else {
            return OnboardSensorSnapshot {
                estimated_vehicle: truth,
                air_data: None,
                cockpit,
                specific_force_frd_m_s2: force,
            };
        };
        let estimated_vehicle = self.display_estimated_vehicle_state();
        let now = self.now();
        let mut air_data = self
            .sim_epoch()
            .utc_at(now)
            .ok()
            .and_then(|utc| self.sensors.air_data(now, utc, &truth, &errors));
        if let (Some(reading), Some(pec)) = (air_data.as_mut(), self.pitot_static_calibration()) {
            let ias_m_s = pec.cas_to_ias_at_flap_deg_m_s(
                self.articulation().flap_deg,
                reading.value.calibrated_airspeed_m_s(),
            );
            reading.value = reading.value.with_pec_indicated_airspeed_m_s(ias_m_s);
        }
        OnboardSensorSnapshot {
            estimated_vehicle,
            air_data,
            cockpit: sensed_cockpit(cockpit, &estimated_vehicle),
            specific_force_frd_m_s2: sensed_specific_force(force, &errors),
        }
    }
}
