//! The JSBSim Cessna 172 as a flight controller's airframe, starting in
//! trimmed level flight.
//!
//! The four actuator lanes are throttle, aileron, elevator and rudder with
//! the surfaces neutral at one half. Until the flight controller is armed
//! the airframe holds the controls JSBSim trimmed it with, as a pilot would
//! before engaging an autopilot. JSBSim takes its density and temperature
//! from its own standard atmosphere and its wind from the shared air field,
//! so the aeroplane drifts with the same wind its sensors read.

use glam::DVec3;
use sim_atmosphere::AtmosphereField;
use sim_aviate::{DirectActuatorCmd, direct_fixed_wing_controls};
use sim_core::{
    Block, ControlInputs, EngineStartMode, JsbsimFdmBlock, JsbsimFdmConfig, KinematicInput,
    TickSpan, VehicleState,
};
use sim_devices::SensedTruth;
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime, ecef_to_geodetic};
use sim_pack::{LoadedSyspack, PackError, builtin_c172p_package, builtin_c172s_package};

use crate::airframe::specific_force_body_m_s2;
use crate::{Airframe, ChainError, SharedAir};

#[cfg(test)]
mod tests;

/// Angle of attack the trim search starts from, radians.
const TRIM_SEARCH_ALPHA_RAD: f64 = 0.035;

/// Cessna 172 sub-variant flown by [`C172Airframe`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum C172Variant {
    /// Cessna 172S Skyhawk SP (`vehicles/c172s`, 180 BHP IO-360-L2A).
    #[default]
    C172s,
    /// Cessna 172P Skyhawk II / Reims F172P (`vehicles/c172p`, 160 BHP O-320-D2J).
    C172p,
}

impl C172Variant {
    /// Loads and SHA-256-verifies the built-in syspack for this variant.
    pub fn load_package(self) -> Result<LoadedSyspack, PackError> {
        match self {
            Self::C172s => builtin_c172s_package(),
            Self::C172p => builtin_c172p_package(),
        }
    }
}

/// Where and how fast the aeroplane starts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct C172Start {
    /// Starting position.
    pub position: Geodetic,
    /// Heading from true north, radians.
    pub heading_rad: f64,
    /// True airspeed, m/s.
    pub speed_m_s: f64,
    /// Integration period.
    pub period: TickSpan,
    /// Instant of the start, for the air field.
    pub utc: UtcTime,
}

/// The JSBSim C172 and the air its sensors sit in.
pub struct C172Airframe {
    fdm: JsbsimFdmBlock,
    air: SharedAir,
    utc: UtcTime,
    trim: ControlInputs,
    specific_force_body_m_s2: DVec3,
}

impl C172Airframe {
    /// The Cessna 172S Skyhawk SP trimmed for level flight at `start`, engine running.
    pub fn trimmed(start: &C172Start, air: SharedAir) -> Result<Self, ChainError> {
        Self::trimmed_variant(C172Variant::C172s, start, air)
    }

    /// The selected Cessna 172 `variant` trimmed for level flight at `start`, engine running.
    pub fn trimmed_variant(
        variant: C172Variant,
        start: &C172Start,
        air: SharedAir,
    ) -> Result<Self, ChainError> {
        let package = variant
            .load_package()
            .map_err(|e| ChainError::Package(Box::new(e)))?;
        let resolver = package
            .build_jsbsim_resolver()
            .map_err(|e| ChainError::Package(Box::new(e)))?;
        let (aircraft_path, model) = package
            .jsbsim_model()
            .map_err(|e| ChainError::Package(Box::new(e)))?;
        let mut fdm = JsbsimFdmBlock::new(&JsbsimFdmConfig {
            resolver: &resolver,
            aircraft_path,
            model,
            period: start.period,
            initial: start.position,
            heading_rad: start.heading_rad,
            speed_m_s: start.speed_m_s,
            alpha_rad: TRIM_SEARCH_ALPHA_RAD,
            engine_start: EngineStartMode::Running,
        })
        .map_err(|e| ChainError::Plant(Box::new(e)))?;
        set_wind(&mut fdm, &air, start.utc)?;
        let trim = fdm
            .trim_level_flight()
            .map_err(|e| ChainError::Trim(Box::new(e)))?;
        let state = fdm.output();
        // Unaccelerated flight: the specific force is the reaction to weight.
        let level = specific_force_body_m_s2(&state, &state, start.period.as_seconds());
        Ok(Self {
            fdm,
            air,
            utc: start.utc,
            trim,
            specific_force_body_m_s2: level,
        })
    }

    /// The JSBSim flight model, to read its articulation and systems.
    pub const fn fdm(&self) -> &JsbsimFdmBlock {
        &self.fdm
    }

    /// The specific force of the last step, body forward-right-down axes,
    /// m/s².
    pub const fn specific_force_body_m_s2(&self) -> DVec3 {
        self.specific_force_body_m_s2
    }

    /// The controls JSBSim trimmed level flight with: the throttle that
    /// holds the starting speed, and the stick with the pitch trim taken up.
    pub const fn trim(&self) -> ControlInputs {
        self.trim
    }
}

impl Airframe for C172Airframe {
    fn period(&self) -> TickSpan {
        self.fdm.period()
    }

    fn state(&self) -> VehicleState {
        self.fdm.output()
    }

    fn sensed(&self) -> Result<SensedTruth, ChainError> {
        let vehicle = self.fdm.output();
        let position = ecef_to_geodetic(vehicle.position_ecef);
        Ok(SensedTruth {
            vehicle,
            specific_force_body_m_s2: self.specific_force_body_m_s2,
            angular_rate_body_rad_s: vehicle.inertial_rate_body(&BodyEllipsoid::WGS84),
            air: self
                .air
                .sample(&position, self.utc)
                .map_err(ChainError::Air)?,
        })
    }

    fn fly(&mut self, command: &DirectActuatorCmd) -> Result<(), ChainError> {
        let controls = if command.armed {
            direct_fixed_wing_controls(command)
        } else {
            self.trim
        };
        let before = self.fdm.output();
        set_wind(&mut self.fdm, &self.air, self.utc)?;
        self.fdm
            .step(&KinematicInput { controls })
            .map_err(|e| ChainError::Plant(Box::new(e)))?;
        let dt_s = self.fdm.period().as_seconds();
        self.specific_force_body_m_s2 = specific_force_body_m_s2(&before, &self.fdm.output(), dt_s);
        self.utc = self.utc.advanced_by(dt_s);
        Ok(())
    }
}

/// Gives `fdm` the wind of `air` where the aeroplane is at `utc`.
fn set_wind(fdm: &mut JsbsimFdmBlock, air: &SharedAir, utc: UtcTime) -> Result<(), ChainError> {
    let position = ecef_to_geodetic(fdm.output().position_ecef);
    let sample = air.sample(&position, utc).map_err(ChainError::Air)?;
    fdm.set_wind_ned_m_s(sample.wind_ned_m_s)
        .map_err(|e| ChainError::Plant(Box::new(e)))
}
