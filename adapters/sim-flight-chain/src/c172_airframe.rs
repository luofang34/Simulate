//! The JSBSim Cessna 172 as a flight controller's airframe, starting in
//! trimmed level flight.
//!
//! The four actuator lanes are throttle, aileron, elevator and rudder with
//! the surfaces neutral at one half. Until the flight controller is armed,
//! and while its servos are released, the pilot's controls fly the airframe;
//! they start as the controls JSBSim trimmed it with. JSBSim takes its density and temperature
//! from its own standard atmosphere and its wind from the shared air field,
//! so the aeroplane drifts with the same wind its sensors read.

use glam::DVec3;
use sim_atmosphere::AtmosphereField;
use sim_aviate::{DirectActuatorCmd, direct_fixed_wing_controls};
use sim_bodies::{BodyDescriptor, BodyFields};
use sim_core::{
    Block, ControlInputs, EngineStartMode, JsbsimFdmBlock, JsbsimFdmConfig, JsbsimFdmSnapshot,
    KinematicInput, SystemControls, TickSpan, VehicleState,
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
    /// The Earth's catalog fields. JSBSim integrates its own gravity; the
    /// sensed specific force and the board's devices read these.
    fields: BodyFields,
    utc: UtcTime,
    trim: ControlInputs,
    /// What the pilot holds on the yoke, pedals and throttle.
    pilot: ControlInputs,
    /// Whether the autopilot's servos are clutched to the controls.
    servos_engaged: bool,
    specific_force_body_m_s2: DVec3,
}

/// What changes as a [`C172Airframe`] flies. The air field and the trim it started with
/// stay the airframe's own.
#[derive(Debug, Clone, PartialEq)]
pub struct C172AirframeSnapshot {
    fdm: JsbsimFdmSnapshot,
    utc: UtcTime,
    pilot: ControlInputs,
    servos_engaged: bool,
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
        let fields = BodyDescriptor::earth().map_err(ChainError::Body)?.fields();
        // Unaccelerated flight: the specific force is the reaction to weight.
        let level = specific_force_body_m_s2(
            fields.gravity.as_ref(),
            &state,
            &state,
            start.period.as_seconds(),
        );
        Ok(Self {
            fdm,
            air,
            fields,
            utc: start.utc,
            trim,
            pilot: trim,
            servos_engaged: true,
            specific_force_body_m_s2: level,
        })
    }

    /// What the pilot holds on the controls. It flies the aeroplane whenever the flight
    /// controller does not: before it is armed, and while its servos are released.
    pub const fn set_pilot_controls(&mut self, controls: ControlInputs) {
        self.pilot = controls;
    }

    /// What the pilot holds on the controls.
    pub const fn pilot_controls(&self) -> ControlInputs {
        self.pilot
    }

    /// Clutches the autopilot's servos to the controls, or releases them. A flight
    /// controller keeps flying its laws while released, as an autopilot computer does
    /// with its servos out; it refuses to disarm in the air, so the clutch and not the
    /// arming state is what hands the aeroplane to the pilot.
    pub const fn set_servos_engaged(&mut self, engaged: bool) {
        self.servos_engaged = engaged;
    }

    /// Whether the autopilot's servos are clutched to the controls.
    pub const fn servos_engaged(&self) -> bool {
        self.servos_engaged
    }

    /// Sets what no flight controller lane reaches and the pilot always works by hand:
    /// flaps, wheel brakes, mixture and the like, from the next step on.
    pub fn set_system_controls(&mut self, controls: SystemControls) {
        self.fdm.set_system_controls(controls);
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
    type Snapshot = C172AirframeSnapshot;

    fn snapshot(&self) -> C172AirframeSnapshot {
        // Every field is named so that one added to the airframe must be placed here.
        let Self {
            fdm,
            air: _,
            fields: _,
            utc,
            trim: _,
            pilot,
            servos_engaged,
            specific_force_body_m_s2,
        } = self;
        C172AirframeSnapshot {
            fdm: fdm.snapshot(),
            utc: *utc,
            pilot: *pilot,
            servos_engaged: *servos_engaged,
            specific_force_body_m_s2: *specific_force_body_m_s2,
        }
    }

    fn restore(&mut self, snapshot: &C172AirframeSnapshot) -> Result<(), ChainError> {
        self.fdm
            .try_restore(&snapshot.fdm)
            .map_err(|e| ChainError::Plant(Box::new(e)))?;
        self.utc = snapshot.utc;
        self.pilot = snapshot.pilot;
        self.servos_engaged = snapshot.servos_engaged;
        self.specific_force_body_m_s2 = snapshot.specific_force_body_m_s2;
        Ok(())
    }

    fn period(&self) -> TickSpan {
        self.fdm.period()
    }

    fn state(&self) -> VehicleState {
        self.fdm.output()
    }

    fn fields(&self) -> &BodyFields {
        &self.fields
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
        let controls = if command.armed && self.servos_engaged {
            direct_fixed_wing_controls(command)
        } else {
            self.pilot
        };
        let before = self.fdm.output();
        set_wind(&mut self.fdm, &self.air, self.utc)?;
        self.fdm
            .step(&KinematicInput { controls })
            .map_err(|e| ChainError::Plant(Box::new(e)))?;
        let dt_s = self.fdm.period().as_seconds();
        self.specific_force_body_m_s2 = specific_force_body_m_s2(
            self.fields.gravity.as_ref(),
            &before,
            &self.fdm.output(),
            dt_s,
        );
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
