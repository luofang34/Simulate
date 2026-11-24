use std::sync::Arc;

use mbd_design::{DesignAuditReport, audit_verified_syspack};
use mbd_multibody::WheelState;
use mbd_systems::{
    AbepThrusterState, AirbreathingEngineState, AtomicOxygenErosionState, ElectricThrusterState,
    LiquidRocketState, SolidMotorState, TpsAblationState,
};
use sim_atmosphere::AtmosphereField;
use sim_core::{
    MultibodySubsystems, MultibodyVehicleBlock, MultibodyVehicleConfig, PropulsionAssembly,
};
use sim_geo::GravityField;

use super::LoadedSyspack;
use crate::PackError;

impl LoadedSyspack {
    /// Runs forward/consistency MBD audit across component mass properties, tank
    /// geometry capacity, subsystem builders, and provenance coverage via [`mbd_design`].
    pub fn audit_design(&self) -> Result<DesignAuditReport, PackError> {
        Ok(audit_verified_syspack(&self.verified)?)
    }

    /// Instantiates a complete [`MultibodyVehicleBlock`] wiring component tree,
    /// aerodynamic elements, fluids, propulsion, electrical bus, thermal network, TPS,
    /// atomic-oxygen coatings, robotic manipulators, and wheels from this verified package.
    /// `gravity` is the truth gravity of `config.body`, the instance its catalog entry
    /// holds and the vehicle's sensors are handed too.
    pub fn build_multibody_vehicle(
        &self,
        config: MultibodyVehicleConfig,
        atmosphere: Box<dyn AtmosphereField>,
        gravity: Arc<dyn GravityField>,
    ) -> Result<MultibodyVehicleBlock, PackError> {
        let aero = self.verified.build_aero_elements()?;
        let propulsion = self.build_propulsion_assembly()?;
        let subsystems = self.build_multibody_subsystems(propulsion)?;

        MultibodyVehicleBlock::new(
            config,
            self.verified.tree().clone(),
            aero,
            subsystems,
            atmosphere,
            gravity,
        )
        .map_err(|err| PackError::DeviceBuild {
            device: "multibody_vehicle".to_owned(),
            source: Box::new(err),
        })
    }

    fn build_propulsion_assembly(&self) -> Result<PropulsionAssembly, PackError> {
        Ok(PropulsionAssembly {
            liquid_rockets: self
                .verified
                .build_liquid_rockets()?
                .into_iter()
                .map(|s| (s, LiquidRocketState::default()))
                .collect(),
            solid_motors: self
                .verified
                .build_solid_motors()?
                .into_iter()
                .map(|s| {
                    let st = SolidMotorState::loaded(&s);
                    (s, st)
                })
                .collect(),
            electric_thrusters: self
                .verified
                .build_electric_thrusters()?
                .into_iter()
                .map(|s| (s, ElectricThrusterState::default()))
                .collect(),
            abep_thrusters: self
                .verified
                .build_abep_thrusters()?
                .into_iter()
                .map(|s| (s, AbepThrusterState::default()))
                .collect(),
            airbreathing_engines: self
                .verified
                .build_airbreathing_engines()?
                .into_iter()
                .map(|s| (s, AirbreathingEngineState::default()))
                .collect(),
        })
    }

    fn build_multibody_subsystems(
        &self,
        propulsion: PropulsionAssembly,
    ) -> Result<MultibodySubsystems, PackError> {
        Ok(MultibodySubsystems {
            fluids: self.verified.build_fluid_network()?,
            propulsion,
            electrical: self.verified.build_electrical_bus()?,
            thermal: self.verified.build_thermal_network()?,
            tps_shields: self
                .verified
                .build_tps_shields()?
                .into_iter()
                .map(|s| {
                    let st = TpsAblationState::fresh(&s);
                    (s, st)
                })
                .collect(),
            ao_coatings: self
                .verified
                .build_ao_coatings()?
                .into_iter()
                .map(|s| (s, AtomicOxygenErosionState::default()))
                .collect(),
            manipulators: self.verified.build_manipulators()?,
            wheels: self
                .verified
                .build_wheel_specs()?
                .into_iter()
                .map(|s| (s, WheelState::default()))
                .collect(),
            pinned_dwell_s: 0.25,
            ..MultibodySubsystems::default()
        })
    }
}
