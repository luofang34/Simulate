//! The X500 quadrotor airframe as the flight controller's plant: a rigid body
//! driven by four rotors, resting on a pad until they lift it.
//!
//! The body has no aerodynamic drag and its rotors answer a command at once.
//! The rigid body has no ground contact, so the pad stands in for the ground:
//! it holds the vehicle still while the rotors carry less than its weight and
//! plays no part afterwards.

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, Placement, VacuumAero};
use sim_atmosphere::StandardAtmosphere;
use sim_aviate::{AviateStimulus, DirectActuatorCmd, RotorLayout};
use sim_core::{
    Block, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime};
use world_core::{GeodeticPosition, normal_gravity};

use crate::ChainError;

/// Weight the Aviate X500 hover trim is tuned for, newtons.
const X500_WEIGHT_N: f64 = 20.25;
/// Principal moments of inertia about body x, y and z, kg·m².
const X500_INERTIA_KG_M2: DVec3 = DVec3::new(0.029, 0.029, 0.055);
const AIRFRAME_TEMPERATURE_K: f64 = 293.15;

/// What the flight controller's sensors feel at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlantStimulus {
    /// At rest on the pad: the accelerometer reads the pad's reaction.
    Resting {
        /// Centre-of-gravity state.
        vehicle: VehicleState,
        /// Specific force in body forward-right-down axes, m/s².
        specific_force_frd_m_s2: DVec3,
    },
    /// Flying: the sensed kinematics at the airframe's mounting site.
    Flying(AviateStimulus),
}

/// The X500 rigid body and its rotors.
pub struct X500Plant {
    body: RigidBody6DofBlock,
    rotors: RotorLayout,
    gravity_m_s2: f64,
    on_pad: bool,
}

impl X500Plant {
    /// The airframe level on a pad at `pad`, nose toward `heading_rad` from
    /// true north, integrated every `period`.
    pub fn on_pad(pad: Geodetic, heading_rad: f64, period: TickSpan) -> Result<Self, ChainError> {
        let (lat_deg, lon_deg) = (pad.lat_rad.to_degrees(), pad.lon_rad.to_degrees());
        let gravity_m_s2 = normal_gravity(GeodeticPosition::wgs84(lat_deg, lon_deg, pad.height_m))
            .map_err(|_| ChainError::PadGravity { lat_deg, lon_deg })?;
        let spec = ComponentSpec {
            name: "X500 Airframe".to_owned(),
            placement: Placement::ROOT,
            dry_mass_kg: X500_WEIGHT_N / gravity_m_s2,
            dry_inertia_body: DMat3::from_diagonal(X500_INERTIA_KG_M2),
            thermal_capacity_j_k: 0.0,
            nominal_power_w: 0.0,
        };
        let tree =
            ComponentTree::new([(spec, ComponentState::at_temperature(AIRFRAME_TEMPERATURE_K))])
                .map_err(ChainError::Tree)?;
        let config = RigidBodyConfig {
            body: BodyEllipsoid::WGS84,
            period,
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: VehicleState::level_flight(&pad, heading_rad, 0.0),
        };
        let body = RigidBody6DofBlock::new(
            config,
            tree,
            Box::new(VacuumAero),
            Box::new(StandardAtmosphere),
        )
        .map_err(|error| ChainError::Plant(Box::new(error)))?;
        Ok(Self {
            body,
            rotors: RotorLayout::x500(),
            gravity_m_s2,
            on_pad: true,
        })
    }

    /// The integration period.
    pub fn period(&self) -> TickSpan {
        self.body.period()
    }

    /// The true vehicle state.
    pub fn state(&self) -> VehicleState {
        self.body.output()
    }

    /// Whether the pad still carries the vehicle.
    pub const fn is_on_pad(&self) -> bool {
        self.on_pad
    }

    /// What the flight controller's sensors feel now.
    pub fn stimulus(&self) -> Result<PlantStimulus, ChainError> {
        let vehicle = self.body.output();
        if self.on_pad {
            return Ok(PlantStimulus::Resting {
                vehicle,
                specific_force_frd_m_s2: DVec3::new(0.0, 0.0, -self.gravity_m_s2),
            });
        }
        let site = self
            .body
            .site_kinematics(ComponentId(0))
            .map_err(ChainError::Tree)?;
        Ok(PlantStimulus::Flying(AviateStimulus { vehicle, site }))
    }

    /// One period under the flight controller's rotor command.
    pub fn step(&mut self, command: &DirectActuatorCmd) -> Result<(), ChainError> {
        let thrusters = self
            .rotors
            .direct_thrusters(command)
            .map_err(ChainError::Rotors)?;
        if self.on_pad {
            let lift_n: f64 = thrusters.iter().map(|wrench| -wrench.force_body_n.z).sum();
            if lift_n <= X500_WEIGHT_N {
                return Ok(());
            }
            self.on_pad = false;
        }
        self.body
            .step(&RigidBodyInput {
                thrusters,
                ..RigidBodyInput::default()
            })
            .map_err(|error| ChainError::Plant(Box::new(error)))
    }
}
