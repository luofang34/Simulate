//! Built-in hangar airframe descriptors, aircraft family metadata, and flight-dynamics badges.

use sim_scenario::ScenarioProfile;

use crate::controls::{ControlPreset, VehicleClass};
use crate::vehicles::FreeFlightVehicle;

/// Flight-dynamics engine behind a vehicle, shown as a badge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FdmEngine {
    /// JSBSim flight dynamics.
    Jsbsim,
    /// Aviate flight controller in the loop.
    AviateXil,
    /// Six-degree-of-freedom rigid body.
    RigidBody,
}

impl FdmEngine {
    /// Badge text.
    #[must_use]
    pub const fn badge(self) -> &'static str {
        match self {
            Self::Jsbsim => "JSBSIM",
            Self::AviateXil => "AVIATE XIL",
            Self::RigidBody => "6-DOF RIGIDBODY",
        }
    }

    /// The engine flying free-flight plant `vehicle`.
    #[must_use]
    pub const fn of(vehicle: FreeFlightVehicle) -> Self {
        match vehicle {
            FreeFlightVehicle::JsbsimC172
            | FreeFlightVehicle::JsbsimC172p
            | FreeFlightVehicle::JsbsimP38g => Self::Jsbsim,
            FreeFlightVehicle::AviateX500Direct
            | FreeFlightVehicle::AviateFixedWingDirect
            | FreeFlightVehicle::C172Mission => Self::AviateXil,
            FreeFlightVehicle::Kinematic => Self::RigidBody,
        }
    }
}

/// The 3D model the hangar previews.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HangarModel {
    /// The packaged Cessna 172S Skyhawk SP (`N642SP` default).
    C172,
    /// The packaged Cessna 172P Skyhawk II / Reims F172P (`D-ECJB` default).
    C172p,
    /// The procedural Lockheed P-38G Lightning (`"Miss Virginia"`) twin-boom fighter.
    P38g,
    /// The procedural X500 quadrotor.
    X500,
    /// The procedural twin-boom corridor UAV.
    FixedWingUav,
    /// The procedural Apollo lunar module.
    LunarLander,
    /// The packaged Raketa single-stage test rocket (`retro_rocket_v4.usd`).
    Raketa,
}

/// How flying a vehicle starts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleLaunch {
    /// Hand-flown free flight on this plant.
    Free(FreeFlightVehicle),
    /// A scripted scenario that flies this vehicle.
    Scenario(ScenarioProfile),
    /// A spaceflight from the launch pad.
    Space,
}

/// How flying one hangar airframe starts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AirframeLaunch {
    /// Hand-flown free flight on any of these plants (flight-dynamics modes), the first
    /// being the default.
    Free(Vec<FreeFlightVehicle>),
    /// A scripted scenario that flies this airframe.
    Scenario(ScenarioProfile),
    /// A spaceflight from the launch pad.
    Space,
}

/// An airframe equals every launch it can start, so the flying launch finds its row.
impl PartialEq<VehicleLaunch> for AirframeLaunch {
    fn eq(&self, other: &VehicleLaunch) -> bool {
        match (self, other) {
            (Self::Free(modes), VehicleLaunch::Free(vehicle)) => modes.contains(vehicle),
            (Self::Scenario(a), VehicleLaunch::Scenario(b)) => a == b,
            (Self::Space, VehicleLaunch::Space) => true,
            _ => false,
        }
    }
}

/// One distinct airframe in the hangar registry.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleDescriptor {
    /// Display name.
    pub title: String,
    /// Package aircraft identifier in [`crate::livery_session::LiverySession`], when packaged.
    pub aircraft_id: Option<&'static str>,
    /// Shared aircraft family title when grouped with sibling variants (e.g. `Cessna 172 Skyhawk`).
    pub family_title: Option<&'static str>,
    /// Class, which picks the automatic control preset.
    pub class: VehicleClass,
    /// Role shown under the name, e.g. `General Aviation`.
    pub role: &'static str,
    /// Typical cruise speed, knots (`0` when not meaningful).
    pub cruise_kt: f64,
    /// Typical mass, kilograms.
    pub mass_kg: f64,
    /// What flying it starts.
    pub launch: AirframeLaunch,
    /// The model the hangar shows.
    pub model: HangarModel,
}

impl VehicleDescriptor {
    /// The automatic control preset for this vehicle.
    #[must_use]
    pub const fn default_preset(&self) -> ControlPreset {
        ControlPreset::default_for(self.class)
    }

    /// Free-flight plants this airframe can fly on (empty for scenario-only airframes).
    #[must_use]
    pub fn modes(&self) -> &[FreeFlightVehicle] {
        match &self.launch {
            AirframeLaunch::Free(modes) => modes,
            AirframeLaunch::Scenario(_) | AirframeLaunch::Space => &[],
        }
    }

    /// `preferred` when this airframe flies it, else its default mode.
    #[must_use]
    pub fn mode_or_default(
        &self,
        preferred: Option<FreeFlightVehicle>,
    ) -> Option<FreeFlightVehicle> {
        preferred
            .filter(|m| self.modes().contains(m))
            .or_else(|| self.modes().first().copied())
    }

    /// The launch flying this airframe on `mode` (its default when `mode` is not one of its
    /// own) starts.
    #[must_use]
    pub fn launch_for(&self, mode: Option<FreeFlightVehicle>) -> Option<VehicleLaunch> {
        match &self.launch {
            AirframeLaunch::Free(_) => self.mode_or_default(mode).map(VehicleLaunch::Free),
            AirframeLaunch::Scenario(profile) => Some(VehicleLaunch::Scenario(*profile)),
            AirframeLaunch::Space => Some(VehicleLaunch::Space),
        }
    }

    /// Flight-dynamics engine of `mode` (or the default mode).
    #[must_use]
    pub fn fdm(&self, mode: Option<FreeFlightVehicle>) -> FdmEngine {
        self.mode_or_default(mode)
            .map_or(FdmEngine::RigidBody, FdmEngine::of)
    }

    /// The built-in airframes, each with its own model.
    #[must_use]
    pub fn builtins() -> Vec<Self> {
        vec![
            Self {
                title: "C172S Skyhawk SP".to_owned(),
                aircraft_id: Some("c172s"),
                family_title: Some("Cessna 172 Skyhawk"),
                class: VehicleClass::FixedWing,
                role: "General Aviation",
                cruise_kt: 122.0,
                mass_kg: 1_157.0,
                launch: AirframeLaunch::Free(vec![
                    FreeFlightVehicle::JsbsimC172,
                    FreeFlightVehicle::AviateFixedWingDirect,
                    FreeFlightVehicle::C172Mission,
                ]),
                model: HangarModel::C172,
            },
            Self {
                title: "C172P Skyhawk II".to_owned(),
                aircraft_id: Some("c172p"),
                family_title: Some("Cessna 172 Skyhawk"),
                class: VehicleClass::FixedWing,
                role: "General Aviation",
                cruise_kt: 118.0,
                mass_kg: 1_089.0,
                launch: AirframeLaunch::Free(vec![FreeFlightVehicle::JsbsimC172p]),
                model: HangarModel::C172p,
            },
            Self {
                title: "Lockheed P-38G Lightning".to_owned(),
                aircraft_id: Some("p38g"),
                family_title: None,
                class: VehicleClass::FixedWing,
                role: "Twin-Boom Fighter",
                cruise_kt: 265.0,
                mass_kg: 6_813.0,
                launch: AirframeLaunch::Free(vec![FreeFlightVehicle::JsbsimP38g]),
                model: HangarModel::P38g,
            },
            Self {
                title: "Corridor UAV".to_owned(),
                aircraft_id: None,
                family_title: None,
                class: VehicleClass::FixedWing,
                role: "Fixed-Wing Testbed",
                cruise_kt: 95.0,
                mass_kg: 750.0,
                launch: AirframeLaunch::Free(vec![FreeFlightVehicle::Kinematic]),
                model: HangarModel::FixedWingUav,
            },
            Self {
                title: "Holybro X500 Quadrotor".to_owned(),
                aircraft_id: None,
                family_title: None,
                class: VehicleClass::Multirotor,
                role: "Multirotor",
                cruise_kt: 24.0,
                mass_kg: 2.0,
                launch: AirframeLaunch::Free(FreeFlightVehicle::x500_plants()),
                model: HangarModel::X500,
            },
            Self {
                title: "Apollo LM Eagle".to_owned(),
                aircraft_id: None,
                family_title: None,
                class: VehicleClass::Spacecraft,
                role: "Spacecraft",
                cruise_kt: 0.0,
                mass_kg: 15_103.0,
                launch: AirframeLaunch::Scenario(ScenarioProfile::Apollo11LunarLanding),
                model: HangarModel::LunarLander,
            },
            Self::raketa(),
        ]
    }

    /// Descriptor for the `Raketa` single-stage test rocket (`retro_rocket_v4.usd`, `12,000 kg`).
    #[must_use]
    pub fn raketa() -> Self {
        Self {
            title: "Raketa Single-Stage Rocket".to_owned(),
            aircraft_id: None,
            family_title: None,
            class: VehicleClass::Spacecraft,
            role: "Spacecraft",
            cruise_kt: 0.0,
            mass_kg: 12_000.0,
            launch: AirframeLaunch::Space,
            model: HangarModel::Raketa,
        }
    }
}

/// Glyph-safe flight-dynamics mode label for `vehicle`.
#[must_use]
pub const fn mode_label(vehicle: FreeFlightVehicle) -> &'static str {
    match vehicle {
        FreeFlightVehicle::JsbsimC172
        | FreeFlightVehicle::JsbsimC172p
        | FreeFlightVehicle::JsbsimP38g => "JSBSIM",
        FreeFlightVehicle::AviateFixedWingDirect => "AVIATE FBW",
        FreeFlightVehicle::Kinematic => "KINEMATIC",
        FreeFlightVehicle::AviateX500Direct => "AVIATE DIRECT",
        FreeFlightVehicle::C172Mission => "PILOTAGE MISSION",
    }
}
