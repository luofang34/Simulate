/// Selectable interactive free-flight vehicle and flight-control plant.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum FreeFlightVehicle {
    /// SHA-256-verified JSBSim Cessna 172S Skyhawk SP (`vehicles/c172s`) with `SystemControls`.
    #[default]
    JsbsimC172,
    /// SHA-256-verified JSBSim Cessna 172P Skyhawk II / Reims F172P (`vehicles/c172p`).
    JsbsimC172p,
    /// SHA-256-verified JSBSim Lockheed P-38G Lightning (`vehicles/p38g`) twin-boom fighter.
    JsbsimP38g,
    /// Tier-0 `AviateDirectRunner` (`Ekf` + `MultirotorController` + `QuadXMixerX500`) on 6-DoF X500.
    AviateX500Direct,
    /// Tier-0 `AviateDirectRunner` (`Ekf` + `FixedWingController`) driving the JSBSim C172S.
    AviateFixedWingDirect,
    /// The C172S on the vehicle's own control chain: Pilotage's host core flies the first
    /// shareable link's route through the Aviate flight controller, Navigate's sets
    /// navigating on the same devices.
    C172Mission,
    /// Fast kinematic 6-DoF free-flight model.
    Kinematic,
}

impl FreeFlightVehicle {
    /// Whether the airframe predates head-up and glass displays, so its cockpit
    /// opens on its own panel with the conformal HUD and glass dock hidden.
    #[must_use]
    pub const fn period_cockpit(self) -> bool {
        matches!(self, Self::JsbsimP38g)
    }

    /// Package aircraft identifier in [`crate::livery_session::LiverySession`], when this
    /// vehicle flies a packaged airframe with selectable liveries.
    #[must_use]
    pub const fn aircraft_id(self) -> Option<&'static str> {
        match self {
            Self::JsbsimC172 | Self::AviateFixedWingDirect | Self::C172Mission => Some("c172s"),
            Self::JsbsimC172p => Some("c172p"),
            Self::JsbsimP38g => Some("p38g"),
            Self::AviateX500Direct | Self::Kinematic => None,
        }
    }

    /// Cycles to the previous free-flight vehicle.
    #[must_use]
    pub const fn prev(self) -> Self {
        match self {
            Self::JsbsimC172 => Self::Kinematic,
            Self::JsbsimC172p => Self::JsbsimC172,
            Self::JsbsimP38g => Self::JsbsimC172p,
            Self::AviateX500Direct => Self::JsbsimP38g,
            Self::AviateFixedWingDirect => Self::AviateX500Direct,
            Self::C172Mission => Self::AviateFixedWingDirect,
            Self::Kinematic => Self::C172Mission,
        }
    }

    /// Cycles to the next free-flight vehicle.
    #[must_use]
    pub const fn next(self) -> Self {
        match self {
            Self::JsbsimC172 => Self::JsbsimC172p,
            Self::JsbsimC172p => Self::JsbsimP38g,
            Self::JsbsimP38g => Self::AviateX500Direct,
            Self::AviateX500Direct => Self::AviateFixedWingDirect,
            Self::AviateFixedWingDirect => Self::C172Mission,
            Self::C172Mission => Self::Kinematic,
            Self::Kinematic => Self::JsbsimC172,
        }
    }

    /// The X500's plants.
    #[must_use]
    pub fn x500_plants() -> Vec<Self> {
        vec![Self::AviateX500Direct]
    }

    /// Short title displayed in the window title bar and logs.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::JsbsimC172 => "C172S Skyhawk SP (JSBSim)",
            Self::JsbsimC172p => "C172P Skyhawk II (JSBSim)",
            Self::JsbsimP38g => "Lockheed P-38G Lightning (JSBSim)",
            Self::AviateX500Direct => "X500 Quadrotor (Aviate Direct)",
            Self::AviateFixedWingDirect => "Fixed-Wing FBW (Aviate + JSBSim)",
            Self::C172Mission => "C172 Mission (Pilotage -> Aviate)",
            Self::Kinematic => "Kinematic Free Flight",
        }
    }
}
