/// Selectable interactive free-flight vehicle and flight-control plant.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum FreeFlightVehicle {
    /// SHA-256-verified JSBSim Cessna 172 Skyhawk (`vehicles/c172`) with `SystemControls`.
    #[default]
    JsbsimC172,
    /// SHA-256-verified JSBSim Lockheed P-38G Lightning (`vehicles/p38g`) twin-boom fighter.
    JsbsimP38g,
    /// Tier-0 `AviateDirectRunner` (`Ekf` + `MultirotorController` + `QuadXMixerX500`) on 6-DoF X500.
    AviateX500Direct,
    /// Tier-0 `AviateDirectRunner` (`Ekf` + `FixedWingController`) driving the JSBSim C172.
    AviateFixedWingDirect,
    /// The X500 on the vehicle's own control chain: Pilotage's mission engine commands the
    /// Aviate flight controller in MAVLink and flies a route from a pad. Desktop only.
    X500Mission,
    /// Fast kinematic 6-DoF free-flight model.
    Kinematic,
}

/// Whether this build can fly [`FreeFlightVehicle::X500Mission`].
const MISSION_CHAIN_BUILT: bool = cfg!(not(target_arch = "wasm32"));

impl FreeFlightVehicle {
    /// Whether the airframe predates head-up and glass displays, so its cockpit
    /// opens on its own panel with the conformal HUD and glass dock hidden.
    #[must_use]
    pub const fn period_cockpit(self) -> bool {
        matches!(self, Self::JsbsimP38g)
    }

    /// Cycles to the previous free-flight vehicle.
    #[must_use]
    pub const fn prev(self) -> Self {
        match self {
            Self::JsbsimC172 => Self::Kinematic,
            Self::JsbsimP38g => Self::JsbsimC172,
            Self::AviateX500Direct => Self::JsbsimP38g,
            Self::X500Mission => Self::AviateX500Direct,
            Self::AviateFixedWingDirect if MISSION_CHAIN_BUILT => Self::X500Mission,
            Self::AviateFixedWingDirect => Self::AviateX500Direct,
            Self::Kinematic => Self::AviateFixedWingDirect,
        }
    }

    /// Cycles to the next free-flight vehicle.
    #[must_use]
    pub const fn next(self) -> Self {
        match self {
            Self::JsbsimC172 => Self::JsbsimP38g,
            Self::JsbsimP38g => Self::AviateX500Direct,
            Self::AviateX500Direct if MISSION_CHAIN_BUILT => Self::X500Mission,
            Self::AviateX500Direct | Self::X500Mission => Self::AviateFixedWingDirect,
            Self::AviateFixedWingDirect => Self::Kinematic,
            Self::Kinematic => Self::JsbsimC172,
        }
    }

    /// The X500's plants: hand-flown through Aviate first, then, where it is built, the
    /// Pilotage mission.
    #[must_use]
    pub fn x500_plants() -> Vec<Self> {
        let mission = MISSION_CHAIN_BUILT.then_some(Self::X500Mission);
        std::iter::once(Self::AviateX500Direct)
            .chain(mission)
            .collect()
    }

    /// Short title displayed in the window title bar and logs.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::JsbsimC172 => "C172 Skyhawk (JSBSim)",
            Self::JsbsimP38g => "Lockheed P-38G Lightning (JSBSim)",
            Self::AviateX500Direct => "X500 Quadrotor (Aviate Direct)",
            Self::X500Mission => "X500 Mission (Pilotage -> Aviate)",
            Self::AviateFixedWingDirect => "Fixed-Wing FBW (Aviate + JSBSim)",
            Self::Kinematic => "Kinematic Free Flight",
        }
    }
}
