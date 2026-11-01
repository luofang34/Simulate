use crate::ScenarioWorld;

/// One of the deterministic flights.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ScenarioProfile {
    /// Straight and gently banking flight across the Delaware corridor,
    /// crossing many map cells.
    CorridorNominalCruise,
    /// A cold start off the corridor, 900 m to the side and 200 m high,
    /// converging back onto it.
    CorridorOffSeedAcquisition,
    /// Steep banks and pitch excursions that take the horizon out of a nose
    /// camera's view.
    CorridorAggressiveBank,
    /// A tight orbit inside one map cell, whose repeat anchors must not count
    /// as independent fixes.
    CorridorSameCellLoiter,
    /// Eagle's final approach to the Sea of Tranquility.
    Apollo11LunarLanding,
    /// Columbia's Command Module from entry interface to subsonic speed.
    Apollo11CapsuleReentry,
    /// C172 flying a closed-loop left-hand traffic pattern at `47N` (Central Jersey
    /// Regional, uncontrolled CTAF `122.700 MHz`, Runway `07`).
    Airport47NUncontrolledPattern,
    /// C172 flying a closed-loop left-hand traffic pattern at `KEWR` (Newark Liberty
    /// Class B towered `118.300 MHz`, Runway `22L`).
    AirportKewrToweredPattern,
}

impl ScenarioProfile {
    /// Every profile, in key order.
    pub const ALL: [Self; 8] = [
        Self::CorridorNominalCruise,
        Self::CorridorOffSeedAcquisition,
        Self::CorridorAggressiveBank,
        Self::CorridorSameCellLoiter,
        Self::Apollo11LunarLanding,
        Self::Apollo11CapsuleReentry,
        Self::Airport47NUncontrolledPattern,
        Self::AirportKewrToweredPattern,
    ];

    /// The profile on number key `digit`, `1` to `8`.
    pub const fn from_digit(digit: u8) -> Option<Self> {
        match digit {
            1 => Some(Self::CorridorNominalCruise),
            2 => Some(Self::CorridorOffSeedAcquisition),
            3 => Some(Self::CorridorAggressiveBank),
            4 => Some(Self::CorridorSameCellLoiter),
            5 => Some(Self::Apollo11LunarLanding),
            6 => Some(Self::Apollo11CapsuleReentry),
            7 => Some(Self::Airport47NUncontrolledPattern),
            8 => Some(Self::AirportKewrToweredPattern),
            _ => None,
        }
    }

    /// A short name for displays.
    pub const fn title(self) -> &'static str {
        match self {
            Self::CorridorNominalCruise => "Delaware Corridor Cruise",
            Self::CorridorOffSeedAcquisition => "Off-Seed Acquisition",
            Self::CorridorAggressiveBank => "Aggressive Bank",
            Self::CorridorSameCellLoiter => "Same-Cell Loiter",
            Self::Apollo11LunarLanding => "Apollo 11 Lunar Landing",
            Self::Apollo11CapsuleReentry => "Apollo 11 CM Reentry",
            Self::Airport47NUncontrolledPattern => "47N C172 Pattern (CTAF)",
            Self::AirportKewrToweredPattern => "KEWR C172 Pattern (Tower)",
        }
    }

    /// The terrain the profile flies over.
    pub const fn world(self) -> ScenarioWorld {
        match self {
            Self::CorridorNominalCruise
            | Self::CorridorOffSeedAcquisition
            | Self::CorridorAggressiveBank
            | Self::CorridorSameCellLoiter => ScenarioWorld::DelawareCorridor,
            Self::Apollo11LunarLanding => ScenarioWorld::LunarTranquility,
            Self::Apollo11CapsuleReentry => ScenarioWorld::EarthGlobe,
            Self::Airport47NUncontrolledPattern => ScenarioWorld::CentralJersey47N,
            Self::AirportKewrToweredPattern => ScenarioWorld::NewarkKewr,
        }
    }
}
