//! The scenario browser: launchable scenarios grouped by category, with one-line briefs.

use sim_scenario::ScenarioProfile;

/// Scenario browser group.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScenarioCategory {
    /// Hand-flown sandbox.
    FreeFlight,
    /// Airport traffic patterns.
    TrafficPattern,
    /// Visual-navigation corridor profiles.
    Corridor,
    /// Apollo spacecraft.
    Spacecraft,
}

impl ScenarioCategory {
    /// Display name.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::FreeFlight => "Free Flight Sandbox",
            Self::TrafficPattern => "Traffic Pattern",
            Self::Corridor => "Visual Navigation Corridor",
            Self::Spacecraft => "Spacecraft & Lunar",
        }
    }
}

/// One launchable scenario.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenarioEntry {
    /// The profile, or `None` for free flight.
    pub profile: Option<ScenarioProfile>,
    /// Browser group.
    pub category: ScenarioCategory,
    /// One-line mission brief.
    pub briefing: &'static str,
}

impl ScenarioEntry {
    /// Display title.
    #[must_use]
    pub fn title(&self) -> &'static str {
        self.profile.map_or("Free Flight", ScenarioProfile::title)
    }

    /// Free flight followed by every profile, grouped by category.
    #[must_use]
    pub fn all() -> Vec<Self> {
        let mut entries: Vec<Self> = std::iter::once(Self {
            profile: None,
            category: ScenarioCategory::FreeFlight,
            briefing: "Hand-fly the selected vehicle from the selected spawn.",
        })
        .chain(ScenarioProfile::ALL.into_iter().map(|profile| Self {
            profile: Some(profile),
            category: category_of(profile),
            briefing: briefing_of(profile),
        }))
        .collect();
        entries.sort_by_key(|e| e.category as u8);
        entries
    }
}

const fn category_of(profile: ScenarioProfile) -> ScenarioCategory {
    match profile {
        ScenarioProfile::CorridorNominalCruise
        | ScenarioProfile::CorridorOffSeedAcquisition
        | ScenarioProfile::CorridorAggressiveBank
        | ScenarioProfile::CorridorSameCellLoiter => ScenarioCategory::Corridor,
        ScenarioProfile::Apollo11LunarLanding | ScenarioProfile::Apollo11CapsuleReentry => {
            ScenarioCategory::Spacecraft
        }
        ScenarioProfile::Airport47NUncontrolledPattern
        | ScenarioProfile::AirportKewrToweredPattern => ScenarioCategory::TrafficPattern,
    }
}

const fn briefing_of(profile: ScenarioProfile) -> &'static str {
    match profile {
        ScenarioProfile::CorridorNominalCruise => {
            "Straight and gently banking cruise across many corridor tiles and map cells."
        }
        ScenarioProfile::CorridorOffSeedAcquisition => {
            "Cold start 900 m off track and 200 m high - converging onto the corridor."
        }
        ScenarioProfile::CorridorAggressiveBank => {
            "Steep banks and pitch excursions that take the horizon out of view."
        }
        ScenarioProfile::CorridorSameCellLoiter => {
            "Tight orbit inside one map cell - repeat anchors are not independent fixes."
        }
        ScenarioProfile::Apollo11LunarLanding => "Eagle powered descent to the Sea of Tranquility.",
        ScenarioProfile::Apollo11CapsuleReentry => {
            "Columbia command module from entry interface to subsonic speed."
        }
        ScenarioProfile::Airport47NUncontrolledPattern => {
            "C172 left-hand pattern at 47N - CTAF 122.700 - runway 07."
        }
        ScenarioProfile::AirportKewrToweredPattern => {
            "C172 left-hand pattern at KEWR Class B - tower 118.300 - runway 22L."
        }
    }
}
