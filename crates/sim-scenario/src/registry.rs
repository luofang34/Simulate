//! An open list of what a pilot can fly, for scenario browsers: every
//! built-in [`ScenarioProfile`], a free flight at each built-in aerodrome,
//! and anything registered at runtime.

use std::collections::BTreeMap;

use sim_scenery::AerodromeId;

use crate::ScenarioProfile;

#[cfg(test)]
mod tests;

/// The shelf a scenario sits on in a browser.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ScenarioCategory {
    /// Unscripted flight from a spawn point.
    FreeFlight,
    /// Closed circuits at an aerodrome.
    TrafficPattern,
    /// Visual-navigation corridor flights.
    VisualCorridor,
    /// Apollo spacecraft: lunar descent and Earth entry.
    SpacecraftLunar,
    /// Software- and hardware-in-the-loop verification rigs.
    XilVerification,
}

/// One entry in a [`ScenarioRegistry`].
#[derive(Debug, Clone, PartialEq)]
pub struct ScenarioDescriptor {
    /// Stable identifier, unique in a registry.
    pub id: String,
    /// Display title.
    pub title: String,
    /// Browser shelf.
    pub category: ScenarioCategory,
    /// One or two sentences on what the scenario exercises.
    pub summary: String,
    /// The vehicle the scenario is built around.
    pub recommended_vehicle: String,
    /// The aerodrome code it starts at or near, if any.
    pub aerodrome: Option<String>,
    /// Free-form search tags.
    pub tags: Vec<String>,
    /// The built-in profile that flies it, if any.
    pub profile: Option<ScenarioProfile>,
}

impl ScenarioDescriptor {
    /// The descriptor of a built-in profile.
    #[must_use]
    pub fn of_profile(profile: ScenarioProfile) -> Self {
        let (category, vehicle, aerodrome, summary, tags): (_, _, _, _, &[&str]) = match profile {
            ScenarioProfile::CorridorNominalCruise
            | ScenarioProfile::CorridorOffSeedAcquisition
            | ScenarioProfile::CorridorAggressiveBank
            | ScenarioProfile::CorridorSameCellLoiter => (
                ScenarioCategory::VisualCorridor,
                "Cessna 172",
                None,
                "A scripted flight over the synthetic Delaware corridor feeding the two-tier visual navigation harness.",
                &["delaware", "visual-navigation", "corridor"],
            ),
            ScenarioProfile::Apollo11LunarLanding => (
                ScenarioCategory::SpacecraftLunar,
                "Apollo LM Eagle",
                None,
                "Eagle's powered descent to the Sea of Tranquility.",
                &["apollo", "moon", "landing"],
            ),
            ScenarioProfile::Apollo11CapsuleReentry => (
                ScenarioCategory::SpacecraftLunar,
                "Apollo CM Columbia",
                None,
                "Columbia's Command Module from entry interface to subsonic speed.",
                &["apollo", "reentry"],
            ),
            ScenarioProfile::Airport47NUncontrolledPattern => (
                ScenarioCategory::TrafficPattern,
                "Cessna 172",
                Some(AerodromeId::CentralJersey47N),
                "A closed-loop left-hand circuit at an uncontrolled CTAF field.",
                &["pattern", "ctaf", "new-jersey"],
            ),
            ScenarioProfile::AirportKewrToweredPattern => (
                ScenarioCategory::TrafficPattern,
                "Cessna 172",
                Some(AerodromeId::NewarkKewr),
                "A closed-loop left-hand circuit inside Class B airspace.",
                &["pattern", "towered", "class-b", "new-jersey"],
            ),
        };
        Self {
            id: format!("profile/{profile:?}"),
            title: profile.title().to_owned(),
            category,
            summary: summary.to_owned(),
            recommended_vehicle: vehicle.to_owned(),
            aerodrome: aerodrome.map(|id| id.code().to_owned()),
            tags: tags.iter().map(|tag| (*tag).to_owned()).collect(),
            profile: Some(profile),
        }
    }

    /// A free flight from `aerodrome`'s primary runway.
    #[must_use]
    pub fn free_flight_at(aerodrome: AerodromeId) -> Self {
        Self {
            id: format!("free-flight/{}", aerodrome.code()),
            title: format!("Free Flight: {}", aerodrome.title()),
            category: ScenarioCategory::FreeFlight,
            summary: format!(
                "Unscripted flight from runway {} with the stick in hand.",
                aerodrome.primary_runway()
            ),
            recommended_vehicle: "Cessna 172".to_owned(),
            aerodrome: Some(aerodrome.code().to_owned()),
            tags: vec!["free-flight".to_owned()],
            profile: None,
        }
    }

    /// Whether `query`, without case, appears in the id, title, summary,
    /// vehicle, aerodrome or a tag. An empty query matches everything.
    #[must_use]
    pub fn matches(&self, query: &str) -> bool {
        let needle = query.trim().to_lowercase();
        let fields = [
            &self.id,
            &self.title,
            &self.summary,
            &self.recommended_vehicle,
        ];
        fields
            .into_iter()
            .chain(self.aerodrome.as_ref())
            .chain(&self.tags)
            .any(|field| field.to_lowercase().contains(&needle))
    }
}

/// Scenarios by id, open to registration at runtime.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ScenarioRegistry {
    entries: BTreeMap<String, ScenarioDescriptor>,
}

impl ScenarioRegistry {
    /// A registry with nothing in it.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Every built-in profile and a free flight at each built-in aerodrome.
    #[must_use]
    pub fn with_builtins() -> Self {
        let mut registry = Self::new();
        for profile in ScenarioProfile::ALL {
            registry.register(ScenarioDescriptor::of_profile(profile));
        }
        for aerodrome in AerodromeId::ALL {
            registry.register(ScenarioDescriptor::free_flight_at(aerodrome));
        }
        registry
    }

    /// Adds `descriptor`, returning any it replaced under the same id.
    pub fn register(&mut self, descriptor: ScenarioDescriptor) -> Option<ScenarioDescriptor> {
        self.entries.insert(descriptor.id.clone(), descriptor)
    }

    /// The descriptor with `id`.
    #[must_use]
    pub fn get(&self, id: &str) -> Option<&ScenarioDescriptor> {
        self.entries.get(id)
    }

    /// Every descriptor, by id.
    pub fn iter(&self) -> impl Iterator<Item = &ScenarioDescriptor> {
        self.entries.values()
    }

    /// How many scenarios are registered.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether nothing is registered.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The descriptors on one shelf, by id.
    #[must_use]
    pub fn in_category(&self, category: ScenarioCategory) -> Vec<&ScenarioDescriptor> {
        self.iter()
            .filter(|descriptor| descriptor.category == category)
            .collect()
    }

    /// Descriptors matching `query` (see [`ScenarioDescriptor::matches`]),
    /// by id.
    #[must_use]
    pub fn search(&self, query: &str) -> Vec<&ScenarioDescriptor> {
        self.iter()
            .filter(|descriptor| descriptor.matches(query))
            .collect()
    }
}
