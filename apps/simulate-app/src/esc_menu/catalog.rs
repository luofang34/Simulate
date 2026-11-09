//! What the Flight Hub lists: the vehicle registry, aerodromes with their runway ends and
//! stands, scenario briefs, and the small enums its selectors cycle.

pub mod airports;
pub mod scenarios;
mod selectors;

use sim_scenario::ScenarioProfile;
use sim_scenery::{AerodromeId, AirportIdx, OsmAerodromeDiagram};

use crate::controls::{ControlPreset, VehicleClass};
pub use crate::flight::SpawnMode;
use crate::vehicles::FreeFlightVehicle;
pub use scenarios::ScenarioEntry;
pub use selectors::{AvionicsLayout, RateChoice, SceneryMode};

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
            FreeFlightVehicle::JsbsimC172 | FreeFlightVehicle::JsbsimP38g => Self::Jsbsim,
            FreeFlightVehicle::AviateX500Direct | FreeFlightVehicle::AviateFixedWingDirect => {
                Self::AviateXil
            }
            FreeFlightVehicle::Kinematic => Self::RigidBody,
        }
    }
}

/// The 3D model the hangar previews.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HangarModel {
    /// The packaged Cessna 172 `.usdc`.
    C172,
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
}

/// How flying one hangar airframe starts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AirframeLaunch {
    /// Hand-flown free flight on any of these plants (flight-dynamics modes), the first
    /// being the default.
    Free(Vec<FreeFlightVehicle>),
    /// A scripted scenario that flies this airframe.
    Scenario(ScenarioProfile),
}

/// An airframe equals every launch it can start, so the flying launch finds its row.
impl PartialEq<VehicleLaunch> for AirframeLaunch {
    fn eq(&self, other: &VehicleLaunch) -> bool {
        match (self, other) {
            (Self::Free(modes), VehicleLaunch::Free(vehicle)) => modes.contains(vehicle),
            (Self::Scenario(a), VehicleLaunch::Scenario(b)) => a == b,
            _ => false,
        }
    }
}

/// One distinct airframe in the hangar registry.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleDescriptor {
    /// Display name.
    pub title: String,
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
            AirframeLaunch::Scenario(_) => &[],
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
        let free = |title: &str, class, role, cruise_kt, mass_kg, modes, model| Self {
            title: title.to_owned(),
            class,
            role,
            cruise_kt,
            mass_kg,
            launch: AirframeLaunch::Free(modes),
            model,
        };
        vec![
            free(
                "C172 Skyhawk",
                VehicleClass::FixedWing,
                "General Aviation",
                110.0,
                1_043.0,
                vec![
                    FreeFlightVehicle::JsbsimC172,
                    FreeFlightVehicle::AviateFixedWingDirect,
                ],
                HangarModel::C172,
            ),
            free(
                "Lockheed P-38G Lightning",
                VehicleClass::FixedWing,
                "Twin-Boom Fighter",
                265.0,
                6_813.0,
                vec![FreeFlightVehicle::JsbsimP38g],
                HangarModel::P38g,
            ),
            free(
                "Corridor UAV",
                VehicleClass::FixedWing,
                "Fixed-Wing Testbed",
                95.0,
                750.0,
                vec![FreeFlightVehicle::Kinematic],
                HangarModel::FixedWingUav,
            ),
            free(
                "Holybro X500 Quadrotor",
                VehicleClass::Multirotor,
                "Multirotor",
                24.0,
                2.0,
                vec![FreeFlightVehicle::AviateX500Direct],
                HangarModel::X500,
            ),
            Self {
                title: "Apollo LM Eagle".to_owned(),
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
            class: VehicleClass::Spacecraft,
            role: "Spacecraft",
            cruise_kt: 0.0,
            mass_kg: 12_000.0,
            launch: AirframeLaunch::Scenario(ScenarioProfile::Apollo11LunarLanding),
            model: HangarModel::Raketa,
        }
    }
}

/// Glyph-safe flight-dynamics mode label for `vehicle`.
#[must_use]
pub const fn mode_label(vehicle: FreeFlightVehicle) -> &'static str {
    match vehicle {
        FreeFlightVehicle::JsbsimC172 | FreeFlightVehicle::JsbsimP38g => "JSBSIM",
        FreeFlightVehicle::AviateFixedWingDirect => "AVIATE FBW",
        FreeFlightVehicle::Kinematic => "KINEMATIC",
        FreeFlightVehicle::AviateX500Direct => "AVIATE DIRECT",
    }
}

/// What kind of place an aerodrome entry is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AerodromeKind {
    /// An OpenStreetMap airport diagram.
    Osm(AerodromeId),
    /// A surveyed airport of [`sim_scenery::AirportDatabase::builtin`], flown from its
    /// runway records.
    Database(AirportIdx),
    /// The Delaware visual-navigation corridor.
    DelawareCorridor,
    /// Tranquility Base on the Moon (flown as the Apollo landing).
    Tranquility,
}

/// Formats a compass bearing in standard aviation `001..=360` whole degrees (`0°` -> `360`).
#[must_use]
pub fn aviation_heading_deg(deg: f64) -> u16 {
    if !deg.is_finite() {
        return 360;
    }
    let rounded = deg.rem_euclid(360.0).round() as u16;
    if rounded == 0 || rounded >= 360 {
        360
    } else {
        rounded
    }
}

/// One runway end.
#[derive(Debug, Clone, PartialEq)]
pub struct RunwayEntry {
    /// Designator, e.g. `22L` (assigned from magnetic heading rounded to `10°`).
    pub designator: String,
    /// Magnetic bearing, degrees clockwise from magnetic north.
    pub magnetic_bearing_deg: f64,
    /// True bearing, degrees clockwise from true north.
    pub true_bearing_deg: f64,
    /// Length, metres.
    pub length_m: f64,
}

impl RunwayEntry {
    /// Runway-end chip label following aviation convention: True-referenced polar runways
    /// (`02T`) display only True bearing (`RWY 02T 020T`); Magnetic runways display both
    /// Magnetic and True bearings (`RWY 22L 219M/206T`) when `max_chars` permits, falling back
    /// to Magnetic only (`RWY 22L 219M`, then `22L 219M`) when horizontal space is tight.
    #[must_use]
    pub fn chip_label(&self, max_chars: usize) -> String {
        let tru = aviation_heading_deg(self.true_bearing_deg);
        if self.designator.ends_with('T') {
            let full = format!("RWY {} {tru:03}T", self.designator);
            if full.chars().count() <= max_chars {
                return full;
            }
            let compact = format!("{} {tru:03}T", self.designator);
            if compact.chars().count() <= max_chars {
                return compact;
            }
            return self.designator.clone();
        }
        let mag = aviation_heading_deg(self.magnetic_bearing_deg);
        let full = format!("RWY {} {mag:03}M/{tru:03}T", self.designator);
        if full.chars().count() <= max_chars {
            return full;
        }
        let mag_only = format!("RWY {} {mag:03}M", self.designator);
        if mag_only.chars().count() <= max_chars {
            return mag_only;
        }
        let compact = format!("{} {mag:03}M", self.designator);
        if compact.chars().count() <= max_chars {
            return compact;
        }
        self.designator.clone()
    }
}

/// One searchable place to spawn.
#[derive(Debug, Clone, PartialEq)]
pub struct AerodromeEntry {
    /// Identifier, e.g. `KEWR`.
    pub code: String,
    /// Full name.
    pub name: String,
    /// Kind of place.
    pub kind: AerodromeKind,
    /// Field elevation, metres.
    pub field_elevation_m: f64,
    /// Published magnetic variation, degrees east positive (`None` off-Earth).
    pub mag_var_deg: Option<f64>,
    /// Runway ends.
    pub runways: Vec<RunwayEntry>,
    /// Ramp / stand labels.
    pub stands: Vec<String>,
    /// Reference point latitude, degrees, when the place has one on Earth.
    pub lat_deg: Option<f64>,
    /// Reference point longitude, degrees, when the place has one on Earth.
    pub lon_deg: Option<f64>,
    /// Where it is and what to call, e.g. `Newark, US-NJ - TWR 118.300`.
    pub location_hint: String,
    /// The airport database record, when the place has one.
    pub airport_idx: Option<AirportIdx>,
}

impl AerodromeEntry {
    /// Builds the entry for an OSM diagram, with its database record's location and
    /// published magnetic variation.
    #[must_use]
    pub fn from_diagram(diagram: &OsmAerodromeDiagram) -> Self {
        let graph = &diagram.surface_graph;
        let record = sim_scenery::AirportDatabase::builtin().by_code(diagram.id.code());
        let mag_var_deg = record.map(|(_, r)| f64::from(r.mag_var_deg));
        Self {
            code: diagram.id.code().to_owned(),
            name: diagram.name.clone(),
            kind: AerodromeKind::Osm(diagram.id),
            field_elevation_m: diagram.field_elevation_m,
            mag_var_deg,
            runways: graph
                .runway_ends
                .iter()
                .map(|end| {
                    let true_bearing_deg = end.true_bearing_rad.to_degrees().rem_euclid(360.0);
                    let surveyed_mag = record.and_then(|(_, r)| {
                        r.runways
                            .iter()
                            .flat_map(|rw| rw.ends.iter())
                            .find(|e| e.designator.as_str() == end.designator.as_str())
                            .map(|e| f64::from(e.magnetic_heading_deg).rem_euclid(360.0))
                    });
                    let magnetic_bearing_deg = surveyed_mag.unwrap_or_else(|| {
                        (true_bearing_deg - mag_var_deg.unwrap_or(0.0)).rem_euclid(360.0)
                    });
                    RunwayEntry {
                        designator: end.designator.as_str().to_owned(),
                        magnetic_bearing_deg,
                        true_bearing_deg,
                        length_m: end.length_m,
                    }
                })
                .collect(),
            stands: graph
                .parking_spots
                .iter()
                .map(|s| s.label.clone())
                .collect(),
            lat_deg: Some(graph.reference_point.latitude_deg),
            lon_deg: Some(graph.reference_point.longitude_deg),
            location_hint: record.map_or_else(String::new, |(_, r)| airports::location_hint(r)),
            airport_idx: record.map(|(idx, _)| idx),
        }
    }

    /// Spawn modes this place supports.
    #[must_use]
    pub fn spawn_modes(&self) -> Vec<SpawnMode> {
        match self.kind {
            AerodromeKind::Osm(_) | AerodromeKind::Database(_) => SpawnMode::ALL
                .into_iter()
                .filter(|m| *m != SpawnMode::Ramp || !self.stands.is_empty())
                .filter(|m| *m == SpawnMode::Overhead || !self.runways.is_empty())
                .collect(),
            AerodromeKind::DelawareCorridor | AerodromeKind::Tranquility => {
                vec![SpawnMode::Overhead]
            }
        }
    }

    /// Whether `query` (case-insensitive) matches the code or name.
    #[must_use]
    pub fn matches(&self, query: &str) -> bool {
        let query = query.trim().to_ascii_uppercase();
        query.is_empty()
            || self.code.to_ascii_uppercase().contains(&query)
            || self.name.to_ascii_uppercase().contains(&query)
    }
}

/// Everything the hub lists, built once from the built-ins and the loaded diagrams.
#[derive(Debug, Clone, PartialEq)]
pub struct HubCatalog {
    /// Vehicle registry.
    pub vehicles: Vec<VehicleDescriptor>,
    /// Spawnable places.
    pub aerodromes: Vec<AerodromeEntry>,
    /// Launchable scenarios.
    pub scenarios: Vec<ScenarioEntry>,
    /// Whether [`Self::aerodromes`] lists the whole airport database, in its record
    /// order, so searches rank through the database's own index.
    pub global_airports: bool,
}

impl HubCatalog {
    /// The built-in vehicles and scenarios plus `diagrams`, the corridor, and Tranquility.
    #[cfg(test)]
    #[must_use]
    pub fn new<'a>(diagrams: impl IntoIterator<Item = &'a OsmAerodromeDiagram>) -> Self {
        let mut aerodromes: Vec<AerodromeEntry> = diagrams
            .into_iter()
            .map(AerodromeEntry::from_diagram)
            .collect();
        aerodromes.extend(airports::special_entries());
        Self {
            vehicles: VehicleDescriptor::builtins(),
            aerodromes,
            scenarios: ScenarioEntry::all(),
            global_airports: false,
        }
    }

    /// Registry index of the airframe flying `vehicle`.
    #[must_use]
    pub fn airframe_of(&self, vehicle: FreeFlightVehicle) -> Option<usize> {
        self.vehicles
            .iter()
            .position(|v| v.modes().contains(&vehicle))
    }

    /// Indices of aerodromes matching `query`: over the airport database, its ranked
    /// search (exact codes first) followed by the matching corridor and Moon entries;
    /// otherwise every entry whose code or name contains `query`.
    #[must_use]
    pub fn filter_aerodromes(&self, query: &str) -> Vec<usize> {
        if self.global_airports {
            return airports::ranked_matches(self, query);
        }
        self.aerodromes
            .iter()
            .enumerate()
            .filter(|(_, a)| a.matches(query))
            .map(|(i, _)| i)
            .collect()
    }
}
