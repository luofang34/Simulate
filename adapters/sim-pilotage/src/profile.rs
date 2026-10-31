use std::collections::BTreeMap;

use mbd_core::{ComponentState, ComponentTree};
use pilotage_aircraft::{AircraftProfile, CruiseModel, EnergyStore, Loading, Station, Tank};
use sim_pack::{ComponentRecord, LoadedSyspack, SyspackManifest, SystemKind};

use crate::PilotageBridgeError;

#[cfg(test)]
mod tests;

/// The profile schema this bridge writes.
const PROFILE_SCHEMA_VERSION: u32 = 1;

/// A loading station on one package component.
#[derive(Debug, Clone, PartialEq)]
pub struct StationSite {
    /// Station name in the profile.
    pub name: String,
    /// Key of the component that carries the load.
    pub component: String,
    /// Maximum load, kg.
    pub max_kg: f64,
}

/// A fuel tank in one package component.
#[derive(Debug, Clone, PartialEq)]
pub struct TankSite {
    /// Tank name in the profile.
    pub name: String,
    /// Key of the component that holds the fuel.
    pub component: String,
    /// Usable fuel, litres.
    pub usable_l: f64,
}

/// Where the aircraft stores its energy.
#[derive(Debug, Clone, PartialEq)]
pub enum EnergySpec {
    /// Fuel in tanks.
    Fuel {
        /// Fuel density, kg/L.
        density_kg_per_l: f64,
        /// The tanks.
        tanks: Vec<TankSite>,
    },
    /// A battery, whose mass the package already counts as structure.
    Battery {
        /// Usable energy when full, Wh.
        usable_wh: f64,
    },
}

/// What a manifest cannot state about an aircraft: limits, performance, and
/// which of its components take a load.
#[derive(Debug, Clone, PartialEq)]
pub struct AircraftSpec {
    /// ICAO type designator.
    pub type_designator: String,
    /// Registration, for one airframe.
    pub registration: Option<String>,
    /// Source of the data.
    pub source: String,
    /// Maximum takeoff weight, kg.
    pub max_takeoff_weight_kg: f64,
    /// CG envelope as (weight kg, arm m) vertices, arms as [`aircraft_profile`]
    /// defines them.
    pub cg_envelope: Vec<[f64; 2]>,
    /// Loading stations.
    pub stations: Vec<StationSite>,
    /// Fuel or battery.
    pub energy: EnergySpec,
    /// Planned cruise.
    pub cruise: CruiseModel,
}

impl AircraftSpec {
    fn tanks(&self) -> &[TankSite] {
        match &self.energy {
            EnergySpec::Fuel { tanks, .. } => tanks,
            EnergySpec::Battery { .. } => &[],
        }
    }

    fn density_kg_per_l(&self) -> Option<f64> {
        match self.energy {
            EnergySpec::Fuel {
                density_kg_per_l, ..
            } => Some(density_kg_per_l),
            EnergySpec::Battery { .. } => None,
        }
    }

    /// Each component carries one station or tank, so a component's load can
    /// be read back as that site's.
    fn sites_by_component(&self) -> Result<BTreeMap<&str, &str>, PilotageBridgeError> {
        let stations = self.stations.iter().map(|s| (&s.component, &s.name));
        let tanks = self.tanks().iter().map(|t| (&t.component, &t.name));
        let mut sites = BTreeMap::new();
        for (component, name) in stations.chain(tanks) {
            if let Some(first) = sites.insert(component.as_str(), name.as_str()) {
                return Err(PilotageBridgeError::SharedComponent {
                    first: first.to_owned(),
                    second: name.clone(),
                    component: component.clone(),
                });
            }
        }
        Ok(sites)
    }
}

/// Weight-and-balance arm of a body-axis `x`: flight manuals count arms
/// positive aft of the datum, and body `x` is positive forward. The datum is
/// the package's root origin.
fn arm_of(body_x_m: f64) -> f64 {
    -body_x_m
}

fn unknown_component(site: &str, component: &str) -> PilotageBridgeError {
    PilotageBridgeError::UnknownComponent {
        site: site.to_owned(),
        component: component.to_owned(),
    }
}

fn arm_at(
    package: &LoadedSyspack,
    site: &str,
    component: &str,
) -> Result<f64, PilotageBridgeError> {
    let id = package
        .component_id(component)
        .map_err(|_| unknown_component(site, component))?;
    let (position, _) = package
        .tree()
        .world_transform_in_root(id)
        .map_err(PilotageBridgeError::Tree)?;
    Ok(arm_of(position.x))
}

/// Structure alone: the tree with every load taken off.
fn empty_mass(package: &LoadedSyspack) -> Result<(f64, f64), PilotageBridgeError> {
    let mut tree: ComponentTree = package.tree().clone();
    let unloaded: Vec<ComponentState> = tree
        .states()
        .iter()
        .map(|state| ComponentState {
            fluid_mass_kg: 0.0,
            payload_mass_kg: 0.0,
            ..*state
        })
        .collect();
    tree.restore_states(&unloaded)
        .map_err(PilotageBridgeError::Tree)?;
    let mass = tree.mass_properties().map_err(PilotageBridgeError::Tree)?;
    Ok((mass.total_mass_kg, arm_of(mass.cg_body_m.x)))
}

/// The profile of a package under `spec`.
///
/// Empty weight and arm come from the component tree with every load removed,
/// so they match what the simulation flies. Arms are positive aft of the
/// package's root origin. Refuses a non-aircraft kind, an unknown or shared
/// component, and anything Pilotage's own validation refuses.
pub fn aircraft_profile(
    package: &LoadedSyspack,
    spec: &AircraftSpec,
) -> Result<AircraftProfile, PilotageBridgeError> {
    let kind = package.manifest().kind;
    if !matches!(
        kind,
        SystemKind::FixedWing | SystemKind::Rotorcraft | SystemKind::Multirotor
    ) {
        return Err(PilotageBridgeError::UnsupportedKind { kind });
    }
    spec.sites_by_component()?;
    let (empty_weight_kg, empty_arm_m) = empty_mass(package)?;
    let stations = spec
        .stations
        .iter()
        .map(|site| {
            Ok(Station {
                name: site.name.clone(),
                arm_m: arm_at(package, &site.name, &site.component)?,
                max_kg: site.max_kg,
            })
        })
        .collect::<Result<Vec<_>, PilotageBridgeError>>()?;
    let energy = match &spec.energy {
        EnergySpec::Battery { usable_wh } => EnergyStore::Battery {
            usable_wh: *usable_wh,
        },
        EnergySpec::Fuel {
            density_kg_per_l,
            tanks,
        } => EnergyStore::Fuel {
            density_kg_per_l: *density_kg_per_l,
            tanks: tanks
                .iter()
                .map(|site| {
                    Ok(Tank {
                        name: site.name.clone(),
                        arm_m: arm_at(package, &site.name, &site.component)?,
                        usable_l: site.usable_l,
                    })
                })
                .collect::<Result<Vec<_>, PilotageBridgeError>>()?,
        },
    };
    let profile = AircraftProfile {
        schema_version: PROFILE_SCHEMA_VERSION,
        type_designator: spec.type_designator.clone(),
        registration: spec.registration.clone(),
        source: spec.source.clone(),
        empty_weight_kg,
        empty_arm_m,
        max_takeoff_weight_kg: spec.max_takeoff_weight_kg,
        energy,
        stations,
        cg_envelope: spec.cg_envelope.clone(),
        cruise: spec.cruise.clone(),
    };
    profile.validate().map_err(PilotageBridgeError::Aircraft)?;
    Ok(profile)
}

/// The loading a package starts with: each station's payload and each tank's
/// fluid, converted to litres. A battery starts full, as the simulation has no
/// state of charge.
pub fn package_loading(
    package: &LoadedSyspack,
    spec: &AircraftSpec,
    profile: &AircraftProfile,
) -> Result<Loading, PilotageBridgeError> {
    let load = |site: &str, component: &str| {
        let id = package
            .component_id(component)
            .map_err(|_| unknown_component(site, component))?;
        package
            .tree()
            .state(id)
            .copied()
            .map_err(PilotageBridgeError::Tree)
    };
    let stations_kg = spec
        .stations
        .iter()
        .map(|site| {
            Ok((
                site.name.clone(),
                load(&site.name, &site.component)?.payload_mass_kg,
            ))
        })
        .collect::<Result<Vec<_>, PilotageBridgeError>>()?;
    let density = spec.density_kg_per_l().unwrap_or(1.0);
    let fuel_l = spec
        .tanks()
        .iter()
        .map(|site| {
            let fluid = load(&site.name, &site.component)?.fluid_mass_kg;
            Ok((site.name.clone(), fluid / density))
        })
        .collect::<Result<Vec<_>, PilotageBridgeError>>()?;
    let battery_wh = match spec.energy {
        EnergySpec::Battery { usable_wh } => Some(usable_wh),
        EnergySpec::Fuel { .. } => None,
    };
    Ok(Loading {
        profile: profile.id().map_err(PilotageBridgeError::Aircraft)?,
        revision: 0,
        stations_kg,
        fuel_l,
        battery_wh,
    })
}

fn component_of<'a>(
    manifest: &'a mut SyspackManifest,
    key: &str,
    site: &str,
) -> Result<&'a mut ComponentRecord, PilotageBridgeError> {
    manifest
        .components
        .iter_mut()
        .find(|record| record.key == key)
        .ok_or_else(|| unknown_component(site, key))
}

/// `manifest` with `loading` written into its components, so the same flight's
/// weight and balance can be flown. A loading Pilotage refuses for `profile`
/// is refused here, and components with no station or tank keep their load.
pub fn apply_loading(
    manifest: &SyspackManifest,
    spec: &AircraftSpec,
    profile: &AircraftProfile,
    loading: &Loading,
) -> Result<SyspackManifest, PilotageBridgeError> {
    loading
        .remaining(profile)
        .map_err(PilotageBridgeError::Aircraft)?;
    let mut loaded = manifest.clone();
    for (name, kg) in &loading.stations_kg {
        let site = spec.stations.iter().find(|site| site.name == *name);
        let site = site.ok_or_else(|| PilotageBridgeError::UnknownSite { site: name.clone() })?;
        component_of(&mut loaded, &site.component, name)?.initial_payload_mass_kg = *kg;
    }
    let density = spec.density_kg_per_l().unwrap_or(0.0);
    for (name, litres) in &loading.fuel_l {
        let site = spec.tanks().iter().find(|site| site.name == *name);
        let site = site.ok_or_else(|| PilotageBridgeError::UnknownSite { site: name.clone() })?;
        component_of(&mut loaded, &site.component, name)?.initial_fluid_mass_kg = litres * density;
    }
    Ok(loaded)
}
