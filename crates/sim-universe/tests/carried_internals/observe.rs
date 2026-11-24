//! What the carried-internals tests read of the spacecraft and of the orbital
//! tier holding it.

use glam::DVec3;
use sim_core::Block;
use sim_orbital::{ContentHash, ExternalMassProperties};
use sim_universe::{MultibodyLocalPlant, Universe};

use super::SHIP;
use super::spacecraft::{ROOT, TANK};

/// What the test reads of the spacecraft's internals.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Internals {
    pub tank_kg: f64,
    pub battery_wh: f64,
    pub temperature_k: f64,
    pub thrust_n: f64,
    pub lit: f64,
}

/// Where in its orbit the spacecraft is, as its bus sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Phase {
    /// In full sunlight.
    Sunlit,
    /// Short of full sunlight with the thruster running on the battery.
    OnBattery,
    /// In the umbra with the thruster shed.
    Shed,
}

impl Internals {
    pub fn phase(&self) -> Phase {
        if self.lit == 1.0 {
            Phase::Sunlit
        } else if self.thrust_n > 0.0 {
            Phase::OnBattery
        } else {
            Phase::Shed
        }
    }
}

#[cfg(test)]
pub fn ship(universe: &Universe) -> &MultibodyLocalPlant {
    universe
        .carried::<MultibodyLocalPlant>(SHIP)
        .or_else(|| universe.local::<MultibodyLocalPlant>(SHIP))
        .unwrap()
}

#[cfg(test)]
pub fn internals(universe: &Universe) -> Internals {
    let plant = ship(universe);
    let subsystems = plant.block().subsystems();
    Internals {
        tank_kg: subsystems.fluids.tank_state(TANK).unwrap().mass_kg,
        battery_wh: subsystems.electrical.batteries()[0].1.energy_wh,
        temperature_k: subsystems.thermal.temperature_k(ROOT).unwrap(),
        thrust_n: plant.block().telemetry().total_thrust_body_n.length(),
        lit: plant.input().sun_visibility,
    }
}

/// Everything that must match between two runs: the orbital tier's full
/// replay hash and every bit of the plant, printed.
#[cfg(test)]
pub fn fingerprint(universe: &Universe) -> (ContentHash, String) {
    let plant = ship(universe);
    (
        universe.orbital().universe_hash(),
        format!("{:?} {:?}", plant.block().snapshot(), plant.input()),
    )
}

/// The thrust the orbital tier is holding for the vessel, N.
#[cfg(test)]
pub fn latched_thrust_n(universe: &Universe) -> f64 {
    let held = universe
        .orbital()
        .committed_checkpoint(SHIP)
        .unwrap()
        .external
        .loads;
    DVec3::from_array(held.force_body_n).length()
}

/// The mass the orbital tier is holding for the vessel.
#[cfg(test)]
pub fn held_mass(universe: &Universe) -> ExternalMassProperties {
    let held = universe
        .orbital()
        .committed_checkpoint(SHIP)
        .unwrap()
        .external
        .loads;
    held.mass.unwrap()
}

#[cfg(test)]
pub fn relative(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}
