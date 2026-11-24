//! Independent physical & operational reference suite for `sim-universe`
//! multi-vessel proximity topology, tiering discipline, checkpoint/slicing
//! invariance, and error contracts.

#[cfg(test)]
#[path = "universe_topology_reference/checkpoint_slicing.rs"]
mod checkpoint_slicing;
#[cfg(test)]
#[path = "universe_topology_reference/error_contracts.rs"]
mod error_contracts;
#[cfg(test)]
#[path = "universe_topology_reference/mission_lifecycle.rs"]
mod mission_lifecycle;
#[cfg(test)]
#[path = "universe_topology_reference/models.rs"]
mod models;
