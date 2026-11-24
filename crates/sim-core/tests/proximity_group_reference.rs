//! Independent physical reference verification suite for `sim-core`
//! proximity-group coupled multi-vessel integration:
//! 1. Three-vessel articulated chain (`A --tether-- B --docking-- C`)
//! 2. Stiff tether reel-in, rebound, Rosenbrock23/DormandPrince54, and cut
//! 3. Orbital-radius (`r ~ 7e6 m`) CM-relative angular momentum conservation

#[cfg(test)]
#[path = "proximity_group_reference/chain_dynamics.rs"]
mod chain_dynamics;
#[cfg(test)]
#[path = "proximity_group_reference/models.rs"]
mod models;
#[cfg(test)]
#[path = "proximity_group_reference/orbital_cm_momentum.rs"]
mod orbital_cm_momentum;
#[cfg(test)]
#[path = "proximity_group_reference/tether_rebound.rs"]
mod tether_rebound;
