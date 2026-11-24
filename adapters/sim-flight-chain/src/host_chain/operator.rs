//! What a crew does to the host in flight: hand it a new plan, and select a heading, an
//! altitude or an airspeed in place of the plan's.

use aviate_core::control::VehicleController;
use aviate_core::mixer::Mixer;
use pilotage_mission::SelectedTargets;

use super::{HostChain, RouteFix, waypoint_plan};
use crate::{Airframe, ChainError};

impl<A, C, M> HostChain<A, C, M>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    /// Hands the host the plan through `fixes` now. A plan that waited for its loading
    /// time is replaced; a plan already flying goes on until the host reports that the
    /// new one took over.
    pub fn load_plan(&mut self, fixes: &[RouteFix]) -> Result<(), ChainError> {
        self.plan = None;
        let record = self
            .core
            .load_mission(waypoint_plan(fixes))
            .map_err(|source| ChainError::Host(Box::new(source)))?;
        self.record = Some(record);
        // The host acts on the plan in its next cycle, not at its next scheduled wake.
        self.next_wake = self.host_now();
        Ok(())
    }

    /// Selects `targets` in place of the plan's; a selection with no target flies the
    /// plan. The host refuses one before a plan is loaded.
    pub fn select_targets(&mut self, targets: SelectedTargets) -> Result<(), ChainError> {
        self.core
            .select_targets(targets)
            .map_err(|source| ChainError::Host(Box::new(source)))
    }

    /// The selection in effect.
    pub fn selected_targets(&self) -> SelectedTargets {
        self.core.selected_targets()
    }
}
