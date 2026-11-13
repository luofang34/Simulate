//! Livery selection, cycling, and external livery/variant pack import on [`Flight`].

#[cfg(test)]
use sim_pack::VehicleLivery;

use super::Flight;
#[cfg(test)]
use crate::error::AppSimError;
use crate::livery_session::{LiverySession, OwnshipLiveryRef};

impl Flight {
    /// Borrows the live aircraft variant and livery session.
    #[must_use]
    pub const fn liveries(&self) -> &LiverySession {
        &self.liveries
    }

    /// Mutably borrows the live aircraft variant and livery session.
    pub fn liveries_mut(&mut self) -> &mut LiverySession {
        &mut self.liveries
    }

    /// Selects `livery_id` (or `tail_code`) on `aircraft_id`.
    #[cfg(test)]
    pub fn select_livery(
        &mut self,
        aircraft_id: &str,
        livery_id: &str,
    ) -> Result<VehicleLivery, AppSimError> {
        self.liveries.select_livery(aircraft_id, livery_id)
    }

    /// Cycles to the next (`forward = true`) or previous livery on the active free-flight
    /// aircraft when it exposes a livery catalog.
    #[cfg(test)]
    pub fn cycle_active_livery(
        &mut self,
        forward: bool,
    ) -> Result<Option<VehicleLivery>, AppSimError> {
        let Some(aircraft_id) = self.free_vehicle().and_then(|v| v.aircraft_id()) else {
            return Ok(None);
        };
        self.liveries.cycle_livery(aircraft_id, forward)
    }

    /// Returns the active OpenUSD ownship livery asset override for the currently flying
    /// free-flight vehicle, when one has been explicitly selected or imported.
    #[must_use]
    pub fn active_ownship_livery_override(&self) -> Option<OwnshipLiveryRef<'_>> {
        if self.profile().is_some() {
            return None;
        }
        let aircraft_id = self.free_vehicle()?.aircraft_id()?;
        self.liveries.active_usd_override(aircraft_id)
    }
}
