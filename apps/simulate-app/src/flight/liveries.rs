//! Livery selection, cycling, and external livery/variant pack import on [`Flight`].

#[cfg(test)]
use sim_pack::VehicleLivery;

use super::Flight;
#[cfg(test)]
use crate::error::AppSimError;
use crate::livery_session::{LiverySession, OwnshipLiveryRef};
use crate::vehicles::FreeFlightVehicle;

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

    /// Returns the package aircraft id of `vehicle` (built-in or imported external plugin).
    #[must_use]
    pub fn vehicle_aircraft_id(&self, vehicle: FreeFlightVehicle) -> Option<&str> {
        vehicle.aircraft_id().or_else(|| {
            vehicle
                .external_slot()
                .and_then(|slot| self.liveries.external_meta(slot))
                .map(|meta| meta.aircraft_id.as_str())
        })
    }

    /// Cycles to the next (`forward = true`) or previous livery on the active free-flight
    /// aircraft when it exposes a livery catalog.
    #[cfg(test)]
    pub fn cycle_active_livery(
        &mut self,
        forward: bool,
    ) -> Result<Option<VehicleLivery>, AppSimError> {
        let Some(aircraft_id) = self
            .free_vehicle()
            .and_then(|v| self.vehicle_aircraft_id(v))
            .map(str::to_owned)
        else {
            return Ok(None);
        };
        self.liveries.cycle_livery(&aircraft_id, forward)
    }

    /// Returns the active OpenUSD ownship livery asset override for the currently flying
    /// free-flight vehicle, when one has been explicitly selected or imported.
    #[must_use]
    pub fn active_ownship_livery_override(&self) -> Option<OwnshipLiveryRef<'_>> {
        if self.profile().is_some() {
            return None;
        }
        let aircraft_id = self.vehicle_aircraft_id(self.free_vehicle()?)?;
        self.liveries.active_usd_override(aircraft_id)
    }
}
