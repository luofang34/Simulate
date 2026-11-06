//! Flying a hangar airframe and spawning at a hub aerodrome, including any airport of the
//! built-in database.

use sim_scenery::{AirportDatabase, AirportRecord, CatalogVerticalDatum};
use tracing::info;
use winit::event_loop::ActiveEventLoop;

use super::super::FlightApp;
use crate::esc_menu::catalog::{AerodromeKind, VehicleLaunch};
use crate::esc_menu::{SceneryMode, SpawnMode};
use crate::vehicles::FreeFlightVehicle;

impl FlightApp {
    /// Flies airframe `index` on `mode`, else on the plant flying now when the airframe
    /// has it, else on its default.
    pub(super) fn apply_vehicle(
        &mut self,
        event_loop: &ActiveEventLoop,
        index: usize,
        mode: Option<FreeFlightVehicle>,
    ) -> bool {
        let Some(vehicle) = self.catalog.vehicles.get(index) else {
            return false;
        };
        let title = vehicle.title.clone();
        let launch = vehicle.launch_for(mode.or(self.flight.free_vehicle()));
        match launch {
            Some(VehicleLaunch::Free(plant)) => match self.flight.select_vehicle(plant) {
                Ok(()) => {
                    self.after_spawn();
                    info!(vehicle = title, plant = plant.title(), "flying vehicle");
                }
                Err(err) => {
                    self.fail(event_loop, "switching vehicle failed", &err);
                    return false;
                }
            },
            Some(VehicleLaunch::Scenario(profile)) => self.select(event_loop, Some(profile)),
            None => return false,
        }
        true
    }

    pub(super) fn spawn(
        &mut self,
        event_loop: &ActiveEventLoop,
        index: usize,
        runway: usize,
        mode: SpawnMode,
    ) -> bool {
        let Some(entry) = self.catalog.aerodromes.get(index) else {
            return false;
        };
        // The EFB shows the chosen runway end's plate, whatever the supervisor's default.
        let efb_airport = match entry.kind {
            AerodromeKind::Osm(_) | AerodromeKind::Database(_) => Some((
                entry.code.clone(),
                entry
                    .runways
                    .get(runway)
                    .map_or_else(String::new, |r| r.designator.clone()),
            )),
            AerodromeKind::DelawareCorridor | AerodromeKind::Tranquility => None,
        };
        let result = match entry.kind {
            AerodromeKind::Osm(id) => {
                let Some(rig) = self.cockpit_rig.as_mut() else {
                    return false;
                };
                rig.supervisor_mut().set_free_aerodrome(id);
                if let Some(idx) = entry.airport_idx
                    && let Some(record) = AirportDatabase::builtin().get(idx)
                {
                    let designator = entry.runways.get(runway).map(|r| r.designator.as_str());
                    let end = record_runway_end_index(
                        record,
                        rig.supervisor().vertical_datum(),
                        designator,
                    );
                    rig.supervisor_mut().set_free_airport(idx, end);
                }
                let Some(diagram) = rig.supervisor().diagram(id).cloned() else {
                    return false;
                };
                self.flight.spawn_at_aerodrome(&diagram, runway, mode)
            }
            AerodromeKind::Database(idx) => {
                let designator = entry.runways.get(runway).map(|r| r.designator.clone());
                let code = entry.code.clone();
                let Some(result) =
                    self.spawn_database_airport(idx, &code, designator.as_deref(), mode)
                else {
                    return false;
                };
                result
            }
            AerodromeKind::DelawareCorridor => self.flight.spawn_delaware(),
            AerodromeKind::Tranquility => {
                self.select(
                    event_loop,
                    Some(sim_scenario::ScenarioProfile::Apollo11LunarLanding),
                );
                return true;
            }
        };
        match result {
            Ok(()) => {
                self.after_spawn();
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    rig.invalidate_chart();
                    rig.set_efb_airport(efb_airport);
                }
                true
            }
            Err(err) => {
                self.fail(event_loop, "spawning failed", &err);
                false
            }
        }
    }

    fn spawn_database_airport(
        &mut self,
        idx: sim_scenery::AirportIdx,
        code: &str,
        designator: Option<&str>,
        mode: SpawnMode,
    ) -> Option<Result<(), crate::error::AppSimError>> {
        let datum = self.cockpit_rig.as_ref().map_or_else(
            || match self.hub.scenery {
                SceneryMode::Online => CatalogVerticalDatum::OrthometricMslEgm96,
                SceneryMode::Offline => CatalogVerticalDatum::EllipsoidWgs84,
            },
            |rig| rig.supervisor().vertical_datum(),
        );
        let record = AirportDatabase::builtin().get(idx)?;
        let end = record_runway_end_index(record, datum, designator);
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.supervisor_mut().set_free_airport(idx, end);
        }
        info!(airport = code, runway = ?designator, "spawning at database airport");
        Some(self.flight.spawn_at_airport_record(record, end, mode))
    }
}

/// Index of runway end `designator` in `record`'s surface graph for `datum` (or `0`).
pub(super) fn record_runway_end_index(
    record: &AirportRecord,
    datum: CatalogVerticalDatum,
    designator: Option<&str>,
) -> usize {
    let graph = record.to_flightbag_graph(datum);
    designator
        .and_then(|d| {
            graph
                .runway_ends
                .iter()
                .position(|end| end.designator.as_str() == d)
        })
        .unwrap_or(0)
}
