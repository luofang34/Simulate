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
            Some(VehicleLaunch::Space) => match self.flight.fly_raketa() {
                Ok(()) => {
                    self.after_spawn();
                    info!(vehicle = title, "spaceflight on the pad");
                }
                Err(err) => {
                    self.fail(event_loop, "starting the spaceflight failed", &err);
                    return false;
                }
            },
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
            AerodromeKind::Osm(_) | AerodromeKind::Database(_) | AerodromeKind::AdHoc(_) => Some((
                entry.code.clone(),
                entry
                    .runways
                    .get(runway)
                    .map_or_else(String::new, |r| r.designator.clone()),
            )),
            AerodromeKind::DelawareCorridor | AerodromeKind::Tranquility => None,
        };
        if matches!(entry.kind, AerodromeKind::Tranquility) {
            self.select(
                event_loop,
                Some(sim_scenario::ScenarioProfile::Apollo11LunarLanding),
            );
            return true;
        }
        let Some(result) = self.dispatch_spawn(index, runway, mode) else {
            return false;
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

    fn dispatch_spawn(
        &mut self,
        index: usize,
        runway: usize,
        mode: SpawnMode,
    ) -> Option<Result<(), crate::error::AppSimError>> {
        let entry = self.catalog.aerodromes.get(index)?;
        let kind = entry.kind;
        let airport_idx = entry.airport_idx;
        let code = entry.code.clone();
        let designator = entry.runways.get(runway).map(|r| r.designator.clone());
        match kind {
            AerodromeKind::Osm(id) => {
                self.spawn_osm_aerodrome(id, airport_idx, designator.as_deref(), runway, mode)
            }
            AerodromeKind::Database(idx) => {
                self.spawn_database_airport(idx, &code, designator.as_deref(), mode)
            }
            AerodromeKind::AdHoc(ad_hoc_idx) => {
                self.spawn_ad_hoc_airport(ad_hoc_idx, designator.as_deref(), mode)
            }
            AerodromeKind::DelawareCorridor => Some(self.flight.spawn_delaware()),
            AerodromeKind::Tranquility => None,
        }
    }

    fn spawn_osm_aerodrome(
        &mut self,
        id: sim_scenery::AerodromeId,
        airport_idx: Option<sim_scenery::AirportIdx>,
        designator: Option<&str>,
        runway: usize,
        mode: SpawnMode,
    ) -> Option<Result<(), crate::error::AppSimError>> {
        let rig = self.cockpit_rig.as_mut()?;
        rig.supervisor_mut().set_free_aerodrome(id);
        if let Some(idx) = airport_idx
            && let Some(record) = AirportDatabase::builtin().get(idx)
        {
            let end =
                record_runway_end_index(record, rig.supervisor().vertical_datum(), designator);
            rig.supervisor_mut().set_free_airport(idx, end);
        }
        let diagram = rig.supervisor().diagram(id).cloned()?;
        Some(self.flight.spawn_at_aerodrome(&diagram, runway, mode))
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

    pub(in crate::app) fn spawn_ad_hoc_airport(
        &mut self,
        ad_hoc_idx: usize,
        designator: Option<&str>,
        mode: SpawnMode,
    ) -> Option<Result<(), crate::error::AppSimError>> {
        let aerodrome = self.catalog.ad_hoc.get(ad_hoc_idx)?.clone();
        let datum = self.cockpit_rig.as_ref().map_or_else(
            || match self.hub.scenery {
                SceneryMode::Online => CatalogVerticalDatum::OrthometricMslEgm96,
                SceneryMode::Offline => CatalogVerticalDatum::EllipsoidWgs84,
            },
            |rig| rig.supervisor().vertical_datum(),
        );
        let end = record_runway_end_index(&aerodrome.airport, datum, designator);
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.register_extra_airport(aerodrome.airport.clone(), &aerodrome.diagram_geojson);
            rig.supervisor_mut()
                .set_custom_airport(aerodrome.airport.clone(), end);
        }
        info!(
            airport = aerodrome.airport.ids.primary.as_str(),
            runway = ?designator,
            "spawning at ad-hoc aerodrome"
        );
        Some(
            self.flight
                .spawn_at_airport_record(&aerodrome.airport, end, mode),
        )
    }

    /// Registers every catalog ad-hoc aerodrome's surface diagram and plate source into
    /// `cockpit_rig`.
    pub(in crate::app) fn sync_ad_hoc_to_rig(&mut self) {
        let Some(rig) = self.cockpit_rig.as_mut() else {
            return;
        };
        for aerodrome in &self.catalog.ad_hoc {
            rig.register_extra_airport(aerodrome.airport.clone(), &aerodrome.diagram_geojson);
        }
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
