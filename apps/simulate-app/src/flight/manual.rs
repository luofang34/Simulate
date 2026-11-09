//! Free-flight spawns: over an airport runway, or taking manual control out of a scripted
//! scenario at its live state.

use glam::DVec3;
use sim_core::VehicleState;
use sim_geo::{BodyEllipsoid, LocalFrame, ecef_to_geodetic};
use sim_scenery::{AirportRecord, OsmAerodromeDiagram};
use tracing::info;

use super::Flight;
use super::airport_spawn::SpawnSite;
use super::mode::{FreeSpawn, Mode};
use crate::error::AppSimError;
use crate::vehicles::FreeFlightVehicle;
use crate::world_surface::WorldSurfaceHandle;

/// Where a spawn at an aerodrome puts the vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpawnMode {
    /// Lined up on the runway threshold, engine idle, ready for takeoff.
    Threshold,
    /// Three nautical miles out on a stabilized 3° approach.
    ShortFinal,
    /// Over the field at pattern / cruise height.
    Overhead,
    /// Parked cold on a ramp stand.
    Ramp,
}

impl SpawnMode {
    /// Every mode in display order.
    pub const ALL: [Self; 4] = [
        Self::Threshold,
        Self::ShortFinal,
        Self::Overhead,
        Self::Ramp,
    ];

    /// Display name.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::Threshold => "Runway Threshold",
            Self::ShortFinal => "Short Final 3 NM",
            Self::Overhead => "Overhead / Cruise",
            Self::Ramp => "Ramp / Stand",
        }
    }
}

/// Highest ellipsoidal height a takeover may start free flight at (the light aircraft and
/// the X500 cannot fly a reentry capsule's state), metres.
const TAKEOVER_CEILING_M: f64 = 6_000.0;

/// Hover height the Aviate X500 holds above its spawn field, metres (see
/// [`crate::vehicles::FreeFlightPlant::new_at`]).
const X500_HOVER_AGL_M: f64 = 110.0;

impl Flight {
    /// Respawns the preferred free-flight vehicle over `diagram`'s active runway; later free
    /// flights and restarts start there too.
    pub fn respawn_at_diagram(&mut self, diagram: &OsmAerodromeDiagram) -> Result<(), AppSimError> {
        self.surface.set_extra_runways(Vec::new());
        let site = SpawnSite::of_diagram(diagram, None, self.surface.vertical_datum());
        self.respawn_free(site.spawn(SpawnMode::Overhead, &self.surface))
    }

    /// Respawns the preferred vehicle at `diagram`'s runway end `runway` (else its primary
    /// runway) in `mode`; later free flights and restarts start there too.
    pub fn spawn_at_aerodrome(
        &mut self,
        diagram: &OsmAerodromeDiagram,
        runway: usize,
        mode: SpawnMode,
    ) -> Result<(), AppSimError> {
        self.surface.set_extra_runways(Vec::new());
        let site = SpawnSite::of_diagram(diagram, Some(runway), self.surface.vertical_datum());
        self.respawn_free(site.spawn(mode, &self.surface))?;
        info!(
            airport = diagram.id.code(),
            mode = mode.title(),
            "spawned at aerodrome"
        );
        Ok(())
    }

    /// Respawns the preferred vehicle at the Delaware corridor default.
    pub fn spawn_delaware(&mut self) -> Result<(), AppSimError> {
        self.surface.set_extra_runways(Vec::new());
        self.respawn_free(FreeSpawn::Delaware)
    }

    /// Whether free flight starts parked on the ground (throttle trims to idle).
    #[must_use]
    pub const fn spawned_on_ground(&self) -> bool {
        matches!(self.free_spawn, FreeSpawn::Ground { .. })
    }

    /// Starts free flight on the preferred vehicle at `spawn`; later free flights and
    /// restarts start there too.
    pub(super) fn respawn_free(&mut self, spawn: FreeSpawn) -> Result<(), AppSimError> {
        self.mode = Mode::free(self.preferred_vehicle, spawn, &self.surface)?;
        self.free_spawn = spawn;
        self.begin_flight()
    }

    /// Hands the pilot the stick: in a scripted scenario, starts free flight on the
    /// preferred vehicle at the scenario's live position and heading (restarts return
    /// there). Already in free flight, does nothing. Scenarios off Earth are refused, since
    /// every free-flight plant flies the WGS84 atmosphere.
    pub fn take_manual_control(&mut self) -> Result<(), AppSimError> {
        let Mode::Scenario(run) = &self.mode else {
            return Ok(());
        };
        let profile = run.profile();
        if profile.world().body() != BodyEllipsoid::WGS84 {
            return Err(AppSimError::ManualControlUnavailable {
                scenario: profile.title(),
                detail: "free-flight plants fly only over Earth",
            });
        }
        let state = run.vehicle();
        if ecef_to_geodetic(state.position_ecef).height_m > TAKEOVER_CEILING_M {
            return Err(AppSimError::ManualControlUnavailable {
                scenario: profile.title(),
                detail: "the vehicle is above the free-flight plants' ceiling",
            });
        }
        let spawn = takeover_spawn(&state, self.preferred_vehicle, &self.surface);
        self.respawn_free(spawn)?;
        info!(
            scenario = profile.title(),
            vehicle = self.preferred_vehicle.title(),
            "took manual control"
        );
        Ok(())
    }
}

/// A spawn reproducing `state`'s position and heading for `vehicle`.
fn takeover_spawn(
    state: &VehicleState,
    vehicle: FreeFlightVehicle,
    surface: &WorldSurfaceHandle,
) -> FreeSpawn {
    let spawn = ecef_to_geodetic(state.position_ecef);
    let ground_m = surface
        .ellipsoidal_ground_m(&spawn)
        .unwrap_or(spawn.height_m - X500_HOVER_AGL_M);
    // The X500 spawns hovering a fixed height over the field it is given, so hand it the
    // field that puts the hover exactly at the scenario's height.
    let field_elev_m = if vehicle == FreeFlightVehicle::AviateX500Direct {
        spawn.height_m - X500_HOVER_AGL_M
    } else {
        ground_m.min(spawn.height_m)
    };
    FreeSpawn::At {
        spawn,
        heading_rad: heading_rad(state),
        field_elev_m,
    }
}

/// True heading of `state`'s nose, radians from north.
pub(super) fn heading_rad(state: &VehicleState) -> f64 {
    let local = LocalFrame::at(&ecef_to_geodetic(state.position_ecef));
    let nose = state.attitude * DVec3::X;
    nose.dot(local.east()).atan2(nose.dot(local.north()))
}

impl Flight {
    /// Respawns the preferred vehicle at any airport of the airport database, at its
    /// runway end `runway_end` (else its first) in `mode`; later free flights and restarts
    /// start there too. A built-in OpenStreetMap aerodrome spawns from its mapped diagram;
    /// any other airport's runways are laid into the world surface first.
    pub fn spawn_at_airport_record(
        &mut self,
        record: &AirportRecord,
        runway_end: usize,
        mode: SpawnMode,
    ) -> Result<(), AppSimError> {
        let datum = self.surface.vertical_datum();
        if let Some(diagram) = record
            .osm_aerodrome()
            .and_then(|id| OsmAerodromeDiagram::builtin(id).ok())
        {
            self.surface.set_extra_runways(Vec::new());
            // The record numbers its surveyed runway ends; the diagram its mapped ones.
            let designator = record
                .to_flightbag_graph(datum)
                .runway_ends
                .get(runway_end)
                .map(|end| end.designator);
            let mapped = diagram
                .surface_graph
                .runway_ends
                .iter()
                .position(|end| Some(end.designator) == designator);
            return self.spawn_at_aerodrome(&diagram, mapped.unwrap_or(runway_end), mode);
        }
        self.surface.set_extra_runways(record.runway_strips(datum));
        let site = SpawnSite::of_record(record, runway_end, datum);
        self.respawn_free(site.spawn(mode, &self.surface))?;
        info!(
            airport = record.ids.primary.as_str(),
            mode = mode.title(),
            "spawned at airport"
        );
        Ok(())
    }

    /// Respawns the preferred vehicle over `record`'s first runway end.
    #[cfg(test)]
    pub fn respawn_at_airport_record(&mut self, record: &AirportRecord) -> Result<(), AppSimError> {
        self.spawn_at_airport_record(record, 0, SpawnMode::Overhead)
    }

    /// Respawns on `record`'s runway `designator` at longitudinal `station_m` relative to
    /// its surveyed landing threshold (`< 0` on approach glidepath, `>= 0` on ground rollout).
    #[cfg(test)]
    pub fn spawn_at_landing_station(
        &mut self,
        record: &AirportRecord,
        designator: &str,
        station_m: f64,
    ) -> Result<(), AppSimError> {
        use sim_geo::{Geodetic, geodetic_to_ecef};
        let datum = self.surface.vertical_datum();
        if record.osm_aerodrome().is_some() {
            self.surface.set_extra_runways(Vec::new());
        } else {
            self.surface.set_extra_runways(record.runway_strips(datum));
        }
        let landing =
            record
                .landing_runway(designator, datum)
                .ok_or_else(|| AppSimError::Screenshot {
                    detail: format!("missing runway {designator} at {}", record.ids.primary),
                })?;
        let thr_frame = LocalFrame::at(&landing.threshold);
        let (sin_b, cos_b) = landing.true_bearing_rad.sin_cos();
        let ahead = thr_frame.north() * cos_b + thr_frame.east() * sin_b;
        let moved = ecef_to_geodetic(geodetic_to_ecef(&landing.threshold) + ahead * station_m);
        let dest_frame = LocalFrame::at(&moved);
        let heading_rad = ahead
            .dot(dest_frame.east())
            .atan2(ahead.dot(dest_frame.north()))
            .rem_euclid(std::f64::consts::TAU);
        let spawn = if station_m < 0.0 {
            let thr_ground = self
                .surface
                .ellipsoidal_ground_m(&landing.threshold)
                .unwrap_or(landing.threshold.height_m);
            let above_m = (-station_m) * landing.glidepath_rad.tan() + 15.0;
            let pos = Geodetic {
                height_m: thr_ground + above_m,
                ..moved
            };
            FreeSpawn::At {
                spawn: pos,
                heading_rad,
                field_elev_m: self
                    .surface
                    .ellipsoidal_ground_m(&pos)
                    .unwrap_or(thr_ground),
            }
        } else {
            let frac = (station_m / landing.landing_length_m.max(1.0)).clamp(0.0, 1.0);
            let charted_h = landing.threshold.height_m
                + (landing.far_end.height_m - landing.threshold.height_m) * frac;
            let base = Geodetic {
                height_m: charted_h,
                ..moved
            };
            let ground_h = self
                .surface
                .ellipsoidal_ground_m(&base)
                .unwrap_or(charted_h);
            FreeSpawn::Ground {
                spawn: Geodetic {
                    height_m: ground_h,
                    ..moved
                },
                heading_rad,
                cold: false,
            }
        };
        self.respawn_free(spawn)
    }
}
