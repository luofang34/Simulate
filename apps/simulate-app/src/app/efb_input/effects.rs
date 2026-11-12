//! Host side-effects, 3D terrain threshold raycasting, and frame-by-frame LNAV/autoland
//! guidance stepping for [`crate::efb_plan::EfbWorkspace`].

use sim_core::ControlInputs;
use sim_geo::{BodyEllipsoid, LocalFrame, ecef_to_geodetic};
use sim_input::Control;
use sim_maplibre::EfbDisplayMode;
use sim_scenario::SimTimeScale;
use sim_scenery::{AirportDatabase, CatalogVerticalDatum};
use tracing::{info, warn};

use super::super::FlightApp;
use crate::efb_plan::{EfbAutopilotPhase, EfbHostEffect, EfbPlanAction, EfbTab};
use crate::esc_menu::SceneryMode;
use crate::map_rig::EfbCommand;

const KNOTS_PER_M_S: f64 = 1.943_844_492_440_6;
const MANUAL_OVERRIDE_STICK: f64 = 0.28;

impl FlightApp {
    /// Ownship `[lat_deg, lon_deg]` and horizontal ground speed in knots, read from the
    /// onboard navigation estimate rather than simulation truth.
    pub(in crate::app) fn ownship_deg_and_gs_kt(&self) -> ([f64; 2], f64) {
        let est = self.flight.estimated_vehicle_state();
        let geo = ecef_to_geodetic(est.position_ecef);
        let frame = LocalFrame::at(&geo);
        let enu_v = frame.ecef_to_enu_vector(est.velocity_ecef);
        let gs_kt = enu_v.x.hypot(enu_v.y) * KNOTS_PER_M_S;
        ([geo.lat_deg(), geo.lon_deg()], gs_kt)
    }

    /// Publishes the active `EfbWorkspace` flight plan and leg progress into `CockpitRig`.
    pub(in crate::app) fn sync_efb_route_to_rig(&mut self) {
        let (_, gs_kt) = self.ownship_deg_and_gs_kt();
        let (plan, progress) = self.efb_workspace.maplibre_plan(gs_kt.max(105.0));
        let dest_plate = self
            .efb_workspace
            .tokens
            .last()
            .and_then(|tok| tok.airport_and_runway())
            .map(|(code, rwy)| (code.to_owned(), rwy.unwrap_or("").to_owned()));
        if let Some(rig) = self.cockpit_rig.as_mut() {
            if let Err(err) = rig.set_efb_route(&plan, progress) {
                warn!(error = %err, "updating EFB route portrayal failed");
            }
            if let Some(plate) = dest_plate {
                rig.set_efb_airport(Some(plate));
            }
        }
    }

    /// Applies an [`EfbPlanAction`] on `self.efb_workspace` and executes any host effect.
    pub(super) fn apply_efb_plan_action(&mut self, action: EfbPlanAction) {
        let (own_deg, _) = self.ownship_deg_and_gs_kt();
        let surface = self.flight.world_surface().clone();
        let effect = self
            .efb_workspace
            .apply_action(action, own_deg, &mut self.catalog, &surface);
        match effect {
            EfbHostEffect::RouteUpdated => self.sync_efb_route_to_rig(),
            EfbHostEffect::JumpMap {
                lat_deg,
                lon_deg,
                zoom,
            } => {
                self.sync_efb_route_to_rig();
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    rig.efb_jump_to(lat_deg, lon_deg, zoom);
                }
            }
            EfbHostEffect::EnterWorld3dPicker => {
                self.sync_efb_route_to_rig();
                self.efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner));
            }
            EfbHostEffect::CreatedAdHoc { code, focus_deg } => {
                self.sync_ad_hoc_to_rig();
                self.sync_efb_route_to_rig();
                if let (Some(rig), Some(&lat), Some(&lon)) = (
                    self.cockpit_rig.as_mut(),
                    focus_deg.first(),
                    focus_deg.get(1),
                ) {
                    rig.efb_jump_to(lat, lon, Some(14.2));
                }
                info!(airport = code, "created and registered ad-hoc runway");
            }
            EfbHostEffect::SpawnAtRunway {
                code,
                designator,
                station_m,
                engage_autoland,
            } => {
                self.spawn_from_efb(&code, &designator, station_m, engage_autoland);
            }
        }
    }

    fn spawn_from_efb(
        &mut self,
        code: &str,
        designator: &str,
        station_m: f64,
        engage_autoland: bool,
    ) {
        let datum = self.cockpit_rig.as_ref().map_or_else(
            || match self.hub.scenery {
                SceneryMode::Online => CatalogVerticalDatum::OrthometricMslEgm96,
                SceneryMode::Offline => CatalogVerticalDatum::EllipsoidWgs84,
            },
            |rig| rig.supervisor().vertical_datum(),
        );
        let resolved = if let Some(adhoc) = self.catalog.ad_hoc_by_code(code).cloned() {
            let end = end_index_for(&adhoc.airport, datum, designator);
            if let Some(rig) = self.cockpit_rig.as_mut() {
                rig.register_extra_airport(adhoc.airport.clone(), &adhoc.diagram_geojson);
                rig.supervisor_mut()
                    .set_custom_airport(adhoc.airport.clone(), end);
            }
            Some(adhoc.airport)
        } else if let Some((idx, record)) = AirportDatabase::builtin().by_code(code) {
            let end = end_index_for(record, datum, designator);
            if let Some(rig) = self.cockpit_rig.as_mut() {
                rig.supervisor_mut().set_free_airport(idx, end);
            }
            Some(record.clone())
        } else {
            None
        };
        let Some(record) = resolved else {
            self.efb_workspace.status_banner = format!("AIRPORT {code} NOT FOUND");
            return;
        };
        if let Err(err) = self
            .flight
            .spawn_at_landing_station(&record, designator, station_m)
        {
            warn!(error = %err, code, designator, "EFB runway spawn failed");
            self.efb_workspace.status_banner = format!("SPAWN FAILED AT {code}/{designator}");
            return;
        }
        self.after_spawn();
        if engage_autoland {
            self.efb_workspace.autopilot.engage();
            if station_m < 0.0 {
                self.flight.update_system_controls(|sys| {
                    sys.flaps_norm = 0.67;
                    sys.wheel_brake_norm = 0.0;
                });
            }
        } else {
            self.efb_workspace.autopilot.disengage();
        }
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.invalidate_chart();
            rig.set_efb_airport(Some((code.to_owned(), designator.to_owned())));
        }
        self.sync_efb_route_to_rig();
    }

    /// Captures a threshold click directly in the 3D viewport via raycast against the
    /// authoritative terrain/bridge surface.
    pub(in crate::app) fn world_3d_pick_pressed(&mut self) {
        let (mode, _) = self.view_and_time_scale();
        let body = self.flight.world().body();
        let Ok(base_cam) = self.flight.display_camera(mode) else {
            return;
        };
        let camera = self
            .look
            .apply_on(body, mode, base_cam, &self.flight.display_vehicle_state());
        let Some((hit, _range_m)) = self.flight.world_surface().raycast_cursor_geodetic(
            &camera,
            self.cursor_px,
            self.surface_size(),
        ) else {
            self.efb_workspace.status_banner =
                "3D PICK MISSED TERRAIN - CLICK GROUND OR BRIDGE".to_owned();
            return;
        };
        let completed = self
            .efb_workspace
            .ad_hoc
            .record_point(hit.lat_deg(), hit.lon_deg());
        self.efb_workspace
            .status_banner
            .clone_from(&self.efb_workspace.ad_hoc.status);
        if completed {
            self.efb_workspace.tab = EfbTab::AdHocRwy;
            self.efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb));
        }
    }

    /// Updates lateral leg sequencing and steps `EfbAutopilot` in free flight, returning the
    /// controls to feed into `Flight::advance_to_now`.
    pub(in crate::app) fn step_efb_guidance(
        &mut self,
        manual: ControlInputs,
        time_scale: SimTimeScale,
    ) -> ControlInputs {
        let (own_deg, _) = self.ownship_deg_and_gs_kt();
        let prev_progress = self.efb_workspace.progress;
        self.efb_workspace.update_sequencing(own_deg);
        if self.efb_workspace.progress != prev_progress {
            self.sync_efb_route_to_rig();
        }
        let can_couple = self.flight.profile().is_none()
            && !self.flight.in_space()
            && !self.flight.shows_page()
            && self.flight.world().body() == BodyEllipsoid::WGS84;
        if !can_couple {
            self.efb_workspace.autopilot.disengage();
        }
        if self.efb_workspace.autopilot.is_engaged()
            && (manual.roll.abs() > MANUAL_OVERRIDE_STICK
                || manual.pitch.abs() > MANUAL_OVERRIDE_STICK)
        {
            self.efb_workspace.autopilot.disengage();
            self.efb_workspace.status_banner = "AP DISCONNECTED - MANUAL STICK OVERRIDE".to_owned();
        }
        let controls =
            if self.efb_workspace.autopilot.is_engaged() && time_scale != SimTimeScale::Paused {
                self.compute_autopilot_controls(manual, time_scale)
            } else {
                manual
            };
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.set_efb_autopilot_phase(self.efb_workspace.autopilot.phase());
        }
        controls
    }

    fn compute_autopilot_controls(
        &mut self,
        fallback: ControlInputs,
        time_scale: SimTimeScale,
    ) -> ControlInputs {
        let est = self.flight.estimated_vehicle_state();
        let art = self.flight.articulation();
        let geo = ecef_to_geodetic(est.position_ecef);
        let surface = self.flight.world_surface();
        let undulation_m = surface.geoid_undulation_m(geo.lat_rad, geo.lon_rad);
        let landing_rwy = self
            .efb_workspace
            .destination_landing_runway(&self.catalog, surface)
            .map(|(_, geom)| geom);
        let dt_s = (1.0 / 60.0) * time_scale.factor().clamp(0.1, 4.0);
        let Some(out) = self.efb_workspace.autopilot.step(
            &est,
            &art,
            &self.efb_workspace.tokens,
            self.efb_workspace.progress,
            landing_rwy.as_ref(),
            undulation_m,
            dt_s,
        ) else {
            return fallback;
        };
        let stopped = out.phase == EfbAutopilotPhase::Rollout && est.velocity_ecef.length() < 0.25;
        self.flight.update_system_controls(|sys| {
            sys.flaps_norm = out.flaps_norm;
            sys.wheel_brake_norm = if stopped { 1.0 } else { out.wheel_brake_norm };
        });
        self.mapper.set(Control::Throttle, out.controls.throttle);
        out.controls
    }
}

fn end_index_for(
    record: &sim_scenery::AirportRecord,
    datum: CatalogVerticalDatum,
    designator: &str,
) -> usize {
    record
        .to_flightbag_graph(datum)
        .runway_ends
        .iter()
        .position(|e| e.designator.as_str() == designator)
        .unwrap_or(0)
}
