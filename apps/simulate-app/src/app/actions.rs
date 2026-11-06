//! Discrete operator commands from the keyboard and the `Esc` menu.

use sim_scenario::ScenarioProfile;
use tracing::{info, warn};
use winit::event_loop::ActiveEventLoop;

use super::FlightApp;
use super::hub::shown_layout;
use crate::error::AppSimError;
use crate::esc_menu::catalog::VehicleLaunch;
use crate::esc_menu::{MenuCommand, MenuInput, MenuOutcome, Step};
use crate::keyboard::{CockpitAction, apply_aircraft_system_action};
use crate::map_rig::{EfbCommand, MapLibreRig};

impl FlightApp {
    pub(super) fn select(
        &mut self,
        event_loop: &ActiveEventLoop,
        profile: Option<ScenarioProfile>,
    ) {
        let name = profile.map_or("free flight", ScenarioProfile::title);
        info!(scenario = name, "selected");
        if let Err(err) = self.flight.select(profile) {
            self.fail(event_loop, "the scenario could not start", &err);
            return;
        }
        self.after_spawn();
        self.invalidate_chart();
        self.follow_supervisor_airport();
    }

    /// Points the EFB back at the supervisor's active aerodrome.
    pub(super) fn follow_supervisor_airport(&mut self) {
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.set_efb_airport(None);
        }
    }

    /// Leaves the scripted scenario for hand flying at its live state, with the throttle
    /// set to the vehicle's airborne trim.
    pub(super) fn take_manual_control(&mut self, event_loop: &ActiveEventLoop) {
        if self.flight.profile().is_none() {
            return;
        }
        match self.flight.take_manual_control() {
            Ok(()) => {
                self.after_spawn();
                self.invalidate_chart();
            }
            Err(err @ AppSimError::ManualControlUnavailable { .. }) => {
                warn!(%err, "manual control refused");
            }
            Err(err) => self.fail(event_loop, "taking manual control failed", &err),
        }
    }

    fn invalidate_chart(&mut self) {
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.invalidate_chart();
        }
    }

    fn cycle_avionics_preset(&mut self, event_loop: &ActiveEventLoop) {
        let Some(rig) = self.cockpit_rig.as_mut() else {
            return;
        };
        match rig.cycle_avionics_preset() {
            Ok(Some(preset)) => info!(preset = preset.title(), "selected avionics preset"),
            Ok(None) => {}
            Err(err) => self.fail(event_loop, "switching avionics preset failed", &err),
        }
    }

    /// Steps the free-flight airport and respawns over its active runway.
    fn switch_airport(&mut self, event_loop: &ActiveEventLoop, step: Step) {
        let Some(rig) = self.cockpit_rig.as_mut() else {
            return;
        };
        let id = rig.supervisor_mut().step_aerodrome(step == Step::Next);
        rig.invalidate_chart();
        rig.set_efb_airport(None);
        let Some(diag) = rig.supervisor().diagram(id).cloned() else {
            return;
        };
        if let Err(err) = self.flight.respawn_at_diagram(&diag) {
            self.fail(event_loop, "respawning at airport failed", &err);
            return;
        }
        self.after_spawn();
        info!(
            airport = diag.id.code(),
            name = diag.name,
            "switched active airport"
        );
    }

    /// Steps the free-flight vehicle and enters free flight on it.
    fn switch_vehicle(&mut self, event_loop: &ActiveEventLoop, step: Step) {
        let current = self.flight.preferred_vehicle();
        let vehicle = match step {
            Step::Next => current.next(),
            Step::Prev => current.prev(),
        };
        match self.flight.select_vehicle(vehicle) {
            Ok(()) => {
                self.after_spawn();
                info!(vehicle = vehicle.title(), "selected free-flight vehicle");
            }
            Err(err) => self.fail(event_loop, "switching free-flight vehicle failed", &err),
        }
    }

    pub(super) fn handle_cockpit_action(
        &mut self,
        event_loop: &ActiveEventLoop,
        action: CockpitAction,
    ) {
        match action {
            CockpitAction::CycleAirport => self.switch_airport(event_loop, Step::Next),
            CockpitAction::CycleVehicle => self.switch_vehicle(event_loop, Step::Next),
            CockpitAction::CycleAvionicsPreset => self.cycle_avionics_preset(event_loop),
            CockpitAction::CycleSkyTime => {
                let mode = self.sky.cycle(Step::Next);
                info!(mode = mode.label(), "selected sky ephemeris time");
            }
            CockpitAction::CycleSensorRealism => {
                let realism = self.flight.cycle_sensor_realism();
                info!(realism = realism.title(), "selected onboard sensor realism");
            }
            CockpitAction::ToggleEfb => self.efb_command(EfbCommand::ToggleExpanded),
            CockpitAction::CycleMagnetos if self.flight.profile().is_some() => {
                self.take_manual_control(event_loop);
            }
            CockpitAction::FlapsDown
            | CockpitAction::FlapsUp
            | CockpitAction::CycleMagnetos
            | CockpitAction::MixtureRich
            | CockpitAction::MixtureLean => {
                apply_aircraft_system_action(action, &mut self.flight);
            }
            CockpitAction::CycleViewport
            | CockpitAction::CycleVirtualCamera
            | CockpitAction::ToggleInstruments
            | CockpitAction::ToggleHud
            | CockpitAction::TogglePause
            | CockpitAction::CycleWarp => self.handle_view_action(action),
        }
        if matches!(
            action,
            CockpitAction::ToggleInstruments
                | CockpitAction::ToggleHud
                | CockpitAction::CycleAvionicsPreset
        ) {
            self.sync_hub_layout();
        }
    }

    /// Mirrors the cockpit's instrument fit into the hub after a hotkey changed it.
    fn sync_hub_layout(&mut self) {
        if let Some(rig) = self.cockpit_rig.as_ref() {
            self.hub.layout = shown_layout(
                rig.shows_instruments(),
                rig.maps().map(MapLibreRig::preset),
                self.hub.layout,
            );
        }
    }

    fn handle_view_action(&mut self, action: CockpitAction) {
        let Some(rig) = self.cockpit_rig.as_mut() else {
            return;
        };
        match action {
            CockpitAction::CycleViewport => {
                let mode = rig.supervisor_mut().cycle_viewport_mode();
                self.look.reset();
                rig.invalidate_chart();
                info!(?mode, "switched operator viewport mode");
            }
            CockpitAction::CycleVirtualCamera => {
                let spec = rig
                    .supervisor_mut()
                    .cycle_virtual_camera(self.flight.profile());
                self.look.reset();
                info!(camera = spec.title, id = spec.id, "selected virtual camera");
            }
            CockpitAction::ToggleInstruments => {
                let visible = rig.toggle_instruments();
                info!(visible, "toggled glass-cockpit instrument dock");
            }
            CockpitAction::ToggleHud => {
                let visible = rig.toggle_conformal_hud();
                info!(visible, "toggled conformal HUD overlay");
            }
            CockpitAction::TogglePause => {
                let scale = rig.supervisor_mut().toggle_pause();
                self.flight.note_wall_break();
                info!(?scale, "toggled simulation pause");
            }
            CockpitAction::CycleWarp => {
                let scale = rig.supervisor_mut().cycle_warp();
                info!(?scale, "updated simulation time warp");
            }
            _ => {}
        }
    }

    pub(super) fn handle_menu_input(&mut self, event_loop: &ActiveEventLoop, input: MenuInput) {
        let view = self.hub_view();
        let outcome = self.esc_menu.handle(input, &view, &self.catalog);
        self.apply_menu_outcome(event_loop, outcome);
    }

    pub(super) fn apply_menu_outcome(
        &mut self,
        event_loop: &ActiveEventLoop,
        outcome: MenuOutcome,
    ) {
        match outcome {
            MenuOutcome::Idle => {}
            MenuOutcome::Opened => {
                self.keyboard
                    .release_all(&mut self.mapper, &mut self.flight);
                self.flight.note_wall_break();
                if let Some(index) = self.flying_vehicle_index() {
                    self.esc_menu.select_vehicle(index);
                }
                self.hub.drag = None;
                info!("opened flight hub; simulation held");
            }
            MenuOutcome::Closed => {
                self.controls.cancel_capture();
                self.controls.clear_preview();
                self.controls.sync_live(self.mapper.controls());
                self.hub.drag = None;
                self.hub_preview = None;
                self.flight.note_wall_break();
                info!("closed flight hub; simulation resumed");
            }
            MenuOutcome::Command(command) => self.apply_menu_command(event_loop, command),
        }
    }

    /// Registry index of the vehicle flying now.
    fn flying_vehicle_index(&self) -> Option<usize> {
        let launch = match (self.flight.free_vehicle(), self.flight.profile()) {
            (Some(vehicle), _) => VehicleLaunch::Free(vehicle),
            (None, Some(profile)) => VehicleLaunch::Scenario(profile),
            (None, None) => return None,
        };
        self.catalog
            .vehicles
            .iter()
            .position(|v| v.launch == launch)
    }

    fn close_menu(&mut self, event_loop: &ActiveEventLoop) {
        if self.esc_menu.is_open() {
            let view = self.hub_view();
            let outcome = self.esc_menu.handle(MenuInput::Back, &view, &self.catalog);
            self.apply_menu_outcome(event_loop, outcome);
        }
    }

    fn apply_menu_command(&mut self, event_loop: &ActiveEventLoop, command: MenuCommand) {
        match command {
            MenuCommand::Resume => self.close_menu(event_loop),
            MenuCommand::Rewind { seconds } => match self
                .flight
                .rewind_by_seconds(f64::from(seconds))
            {
                Ok(_) => self.invalidate_chart(),
                Err(err @ AppSimError::NoCheckpoint { .. }) => warn!(%err, "nothing to rewind to"),
                Err(err) => self.fail(event_loop, "rewinding the flight failed", &err),
            },
            MenuCommand::Restart => match self.flight.restart() {
                Ok(()) => {
                    self.after_spawn();
                    self.invalidate_chart();
                    info!("restarted flight");
                }
                Err(err) => self.fail(event_loop, "restarting the flight failed", &err),
            },
            MenuCommand::TakeManualControl => {
                self.take_manual_control(event_loop);
                if self.flight.profile().is_none() {
                    self.close_menu(event_loop);
                }
            }
            MenuCommand::Sky(step) => {
                let mode = self.sky.cycle(step);
                info!(mode = mode.label(), "selected sky ephemeris time");
            }
            MenuCommand::Quit => event_loop.exit(),
            other => {
                if self.apply_hub_command(event_loop, other) && !self.failed {
                    self.close_menu(event_loop);
                }
            }
        }
    }
}
