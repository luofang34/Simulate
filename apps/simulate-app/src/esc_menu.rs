//! The `Esc` Flight Hub: five tabs (flight & timeline, aircraft hangar, world & aerodromes,
//! scenarios, controls) driven by keyboard, gamepad, or mouse.
//!
//! [`EscMenu`] is a pure state machine turning [`MenuInput`]s into [`MenuOutcome`]s against
//! the [`HubView`] last laid out for it; the app applies the commands. [`view`] lays the
//! tabs out as hit-testable widgets, [`scene`] tessellates a view, and [`MenuOverlay`]
//! draws it on the GPU at the swapchain's physical resolution.

pub mod catalog;
mod flight_chrome;
mod keymap;
mod navigate;
mod render;
pub mod scene;
#[cfg(test)]
mod tests;
pub mod view;

use crate::vehicles::FreeFlightVehicle;
pub use catalog::{AvionicsLayout, HubCatalog, RateChoice, SceneryMode, SpawnMode};
pub use keymap::{menu_input_for_code, menu_input_for_key};
pub use render::{MenuOverlay, OverlayContent};
pub use scene::Highlights;
pub use view::{ControlsStatus, HubView, MenuCommand, MenuStatus, WidgetId};

/// One page of the hub.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum HubTab {
    /// Resume, restart, rewind, rate, sky, avionics, scenery, and pacing.
    #[default]
    Flight,
    /// Vehicle registry with the live 3D preview.
    Hangar,
    /// Aerodrome search, runway ends, and spawn modes.
    World,
    /// Categorized scenario browser.
    Scenarios,
    /// Control presets, live telemetry, and rebinding.
    Controls,
}

impl HubTab {
    /// Every tab in bar order.
    pub const ALL: [Self; 5] = [
        Self::Flight,
        Self::Hangar,
        Self::World,
        Self::Scenarios,
        Self::Controls,
    ];

    /// Tab bar label.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::Flight => "1 FLIGHT & TIMELINE",
            Self::Hangar => "2 AIRCRAFT HANGAR",
            Self::World => "3 WORLD & AERODROMES",
            Self::Scenarios => "4 SCENARIOS",
            Self::Controls => "5 CONTROLS & PRESETS",
        }
    }

    fn index(self) -> usize {
        Self::ALL.iter().position(|&t| t == self).unwrap_or(0)
    }

    fn offset(self, forward: bool) -> Self {
        let count = Self::ALL.len();
        let next = if forward {
            self.index().wrapping_add(1) % count
        } else {
            self.index().wrapping_add(count.saturating_sub(1)) % count
        };
        Self::ALL.get(next).copied().unwrap_or(self)
    }
}

/// Direction a selector or stepper steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Step {
    /// The previous value.
    Prev,
    /// The next value.
    Next,
}

/// A discrete hub input from any device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuInput {
    /// Open or close the hub (`Escape`, gamepad `Start`).
    Toggle,
    /// Move focus up.
    Up,
    /// Move focus down.
    Down,
    /// Move focus left.
    Left,
    /// Move focus right.
    Right,
    /// Activate the focused widget.
    Confirm,
    /// Close the hub (gamepad `East`).
    Back,
    /// Next tab (`Tab`, gamepad `RB`).
    NextTab,
    /// Previous tab (`Shift+Tab`, gamepad `LB`).
    PrevTab,
    /// Jump to a tab (`F1`–`F5`).
    Tab(HubTab),
    /// A typed character for the aerodrome search.
    Char(char),
    /// Delete the last search character.
    Backspace,
    /// Scroll the tab's list by this many rows (positive is down).
    Scroll(i32),
}

/// The result of feeding the hub one input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuOutcome {
    /// Nothing for the app to do.
    Idle,
    /// The hub just opened.
    Opened,
    /// The hub just closed.
    Closed,
    /// The hub stays open and the app must apply this command.
    Command(MenuCommand),
}

/// Open state, tab, focus, hover, list selections, search text, and spawn choices.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EscMenu {
    open: bool,
    tab: HubTab,
    focus: Option<WidgetId>,
    hovered: Option<WidgetId>,
    vehicle: usize,
    vehicle_mode: Option<FreeFlightVehicle>,
    vehicle_scroll: usize,
    aerodrome: usize,
    aerodrome_scroll: usize,
    search: String,
    runway: usize,
    spawn_mode: Option<SpawnMode>,
    scenario: usize,
    scenario_scroll: usize,
}

impl EscMenu {
    /// Whether the hub is shown (and the aircraft receives no input).
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.open
    }

    /// Shown tab.
    #[must_use]
    pub const fn tab(&self) -> HubTab {
        self.tab
    }

    /// Widget with keyboard / gamepad focus.
    #[cfg(test)]
    #[must_use]
    pub const fn focus(&self) -> Option<WidgetId> {
        self.focus
    }

    /// Widget under the mouse.
    #[cfg(test)]
    #[must_use]
    pub const fn hovered(&self) -> Option<WidgetId> {
        self.hovered
    }

    /// Selected registry vehicle.
    #[must_use]
    pub const fn vehicle(&self) -> usize {
        self.vehicle
    }

    /// Flight-dynamics mode the operator chose for the selected airframe, if any.
    #[must_use]
    pub const fn vehicle_mode(&self) -> Option<FreeFlightVehicle> {
        self.vehicle_mode
    }

    /// First listed registry row.
    #[must_use]
    pub const fn vehicle_scroll(&self) -> usize {
        self.vehicle_scroll
    }

    /// Selected catalog aerodrome.
    #[must_use]
    pub const fn aerodrome(&self) -> usize {
        self.aerodrome
    }

    /// First listed filtered aerodrome row.
    #[must_use]
    pub const fn aerodrome_scroll(&self) -> usize {
        self.aerodrome_scroll
    }

    /// Aerodrome search text.
    #[must_use]
    pub fn search(&self) -> &str {
        &self.search
    }

    /// Selected runway end.
    #[must_use]
    pub const fn runway(&self) -> usize {
        self.runway
    }

    /// Selected spawn mode, if the operator picked one.
    #[must_use]
    pub const fn spawn_mode(&self) -> Option<SpawnMode> {
        self.spawn_mode
    }

    /// Selected catalog scenario.
    #[must_use]
    pub const fn scenario(&self) -> usize {
        self.scenario
    }

    /// First listed scenario row.
    #[must_use]
    pub const fn scenario_scroll(&self) -> usize {
        self.scenario_scroll
    }

    /// Focus and hover to paint.
    #[must_use]
    pub const fn highlights(&self) -> Highlights {
        Highlights {
            focus: self.focus,
            hovered: self.hovered,
        }
    }

    /// Highlights `vehicle` in the hangar (the vehicle now flying).
    pub fn select_vehicle(&mut self, vehicle: usize) {
        self.vehicle = vehicle;
    }

    /// Selects the airframe flying `active`, showing `active` as its mode.
    pub fn sync_vehicle(&mut self, catalog: &HubCatalog, active: FreeFlightVehicle) {
        if let Some(index) = catalog.airframe_of(active) {
            self.vehicle = index;
            self.vehicle_mode = Some(active);
        }
    }

    /// Feeds one input against `view`, the hub as last laid out.
    pub fn handle(
        &mut self,
        input: MenuInput,
        view: &HubView,
        catalog: &HubCatalog,
    ) -> MenuOutcome {
        if !self.open {
            return if input == MenuInput::Toggle {
                self.open = true;
                self.vehicle_mode = None;
                self.focus = None;
                self.hovered = None;
                MenuOutcome::Opened
            } else {
                MenuOutcome::Idle
            };
        }
        match input {
            MenuInput::Toggle | MenuInput::Back => self.close(),
            MenuInput::Up => self.navigate(view, catalog, navigate::Dir::Up),
            MenuInput::Down => self.navigate(view, catalog, navigate::Dir::Down),
            MenuInput::Left => self.navigate(view, catalog, navigate::Dir::Left),
            MenuInput::Right => self.navigate(view, catalog, navigate::Dir::Right),
            MenuInput::Confirm => match self.focus {
                Some(id) => self.activate(id, catalog),
                None => MenuOutcome::Idle,
            },
            MenuInput::NextTab => self.set_tab(self.tab.offset(true)),
            MenuInput::PrevTab => self.set_tab(self.tab.offset(false)),
            MenuInput::Tab(tab) => self.set_tab(tab),
            MenuInput::Char(ch) => self.type_char(ch, catalog),
            MenuInput::Backspace => {
                if self.tab == HubTab::World && self.search.pop().is_some() {
                    self.after_search(catalog);
                }
                MenuOutcome::Idle
            }
            MenuInput::Scroll(rows) => {
                self.scroll(rows, catalog);
                MenuOutcome::Idle
            }
        }
    }

    /// Highlights the widget under `cursor_px` (physical window pixels).
    pub fn hover_at(&mut self, cursor_px: [f64; 2], view: &HubView) {
        if self.open {
            self.hovered = view.hit(cursor_px);
        }
    }

    /// Activates the widget under `cursor_px`.
    pub fn click_at(
        &mut self,
        cursor_px: [f64; 2],
        view: &HubView,
        catalog: &HubCatalog,
    ) -> MenuOutcome {
        if !self.open {
            return MenuOutcome::Idle;
        }
        match view.hit(cursor_px) {
            Some(WidgetId::TimelineBar) => {
                self.focus = Some(WidgetId::TimelineBar);
                view.timeline_permille(cursor_px)
                    .map_or(MenuOutcome::Idle, |permille| {
                        let scrub = view::TimelineScrub::Permille(permille);
                        MenuOutcome::Command(MenuCommand::Scrub(scrub))
                    })
            }
            Some(id) => {
                self.focus = Some(id);
                self.activate(id, catalog)
            }
            None => MenuOutcome::Idle,
        }
    }

    fn close(&mut self) -> MenuOutcome {
        self.open = false;
        self.hovered = None;
        MenuOutcome::Closed
    }

    fn set_tab(&mut self, tab: HubTab) -> MenuOutcome {
        self.tab = tab;
        self.focus = Some(WidgetId::Tab(tab));
        self.hovered = None;
        MenuOutcome::Idle
    }

    fn type_char(&mut self, ch: char, catalog: &HubCatalog) -> MenuOutcome {
        if self.tab != HubTab::World || ch.is_control() || self.search.len() >= view::SEARCH_MAX {
            return MenuOutcome::Idle;
        }
        self.search.push(ch.to_ascii_uppercase());
        self.after_search(catalog);
        MenuOutcome::Idle
    }

    /// Keeps the aerodrome selection on a match after the filter changed.
    fn after_search(&mut self, catalog: &HubCatalog) {
        self.aerodrome_scroll = 0;
        let matches = catalog.filter_aerodromes(&self.search);
        if !matches.contains(&self.aerodrome)
            && let Some(&first) = matches.first()
        {
            self.pick_aerodrome(first);
        }
    }

    /// Selects catalog aerodrome `index` (a globe pick, or the airport flying now).
    pub fn select_aerodrome(&mut self, index: usize) {
        self.pick_aerodrome(index);
    }

    fn pick_aerodrome(&mut self, index: usize) {
        if self.aerodrome != index {
            self.runway = 0;
            self.spawn_mode = None;
        }
        self.aerodrome = index;
    }

    fn scroll(&mut self, rows: i32, catalog: &HubCatalog) {
        let (scroll, len, visible) = match self.tab {
            HubTab::Hangar => (
                &mut self.vehicle_scroll,
                catalog.vehicles.len(),
                view::VEHICLE_ROWS,
            ),
            HubTab::World => (
                &mut self.aerodrome_scroll,
                catalog.filter_aerodromes(&self.search).len(),
                view::AERODROME_ROWS,
            ),
            HubTab::Scenarios => (
                &mut self.scenario_scroll,
                catalog.scenarios.len(),
                view::SCENARIO_ROWS,
            ),
            HubTab::Flight | HubTab::Controls => return,
        };
        let max = len.saturating_sub(visible);
        let moved = i64::try_from(*scroll).unwrap_or(0) + i64::from(rows);
        *scroll = usize::try_from(moved.max(0)).unwrap_or(0).min(max);
    }

    fn activate(&mut self, id: WidgetId, catalog: &HubCatalog) -> MenuOutcome {
        let command = match id {
            WidgetId::Tab(tab) => return self.set_tab(tab),
            WidgetId::Resume => return self.close(),
            WidgetId::Search => return MenuOutcome::Idle,
            WidgetId::VehicleRow(index) => {
                if self.vehicle != index {
                    self.vehicle_mode = None;
                }
                self.vehicle = index;
                MenuCommand::PreviewVehicle(index)
            }
            WidgetId::AerodromeRow(index) => {
                self.pick_aerodrome(index);
                return MenuOutcome::Idle;
            }
            WidgetId::RunwayEnd(index) => {
                self.runway = index;
                return MenuOutcome::Idle;
            }
            WidgetId::SpawnChoice(mode) => {
                self.spawn_mode = Some(mode);
                return MenuOutcome::Idle;
            }
            WidgetId::ScenarioRow(index) => {
                self.scenario = index;
                return MenuOutcome::Idle;
            }
            WidgetId::SpawnHere => self.spawn_command(catalog),
            WidgetId::Launch => MenuCommand::LaunchScenario(self.scenario),
            WidgetId::VehicleMode(mode) => {
                self.vehicle_mode = Some(mode);
                return MenuOutcome::Idle;
            }
            WidgetId::ApplyVehicle => MenuCommand::ApplyVehicle {
                vehicle: self.vehicle,
                mode: self.vehicle_mode,
            },
            other => {
                return other
                    .command()
                    .map_or(MenuOutcome::Idle, MenuOutcome::Command);
            }
        };
        MenuOutcome::Command(command)
    }

    /// The spawn the World tab's selections describe.
    #[must_use]
    pub fn spawn_command(&self, catalog: &HubCatalog) -> MenuCommand {
        let modes = catalog
            .aerodromes
            .get(self.aerodrome)
            .map(catalog::AerodromeEntry::spawn_modes)
            .unwrap_or_default();
        let mode = self
            .spawn_mode
            .filter(|m| modes.contains(m))
            .or_else(|| modes.first().copied())
            .unwrap_or(SpawnMode::Overhead);
        MenuCommand::Spawn {
            aerodrome: self.aerodrome,
            runway: self.runway,
            mode,
        }
    }
}
