//! Identities of the hub's interactive widgets and the commands the hub issues.

use sim_input::Control;

use crate::controls::KeyAction;
use crate::esc_menu::catalog::{AvionicsLayout, RateChoice, SceneryMode, SpawnMode};
use crate::esc_menu::{HubTab, Step};
use crate::vehicles::FreeFlightVehicle;

/// What the app must do in response to the hub.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuCommand {
    /// Close the hub.
    Resume,
    /// Restart the active flight.
    Restart,
    /// Exit the event loop.
    Quit,
    /// Leave the scripted scenario for hand flying.
    TakeManualControl,
    /// Scrub back along the rewind timeline by this many simulated seconds.
    Rewind {
        /// Requested rewind span.
        seconds: u32,
    },
    /// Scrub forward, to the live frontier, or to a point along the rewind timeline.
    Scrub(TimelineScrub),
    /// Set the simulation rate.
    Rate(RateChoice),
    /// Cycle the sky time mode.
    Sky(Step),
    /// Set the instrument fit.
    Layout(AvionicsLayout),
    /// Set the tile source.
    Scenery(SceneryMode),
    /// Show registry vehicle `index` in the hangar preview.
    PreviewVehicle(usize),
    /// Fly registry airframe `vehicle`, on flight-dynamics `mode` when the operator chose
    /// one.
    ApplyVehicle {
        /// Registry index.
        vehicle: usize,
        /// Chosen free-flight plant, if any.
        mode: Option<FreeFlightVehicle>,
    },
    /// Return the hangar preview camera to its starting orbit.
    ResetHangarView,
    /// Toggle the hangar turntable.
    ToggleAutoRotate,
    /// Spawn at a catalog aerodrome.
    Spawn {
        /// Catalog aerodrome index.
        aerodrome: usize,
        /// Runway end index within it.
        runway: usize,
        /// Where at the aerodrome.
        mode: SpawnMode,
    },
    /// Launch catalog scenario `index`.
    LaunchScenario(usize),
    /// Step the control preset.
    Preset(Step),
    /// Return the control preset to the vehicle default.
    PresetAuto,
    /// Rebind `control` to the next moved axis.
    RebindAxis(Control),
    /// Invert `control`'s axes.
    InvertAxis(Control),
    /// Step `control`'s deadzone.
    Deadzone(Control, Step),
    /// Step `control`'s expo.
    Expo(Control, Step),
    /// Rebind the key of an action to the next pressed key.
    RebindKey(KeyAction),
}

/// A move along the rewind timeline other than a step back.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimelineScrub {
    /// Forward by this many simulated seconds, toward the live frontier.
    Forward(u32),
    /// Back to the live frontier, exactly as it was left.
    Live,
    /// To this many thousandths of the buffered history (`0` oldest, `1000` newest).
    Permille(u16),
}

/// A vector glyph a widget shows instead of text the panel glyph pack cannot draw.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WidgetIcon {
    /// Text only.
    #[default]
    None,
    /// `<`, the previous value.
    ChevronLeft,
    /// `>`, the next value.
    ChevronRight,
    /// `-`, step down.
    Minus,
    /// `+`, step up.
    Plus,
}

/// Every interactive element of the hub.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WidgetId {
    /// A tab in the bar.
    Tab(HubTab),
    /// Close the hub.
    Resume,
    /// Take manual control of the scenario.
    TakeControl,
    /// Restart the flight.
    Restart,
    /// Quit.
    Quit,
    /// Scrub back this many seconds.
    Rewind(u32),
    /// Scrub forward or to the live frontier.
    Scrub(TimelineScrub),
    /// The rewind timeline bar; a click scrubs to the clicked point.
    TimelineBar,
    /// A simulation-rate chip.
    Rate(RateChoice),
    /// Sky ephemeris arrows.
    Sky(Step),
    /// An avionics-layout chip.
    Layout(AvionicsLayout),
    /// A scenery chip.
    Scenery(SceneryMode),
    /// A registry row (registry index).
    VehicleRow(usize),
    /// Hangar turntable toggle.
    AutoRotate,
    /// Reset the hangar preview camera.
    ResetView,
    /// A flight-dynamics mode chip of the selected airframe.
    VehicleMode(FreeFlightVehicle),
    /// Fly the selected vehicle.
    ApplyVehicle,
    /// The aerodrome search box.
    Search,
    /// An aerodrome row (catalog index).
    AerodromeRow(usize),
    /// A runway-end chip.
    RunwayEnd(usize),
    /// A spawn-mode chip.
    SpawnChoice(SpawnMode),
    /// Spawn at the selection.
    SpawnHere,
    /// A scenario row (catalog index).
    ScenarioRow(usize),
    /// Launch the selected scenario.
    Launch,
    /// Preset arrows.
    Preset(Step),
    /// Return to the automatic preset.
    PresetAuto,
    /// Rebind a control's axis.
    AxisRebind(Control),
    /// Invert a control's axes.
    AxisInvert(Control),
    /// Step a control's deadzone.
    Deadzone(Control, Step),
    /// Step a control's expo.
    Expo(Control, Step),
    /// Rebind an action's key.
    KeyRebind(KeyAction),
}

impl WidgetId {
    /// The command a widget issues on its own, without hub selections.
    #[must_use]
    pub const fn command(self) -> Option<MenuCommand> {
        Some(match self {
            Self::Resume => MenuCommand::Resume,
            Self::TakeControl => MenuCommand::TakeManualControl,
            Self::Restart => MenuCommand::Restart,
            Self::Quit => MenuCommand::Quit,
            Self::Rewind(seconds) => MenuCommand::Rewind { seconds },
            Self::Scrub(scrub) => MenuCommand::Scrub(scrub),
            Self::Rate(rate) => MenuCommand::Rate(rate),
            Self::Sky(step) => MenuCommand::Sky(step),
            Self::Layout(layout) => MenuCommand::Layout(layout),
            Self::Scenery(mode) => MenuCommand::Scenery(mode),
            Self::AutoRotate => MenuCommand::ToggleAutoRotate,
            Self::ResetView => MenuCommand::ResetHangarView,
            Self::Preset(step) => MenuCommand::Preset(step),
            Self::PresetAuto => MenuCommand::PresetAuto,
            Self::AxisRebind(control) => MenuCommand::RebindAxis(control),
            Self::AxisInvert(control) => MenuCommand::InvertAxis(control),
            Self::Deadzone(control, step) => MenuCommand::Deadzone(control, step),
            Self::Expo(control, step) => MenuCommand::Expo(control, step),
            Self::KeyRebind(action) => MenuCommand::RebindKey(action),
            Self::Tab(_)
            | Self::VehicleRow(_)
            | Self::ApplyVehicle
            | Self::VehicleMode(_)
            | Self::TimelineBar
            | Self::Search
            | Self::AerodromeRow(_)
            | Self::RunwayEnd(_)
            | Self::SpawnChoice(_)
            | Self::SpawnHere
            | Self::ScenarioRow(_)
            | Self::Launch => return None,
        })
    }
}
