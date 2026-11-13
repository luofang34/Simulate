//! What the hub shows of the flight, the app, and the control scheme.

use sim_core::ControlInputs;

use crate::controls::{
    AxisRow, CaptureTarget, ControlPreset, InputSource, KeyAction, VehicleClass,
};
use crate::esc_menu::catalog::{AvionicsLayout, RateChoice, SceneryMode};
use crate::flight::PacingStats;
use crate::vehicles::FreeFlightVehicle;

/// Control-scheme state the Controls tab and hangar show.
#[derive(Debug, Clone, PartialEq)]
pub struct ControlsStatus {
    /// Preset in force.
    pub preset: ControlPreset,
    /// Whether it follows the vehicle class.
    pub automatic: bool,
    /// Per-control bindings and tuning.
    pub rows: Vec<AxisRow>,
    /// Key label bound to each action.
    pub keys: Vec<(KeyAction, String)>,
    /// Pending rebind.
    pub capture: Option<CaptureTarget>,
    /// Controls as last commanded by any device.
    pub live: ControlInputs,
    /// Device that last moved a control.
    pub source: InputSource,
    /// Connected controller name.
    pub controller: Option<String>,
}

impl Default for ControlsStatus {
    fn default() -> Self {
        Self {
            preset: ControlPreset::default_for(VehicleClass::FixedWing),
            automatic: true,
            rows: Vec::new(),
            keys: Vec::new(),
            capture: None,
            live: ControlInputs::default(),
            source: InputSource::Keyboard,
            controller: None,
        }
    }
}

/// What the hub shows besides its fixed labels.
#[derive(Debug, Clone, PartialEq)]
pub struct MenuStatus {
    /// Rolling real-time pacing of the simulation.
    pub pacing: PacingStats,
    /// Simulated seconds the rewind history covers.
    pub history_s: f64,
    /// Simulated seconds the shown flight is behind the newest history (`0` at the live
    /// frontier).
    pub rewind_offset_s: f64,
    /// Guidance host rewind/reset generation.
    pub epoch: u32,
    /// `30 Hz` sensor-slot frames rendered so far.
    pub sensor_frames: u64,
    /// Simulated seconds of the latest sensor exposure, once one rendered.
    pub sensor_last_s: Option<f64>,
    /// Active scenario or `FREE FLIGHT`.
    pub scenario: String,
    /// Whether a scripted scenario is flying (manual takeover is offered).
    pub scenario_active: bool,
    /// Active airport code.
    pub airport: String,
    /// Active vehicle.
    pub aircraft: String,
    /// Free-flight plant flying now, if any.
    pub active_vehicle: Option<FreeFlightVehicle>,
    /// Active sky time mode.
    pub sky: String,
    /// Whether the simulation clock is paused under the hub.
    pub paused: bool,
    /// Simulation rate selected for when the hub closes.
    pub rate: RateChoice,
    /// Instrument fit.
    pub layout: AvionicsLayout,
    /// Tile source.
    pub scenery: SceneryMode,
    /// Hangar turntable on.
    pub auto_rotate: bool,
    /// Selectable `(tail_code, name)` liveries of the previewed hangar airframe.
    pub hangar_liveries: Vec<(String, String)>,
    /// Index of the active livery on the previewed hangar airframe.
    pub active_livery_index: Option<usize>,
    /// Display label of the active livery on the previewed hangar airframe.
    pub active_livery_name: Option<String>,
    /// Control scheme.
    pub controls: ControlsStatus,
}

impl Default for MenuStatus {
    fn default() -> Self {
        Self {
            pacing: PacingStats::default(),
            history_s: 0.0,
            rewind_offset_s: 0.0,
            epoch: 0,
            sensor_frames: 0,
            sensor_last_s: None,
            scenario: "FREE FLIGHT".to_owned(),
            scenario_active: false,
            airport: String::new(),
            aircraft: String::new(),
            active_vehicle: None,
            sky: String::new(),
            paused: false,
            rate: RateChoice::Real,
            layout: AvionicsLayout::FullGlass,
            scenery: SceneryMode::Online,
            auto_rotate: true,
            hangar_liveries: Vec::new(),
            active_livery_index: None,
            active_livery_name: None,
            controls: ControlsStatus::default(),
        }
    }
}
