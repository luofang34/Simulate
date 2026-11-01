use sim_core::MagnetoSwitch;
use sim_input::{Control, InputMapper};
use sim_scenario::ScenarioProfile;
use winit::event::{ElementState, KeyEvent};
use winit::keyboard::{KeyCode, PhysicalKey};

use crate::flight::Flight;

/// Throttle travel per second while W or S is held.
const THROTTLE_RATE_PER_S: f64 = 0.5;
const FLAP_STEP: f64 = 1.0 / 3.0;
const MIXTURE_STEP: f64 = 0.10;

/// Discrete operator, viewport, time-scale, or aircraft system command triggered on key press.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CockpitAction {
    /// `Tab`: cycle `OperatorViewportMode` (`Cockpit`, `Chase`, `SensorAperture`, `ChartMap`, `VolumetricScene`).
    CycleViewport,
    /// `V`: cycle free-flight vehicle (`C172`, `LightTrainer`, `Aviate X500`, `Aviate Fixed-Wing`, `Kinematic`).
    CycleVehicle,
    /// `H`: toggle glass-cockpit PFD/MFD/HSI dock visibility.
    ToggleInstruments,
    /// `U`: toggle center conformal HUD symbology in `Cockpit` mode.
    ToggleHud,
    /// `Space`: toggle `SimSupervisor` `Paused` / `RealTime`.
    TogglePause,
    /// `]`: cycle `SimSupervisor` time warp (`1x` → `2x` → `4x` → `1x`).
    CycleWarp,
    /// `F`: extend flaps by one notch on JSBSim aircraft.
    FlapsDown,
    /// `G`: retract flaps by one notch on JSBSim aircraft.
    FlapsUp,
    /// `M`: cycle piston ignition magnetos (`Both` → `Left` → `Right` → `Off` → `Both`).
    CycleMagnetos,
    /// `R`: enrich fuel mixture (`+10%`).
    MixtureRich,
    /// `T`: lean fuel mixture (`-10%`).
    MixtureLean,
}

/// What a number key picks: a scripted scenario (`1`–`6`), or `None` (`0`) for free flight.
pub fn scenario_key(event: &KeyEvent) -> Option<Option<ScenarioProfile>> {
    if event.state != ElementState::Pressed || event.repeat {
        return None;
    }
    let PhysicalKey::Code(code) = event.physical_key else {
        return None;
    };
    let digit = match code {
        KeyCode::Digit0 => return Some(None),
        KeyCode::Digit1 => 1,
        KeyCode::Digit2 => 2,
        KeyCode::Digit3 => 3,
        KeyCode::Digit4 => 4,
        KeyCode::Digit5 => 5,
        KeyCode::Digit6 => 6,
        _ => return None,
    };
    ScenarioProfile::from_digit(digit).map(Some)
}

/// Maps a non-repeating key press to a discrete [`CockpitAction`].
pub fn cockpit_action(event: &KeyEvent) -> Option<CockpitAction> {
    if event.state != ElementState::Pressed || event.repeat {
        return None;
    }
    let PhysicalKey::Code(code) = event.physical_key else {
        return None;
    };
    match code {
        KeyCode::Tab => Some(CockpitAction::CycleViewport),
        KeyCode::KeyV => Some(CockpitAction::CycleVehicle),
        KeyCode::KeyH => Some(CockpitAction::ToggleInstruments),
        KeyCode::KeyU => Some(CockpitAction::ToggleHud),
        KeyCode::Space => Some(CockpitAction::TogglePause),
        KeyCode::BracketRight => Some(CockpitAction::CycleWarp),
        KeyCode::KeyF => Some(CockpitAction::FlapsDown),
        KeyCode::KeyG => Some(CockpitAction::FlapsUp),
        KeyCode::KeyM => Some(CockpitAction::CycleMagnetos),
        KeyCode::KeyR => Some(CockpitAction::MixtureRich),
        KeyCode::KeyT => Some(CockpitAction::MixtureLean),
        _ => None,
    }
}

/// Applies discrete aircraft [`SystemControls`](sim_core::SystemControls) actions to `flight`.
pub fn apply_aircraft_system_action(action: CockpitAction, flight: &mut Flight) {
    match action {
        CockpitAction::FlapsDown => {
            flight.update_system_controls(|sys| {
                sys.flaps_norm = (sys.flaps_norm + FLAP_STEP).clamp(0.0, 1.0);
            });
        }
        CockpitAction::FlapsUp => {
            flight.update_system_controls(|sys| {
                sys.flaps_norm = (sys.flaps_norm - FLAP_STEP).clamp(0.0, 1.0);
            });
        }
        CockpitAction::CycleMagnetos => {
            flight.update_system_controls(|sys| {
                sys.magnetos = match sys.magnetos {
                    MagnetoSwitch::Both | MagnetoSwitch::Start => MagnetoSwitch::Left,
                    MagnetoSwitch::Left => MagnetoSwitch::Right,
                    MagnetoSwitch::Right => MagnetoSwitch::Off,
                    MagnetoSwitch::Off => MagnetoSwitch::Both,
                };
            });
        }
        CockpitAction::MixtureRich => {
            flight.update_system_controls(|sys| {
                sys.mixture_norm = (sys.mixture_norm + MIXTURE_STEP).clamp(0.0, 1.0);
                sys.fuel_cutoff = false;
            });
        }
        CockpitAction::MixtureLean => {
            flight.update_system_controls(|sys| {
                sys.mixture_norm = (sys.mixture_norm - MIXTURE_STEP).clamp(0.0, 1.0);
            });
        }
        CockpitAction::CycleViewport
        | CockpitAction::CycleVehicle
        | CockpitAction::ToggleInstruments
        | CockpitAction::ToggleHud
        | CockpitAction::TogglePause
        | CockpitAction::CycleWarp => {}
    }
}

/// Arrow keys fly (Down is stick back), A/D are rudder, W/S move the throttle,
/// B holds wheel brakes, and E holds the engine starter.
/// The keyboard writes a stick axis only when one of its keys changes, so it
/// never zeroes a gamepad or yoke that is also connected.
#[derive(Debug, Default)]
pub struct Keyboard {
    left: bool,
    right: bool,
    up: bool,
    down: bool,
    rudder_left: bool,
    rudder_right: bool,
    throttle_up: bool,
    throttle_down: bool,
    brakes_held: bool,
    starter_held: bool,
}

impl Keyboard {
    /// Records a key change and updates the stick axes or momentary aircraft switches it affects.
    pub fn handle(&mut self, event: &KeyEvent, mapper: &mut InputMapper, flight: &mut Flight) {
        let PhysicalKey::Code(code) = event.physical_key else {
            return;
        };
        let pressed = event.state == ElementState::Pressed;
        match code {
            KeyCode::KeyB => {
                if self.brakes_held != pressed {
                    self.brakes_held = pressed;
                    flight.update_system_controls(|sys| {
                        sys.wheel_brake_norm = if pressed { 1.0 } else { 0.0 };
                    });
                }
                return;
            }
            KeyCode::KeyE => {
                if self.starter_held != pressed {
                    self.starter_held = pressed;
                    flight.update_system_controls(|sys| {
                        sys.starter = pressed;
                    });
                }
                return;
            }
            _ => {}
        }

        let (flag, control) = match code {
            KeyCode::ArrowLeft => (&mut self.left, Some(Control::Roll)),
            KeyCode::ArrowRight => (&mut self.right, Some(Control::Roll)),
            KeyCode::ArrowUp => (&mut self.up, Some(Control::Pitch)),
            KeyCode::ArrowDown => (&mut self.down, Some(Control::Pitch)),
            KeyCode::KeyA => (&mut self.rudder_left, Some(Control::Yaw)),
            KeyCode::KeyD => (&mut self.rudder_right, Some(Control::Yaw)),
            KeyCode::KeyW => (&mut self.throttle_up, None),
            KeyCode::KeyS => (&mut self.throttle_down, None),
            _ => return,
        };
        if *flag == pressed {
            return;
        }
        *flag = pressed;
        match control {
            Some(Control::Roll) => mapper.set(Control::Roll, axis(self.right, self.left)),
            Some(Control::Pitch) => mapper.set(Control::Pitch, axis(self.down, self.up)),
            Some(Control::Yaw) => {
                mapper.set(Control::Yaw, axis(self.rudder_right, self.rudder_left))
            }
            Some(Control::Throttle) | None => {}
        }
    }

    /// Moves the throttle while its keys are held.
    pub fn update(&self, elapsed_s: f64, mapper: &mut InputMapper) {
        let direction = axis(self.throttle_up, self.throttle_down);
        if direction != 0.0 {
            let throttle = mapper.controls().throttle + direction * THROTTLE_RATE_PER_S * elapsed_s;
            mapper.set(Control::Throttle, throttle);
        }
    }
}

fn axis(positive: bool, negative: bool) -> f64 {
    f64::from(u8::from(positive)) - f64::from(u8::from(negative))
}
