use sim_core::MagnetoSwitch;
use sim_input::{Control, InputMapper};
use sim_scenario::ScenarioProfile;
use winit::event::{ElementState, KeyEvent};
use winit::keyboard::{KeyCode, PhysicalKey};

use crate::controls::{KeyAction, KeyBindings, THROTTLE_RATE_PER_S};
use crate::flight::Flight;

const FLAP_STEP: f64 = 1.0 / 3.0;
const MIXTURE_STEP: f64 = 0.10;

/// Discrete operator, viewport, time-scale, or aircraft system command triggered on key press.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CockpitAction {
    /// `Tab`: cycle `OperatorViewportMode` (`Cockpit`, `Chase`, `SensorAperture`, `ChartMap`, `VolumetricScene`).
    CycleViewport,
    /// `C`: cycle virtual camera (`Nose Nav`, `Belly Nadir`, `Pattern Window`, `Tower PTZ`, `Threshold Final`).
    CycleVirtualCamera,
    /// `K`: cycle home airport (`KDOV` Dover, `47N` Central Jersey, `KEWR` Newark Liberty) in free flight.
    CycleAirport,
    /// `V`: cycle free-flight vehicle (`C172`, `P-38G`, `Aviate X500`, `Aviate Fixed-Wing`, `Kinematic`).
    CycleVehicle,
    /// `H`: toggle glass-cockpit PFD/MFD/HSI dock visibility.
    ToggleInstruments,
    /// `U`: toggle the full-window conformal HUD in `Cockpit` mode.
    ToggleHud,
    /// `P`: cycle the avionics preset (gauges, single moving map, glass cockpit).
    CycleAvionicsPreset,
    /// `Space`: toggle `SimSupervisor` `Paused` / `RealTime`.
    TogglePause,
    /// `]`: cycle `SimSupervisor` time warp (`1x` → `2x` → `4x` → `1x`).
    CycleWarp,
    /// `L`: cycle the sky ephemeris time (`Auto` → `Live UTC` → `Dawn` → `Noon` → `Dusk` → `Night`).
    CycleSkyTime,
    /// `F`: extend flaps by one notch on JSBSim aircraft.
    FlapsDown,
    /// `G`: retract flaps by one notch on JSBSim aircraft.
    FlapsUp,
    /// `Y` / `[`: toggle retractable landing gear up/down on JSBSim aircraft.
    ToggleGear,
    /// `O`: toggle retractable or leading-edge landing lights on JSBSim aircraft.
    ToggleLandingLight,
    /// `M`: take manual control of a scripted scenario; in free flight, cycle piston
    /// ignition magnetos (`Both` → `Left` → `Right` → `Off` → `Both`).
    CycleMagnetos,
    /// `R`: enrich fuel mixture (`+10%`).
    MixtureRich,
    /// `T`: lean fuel mixture (`-10%`).
    MixtureLean,
    /// `N`: cycle onboard sensor realism (`Realistic` → `Degraded navigation` → `Ideal`).
    CycleSensorRealism,
    /// `J`: expand the top-right EFB moving map into the EFB tablet, or minimize it again.
    ToggleEfb,
    /// `Q`: toggle hydraulic Fowler flap `CLOSED` hold valve (`T.O. 01-75F-1`).
    ToggleFlapHoldValve,
    /// `I`: cycle radiator coolant flaps & oil cooler shutters (`20%` → `60%` → `100%` → `0%`).
    CycleCoolingFlaps,
    /// `Z`: toggle Curtiss Electric full feathering on the port propeller (`85°` edge-on).
    ToggleFeatherLeft,
    /// `X`: toggle Curtiss Electric full feathering on the starboard propeller (`85°` edge-on).
    ToggleFeatherRight,
}

/// What a number key picks: a scripted scenario (`1`–`8`), or `None` (`0`) for free flight.
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
        KeyCode::Digit7 => 7,
        KeyCode::Digit8 => 8,
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
        KeyCode::KeyC => Some(CockpitAction::CycleVirtualCamera),
        KeyCode::KeyK => Some(CockpitAction::CycleAirport),
        KeyCode::KeyV => Some(CockpitAction::CycleVehicle),
        KeyCode::KeyH => Some(CockpitAction::ToggleInstruments),
        KeyCode::KeyU => Some(CockpitAction::ToggleHud),
        KeyCode::KeyP => Some(CockpitAction::CycleAvionicsPreset),
        KeyCode::Space => Some(CockpitAction::TogglePause),
        KeyCode::BracketRight => Some(CockpitAction::CycleWarp),
        KeyCode::KeyL => Some(CockpitAction::CycleSkyTime),
        KeyCode::KeyF => Some(CockpitAction::FlapsDown),
        KeyCode::KeyG => Some(CockpitAction::FlapsUp),
        KeyCode::KeyY | KeyCode::BracketLeft => Some(CockpitAction::ToggleGear),
        KeyCode::KeyO => Some(CockpitAction::ToggleLandingLight),
        KeyCode::KeyM => Some(CockpitAction::CycleMagnetos),
        KeyCode::KeyR => Some(CockpitAction::MixtureRich),
        KeyCode::KeyT => Some(CockpitAction::MixtureLean),
        KeyCode::KeyN => Some(CockpitAction::CycleSensorRealism),
        KeyCode::KeyJ => Some(CockpitAction::ToggleEfb),
        KeyCode::KeyQ => Some(CockpitAction::ToggleFlapHoldValve),
        KeyCode::KeyI => Some(CockpitAction::CycleCoolingFlaps),
        KeyCode::KeyZ => Some(CockpitAction::ToggleFeatherLeft),
        KeyCode::KeyX => Some(CockpitAction::ToggleFeatherRight),
        _ => None,
    }
}

/// Applies discrete aircraft [`SystemControls`](sim_core::SystemControls) actions to `flight`.
pub fn apply_aircraft_system_action(action: CockpitAction, flight: &mut Flight) {
    flight.update_system_controls(|sys| match action {
        CockpitAction::FlapsDown => sys.flaps_norm = (sys.flaps_norm + FLAP_STEP).clamp(0.0, 1.0),
        CockpitAction::FlapsUp => sys.flaps_norm = (sys.flaps_norm - FLAP_STEP).clamp(0.0, 1.0),
        CockpitAction::ToggleGear => sys.gear_down = !sys.gear_down,
        CockpitAction::ToggleLandingLight => sys.landing_light = !sys.landing_light,
        CockpitAction::CycleMagnetos => {
            sys.magnetos = match sys.magnetos {
                MagnetoSwitch::Both | MagnetoSwitch::Start => MagnetoSwitch::Left,
                MagnetoSwitch::Left => MagnetoSwitch::Right,
                MagnetoSwitch::Right => MagnetoSwitch::Off,
                MagnetoSwitch::Off => MagnetoSwitch::Both,
            };
        }
        CockpitAction::MixtureRich => {
            sys.mixture_norm = (sys.mixture_norm + MIXTURE_STEP).clamp(0.0, 1.0);
            sys.fuel_cutoff = false;
        }
        CockpitAction::MixtureLean => {
            sys.mixture_norm = (sys.mixture_norm - MIXTURE_STEP).clamp(0.0, 1.0);
        }
        CockpitAction::ToggleFlapHoldValve => sys.flap_valve_closed = !sys.flap_valve_closed,
        CockpitAction::CycleCoolingFlaps => {
            let [cur, _] = sys.coolant_flap_norm;
            let next = if cur < 0.10 {
                0.20
            } else if cur < 0.40 {
                0.60
            } else if cur < 0.85 {
                1.00
            } else {
                0.0
            };
            sys.coolant_flap_norm = [next; 2];
            sys.oil_shutter_norm = [next; 2];
        }
        CockpitAction::ToggleFeatherLeft => {
            let [fl, fr] = sys.prop_feather;
            sys.prop_feather = [!fl, fr];
        }
        CockpitAction::ToggleFeatherRight => {
            let [fl, fr] = sys.prop_feather;
            sys.prop_feather = [fl, !fr];
        }
        CockpitAction::CycleViewport
        | CockpitAction::CycleVirtualCamera
        | CockpitAction::CycleAirport
        | CockpitAction::CycleVehicle
        | CockpitAction::ToggleInstruments
        | CockpitAction::ToggleHud
        | CockpitAction::CycleAvionicsPreset
        | CockpitAction::TogglePause
        | CockpitAction::CycleWarp
        | CockpitAction::CycleSkyTime
        | CockpitAction::CycleSensorRealism
        | CockpitAction::ToggleEfb => {}
    });
}

/// Flight keys (rebindable through [`KeyBindings`]; by default the arrows fly with Down as
/// stick back, A/D are rudder, W/S move the throttle, B holds wheel brakes, and E holds the
/// engine starter). The keyboard writes a stick axis only when one of its keys changes, so
/// it never zeroes a gamepad or yoke that is also connected.
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
    /// Records a key change and updates the stick axes or momentary aircraft switches it
    /// affects; returns whether `event` was a flight key.
    pub fn handle(
        &mut self,
        event: &KeyEvent,
        keys: &KeyBindings,
        mapper: &mut InputMapper,
        flight: &mut Flight,
    ) -> bool {
        let PhysicalKey::Code(code) = event.physical_key else {
            return false;
        };
        let Some(action) = keys.action_for(code) else {
            return false;
        };
        let pressed = event.state == ElementState::Pressed;
        match action {
            KeyAction::Brakes => {
                if self.brakes_held != pressed {
                    self.brakes_held = pressed;
                    flight.update_system_controls(|sys| {
                        sys.wheel_brake_norm = if pressed { 1.0 } else { 0.0 };
                    });
                }
                return true;
            }
            KeyAction::Starter => {
                if self.starter_held != pressed {
                    self.starter_held = pressed;
                    flight.update_system_controls(|sys| sys.starter = pressed);
                }
                return true;
            }
            _ => {}
        }
        let (flag, control) = match action {
            KeyAction::RollLeft => (&mut self.left, Some(Control::Roll)),
            KeyAction::RollRight => (&mut self.right, Some(Control::Roll)),
            KeyAction::PitchDown => (&mut self.up, Some(Control::Pitch)),
            KeyAction::PitchUp => (&mut self.down, Some(Control::Pitch)),
            KeyAction::YawLeft => (&mut self.rudder_left, Some(Control::Yaw)),
            KeyAction::YawRight => (&mut self.rudder_right, Some(Control::Yaw)),
            KeyAction::ThrottleUp => (&mut self.throttle_up, None),
            KeyAction::ThrottleDown | KeyAction::Brakes | KeyAction::Starter => {
                (&mut self.throttle_down, None)
            }
        };
        if *flag == pressed {
            return true;
        }
        *flag = pressed;
        match control {
            Some(Control::Roll) => mapper.set(Control::Roll, axis(self.right, self.left)),
            Some(Control::Pitch) => mapper.set(Control::Pitch, axis(self.down, self.up)),
            Some(Control::Yaw) => {
                mapper.set(Control::Yaw, axis(self.rudder_right, self.rudder_left));
            }
            Some(Control::Throttle) | None => {}
        }
        true
    }

    /// Whether any flight key is held.
    #[must_use]
    pub const fn any_held(&self) -> bool {
        self.left
            || self.right
            || self.up
            || self.down
            || self.rudder_left
            || self.rudder_right
            || self.throttle_up
            || self.throttle_down
    }

    /// Lets go of every held key, centring the stick axes the keyboard was holding off-centre
    /// and releasing brakes and starter, so nothing stays held while the `Esc` menu has the
    /// keyboard.
    pub fn release_all(&mut self, mapper: &mut InputMapper, flight: &mut Flight) {
        if self.left || self.right {
            mapper.set(Control::Roll, 0.0);
        }
        if self.up || self.down {
            mapper.set(Control::Pitch, 0.0);
        }
        if self.rudder_left || self.rudder_right {
            mapper.set(Control::Yaw, 0.0);
        }
        if self.brakes_held || self.starter_held {
            let brakes = self.brakes_held;
            let starter = self.starter_held;
            flight.update_system_controls(|sys| {
                if brakes {
                    sys.wheel_brake_norm = 0.0;
                }
                if starter {
                    sys.starter = false;
                }
            });
        }
        *self = Self::default();
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
