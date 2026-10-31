use sim_input::{Control, InputMapper};
use sim_scenario::ScenarioProfile;
use winit::event::{ElementState, KeyEvent};
use winit::keyboard::{KeyCode, PhysicalKey};

/// Throttle travel per second while W or S is held.
const THROTTLE_RATE_PER_S: f64 = 0.5;

/// What a number key picks: a scenario, or `None` for free flight.
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

/// Arrow keys fly (Down is stick back), A/D are rudder, W/S move the throttle.
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
}

impl Keyboard {
    /// Records a key change and updates the stick axes it affects.
    pub fn handle(&mut self, event: &KeyEvent, mapper: &mut InputMapper) {
        let PhysicalKey::Code(code) = event.physical_key else {
            return;
        };
        let pressed = event.state == ElementState::Pressed;
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
