//! Rebindable keyboard flight keys.

use winit::keyboard::KeyCode;

/// A held flight key's meaning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum KeyAction {
    /// Roll left.
    RollLeft,
    /// Roll right.
    RollRight,
    /// Stick back, nose up.
    PitchUp,
    /// Stick forward, nose down.
    PitchDown,
    /// Left rudder.
    YawLeft,
    /// Right rudder.
    YawRight,
    /// Throttle up while held.
    ThrottleUp,
    /// Throttle down while held.
    ThrottleDown,
    /// Wheel brakes while held.
    Brakes,
    /// Engine starter while held.
    Starter,
}

impl KeyAction {
    /// Every action in display order.
    pub const ALL: [Self; 10] = [
        Self::RollLeft,
        Self::RollRight,
        Self::PitchUp,
        Self::PitchDown,
        Self::YawLeft,
        Self::YawRight,
        Self::ThrottleUp,
        Self::ThrottleDown,
        Self::Brakes,
        Self::Starter,
    ];

    /// Display name.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::RollLeft => "Roll left",
            Self::RollRight => "Roll right",
            Self::PitchUp => "Pitch up",
            Self::PitchDown => "Pitch down",
            Self::YawLeft => "Yaw left",
            Self::YawRight => "Yaw right",
            Self::ThrottleUp => "Throttle up",
            Self::ThrottleDown => "Throttle down",
            Self::Brakes => "Brakes",
            Self::Starter => "Starter",
        }
    }

    const fn default_key(self) -> KeyCode {
        match self {
            Self::RollLeft => KeyCode::ArrowLeft,
            Self::RollRight => KeyCode::ArrowRight,
            Self::PitchUp => KeyCode::ArrowDown,
            Self::PitchDown => KeyCode::ArrowUp,
            Self::YawLeft => KeyCode::KeyA,
            Self::YawRight => KeyCode::KeyD,
            Self::ThrottleUp => KeyCode::KeyW,
            Self::ThrottleDown => KeyCode::KeyS,
            Self::Brakes => KeyCode::KeyB,
            Self::Starter => KeyCode::KeyE,
        }
    }
}

/// The key bound to each [`KeyAction`]; one key drives at most one action.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KeyBindings {
    keys: Vec<(KeyAction, KeyCode)>,
}

impl Default for KeyBindings {
    fn default() -> Self {
        Self {
            keys: KeyAction::ALL
                .into_iter()
                .map(|action| (action, action.default_key()))
                .collect(),
        }
    }
}

impl KeyBindings {
    /// The action `code` drives, if any.
    #[must_use]
    pub fn action_for(&self, code: KeyCode) -> Option<KeyAction> {
        self.keys
            .iter()
            .find(|(_, key)| *key == code)
            .map(|(action, _)| *action)
    }

    /// The key bound to `action`.
    #[must_use]
    pub fn key_for(&self, action: KeyAction) -> KeyCode {
        self.keys
            .iter()
            .find(|(a, _)| *a == action)
            .map_or(action.default_key(), |(_, key)| *key)
    }

    /// Binds `code` to `action`; an action that held `code` takes `action`'s old key, so
    /// every action stays reachable.
    pub fn bind(&mut self, action: KeyAction, code: KeyCode) {
        let old = self.key_for(action);
        for (a, key) in &mut self.keys {
            if *a == action {
                *key = code;
            } else if *key == code {
                *key = old;
            }
        }
    }
}

/// Short glyph-safe label for `code` (`KeyA` → `A`, `ArrowLeft` → `LEFT`, `BracketLeft` →
/// `LBRACKET`); punctuation keys are spelled out because the panel glyph pack cannot draw
/// most of their symbols.
#[must_use]
pub fn key_label(code: KeyCode) -> String {
    if let Some(name) = punctuation_name(code) {
        return name.to_owned();
    }
    let debug = format!("{code:?}");
    let short = debug
        .strip_prefix("Key")
        .or_else(|| debug.strip_prefix("Digit"))
        .or_else(|| debug.strip_prefix("Arrow"))
        .unwrap_or(&debug);
    short.to_ascii_uppercase()
}

const fn punctuation_name(code: KeyCode) -> Option<&'static str> {
    Some(match code {
        KeyCode::BracketLeft => "LBRACKET",
        KeyCode::BracketRight => "RBRACKET",
        KeyCode::Comma => "COMMA",
        KeyCode::Period => "PERIOD",
        KeyCode::Semicolon => "SEMICOLON",
        KeyCode::Quote => "QUOTE",
        KeyCode::Equal => "EQUALS",
        KeyCode::Minus => "MINUS",
        KeyCode::Slash => "SLASH",
        KeyCode::Backslash => "BACKSLASH",
        KeyCode::Backquote => "GRAVE",
        _ => return None,
    })
}
