use super::capture::PendingKind;
use super::{ControlOwner, InputMapper};
use crate::keyboard::action_control;
use crate::{CapturedKey, DeviceId, FlightKeyAction, GamepadButton, KeyId, KeyResponse};

impl InputMapper {
    /// Presses or releases a flight action from the keyboard. Pressing a
    /// stick or throttle action takes that control from any device; the
    /// deflection itself ramps in [`Self::tick`].
    pub fn set_key_action(&mut self, action: FlightKeyAction, pressed: bool) {
        if self.keyboard.set_key(action, pressed) {
            self.on_action(action, pressed);
        }
    }

    /// Routes a physical key through the active preset's keymap, or binds it
    /// if [`Self::start_key_capture`] is pending and the key went down.
    pub fn handle_key(&mut self, key: &KeyId, pressed: bool) -> KeyResponse {
        if let Some(PendingKind::Key(action)) = self.capture.as_ref().map(|c| c.kind)
            && pressed
        {
            self.capture = None;
            self.bind_key(key, action);
            return KeyResponse::Captured(CapturedKey {
                key: key.clone(),
                action,
            });
        }
        match self.presets.get(self.preset).key_action(key) {
            Some(action) => {
                self.set_key_action(action, pressed);
                KeyResponse::Action(action)
            }
            None => KeyResponse::Ignored,
        }
    }

    /// Presses or releases a gamepad button; returns the action it holds
    /// under the device's preset, if any.
    pub fn handle_button(
        &mut self,
        device: DeviceId,
        button: GamepadButton,
        pressed: bool,
    ) -> Option<FlightKeyAction> {
        let preset = self.device_preset(device)?;
        let action = self
            .presets
            .get(preset)
            .buttons
            .iter()
            .find(|b| b.button == button)
            .map(|b| b.action)?;
        if self.keyboard.set_button(action, pressed) {
            self.on_action(action, pressed);
        }
        Some(action)
    }

    /// Advances held-key stick slewing and latched-throttle rates by `dt_s`
    /// seconds; non-finite or non-positive steps are ignored.
    pub fn tick(&mut self, dt_s: f64) {
        if !(dt_s.is_finite() && dt_s > 0.0) {
            return;
        }
        self.keyboard.slew(&self.rates, dt_s);
        self.integrate_throttle(dt_s);
    }

    /// Every action a key or button holds.
    pub fn held_actions(&self) -> Vec<FlightKeyAction> {
        self.keyboard.held()
    }

    /// Releases every held key and button and centres the keyboard stick,
    /// such as when a menu takes the keyboard. The latched throttle stays.
    pub fn release_keys(&mut self) {
        self.keyboard.release_all();
        self.keyboard.centre();
    }

    /// The keys bound to `action` in the active preset.
    pub fn keys_for(&self, action: FlightKeyAction) -> Vec<KeyId> {
        self.presets
            .get(self.preset)
            .keys_for(action)
            .cloned()
            .collect()
    }

    fn on_action(&mut self, action: FlightKeyAction, pressed: bool) {
        if !pressed {
            return;
        }
        if action == FlightKeyAction::ResetTrim {
            self.latched_throttle = self.trim_throttle;
            self.keyboard.centre();
            return;
        }
        if let Some(control) = action_control(action) {
            *self.owners.get_mut(control) = ControlOwner::Keyboard;
        }
    }
}
