/// Normalised pilot control positions, the interface between input devices
/// and whatever flies the aircraft (flight model, autopilot, remote pilot).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ControlInputs {
    /// Roll stick, −1 (full left) to 1 (full right).
    pub roll: f64,
    /// Pitch stick, −1 (full forward, nose down) to 1 (full back, nose up).
    pub pitch: f64,
    /// Rudder pedals, −1 (full left) to 1 (full right).
    pub yaw: f64,
    /// Throttle lever, 0 (idle) to 1 (full).
    pub throttle: f64,
}

impl ControlInputs {
    /// The same inputs limited to their valid ranges; NaN becomes neutral.
    #[must_use]
    pub fn clamped(self) -> Self {
        let axis = |v: f64| if v.is_nan() { 0.0 } else { v.clamp(-1.0, 1.0) };
        Self {
            roll: axis(self.roll),
            pitch: axis(self.pitch),
            yaw: axis(self.yaw),
            throttle: if self.throttle.is_nan() {
                0.0
            } else {
                self.throttle.clamp(0.0, 1.0)
            },
        }
    }
}
