//! Parts of an airframe's moving state shared by every flight model: a
//! propeller shaft, the exterior lights and the cockpit controls.

/// Blade pitch of a propeller at rest, degrees.
pub(crate) const DEFAULT_BLADE_ANGLE_DEG: f64 = 22.0;

/// One propeller shaft's motion and pitch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellerShaft {
    /// Shaft speed, rpm.
    pub rpm: f64,
    /// Rotation direction viewed from the rear (`+1.0` clockwise, `-1.0` counter-clockwise).
    pub sense: f64,
    /// Blade geometric pitch at the reference station, degrees.
    pub blade_angle_deg: f64,
    /// Deterministically integrated shaft angle in `[0, 2π)` radians.
    pub phase_rad: f64,
}

impl Default for PropellerShaft {
    fn default() -> Self {
        Self {
            rpm: 0.0,
            sense: 1.0,
            blade_angle_deg: DEFAULT_BLADE_ANGLE_DEG,
            phase_rad: 0.0,
        }
    }
}

/// The airframe's exterior lights.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AirframeLights {
    /// Anti-collision red fin beacon normalized intensity `[0, 1]`.
    pub beacon_flash_norm: f64,
    /// Integrated electrical/beacon phase in `[0, 2π)` radians.
    pub beacon_phase_rad: f64,
    /// Normalized landing light illumination `[0, 1]` (`1.0` extended and lit, `0.0` off/stowed).
    pub landing_light_norm: f64,
}

/// Where the pilot has set a four-position flap selector valve (the P-38's
/// `UP`, `CLOSED`, `MANEUVER`, `DOWN`), read from the flap command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FlapSelector {
    /// Flaps commanded fully up.
    #[default]
    Up,
    /// The valve closed, holding the flaps where they are.
    Closed,
    /// Flaps commanded to the first stop above up.
    Maneuver,
    /// Flaps commanded beyond the first stop.
    Down,
}

impl FlapSelector {
    /// Flap commands at or below this read as the first stop (`MANEUVER`); the
    /// P-38G's first stop is at a third of the travel.
    const FIRST_STOP_NORM: f64 = 0.34;

    /// The detent a flap command `cmd_norm` (`0` up, `1` full) selects, or
    /// `Closed` while the valve holds the flaps.
    #[must_use]
    pub fn from_command(cmd_norm: f64, valve_closed: bool) -> Self {
        if valve_closed {
            Self::Closed
        } else if cmd_norm.is_nan() || cmd_norm <= 0.01 {
            Self::Up
        } else if cmd_norm <= Self::FIRST_STOP_NORM {
            Self::Maneuver
        } else {
            Self::Down
        }
    }
}

/// How far the canopy's opening panes are open: the top hatch swung up on its
/// hinge and the side windows wound down, each 0 (shut) to 1 (fully open).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CanopyOpening {
    /// The top hatch.
    pub hatch: f64,
    /// The side windows `[left, right]`.
    pub side_windows: [f64; 2],
}

/// The positions of the cockpit's controls as the pilot has set them, which the
/// cockpit's levers, wheel and pedals are drawn at. These are the commands, not
/// the surfaces: a stick-force-limited aileron lags the wheel that commands it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CockpitControls {
    /// Control column, −1 (full forward) to 1 (full back).
    pub pitch_norm: f64,
    /// Control wheel, −1 (full left) to 1 (full right).
    pub roll_norm: f64,
    /// Rudder pedals, −1 (full left) to 1 (full right).
    pub yaw_norm: f64,
    /// Throttles `[left, right]`, 0 (closed) to 1 (open).
    pub throttle_norm: [f64; 2],
    /// Propeller levers `[left, right]`, 0 (coarse) to 1 (fine).
    pub prop_norm: [f64; 2],
    /// Mixture levers `[left, right]`, 0 (idle cut-off) to 1 (full rich).
    pub mixture_norm: [f64; 2],
    /// Trim controls `[aileron, elevator, rudder]`, −1 to 1.
    pub trim_norm: [f64; 3],
    /// The flap selector's detent.
    pub flap_selector: FlapSelector,
    /// Whether the landing-gear lever is down.
    pub gear_lever_down: bool,
    /// The canopy's hatch and side windows as the pilot has left them.
    pub canopy: CanopyOpening,
}

impl Default for CockpitControls {
    fn default() -> Self {
        Self {
            pitch_norm: 0.0,
            roll_norm: 0.0,
            yaw_norm: 0.0,
            throttle_norm: [0.0; 2],
            prop_norm: [1.0; 2],
            mixture_norm: [1.0; 2],
            trim_norm: [0.0; 3],
            flap_selector: FlapSelector::Up,
            gear_lever_down: true,
            canopy: CanopyOpening {
                hatch: 0.0,
                side_windows: [0.0; 2],
            },
        }
    }
}
