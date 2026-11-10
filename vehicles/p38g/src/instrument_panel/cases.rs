//! Instrument cases the P-38G-13 board holds whose instruments the pack cannot
//! describe yet, or whose identity is unresolved. Each is an empty case: a
//! hole, a bezel, a blank face and cover glass where the instrument goes.
//!
//! Positions and sizes are measured on the surviving board of 43-2285
//! (<https://aeroantique.com/products/p-38g-13-lo-lightening-instrument-panel-43-2285>),
//! `[right, up]` from the board's bounding-box centre, about 5 % uncertain.

/// One empty case.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct EmptyCase {
    /// Layout slot name, like a pack's.
    pub(crate) slot: &'static str,
    /// Centre `[right, up]` about the board's centre, m.
    pub(crate) centre: [f32; 2],
    /// Case diameter, m.
    pub(crate) diameter_m: f32,
}

const fn case(slot: &'static str, centre: [f32; 2], diameter_m: f32) -> EmptyCase {
    EmptyCase {
        slot,
        centre,
        diameter_m,
    }
}

/// The empty cases, by what each holds on the real board.
pub(crate) const EMPTY_CASES: [EmptyCase; 5] = [
    // Flap and landing-gear position indicator.
    case("flap_and_gear_position", [0.131, -0.183], 0.085),
    // Carburetor air temperature, refitted on the surviving board.
    case("carburetor_air_temperature", [0.078, 0.174], 0.056),
    // Dual X-pointer indicator in the right group; function unresolved.
    case("dual_indicator_right", [0.266, -0.032], 0.085),
    // Small dual L/R dial below the altimeter; legend unreadable. Raised
    // 4 mm from the measured centre so its tile clears the column cut-out.
    case("dual_indicator_lower", [-0.059, -0.089], 0.056),
    // Ammeter.
    case("ammeter", [0.266, -0.115], 0.061),
];

/// A warning lamp: a red jewel in a bezel, lit by a sensing switch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct WarningLamp {
    /// Name of the lamp.
    pub(crate) name: &'static str,
    /// Centre `[right, up]` about the board's centre, m.
    pub(crate) centre: [f32; 2],
    /// Bezel diameter, m.
    pub(crate) diameter_m: f32,
}

/// The left and right coolant-temperature warning lamps, press to test,
/// beside the coolant gauges. The flight model has no coolant temperature,
/// so nothing lights them.
pub(crate) const WARNING_LAMPS: [WarningLamp; 2] = [
    WarningLamp {
        name: "CoolantWarningLeft",
        centre: [-0.366, -0.089],
        diameter_m: 0.022,
    },
    WarningLamp {
        name: "CoolantWarningRight",
        centre: [-0.366, -0.125],
        diameter_m: 0.022,
    },
];
