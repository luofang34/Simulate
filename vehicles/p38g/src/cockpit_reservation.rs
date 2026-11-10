//! Reserved 3D cockpit station architecture for the Lockheed P-38G-13-LO Lightning.
//!
//! The contract fixes the cockpit's frames, the design eyepoint, the seat reference
//! point, the instrument panel plane the gauges mount on, the gunsight mount, the
//! control column's floor attachment and the side-console envelopes, so interior detail
//! binds to them without altering the exterior airframe.
//!
//! Where each number comes from:
//! - AN 01-75F-2 general arrangement ([`crate::drawing`]): the windscreen base and top
//!   bow, the top hatch's frames, the sill line, the main beam.
//! - LeRoy Weber's P-38L-5-LO inboard profile (Superscale, 1958), a redraw that carries
//!   Lockheed station numbers and agrees with the general arrangement where both show
//!   the same thing (top bow at station 126, canopy top, keel): the instrument panel
//!   frame, the seat-back armour, the two floor levels, the control column and the
//!   rudder pedal pivots. Read off a scan to about `2 in`, and of an L, so each is a
//!   secondary reading.
//! - Derived here: the design eyepoint and the seat reference point (see
//!   [`DESIGN_EYE_FS_IN`]).

use crate::drawing::{
    INCH_M, WINDSCREEN_BASE_FS_IN, WINDSCREEN_TOP_FRAME_FS_IN, frl_to_z, fs_to_y,
};

/// Rudder pedal pivots (secondary reading).
pub const RUDDER_PEDAL_FS_IN: f32 = 95.0;
/// Instrument panel frame: the board stands vertical, `6 in` aft of the windscreen base
/// (secondary reading).
pub const INSTRUMENT_PANEL_FS_IN: f32 = 106.0;
/// Control column's floor attachment, on the starboard side (secondary reading).
pub const CONTROL_COLUMN_FS_IN: f32 = 116.0;
/// Control column off the ship centreline, to starboard (secondary reading).
pub const CONTROL_COLUMN_BL_IN: f32 = 10.6;
/// Forward face of the seat-back armour plate, just ahead of the main beam (secondary
/// reading; the general arrangement draws the hatch's aft frame at `FS 154.3`).
pub const SEAT_ARMOUR_FS_IN: f32 = 155.0;
/// Main beam: the `35 %` chord line (AN 01-75F-2 plan view).
pub const MAIN_BEAM_FS_IN: f32 = 160.0;
/// Floor above the fuselage reference line: ahead of the nose-wheel well and over the
/// retracted nose wheel (secondary readings).
pub const FLOOR_ABOVE_FRL_IN: (f32, f32) = (-9.0, -6.0);
/// Stations between which the floor steps up over the retracted nose wheel.
pub const FLOOR_STEP_FS_IN: (f32, f32) = (128.0, 134.0);
/// Seat back recline from vertical (secondary reading of the seat detail), degrees.
pub const SEAT_BACK_RECLINE_DEG: f32 = 15.0;
/// Design eyepoint station: the head `2 in` ahead of a `3 in` headrest pad on the armour
/// plate, the eye `7 in` ahead of the back of the head. The top hatch's middle is
/// `FS 140.2`.
pub const DESIGN_EYE_FS_IN: f32 = SEAT_ARMOUR_FS_IN - 12.0;
/// Design eyepoint above the fuselage reference line: the canopy's inner top (`40.9 in`)
/// less `4.4 in` from eye to crown, `0.5 in` of helmet and `2 in` of clearance.
pub const DESIGN_EYE_ABOVE_FRL_IN: f32 = 34.0;
/// Seat reference point (seat back meets seat pan) relative to the eyepoint, `(aft,
/// up)`: a `31.5 in` sitting eye height on the reclined back plus a `3 in` seat-pack
/// parachute puts it `33.4 in` below the eye, and the eye sits `7 in` ahead of the back
/// line, which leaves the point `0.5 in` aft of the eye.
pub const SEAT_REFERENCE_FROM_EYE_IN: (f32, f32) = (0.5, -33.4);
/// Top of the rectangular board above the fuselage reference line: as high as a
/// `0.72 m` wide board fits inside the skin at its station.
pub const PANEL_TOP_ABOVE_FRL_IN: f32 = 22.0;
/// Gunsight behind the windscreen: station and height above the fuselage reference line
/// (secondary reading of the L's sight).
pub const GUNSIGHT_IN: (f32, f32) = (122.0, 29.0);

/// 3D rectangular mounting frame for an instrument panel or sub-stage attachment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PanelMountFrame {
    /// Target OpenUSD prim path in the vehicle stage.
    pub prim_path: &'static str,
    /// Center position `(x_right, y_fwd, z_up)` in aircraft metres.
    pub center_m: [f32; 3],
    /// Panel width along aircraft `X` in metres.
    pub width_m: f32,
    /// Panel height along tilted `Z` in metres.
    pub height_m: f32,
    /// Backward tilt angle from vertical in degrees.
    pub tilt_deg: f32,
}

/// Fuselage station (`FS`) frame reservation inside the central gondola.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StationBulkheadSpec {
    /// Lockheed fuselage station designation (e.g. `"FS 106"`).
    pub station_name: &'static str,
    /// Station number in inches aft of the nose datum (`FS 0`).
    pub station_in: f32,
    /// Longitudinal coordinate `y` in aircraft metres (`+Y` forward).
    pub y_m: f32,
    /// What stands at this station.
    pub description: &'static str,
}

/// Complete cockpit interior architectural contract for `P-38G-13-LO`.
#[derive(Clone, Debug, PartialEq)]
pub struct CockpitInteriorContract {
    /// Design pilot eye reference point `(x, y, z)` in aircraft metres.
    pub design_eyepoint_m: [f32; 3],
    /// Seat reference point, where the seat back meets the seat pan, in aircraft metres.
    pub seat_reference_point_m: [f32; 3],
    /// Seat back recline from vertical, degrees.
    pub seat_back_recline_deg: f32,
    /// Main flight and engine instrument panel board.
    pub main_instrument_panel: PanelMountFrame,
    /// Optical reflector gunsight mount behind the windscreen.
    pub reflector_gunsight_mount: PanelMountFrame,
    /// Left side-console shelf (throttles, propeller and mixture levers).
    pub left_throttle_quadrant: PanelMountFrame,
    /// Right side-console shelf (flap lever and cooling controls).
    pub right_flap_cooling_console: PanelMountFrame,
    /// Floor attachment `(x, y, z)` of the control column, which stands on the starboard
    /// side and carries the wheel inboard on an arm.
    pub control_column_base_m: [f32; 3],
    /// Model `y` of the rudder pedal pivots.
    pub rudder_pedal_pivot_y_m: f32,
    /// Model `z` of the floor ahead of the nose-wheel well and over it.
    pub floor_z_m: [f32; 2],
    /// Model `y` between which the floor steps from the first level to the second.
    pub floor_step_y_m: [f32; 2],
    /// Model `y` of the forward face of the seat-back armour plate.
    pub seat_armour_y_m: f32,
    /// Frames bounding the cockpit, nose to tail.
    pub bulkheads: &'static [StationBulkheadSpec],
}

const fn frame(
    station_name: &'static str,
    station_in: f32,
    description: &'static str,
) -> StationBulkheadSpec {
    StationBulkheadSpec {
        station_name,
        station_in,
        y_m: fs_to_y(station_in),
        description,
    }
}

const P38G_BULKHEADS: &[StationBulkheadSpec] = &[
    frame("FS 95", RUDDER_PEDAL_FS_IN, "Rudder pedal pivots"),
    frame(
        "FS 100",
        WINDSCREEN_BASE_FS_IN,
        "Windscreen base on the sill line",
    ),
    frame(
        "FS 106",
        INSTRUMENT_PANEL_FS_IN,
        "Instrument panel frame under the glare shield",
    ),
    frame(
        "FS 126",
        WINDSCREEN_TOP_FRAME_FS_IN,
        "Windscreen top bow and forward frame of the top hatch",
    ),
    frame(
        "FS 155",
        SEAT_ARMOUR_FS_IN,
        "Seat-back armour plate under the hatch's aft frame",
    ),
    frame(
        "FS 160",
        MAIN_BEAM_FS_IN,
        "Main beam carry-through behind the seat",
    ),
];

/// Board height; with [`PANEL_TOP_ABOVE_FRL_IN`] it places the board's centre.
const PANEL_HEIGHT_M: f32 = 0.28;
/// Side consoles: centre off the centreline, shelf height, and stations spanned.
const CONSOLE_X_M: f32 = 0.36;
const CONSOLE_Z_M: f32 = 0.16;
const CONSOLE_FS_IN: (f32, f32) = (112.0, 140.0);

/// Returns the reserved cockpit interior station contract for `P-38G-13-LO`.
#[must_use]
pub fn p38g_cockpit_contract() -> CockpitInteriorContract {
    let console_y = fs_to_y(0.5 * (CONSOLE_FS_IN.0 + CONSOLE_FS_IN.1));
    let console_len = fs_to_y(CONSOLE_FS_IN.0) - fs_to_y(CONSOLE_FS_IN.1);
    let console = |prim_path, side: f32| PanelMountFrame {
        prim_path,
        center_m: [side * CONSOLE_X_M, console_y, CONSOLE_Z_M],
        width_m: 0.14,
        height_m: console_len,
        tilt_deg: 0.0,
    };
    let floor = [
        frl_to_z(FLOOR_ABOVE_FRL_IN.0),
        frl_to_z(FLOOR_ABOVE_FRL_IN.1),
    ];
    let (seat_aft, seat_up) = SEAT_REFERENCE_FROM_EYE_IN;
    CockpitInteriorContract {
        design_eyepoint_m: [
            0.0,
            fs_to_y(DESIGN_EYE_FS_IN),
            frl_to_z(DESIGN_EYE_ABOVE_FRL_IN),
        ],
        seat_reference_point_m: [
            0.0,
            fs_to_y(DESIGN_EYE_FS_IN + seat_aft),
            frl_to_z(DESIGN_EYE_ABOVE_FRL_IN + seat_up),
        ],
        seat_back_recline_deg: SEAT_BACK_RECLINE_DEG,
        main_instrument_panel: PanelMountFrame {
            prim_path: "/P38g/CockpitInterior/InstrumentPanel/Board",
            center_m: [
                0.0,
                fs_to_y(INSTRUMENT_PANEL_FS_IN),
                frl_to_z(PANEL_TOP_ABOVE_FRL_IN) - 0.5 * PANEL_HEIGHT_M,
            ],
            width_m: 0.72,
            height_m: PANEL_HEIGHT_M,
            tilt_deg: 0.0,
        },
        reflector_gunsight_mount: PanelMountFrame {
            prim_path: "/P38g/CockpitInterior/GunsightReflector",
            center_m: [0.0, fs_to_y(GUNSIGHT_IN.0), frl_to_z(GUNSIGHT_IN.1)],
            width_m: 0.12,
            height_m: 0.14,
            tilt_deg: 45.0,
        },
        left_throttle_quadrant: console("/P38g/CockpitInterior/LeftThrottleConsole", -1.0),
        right_flap_cooling_console: console("/P38g/CockpitInterior/RightFlapConsole", 1.0),
        control_column_base_m: [
            CONTROL_COLUMN_BL_IN * INCH_M,
            fs_to_y(CONTROL_COLUMN_FS_IN),
            floor[0],
        ],
        rudder_pedal_pivot_y_m: fs_to_y(RUDDER_PEDAL_FS_IN),
        floor_z_m: floor,
        floor_step_y_m: [fs_to_y(FLOOR_STEP_FS_IN.0), fs_to_y(FLOOR_STEP_FS_IN.1)],
        seat_armour_y_m: fs_to_y(SEAT_ARMOUR_FS_IN),
        bulkheads: P38G_BULKHEADS,
    }
}
