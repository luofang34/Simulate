//! Reserved 3D cockpit station architecture for the Lockheed P-38G-13-LO Lightning.
//!
//! The contract fixes the cockpit's frames, the design eyepoint, the seat reference
//! point, the instrument panel plane the gauges mount on, the gunsight mount, the
//! control column's floor attachment and the side-console envelopes, so interior detail
//! binds to them without altering the exterior airframe.
//!
//! Where each number comes from, in order of authority:
//! - AN 01-75F-2 general arrangement ([`crate::drawing`]): the windscreen base and top
//!   bow, the top hatch's frames, the sill line, the main beam.
//! - T.O. 01-75FG-2 (Lightning I service instructions, 1942), same station datum: the
//!   instrument panel at station 106, the armament compartment's aft bulkhead at
//!   station 95, the gunsight arm 118, and the ring-and-bead sight (ring on the
//!   windscreen frame at station 126, bead at station `97 3/8`, line of sight on
//!   waterline 34.2, three inches to port). Stated for the Lightning I and taken for the
//!   G; the waterline is taken as height above the fuselage reference line.
//! - AN 01-75FF-2 (P-38L-1 erection and maintenance instructions): figure 3's fuselage
//!   frame stations (`... 87 3/4, 95, 105, 115, 126 3/32, 135, 145 9/16, 162 1/4 ...
//!   236`; `126 3/32` is the general arrangement's windscreen top bow and `236` its
//!   gondola tail, so the sheets share a frame), and figure 367's armour arms: seat
//!   bottom 138.0, seat back 147.5, plate behind the seat 156.0.
//! - LeRoy Weber's P-38L-5-LO inboard profile (Superscale, 1958), a redraw read off a
//!   scan to about `2 in`, only for what the manuals do not give: the two floor levels,
//!   the control column's foot and the pedal pivots on the station 95 frame.
//! - Derived here: the design eyepoint and the seat reference point (see
//!   [`DESIGN_EYE_FS_IN`]).

use crate::drawing::{
    INCH_M, WINDSCREEN_BASE_FS_IN, WINDSCREEN_TOP_FRAME_FS_IN, frl_to_z, fs_to_y,
};

/// Rudder pedal pivots: on the station 95 frame, the armament compartment's aft
/// bulkhead (T.O. 01-75FG-2; the Weber drawing puts the pivots on it).
pub const RUDDER_PEDAL_FS_IN: f32 = 95.0;
/// Instrument panel board: station 106 (T.O. 01-75FG-2 section 11), an inch aft of the
/// station 105 frame of AN 01-75FF-2 figure 3; it stands vertical on the Weber drawing.
pub const INSTRUMENT_PANEL_FS_IN: f32 = 106.0;
/// Control column's floor attachment, on the starboard side (secondary reading).
pub const CONTROL_COLUMN_FS_IN: f32 = 116.0;
/// Control column off the ship centreline, to starboard (secondary reading).
pub const CONTROL_COLUMN_BL_IN: f32 = 10.6;
/// Armour plate behind and above the seat, just ahead of the main beam (AN 01-75FF-2
/// figure 367, arm 156.0; the general arrangement draws the hatch's aft frame at
/// `FS 154.3`).
pub const SEAT_ARMOUR_FS_IN: f32 = 156.0;
/// Main beam: the `35 %` chord line (AN 01-75F-2 plan view).
pub const MAIN_BEAM_FS_IN: f32 = 160.0;
/// Floor above the fuselage reference line: ahead of the nose-wheel well and over the
/// retracted nose wheel (secondary readings).
pub const FLOOR_ABOVE_FRL_IN: (f32, f32) = (-9.0, -6.0);
/// Stations between which the floor steps up over the retracted nose wheel.
pub const FLOOR_STEP_FS_IN: (f32, f32) = (128.0, 134.0);
/// Seat back recline from vertical (secondary reading of the seat detail), degrees.
pub const SEAT_BACK_RECLINE_DEG: f32 = 15.0;
/// Design eyepoint station: the head `3 in` ahead of a `3 in` headrest pad on the armour
/// plate, the eye `7 in` ahead of the back of the head. The top hatch's middle is
/// `FS 140.2`; the manuals' pilot arm is 138 and seat-back armour arm 147.5.
pub const DESIGN_EYE_FS_IN: f32 = SEAT_ARMOUR_FS_IN - 13.0;
/// Design eyepoint above the fuselage reference line: the canopy's inner top (`40.9 in`)
/// less `4.4 in` from eye to crown, `0.5 in` of helmet and `2 in` of clearance. The
/// auxiliary sight's line on waterline 34.2 is the same height within `0.2 in`.
pub const DESIGN_EYE_ABOVE_FRL_IN: f32 = 34.0;
/// Seat reference point (seat back meets seat pan) relative to the eyepoint, `(aft,
/// up)`: a `31.5 in` sitting eye height on the reclined back plus a `3 in` seat-pack
/// parachute puts it `33.4 in` below the eye, and the eye sits `7 in` ahead of the back
/// line, which leaves the point `0.5 in` aft of the eye.
pub const SEAT_REFERENCE_FROM_EYE_IN: (f32, f32) = (0.5, -33.4);
/// Top of the board's arched crown above the fuselage reference line. It puts the
/// centre of the board's bounding box at `z = 0.350 m`, where Lockheed's frontal
/// photograph Z-4986 ("P-38 #3474 INSTRUMENT PANEL 12-23-42") shows it: the board
/// rectified by its own hole pattern, the camera solved on the station 126.2 frame
/// and the board plane, good to about `2 cm`.
pub const PANEL_TOP_ABOVE_FRL_IN: f32 = 26.6;
/// Gunsight behind the windscreen: station (T.O. 01-75FG-2 arm 118) and height above
/// the fuselage reference line (read off the Weber drawing's sight).
pub const GUNSIGHT_IN: (f32, f32) = (118.0, 29.0);
/// Auxiliary ring-and-bead sight (T.O. 01-75FG-2 section 11): the bead's station ahead
/// of the windscreen, the ring's on the windscreen frame, the sight line's waterline and
/// its offset to port, inches.
pub const BEAD_SIGHT_FS_IN: f32 = 97.375;
/// Station of the auxiliary sight's ring.
pub const RING_SIGHT_FS_IN: f32 = 126.0;
/// Waterline of the auxiliary sight's line, taken as height above the reference line.
pub const AUX_SIGHT_LINE_ABOVE_FRL_IN: f32 = 34.2;
/// The auxiliary sight line's offset to port of the ship centreline.
pub const AUX_SIGHT_PORT_IN: f32 = 3.0;

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
    /// The board's outline `[right, up]` about the panel frame's centre, m,
    /// clockwise as the pilot sees it from the lower left: an arched crown
    /// and a cut-out at the bottom for the control column.
    pub main_instrument_panel_outline_m: &'static [[f32; 2]],
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

/// The P-38G-13 board's outline, measured on the surviving panel of
/// 43-2285 (<https://aeroantique.com/products/p-38g-13-lo-lightening-instrument-panel-43-2285>)
/// at 0.90 mm per pixel, scaled from its 3-1/8 in and 2-1/4 in cases; about
/// 5 % uncertain for perspective. Two period Lockheed photographs of a P-38G
/// cockpit agree with its layout
/// (<https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_left_wing_061019-F-1234P-004.jpg>).
const P38G_PANEL_OUTLINE_M: &[[f32; 2]] = &[
    [-0.399, -0.237],
    [-0.399, -0.042],
    [-0.383, -0.019],
    [-0.320, -0.010],
    [-0.297, 0.030],
    [-0.270, 0.089],
    [-0.230, 0.138],
    [-0.167, 0.192],
    [-0.095, 0.224],
    [-0.022, 0.235],
    [0.050, 0.234],
    [0.121, 0.217],
    [0.175, 0.188],
    [0.230, 0.156],
    [0.266, 0.102],
    [0.301, 0.030],
    [0.320, 0.003],
    [0.383, -0.037],
    [0.399, -0.064],
    [0.399, -0.237],
    [0.086, -0.237],
    [0.076, -0.132],
    [0.059, -0.121],
    [-0.068, -0.121],
    [-0.086, -0.132],
    [-0.095, -0.237],
];

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
        "Instrument panel board under the glare shield",
    ),
    frame(
        "FS 126",
        WINDSCREEN_TOP_FRAME_FS_IN,
        "Windscreen top bow and forward frame of the top hatch",
    ),
    frame(
        "FS 156",
        SEAT_ARMOUR_FS_IN,
        "Armour plate behind the seat, under the hatch's aft frame",
    ),
    frame(
        "FS 160",
        MAIN_BEAM_FS_IN,
        "Main beam carry-through behind the seat",
    ),
];

/// Height and width of the board's bounding box, measured on the surviving board of
/// 43-2285 (see [`P38G_PANEL_OUTLINE_M`]); with [`PANEL_TOP_ABOVE_FRL_IN`] the height
/// places the board's centre.
const PANEL_HEIGHT_M: f32 = 0.474;
const PANEL_WIDTH_M: f32 = 0.797;
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
            width_m: PANEL_WIDTH_M,
            height_m: PANEL_HEIGHT_M,
            tilt_deg: 0.0,
        },
        main_instrument_panel_outline_m: P38G_PANEL_OUTLINE_M,
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
