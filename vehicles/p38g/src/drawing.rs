//! Reference values read from the P-38G general-arrangement drawing (AN 01-75F-2, page 2)
//! and the mapping between the drawing's stations and the model frame.
//!
//! The digitised drawing lives in `design/an01-75f-2_general_arrangement.json`: the outer
//! silhouette of the side, plan and front views as rings in drawing inches, the sheet
//! registration each view was read with, and every dimension callout used here, each
//! with the sheet pixels it was measured at. The constants below repeat those callouts so
//! the geometry code can use them without parsing; a test keeps the two in step.

/// The digitised AN 01-75F-2 page 2 general arrangement (JSON, drawing inches).
pub const GENERAL_ARRANGEMENT_JSON: &str =
    include_str!("../design/an01-75f-2_general_arrangement.json");

/// Metres per inch.
pub const INCH_M: f32 = 0.0254;
/// Model `y` of fuselage station 0, the gondola nose.
pub const NOSE_Y_M: f32 = 3.55;
/// Model `z` of the thrust line.
pub const THRUST_LINE_Z_M: f32 = 0.04;
/// Thrust line above the fuselage reference line (side view callout `5.078`), inches.
pub const THRUST_LINE_ABOVE_FRL_IN: f32 = 5.078;
/// Model `z` of the fuselage reference line.
pub const FRL_Z_M: f32 = THRUST_LINE_Z_M - THRUST_LINE_ABOVE_FRL_IN * INCH_M;

/// Overall length, gondola nose to rudder trailing edge (side view, `37'-9 15/16"`), inches.
pub const OVERALL_LENGTH_IN: f32 = 453.9375;
/// Wing span (front view, `52'-0"`), inches.
pub const SPAN_IN: f32 = 624.0;
/// Propeller plane station (side view, `61.875` to the propeller centreline), inches.
pub const PROP_PLANE_FS_IN: f32 = 61.875;
/// Root-chord leading-edge station (side view, `119.05`), inches.
pub const ROOT_LE_FS_IN: f32 = 119.05;
/// Root chord at the ship centreline (plan view note `ROOT CHORD 117 NACA 23016`), inches.
pub const ROOT_CHORD_IN: f32 = 117.0;
/// Mean aerodynamic chord leading-edge station (plan view `M.A.C. L.E. AT FUS. STA. 130.51`).
pub const MAC_LE_FS_IN: f32 = 130.51;
/// Mean aerodynamic chord (plan view `M.A.C. 84.25`), inches.
pub const MAC_IN: f32 = 84.25;
/// Chord at wing station 289, where the rounded tip begins (plan view `36`), inches.
pub const TIP_CHORD_WS289_IN: f32 = 36.0;
/// Boom centreline from the ship centreline (plan view `96`), inches.
pub const BOOM_BL_IN: f32 = 96.0;
/// Stabiliser leading-edge station (plan view `L.E. STAB. AT FUS. STA. 397.584`), inches.
pub const STAB_LE_FS_IN: f32 = 397.584;
/// Tailplane span (plan view `261`), inches.
pub const STAB_SPAN_IN: f32 = 261.0;
/// Stabiliser chord ahead of the elevator hinge (plan view `25`), inches.
pub const STAB_CHORD_IN: f32 = 25.0;
/// Elevator chord aft of the hinge (plan view `20`), inches.
pub const ELEVATOR_CHORD_IN: f32 = 20.0;
/// Propeller diameter (plan view `11'-6"`), inches.
pub const PROP_DIAMETER_IN: f32 = 138.0;
/// Main-tyre tread (front view `198`), inches.
pub const MAIN_TREAD_IN: f32 = 198.0;
/// Main axle below the fuselage reference line at the static oleo (front view `54.575`).
pub const MAIN_AXLE_BELOW_FRL_STATIC_IN: f32 = 54.575;
/// Main tyre static rolling radius (front view `STATIC 15.300`, 36 in tyre), inches.
pub const MAIN_ROLLING_RADIUS_IN: f32 = 15.3;
/// Nose axle below the fuselage reference line at the static oleo (side view `STATIC
/// 69.970`, dimensioned to the nose tyre's centre), inches.
pub const NOSE_AXLE_BELOW_FRL_STATIC_IN: f32 = 69.97;
/// Static ground line below the horizontal of the fuselage reference line (side view
/// `5°-33'-46"`): the aircraft stands nose-up by this angle, degrees.
pub const STATIC_GROUND_ANGLE_DEG: f32 = 5.0 + 33.0 / 60.0 + 46.0 / 3600.0;
/// Propeller tip above the static ground line (side view `16.726`), inches.
pub const PROP_TIP_GROUND_CLEARANCE_STATIC_IN: f32 = 16.726;
/// Nose tyre static rolling radius (side view `STATIC 11.600`, 27 in tyre), inches.
pub const NOSE_ROLLING_RADIUS_IN: f32 = 11.6;
/// Nose axle station: centre of the dashed nose tyre on the side view, inches.
pub const NOSE_AXLE_FS_IN: f32 = 54.9;
/// Main axle station: centre of the dashed main tyre on the side view, inches.
pub const MAIN_AXLE_FS_IN: f32 = 173.9;
/// Airframe height, fin top to the static ground line (side view `9'-10 3/8"`), inches.
pub const OVERALL_HEIGHT_IN: f32 = 118.375;
/// Canopy top above the fuselage reference line (front view `41.250`), inches.
pub const CANOPY_TOP_ABOVE_FRL_IN: f32 = 41.25;
/// Gondola bottom below the fuselage reference line (front view `31.010`), inches.
pub const GONDOLA_BOTTOM_BELOW_FRL_IN: f32 = 31.01;

/// Gondola maximum half-breadth (plan view `18.995`), inches.
pub const GONDOLA_HALF_BREADTH_IN: f32 = 18.995;
/// Canopy sill line above the fuselage reference line: the horizontal line under the side
/// windows on the side view, inches.
pub const CANOPY_SILL_ABOVE_FRL_IN: f32 = 25.6;
/// Windscreen base station: where the side view's windscreen line meets the sill line and
/// the plan view's canopy outline closes at the front, inches.
pub const WINDSCREEN_BASE_FS_IN: f32 = 100.0;
/// Windscreen slope above the horizontal on the side view, degrees.
pub const WINDSCREEN_SLOPE_DEG: f32 = 29.3;
/// Windscreen top bow, the forward frame of the top hatch (side view frame line), inches.
pub const WINDSCREEN_TOP_FRAME_FS_IN: f32 = 126.2;
/// Aft frame of the top hatch (side view frame line), inches.
pub const HATCH_AFT_FRAME_FS_IN: f32 = 154.3;
/// Canopy tail station: where the plan view's canopy outline closes at the rear, inches.
pub const CANOPY_TAIL_FS_IN: f32 = 184.5;

/// Rear-view mirror fairing on the canopy top (side view): forward and aft stations and
/// the apex above the fuselage reference line, inches.
pub const MIRROR_FAIRING_FS_IN: (f32, f32) = (129.3, 136.5);
/// Apex of the mirror fairing above the fuselage reference line (side view), inches.
pub const MIRROR_FAIRING_APEX_ABOVE_FRL_IN: f32 = 46.1;
/// Width of the mirror fairing (front view), inches.
pub const MIRROR_FAIRING_WIDTH_IN: f32 = 4.6;

/// Model `y` of fuselage station `fs_in`.
#[must_use]
pub const fn fs_to_y(fs_in: f32) -> f32 {
    NOSE_Y_M - fs_in * INCH_M
}

/// Fuselage station of model `y`, inches.
#[must_use]
pub const fn y_to_fs(y: f32) -> f32 {
    (NOSE_Y_M - y) / INCH_M
}

/// Model `z` of a point `h_in` above the fuselage reference line.
#[must_use]
pub const fn frl_to_z(h_in: f32) -> f32 {
    FRL_Z_M + h_in * INCH_M
}

/// Height of model `z` above the fuselage reference line, inches.
#[must_use]
pub const fn z_to_frl(z: f32) -> f32 {
    (z - FRL_Z_M) / INCH_M
}
