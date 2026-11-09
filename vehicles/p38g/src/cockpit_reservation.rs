//! Reserved 3D cockpit station architecture (`FS 86..182`) and interior geometry
//! for the Lockheed P-38G-13-LO Lightning (`T.O. 01-75F-1`).
//!
//! While full clickable 3D gauge needles are deferred, this module defines the exact
//! structural bulkheads, instrument panel projection quad, gunsight mount, control
//! column socket, and side-console envelopes so a high-detail cockpit sub-stage or
//! `Indicate` 3D panel texture can bind directly without altering the exterior airframe.

use glam::Vec3;

use crate::canopy_frames::canopy_frame_prims;
use crate::mesh::{MeshBuilder, P38Material, P38MeshPrim, P38PartRole, fixed};
use crate::stations::{
    ARMOR_GLASS, CHROME_OLEO, COCKPIT_GREEN, GUNMETAL, INSIGNIA_RED, LEATHER_BROWN, PANEL_BLACK,
    PILOT_SUIT, PROP_YELLOW, WHEEL_WELL_ZINC,
};

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

/// Fuselage station (`FS`) structural bulkhead reservation inside the central gondola.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StationBulkheadSpec {
    /// Lockheed fuselage station designation (e.g. `"FS 88"`, `"FS 154"`).
    pub station_name: &'static str,
    /// Station number in inches aft of the nose datum (`FS 0`).
    pub station_in: f32,
    /// Longitudinal coordinate `y` in aircraft metres (`+Y` forward).
    pub y_m: f32,
    /// Functional role of this structural bulkhead.
    pub description: &'static str,
}

/// Complete cockpit interior architectural contract for `P-38G-13-LO`.
#[derive(Clone, Debug, PartialEq)]
pub struct CockpitInteriorContract {
    /// Design pilot eye reference point `(x, y, z)` in aircraft metres.
    pub design_eyepoint_m: [f32; 3],
    /// `FS 86` main flight & engine instrument panel frame.
    pub main_instrument_panel: PanelMountFrame,
    /// Lynn N-3A optical reflector gunsight mount above the glare coaming.
    pub reflector_gunsight_mount: PanelMountFrame,
    /// Left side-console shelf (`x = -0.42 m`: twin throttles, RPM, mixture, feathering switches).
    pub left_throttle_quadrant: PanelMountFrame,
    /// Right side-console shelf (`x = +0.42 m`: 4-position Fowler flap lever & cooling controls).
    pub right_flap_cooling_console: PanelMountFrame,
    /// Floor socket `(x, y, z)` for the Lockheed spectacles control column.
    pub control_column_floor_socket_m: [f32; 3],
    /// Structural fuselage bulkheads bounding the armament, wheel well, cockpit, and radio bays.
    pub bulkheads: &'static [StationBulkheadSpec],
}

const P38G_BULKHEADS: &[StationBulkheadSpec] = &[
    StationBulkheadSpec {
        station_name: "FS 22",
        station_in: 22.0,
        y_m: 2.98,
        description: "Forward nose gun muzzle trunnion & blast-tube support ring",
    },
    StationBulkheadSpec {
        station_name: "FS 52",
        station_in: 52.0,
        y_m: 2.22,
        description: "Ammunition compartment aft wall & nose landing-gear trunnion frame",
    },
    StationBulkheadSpec {
        station_name: "FS 88",
        station_in: 88.0,
        y_m: 1.36,
        description: "Forward cockpit firewall, rudder pedal bay & main instrument panel mount",
    },
    StationBulkheadSpec {
        station_name: "FS 132",
        station_in: 132.0,
        y_m: 0.18,
        description: "Main wing center-section forward box-spar carry-through bulkhead",
    },
    StationBulkheadSpec {
        station_name: "FS 154",
        station_in: 154.0,
        y_m: -0.38,
        description: "Pilot seat support frame & 3/8-in face-hardened dorsal armor bulkhead",
    },
    StationBulkheadSpec {
        station_name: "FS 182",
        station_in: 182.0,
        y_m: -0.92,
        description: "Aft cockpit rear pressure wall & SCR-274-N Command Set radio rack frame",
    },
];

/// Returns the reserved cockpit interior station contract for `P-38G-13-LO`.
#[must_use]
pub fn p38g_cockpit_contract() -> CockpitInteriorContract {
    CockpitInteriorContract {
        design_eyepoint_m: [0.0, 0.42, 0.68],
        main_instrument_panel: PanelMountFrame {
            prim_path: "/P38g/CockpitInterior/MainInstrumentPanel",
            center_m: [0.0, 1.24, 0.40],
            width_m: 0.72,
            height_m: 0.36,
            tilt_deg: 8.0,
        },
        reflector_gunsight_mount: PanelMountFrame {
            prim_path: "/P38g/CockpitInterior/GunsightReflector",
            center_m: [0.0, 1.18, 0.63],
            width_m: 0.12,
            height_m: 0.14,
            tilt_deg: 45.0,
        },
        left_throttle_quadrant: PanelMountFrame {
            prim_path: "/P38g/CockpitInterior/LeftThrottleConsole",
            center_m: [-0.42, 0.72, 0.16],
            width_m: 0.18,
            height_m: 0.64,
            tilt_deg: 0.0,
        },
        right_flap_cooling_console: PanelMountFrame {
            prim_path: "/P38g/CockpitInterior/RightFlapConsole",
            center_m: [0.42, 0.72, 0.16],
            width_m: 0.18,
            height_m: 0.64,
            tilt_deg: 0.0,
        },
        control_column_floor_socket_m: [0.0, 0.94, -0.26],
        bulkheads: P38G_BULKHEADS,
    }
}

pub(crate) fn cockpit_interior_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    build_cockpit_tub_and_bulkheads(&mut prims);
    build_instrument_panel_and_gunsight(&mut prims);
    build_controls_and_pilot(&mut prims);
    prims.extend(canopy_frame_prims());
    prims
}

fn build_cockpit_tub_and_bulkheads(prims: &mut Vec<P38MeshPrim>) {
    let mut tub = MeshBuilder::new();
    // Recessed cockpit floor & side tub walls (FS 88 to FS 182)
    tub.cuboid(Vec3::new(-0.46, -0.92, -0.28), Vec3::new(0.46, 1.38, -0.22));
    for side in [-1.0_f32, 1.0] {
        tub.cuboid(
            Vec3::new(side * 0.44 - 0.03, -0.92, -0.24),
            Vec3::new(side * 0.44 + 0.03, 1.36, 0.36),
        );
        // Side console shelves (Left: throttle/prop/mixture, Right: Fowler flap & cooling)
        tub.cuboid(
            Vec3::new(side * 0.37 - 0.07, 0.36, 0.12),
            Vec3::new(side * 0.37 + 0.07, 1.08, 0.18),
        );
    }
    // FS 88 forward firewall bulkhead & FS 132 wing center-spar carry-through box
    tub.cuboid(Vec3::new(-0.46, 1.32, -0.26), Vec3::new(0.46, 1.38, 0.46));
    tub.cuboid(Vec3::new(-0.46, -0.02, -0.26), Vec3::new(0.46, 0.24, -0.04));
    // FS 182 aft radio compartment bulkhead
    tub.cuboid(Vec3::new(-0.44, -0.94, -0.24), Vec3::new(0.44, -0.88, 0.46));
    prims.push(fixed(
        "/P38g/CockpitInterior/TubAndBulkheads",
        P38PartRole::CabinInterior,
        COCKPIT_GREEN,
        tub,
    ));

    // FS 22 & FS 52 forward nose gun bay structural frames (zinc chromate)
    let mut nose_frames = MeshBuilder::new();
    nose_frames.tube(
        Vec3::new(0.0, 2.98, 0.02),
        Vec3::new(0.0, 2.95, 0.02),
        0.28,
        0.28,
        14,
    );
    nose_frames.tube(
        Vec3::new(0.0, 2.22, 0.03),
        Vec3::new(0.0, 2.18, 0.03),
        0.44,
        0.44,
        16,
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/NoseStationFrames",
        P38PartRole::CabinInterior,
        WHEEL_WELL_ZINC,
        nose_frames,
    ));
}

fn build_instrument_panel_and_gunsight(prims: &mut Vec<P38MeshPrim>) {
    let mut panel = MeshBuilder::new();
    // Main instrument panel board at FS 86 (y = 1.24) + anti-glare coaming hood
    panel.cuboid(Vec3::new(-0.36, 1.21, 0.18), Vec3::new(0.36, 1.26, 0.54));
    panel.cuboid(Vec3::new(-0.38, 1.14, 0.51), Vec3::new(0.38, 1.62, 0.545));
    // Primary flight six-pack + twin engine manifold/RPM/coolant gauge bezels
    for gx in [-0.26_f32, -0.16, -0.06, 0.06, 0.16, 0.26] {
        for gz in [0.27_f32, 0.37, 0.46] {
            panel.tube(
                Vec3::new(gx, 1.21, gz),
                Vec3::new(gx, 1.185, gz),
                0.036,
                0.036,
                12,
            );
        }
    }
    // Lynn N-3A optical reflector gunsight body above the coaming
    panel.tube(
        Vec3::new(0.0, 1.24, 0.55),
        Vec3::new(0.0, 1.15, 0.62),
        0.034,
        0.028,
        10,
    );
    // Aft SCR-274-N Command Set radio transmitter/receiver boxes behind FS 154
    for rx in [-0.20_f32, 0.0, 0.20] {
        panel.cuboid(
            Vec3::new(rx - 0.07, -0.84, 0.28),
            Vec3::new(rx + 0.07, -0.50, 0.46),
        );
    }
    prims.push(fixed(
        "/P38g/CockpitInterior/MainInstrumentPanel",
        P38PartRole::CabinInterior,
        PANEL_BLACK,
        panel,
    ));

    // Lynn N-3A tilted glass reflector combiner lens
    let mut sight_glass = MeshBuilder::new();
    sight_glass.cuboid(Vec3::new(-0.042, 1.15, 0.61), Vec3::new(0.042, 1.19, 0.69));
    prims.push(fixed(
        "/P38g/CockpitInterior/GunsightReflector",
        P38PartRole::CabinInterior,
        ARMOR_GLASS,
        sight_glass,
    ));
}

fn build_controls_and_pilot(prims: &mut Vec<P38MeshPrim>) {
    let mut hardware = MeshBuilder::new();
    // 3/8-in seat-back armor plate & headrest support, directly behind the pilot
    hardware.cuboid(
        Vec3::new(-0.30, SEAT_SHIFT_Y - 0.40, -0.10),
        Vec3::new(0.30, SEAT_SHIFT_Y - 0.35, 0.78),
    );
    // Lockheed spectacles control column & twin-horn yoke wheel
    hardware.tube(
        Vec3::new(0.0, 0.94, -0.24),
        Vec3::new(0.0, 0.84, 0.34),
        0.022,
        0.018,
        10,
    );
    hardware.tube(
        Vec3::new(-0.15, 0.82, 0.34),
        Vec3::new(0.15, 0.82, 0.34),
        0.015,
        0.015,
        10,
    );
    for side in [-1.0_f32, 1.0] {
        hardware.tube(
            Vec3::new(side * 0.14, 0.82, 0.30),
            Vec3::new(side * 0.14, 0.82, 0.42),
            0.014,
            0.014,
            8,
        );
    }
    // Left console twin-engine throttle, propeller RPM, and mixture levers
    for (ly, lz) in [
        (0.82_f32, 0.25_f32),
        (0.76, 0.24),
        (0.70, 0.23),
        (0.64, 0.23),
    ] {
        hardware.tube(
            Vec3::new(-0.37, ly, 0.17),
            Vec3::new(-0.35, ly + 0.02, lz),
            0.008,
            0.012,
            8,
        );
    }
    // Right console 4-position Fowler maneuver flap lever
    hardware.tube(
        Vec3::new(0.37, 0.78, 0.17),
        Vec3::new(0.35, 0.80, 0.26),
        0.009,
        0.014,
        8,
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/ArmorAndControls",
        P38PartRole::CabinInterior,
        GUNMETAL,
        hardware,
    ));

    // Red Curtiss Electric propeller feathering switch guards & trim handwheels
    let mut red_knobs = MeshBuilder::new();
    for y in [0.90_f32, 0.86] {
        red_knobs.cuboid(
            Vec3::new(-0.41, y - 0.015, 0.18),
            Vec3::new(-0.37, y + 0.015, 0.21),
        );
    }
    prims.push(fixed(
        "/P38g/CockpitInterior/FeatherSwitches",
        P38PartRole::CabinInterior,
        INSIGNIA_RED,
        red_knobs,
    ));
    build_seat_and_pilot_bust(prims);
}

/// Fore-aft offset of the seat, its armour and the pilot figure, which puts the
/// seated pilot's eyes on the contract's design eyepoint.
const SEAT_SHIFT_Y: f32 = 0.42;

fn build_seat_and_pilot_bust(prims: &mut Vec<P38MeshPrim>) {
    // Metal bucket seat, leather headrest pad, and pilot figure
    let mut seat = MeshBuilder::new();
    let fwd = Vec3::new(0.0, SEAT_SHIFT_Y, 0.0);
    seat.cuboid(
        Vec3::new(-0.24, -0.34, -0.12) + fwd,
        Vec3::new(0.24, 0.14, -0.06) + fwd,
    );
    seat.cuboid(
        Vec3::new(-0.24, -0.35, -0.10) + fwd,
        Vec3::new(0.24, -0.29, 0.52) + fwd,
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/BucketSeat",
        P38PartRole::CabinInterior,
        CHROME_OLEO,
        seat,
    ));

    let mut pilot = MeshBuilder::new();
    pilot.ellipsoid(
        Vec3::new(0.0, -0.06, 0.22) + fwd,
        Vec3::new(0.21, 0.15, 0.28),
        14,
        8,
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/PilotTorso",
        P38PartRole::PilotFigure,
        PILOT_SUIT,
        pilot,
    ));

    let mut head = MeshBuilder::new();
    head.ellipsoid(
        Vec3::new(0.0, -0.08, 0.62) + fwd,
        Vec3::new(0.11, 0.12, 0.13),
        14,
        8,
    );
    head.cuboid(
        Vec3::new(-0.11, -0.35, 0.56) + fwd,
        Vec3::new(0.11, -0.29, 0.72) + fwd,
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/PilotHelmet",
        P38PartRole::PilotFigure,
        LEATHER_BROWN,
        head,
    ));

    let mut vest = MeshBuilder::new();
    vest.ellipsoid(
        Vec3::new(0.0, -0.02, 0.28) + fwd,
        Vec3::new(0.17, 0.13, 0.18),
        12,
        6,
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/MaeWestVest",
        P38PartRole::PilotFigure,
        PROP_YELLOW,
        vest,
    ));

    let _unused_type_check: P38Material = COCKPIT_GREEN;
}
