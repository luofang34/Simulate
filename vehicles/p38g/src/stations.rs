//! Calibrated station ordinate tables (`FS`, `BS`, `WS`, `WL`) and 3-view reference
//! contour queries for the Lockheed P-38G-13-LO Lightning (`T.O. 01-75F-1`).
//!
//! Coordinate convention (Simulator / OpenUSD aircraft root frame):
//! - `+X` = Starboard (right wing) in metres (`WS` in inches = `|x| / 0.0254`),
//! - `+Y` = Forward (nose) in metres (`FS 0` = `+3.55 m`, `BS 0` = `+3.18 m`),
//! - `+Z` = Up (dorsal) in metres (`WL 0` = thrust line `+0.04 m`).

use std::f32::consts::FRAC_PI_2;

use glam::Vec3;

use crate::mesh::{P38Material, mat};

/// Exact Lockheed P-38G wingspan (`52 ft 0 in` = `15.850 m`).
pub const OVERALL_SPAN_M: f32 = 15.850;
/// Exact Lockheed P-38G wing semi-span (`26 ft 0 in` = `7.925 m`).
pub const SEMI_SPAN: f32 = 7.925;
/// Exact Lockheed P-38G overall length (`37 ft 10 in` = `11.530 m`, `y = +3.55..-7.98 m`).
pub const OVERALL_LENGTH_M: f32 = 11.530;
/// Airframe height on AN 01-75F-2, fin top to the drawing's inclined static ground line
/// (`9 ft 10 3/8 in` = `3.007 m`). The sheet's `12 ft 10 1/8 in` is the propeller arc.
pub const OVERALL_HEIGHT_M: f32 = 118.375 * 0.0254;
/// Root chord on the ship centreline (AN 01-75F-2 `ROOT CHORD 117 NACA 23016`, `2.972 m`).
pub const ROOT_CHORD_M: f32 = 117.0 * 0.0254;
/// Chord where the rounded tip begins, wing station 289 (`38 in` on the drawn outline,
/// `0.965 m`, `NACA 4412`; the sheet's `36` callout is taken where the tip arc starts).
pub const TIP_CHORD_M: f32 = 38.0 * 0.0254;
/// Span station where the rounded wingtip begins (`WS 289` = `7.341 m`).
pub const TIP_START_X: f32 = 289.0 * 0.0254;
/// Reference wing area (`327.5 sq ft` = `30.43 m²`).
pub const WING_AREA_M2: f32 = 30.43;
/// Outer-wing geometric dihedral angle (`5° 40'` = `5.67°`).
pub const WING_DIHEDRAL_DEG: f32 = 5.67;
/// Boom centreline from the ship centreline (AN 01-75F-2 plan view `96` in = `2.4384 m`).
pub const BOOM_X: f32 = 96.0 * 0.0254;
/// Where the exhaust leaves each boom: the GE B-13 turbine bucket wheel on the boom top,
/// `FS 205.6` (AN 01-75F-2 plan view).
pub const TURBO_NOZZLE_Y: f32 = 3.55 - 205.6 * 0.0254;
/// Outboard end of the aileron (AN 01-75F-2 plan view, `WS 289` = `7.341 m`).
pub const AILERON_END_X: f32 = TIP_START_X;
/// Spinner base and propeller hub frame origin, at the cowl joint (`FS 71.3` = `+1.739 m`).
pub const PROP_HUB_Y: f32 = 3.55 - 71.3 * 0.0254;
/// Propeller blade plane ahead of [`PROP_HUB_Y`]: blades at `FS 61.875` (AN 01-75F-2).
pub const PROP_BLADE_OFFSET_M: f32 = (71.3 - 61.875) * 0.0254;
/// Propeller thrust line vertical coordinate (`+0.04 m`).
pub const PROP_HUB_Z: f32 = 0.04;
/// Curtiss Electric `11 ft 6 in` (`3.505 m` diameter) propeller radius (`1.752 m`).
pub const PROP_RADIUS_M: f32 = 1.752;
/// Elevator hinge line: `25 in` aft of the stabiliser leading edge at `FS 397.584`
/// (AN 01-75F-2 plan view `25` + `20` chord split), `FS 422.584`.
pub const ELEVATOR_HINGE_Y: f32 = 3.55 - 422.584 * 0.0254;
/// Rudder hinge line: `37 in` aft of the empennage rotation point (AN 01-75F-2 side
/// view), `FS 428.4`.
pub const RUDDER_HINGE_Y: f32 = 3.55 - 428.4 * 0.0254;
/// Horizontal tail chord plane, `22 in` above the fuselage reference line. AN 01-75F-2
/// draws it `23.6 in` up on the side view and `20.2 in` up on the front view; this
/// splits them.
pub const HTAIL_Z: f32 = 0.4698;

// Historical PBR material palette for P-38G-13-LO "Miss Virginia" (s/n 43-2264, White 147)
/// ANA 613 Olive Drab No. 41: a dark, flat, brownish lacquer (dielectric).
pub(crate) const OLIVE_DRAB: P38Material = mat([0.16, 0.165, 0.095], 0.0, 0.70);
/// ANA 603 Neutral Gray No. 43, the semi-flat underside lacquer.
pub(crate) const NEUTRAL_GRAY: P38Material = mat([0.30, 0.315, 0.33], 0.0, 0.62);
pub(crate) const CANOPY_FRAME: P38Material = mat([0.21, 0.23, 0.15], 0.12, 0.62);
pub(crate) const INSIGNIA_WHITE: P38Material = mat([0.92, 0.92, 0.90], 0.05, 0.42);
/// Insignia Blue: a dark navy that prints near black on the period's
/// orthochromatic film (the boom stars of 43-2239); approximate.
pub(crate) const INSIGNIA_BLUE: P38Material = mat([0.022, 0.032, 0.085], 0.06, 0.46);
pub(crate) const INSIGNIA_RED: P38Material = mat([0.68, 0.10, 0.10], 0.06, 0.44);
pub(crate) const STENCIL_YELLOW: P38Material = mat([0.88, 0.68, 0.10], 0.05, 0.48);
pub(crate) const CHROME_OLEO: P38Material = mat([0.84, 0.86, 0.89], 0.92, 0.16);
pub(crate) const GUNMETAL: P38Material = mat([0.16, 0.17, 0.19], 0.82, 0.30);
pub(crate) const BRASS_SHELL: P38Material = mat([0.74, 0.58, 0.22], 0.78, 0.32);
pub(crate) const TURBO_STEEL: P38Material = mat([0.36, 0.31, 0.27], 0.78, 0.38);
pub(crate) const EXHAUST_RUST: P38Material = mat([0.30, 0.22, 0.18], 0.58, 0.54);
pub(crate) const PROP_BLACK: P38Material = mat([0.03, 0.03, 0.032], 0.0, 0.55);
pub(crate) const PROP_YELLOW: P38Material = mat([0.92, 0.74, 0.08], 0.08, 0.36);
pub(crate) const TIRE_RUBBER: P38Material = mat([0.035, 0.035, 0.037], 0.0, 0.92);
pub(crate) const WHEEL_WELL_ZINC: P38Material = mat([0.46, 0.52, 0.26], 0.14, 0.58);
pub(crate) const COCKPIT_GREEN: P38Material = mat([0.24, 0.32, 0.21], 0.10, 0.64);
pub(crate) const PANEL_BLACK: P38Material = mat([0.08, 0.09, 0.10], 0.18, 0.52);
pub(crate) const LEATHER_BROWN: P38Material = mat([0.34, 0.21, 0.12], 0.05, 0.72);
pub(crate) const PILOT_SUIT: P38Material = mat([0.42, 0.36, 0.24], 0.04, 0.78);

pub(crate) const CANOPY_GLASS: P38Material = P38Material {
    diffuse_rgb: [0.64, 0.78, 0.84],
    emissive_rgb: [0.02, 0.03, 0.04],
    metallic: 0.08,
    roughness: 0.08,
    opacity: 0.32,
};

pub(crate) const ARMOR_GLASS: P38Material = P38Material {
    diffuse_rgb: [0.46, 0.74, 0.68],
    emissive_rgb: [0.02, 0.04, 0.03],
    metallic: 0.10,
    roughness: 0.06,
    opacity: 0.44,
};

/// Central gondola bulkhead stations (`FS 0..238`, `y = +3.55..-2.50 m`). The keel
/// (`z_bot_hull`) follows the AN 01-75F-2 side view, down to `30 in` below the fuselage
/// reference line under the cockpit:
/// `(y, z_mid, half_w, z_top_hull, z_bot_hull, z_canopy_top, w_canopy)`.
pub const GONDOLA_STATIONS: &[(f32, f32, f32, f32, f32, f32, f32)] = &[
    (3.55, 0.02, 0.03, 0.05, -0.01, 0.05, 0.03), // FS 0: Nose tip
    (3.40, 0.02, 0.18, 0.16, -0.356, 0.16, 0.18), // FS 6: Cannon muzzle port
    (3.15, 0.02, 0.32, 0.29, -0.546, 0.29, 0.32), // FS 16: Staggered .50 cal ports
    (2.76, 0.03, 0.44, 0.41, -0.671, 0.41, 0.44), // FS 31: Forward ammo bay
    (2.20, 0.04, 0.53, 0.49, -0.767, 0.49, 0.53), // FS 53: Nose wheel well & gun breech bay
    (1.68, 0.05, 0.57, 0.52, -0.823, 0.52, 0.46), // FS 74: Windscreen base arch
    (1.16, 0.06, 0.60, 0.53, -0.846, 0.90, 0.44), // FS 94: Armored flat windshield top
    (0.58, 0.06, 0.62, 0.54, -0.848, 0.98, 0.45), // FS 117: Forward canopy / pilot head
    (0.06, 0.05, 0.62, 0.54, -0.815, 0.95, 0.44), // FS 137: Main wing spar carry-through
    (-0.42, 0.04, 0.59, 0.50, -0.749, 0.80, 0.38), // FS 156: Aft pilot armor bulkhead
    (-0.90, 0.04, 0.53, 0.45, -0.658, 0.56, 0.30), // FS 175: Radio compartment & canopy tail
    (-1.56, 0.03, 0.41, 0.35, -0.546, 0.35, 0.22), // FS 201: Gondola aft fairing
    (-2.22, 0.03, 0.24, 0.21, -0.419, 0.21, 0.14), // FS 227: Wing trailing-edge intersection
    (-2.495, 0.03, 0.04, 0.07, -0.05, 0.07, 0.04), // FS 238: Gondola tailcone tip at the wing TE
];

/// Twin engine nacelle & tail-boom bulkhead stations (`FS 71..440`, `y = +1.74..-7.63 m`); the
/// boom top runs level `24.5 in` above the fuselage reference line aft of the wing, as on
/// the AN 01-75F-2 side view, and the cowls are as wide as its plan view (`19.7 in` half-width
/// from `FS 80`, `21.1 in` by `FS 120`):
/// `(y, z_mid, half_w, z_top, z_bot, rad_w_bulge, rad_z_bulge)`.
pub const BOOM_STATIONS: &[(f32, f32, f32, f32, f32, f32, f32)] = &[
    (1.739, 0.04, 0.373, 0.37, -0.29, 0.00, 0.00), // FS 71.3: Cowl lip behind the 26 in spinner
    (1.518, 0.02, 0.499, 0.41, -0.52, 0.00, 0.00), // FS 80: Reduction gear & chin scoop lip
    (1.086, 0.00, 0.505, 0.48, -0.62, 0.00, 0.00), // FS 97: Oil cooler duct & forward V-12
    (0.375, 0.02, 0.536, 0.52, -0.56, 0.00, 0.00), // FS 125: Allison V-1710 mid cylinder block
    (-0.387, 0.04, 0.536, 0.53, -0.50, 0.00, 0.00), // FS 155: Firewall ahead of the main beam
    (-0.768, 0.05, 0.49, 0.51, -0.48, 0.00, 0.00), // FS 170: Main gear bay & turbo well
    (-1.15, 0.05, 0.47, 0.48, -0.46, 0.00, 0.00),  // FS 185: Turbosupercharger impeller center
    (-1.85, 0.05, 0.44, 0.50, -0.42, 0.00, 0.00),  // FS 213: Wing trailing-edge boom junction
    (-2.449, 0.070, 0.378, 0.526, -0.386, 0.000, 0.000), // FS 236: Boom ahead of the radiator
    (-3.105, 0.112, 0.338, 0.533, -0.310, 0.000, 0.000), // FS 262: Ahead of the radiator scoops
    (-3.308, 0.128, 0.317, 0.533, -0.277, 0.000, 0.000), // FS 270: Under the radiator scoops
    (-3.765, 0.164, 0.292, 0.533, -0.206, 0.000, 0.000), // FS 288: Under the radiator scoops
    (-4.146, 0.197, 0.267, 0.533, -0.140, 0.000, 0.000), // FS 303: Radiator exit flaps
    (-4.375, 0.220, 0.254, 0.533, -0.094, 0.000, 0.000), // FS 312: Radiator scoop tail
    (-4.578, 0.234, 0.248, 0.533, -0.066, 0.000, 0.000), // FS 320: Tail boom, top level
    (-5.340, 0.303, 0.196, 0.533, 0.074, 0.000, 0.000), // FS 350: Tail boom
    (-6.102, 0.371, 0.145, 0.543, 0.198, 0.000, 0.000), // FS 380: Tail boom
    (-6.559, 0.408, 0.132, 0.559, 0.256, 0.000, 0.000), // FS 398: Fin and tailplane root
    (-7.626, 0.510, 0.038, 0.536, 0.485, 0.000, 0.000), // FS 440: Tail cone inside the fin
];

/// Fin and rudder profile digitised from the AN 01-75F-2 side view, inner outline
/// (`WL -8.6..+76.3`): `(z, leading_y, trailing_y, half_thickness)`, 12 % thick.
pub const FIN_PROFILE: &[(f32, f32, f32, f32)] = &[
    (-0.307, -7.245, -7.308, 0.004), // WL -8.6: FS 425.0..427.5
    (-0.292, -7.189, -7.367, 0.011), // WL -8.0: FS 422.8..429.8
    (-0.241, -7.090, -7.392, 0.018), // WL -6.0: FS 418.9..430.8
    (-0.191, -6.981, -7.484, 0.030), // WL -4.0: FS 414.6..434.4
    (-0.140, -6.884, -7.598, 0.043), // WL -2.0: FS 410.8..438.9
    (-0.089, -6.823, -7.684, 0.052), // WL +0.0: FS 408.4..442.3
    (0.013, -6.737, -7.796, 0.064),  // WL +4.0: FS 405.0..446.7
    (0.114, -6.663, -7.862, 0.072),  // WL +8.0: FS 402.1..449.3
    (0.216, -6.590, -7.913, 0.079),  // WL +12.0: FS 399.2..451.3
    (0.368, -6.546, -7.946, 0.084),  // WL +18.0: FS 397.5..452.6
    (0.521, -6.529, -7.969, 0.086),  // WL +24.0: FS 396.8..453.5
    (0.673, -6.585, -7.977, 0.084),  // WL +30.0: FS 399.0..453.8
    (0.927, -6.625, -7.923, 0.078),  // WL +40.0: FS 400.6..451.7
    (1.181, -6.689, -7.837, 0.069),  // WL +50.0: FS 403.1..448.3
    (1.435, -6.780, -7.728, 0.057),  // WL +60.0: FS 406.7..444.0
    (1.562, -6.841, -7.667, 0.050),  // WL +65.0: FS 409.1..441.6
    (1.689, -6.930, -7.570, 0.038),  // WL +70.0: FS 412.6..437.8
    (1.765, -7.011, -7.486, 0.028),  // WL +73.0: FS 415.8..434.5
    (1.816, -7.088, -7.408, 0.019),  // WL +75.0: FS 418.8..431.4
    (1.849, -7.207, -7.270, 0.004),  // WL +76.3: FS 423.5..426.0
];

/// Butt lines (inches) the tailplane is lofted through: the tips' arcs, the booms and
/// the centreline.
pub(crate) const TAILPLANE_SECTIONS_BL: [f32; 19] = [
    -130.4, -129.5, -127.5, -124.5, -120.5, -115.0, -108.0, -100.0, -92.0, 0.0, 92.0, 100.0, 108.0,
    115.0, 120.5, 124.5, 127.5, 129.5, 130.4,
];

/// Horizontal-tail planform at butt line `bl_in` (inches, either side): `(leading_fs,
/// trailing_fs)`, or `None` beyond the tip. AN 01-75F-2 plan view: leading edge at
/// `FS 397.584`, `45 in` chord, `261 in` span, and full-chord semicircular tips.
#[must_use]
pub fn tailplane_planform_fs(bl_in: f32) -> Option<(f32, f32)> {
    const LE_FS: f32 = 397.584;
    const CHORD: f32 = 45.0;
    const HALF_SPAN: f32 = 130.5;
    let r = 0.5 * CHORD;
    let tip_centre = HALF_SPAN - r;
    let bl = bl_in.abs();
    if bl > HALF_SPAN {
        return None;
    }
    let half = if bl <= tip_centre {
        r
    } else {
        (r * r - (bl - tip_centre).powi(2)).max(0.0).sqrt()
    };
    Some((LE_FS + r - half, LE_FS + r + half))
}

/// Linearly interpolates the central gondola cross-section at longitudinal coordinate `y`.
#[must_use]
pub fn interpolate_gondola(y: f32) -> (f32, f32, f32, f32, f32, f32) {
    let Some(&first) = GONDOLA_STATIONS.first() else {
        return (0.0, 0.1, 0.1, -0.1, 0.1, 0.1);
    };
    if y >= first.0 {
        return (first.1, first.2, first.3, first.4, first.5, first.6);
    }
    for pair in GONDOLA_STATIONS.windows(2) {
        if let [a, b] = pair
            && y >= b.0
        {
            let t = ((a.0 - y) / (a.0 - b.0).max(1e-5)).clamp(0.0, 1.0);
            let lerp = |u: f32, v: f32| u + (v - u) * t;
            return (
                lerp(a.1, b.1),
                lerp(a.2, b.2),
                lerp(a.3, b.3),
                lerp(a.4, b.4),
                lerp(a.5, b.5),
                lerp(a.6, b.6),
            );
        }
    }
    let last = GONDOLA_STATIONS.last().copied().unwrap_or(first);
    (last.1, last.2, last.3, last.4, last.5, last.6)
}

/// Evaluates a 3D surface point on the central gondola hull at `(y, theta)`.
#[must_use]
pub fn gondola_point(y: f32, theta: f32) -> Vec3 {
    let (z_mid, half_w, z_top, z_bot, _, _) = interpolate_gondola(y);
    let (sin_t, cos_t) = theta.sin_cos();
    let x = sin_t.signum() * sin_t.abs().powf(0.84) * half_w;
    let z = if cos_t >= 0.0 {
        z_mid + cos_t.powf(0.88) * (z_top - z_mid)
    } else {
        z_mid - (-cos_t).powf(0.82) * (z_mid - z_bot)
    };
    Vec3::new(x, y, z)
}

/// Linearly interpolates a tail-boom cross-section at longitudinal coordinate `y`.
#[must_use]
pub fn interpolate_boom(y: f32) -> (f32, f32, f32, f32, f32, f32) {
    let Some(&first) = BOOM_STATIONS.first() else {
        return (0.0, 0.1, 0.1, -0.1, 0.0, 0.0);
    };
    if y >= first.0 {
        return (first.1, first.2, first.3, first.4, first.5, first.6);
    }
    for pair in BOOM_STATIONS.windows(2) {
        if let [a, b] = pair
            && y >= b.0
        {
            let t = ((a.0 - y) / (a.0 - b.0).max(1e-5)).clamp(0.0, 1.0);
            let lerp = |u: f32, v: f32| u + (v - u) * t;
            return (
                lerp(a.1, b.1),
                lerp(a.2, b.2),
                lerp(a.3, b.3),
                lerp(a.4, b.4),
                lerp(a.5, b.5),
                lerp(a.6, b.6),
            );
        }
    }
    let last = BOOM_STATIONS.last().copied().unwrap_or(first);
    (last.1, last.2, last.3, last.4, last.5, last.6)
}

/// Evaluates a 3D surface point on the left (`side = -1`) or right (`side = +1`) boom at `(y, theta)`.
#[must_use]
pub fn boom_point(side: f32, y: f32, theta: f32) -> Vec3 {
    let (z_mid, half_w, z_top, z_bot, rad_w, rad_z) = interpolate_boom(y);
    let (sin_t, cos_t) = theta.sin_cos();
    let side_lobe = (sin_t.abs() * (1.0 - (cos_t + 0.05).powi(2)).max(0.0)).powi(2);
    let w = half_w + rad_w * side_lobe;
    let x_local = sin_t.signum() * sin_t.abs().powf(0.86) * w;
    let z = if cos_t >= 0.0 {
        z_mid + cos_t.powf(0.86) * (z_top + rad_z * side_lobe - z_mid)
    } else {
        z_mid - (-cos_t).powf(0.80) * (z_mid - (z_bot - rad_z * 0.5 * side_lobe))
    };
    Vec3::new(side * BOOM_X + x_local, y, z)
}

/// Returns `(leading_y, trailing_y, z_mid, thickness_ratio, incidence_rad)` at span station `x`.
///
/// AN 01-75F-2 planform: a straight taper from the `117 in` root chord on the ship
/// centreline to `38 in` at `WS 289`, then a rounded tip to `WS 312`; the main beam at
/// `35 %` chord is a straight line at `FS 160`. The chord plane passes through the
/// fuselage reference line on the centreline with `5° 40'` dihedral (`30.958 in` above it at
/// the tip), and the whole wing is rotated `2°` nose-up about the main beam. `z_mid` is the
/// chord plane at the main beam.
#[must_use]
pub fn wing_station(x: f32) -> (f32, f32, f32, f32, f32) {
    use crate::drawing::{INCH_M, frl_to_z, fs_to_y};
    const MAIN_BEAM_FS: f32 = 160.0;
    const BEAM_CHORD_FRACTION: f32 = 0.35;
    let ax = x.abs().min(SEMI_SPAN);
    let bl = ax / INCH_M;
    let root = ROOT_CHORD_M / INCH_M;
    let tip_start = TIP_START_X / INCH_M;
    let tip = TIP_CHORD_M / INCH_M;
    let taper = root - (root - tip) * (bl.min(tip_start) / tip_start);
    let chord_in = if bl <= tip_start {
        taper
    } else {
        let t = ((bl - tip_start) / (SEMI_SPAN / INCH_M - tip_start)).clamp(0.0, 1.0);
        (tip * (1.0 - t * t).max(0.0).powf(0.7)).max(1.5)
    };
    let lead_fs = MAIN_BEAM_FS - BEAM_CHORD_FRACTION * chord_in;
    let trail_fs = MAIN_BEAM_FS + (1.0 - BEAM_CHORD_FRACTION) * chord_in;
    let thick_ratio = 0.160 - 0.040 * (bl / tip_start).min(1.0);
    let z_mid = frl_to_z(bl * WING_DIHEDRAL_DEG.to_radians().tan());
    (
        fs_to_y(lead_fs),
        fs_to_y(trail_fs),
        z_mid,
        thick_ratio,
        2.0_f32.to_radians(),
    )
}

/// Evaluates a 3D point on the cambered `NACA 23016 -> NACA 4412` wing skin.
#[must_use]
pub fn wing_surface_point(x: f32, chord_frac: f32, upper: bool) -> Vec3 {
    let (lead_y, trail_y, z_mid, thick_ratio, inc_rad) = wing_station(x);
    let chord = (lead_y - trail_y).max(0.05);
    let u = chord_frac.clamp(0.0, 1.0);
    let yt = 5.0
        * thick_ratio
        * (0.2969 * u.sqrt() - 0.1260 * u - 0.3516 * u * u + 0.2843 * u.powi(3)
            - 0.1036 * u.powi(4));
    let camber = 0.022
        * if u < 0.30 {
            (2.0 * 0.30 * u - u * u) / (0.30 * 0.30)
        } else {
            ((1.0 - 2.0 * 0.30) + 2.0 * 0.30 * u - u * u) / ((1.0 - 0.30) * (1.0 - 0.30))
        };
    let sign = if upper { 1.0 } else { -1.0 };
    let z_local = (camber + sign * yt) * chord + (0.35 - u) * chord * inc_rad.sin();
    let y = lead_y - u * chord;
    Vec3::new(x, y, z_mid + z_local)
}

/// Linearly interpolates the vertical fin profile at vertical coordinate `z`.
#[must_use]
pub fn interpolate_fin(z: f32) -> (f32, f32, f32) {
    let Some(&first) = FIN_PROFILE.first() else {
        return (-6.0, -7.5, 0.05);
    };
    if z <= first.0 {
        return (first.1, first.2, first.3);
    }
    for pair in FIN_PROFILE.windows(2) {
        if let [a, b] = pair
            && z <= b.0
        {
            let t = ((z - a.0) / (b.0 - a.0).max(1e-5)).clamp(0.0, 1.0);
            return (
                a.1 + (b.1 - a.1) * t,
                a.2 + (b.2 - a.2) * t,
                a.3 + (b.3 - a.3) * t,
            );
        }
    }
    let last = FIN_PROFILE.last().copied().unwrap_or(first);
    (last.1, last.2, last.3)
}

/// Evaluates a 3D surface point on a vertical fin at `(side, z, chord_frac, out_sign)`.
#[must_use]
pub fn fin_point(side: f32, z: f32, chord_frac: f32, out_sign: f32) -> Vec3 {
    let (ly, ty, half_t) = interpolate_fin(z);
    let u = chord_frac.clamp(0.0, 1.0);
    let naca = 5.0
        * (0.2969 * u.sqrt() - 0.1260 * u - 0.3516 * u * u + 0.2843 * u.powi(3)
            - 0.1036 * u.powi(4));
    let y = ly + (ty - ly) * u;
    let x = side * BOOM_X + out_sign * half_t * naca;
    Vec3::new(x, y, z)
}

/// Top-view reference half-width of the central gondola at longitudinal station `y`.
#[must_use]
pub fn three_view_gondola_outline_x(y: f32) -> f32 {
    interpolate_gondola(y).1
}

/// Top-view reference half-width of a tail boom at longitudinal station `y`.
#[must_use]
pub fn three_view_boom_outline_x(y: f32) -> f32 {
    let (_, half_w, _, _, rad_w, _) = interpolate_boom(y);
    let sin_t = FRAC_PI_2.sin();
    let cos_t = FRAC_PI_2.cos();
    let side_lobe = (sin_t.abs() * (1.0 - (cos_t + 0.05).powi(2)).max(0.0)).powi(2);
    half_w + rad_w * side_lobe
}

/// Top-view reference wing leading-edge coordinate `y` at spanwise station `x`.
#[must_use]
pub fn three_view_wing_leading_y(x: f32) -> f32 {
    wing_station(x).0
}

/// Top-view reference wing trailing-edge coordinate `y` at spanwise station `x`.
#[must_use]
pub fn three_view_wing_trailing_y(x: f32) -> f32 {
    wing_station(x).1
}
