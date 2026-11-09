//! Paint finish of the P-38G's parts, keyed by the vehicle crate's USD prim paths.
//!
//! One panel grid in aircraft metres serves every stressed-skin part, so seams run on
//! across part boundaries and under the painted markings: ribs every `0.55 m` of span,
//! frames every `0.50 m` of station and waterline seams every `0.30 m`. Control surfaces
//! are authored in hinge-local axes with the hinge along `x`, so their `x` seams are ribs.

use sim_p38g::{BOOM_X, TURBO_NOZZLE_Y};

use crate::usd_aircraft::finish::{SootStreak, SurfaceDetail};

const SKIN_GRID_M: [f32; 3] = [0.55, 0.50, 0.30];
const RIBS_ONLY_M: [f32; 3] = [0.35, 0.0, 0.0];
/// Wing skins run long spanwise between spars, so their chordwise seams are sparser.
const WING_GRID_M: [f32; 3] = [0.55, 0.95, 0.0];

/// The finish of the part at `prim_path`.
pub(super) fn detail(prim_path: &str) -> SurfaceDetail {
    let path = prim_path.trim_start_matches("/P38g/");
    let skin = |weathering| SurfaceDetail {
        panel_pitch_m: SKIN_GRID_M,
        weathering,
        soot: None,
    };
    match path {
        "Booms/UpperOlive" => SurfaceDetail {
            soot: Some(SootStreak {
                lateral_m: BOOM_X,
                station_m: TURBO_NOZZLE_Y + 0.05,
                length_m: 2.6,
                strength: 1.4,
            }),
            ..skin(1.0)
        },
        "Wing/UpperOlive" => SurfaceDetail {
            panel_pitch_m: WING_GRID_M,
            ..skin(1.0)
        },
        "Wing/LowerGray" => SurfaceDetail {
            panel_pitch_m: WING_GRID_M,
            ..skin(0.45)
        },
        "Booms/LowerGray" | "Gondola/LowerGray" => skin(0.45),
        "Wing/FuelBayPanels" | "Wing/KarmanFillets" => SurfaceDetail {
            panel_pitch_m: [0.0; 3],
            weathering: 1.0,
            soot: None,
        },
        p if p.starts_with("Gondola/")
            || p.starts_with("Booms/")
            || p.starts_with("Decals/")
            || p == "Empennage/VerticalFins"
            || p == "Empennage/HorizontalTail" =>
        {
            skin(1.0)
        }
        p if p.starts_with("Wing/") || p.starts_with("Empennage/") => SurfaceDetail {
            panel_pitch_m: RIBS_ONLY_M,
            weathering: 0.9,
            soot: None,
        },
        p if p.starts_with("Gear/WheelWells") || p.starts_with("Cooling/") => SurfaceDetail {
            panel_pitch_m: [0.0; 3],
            weathering: 0.6,
            soot: None,
        },
        _ => SurfaceDetail::default(),
    }
}
