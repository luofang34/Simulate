//! The plan over terrain: tilted toward the horizon, the map drapes the plan's layers on the
//! relief, and the marks and labels the host lays over it stand on the same ground.

use sim_geo::Geodetic;
use sim_scenery::{TerrariumTile, project_to_tile};

use super::plan::sample_efb_on;
use crate::chart_symbology::screen_positions;
use crate::{
    ChartSymbologyInput, EfbDisplayMode, EfbFollowMode, FixId, FlightPlan, RouteFix, RouteProgress,
};

/// Magenta pixels' centroid within `radius` pixels of `near`, in a `width`-wide frame.
fn magenta_centroid(rgba: &[u8], width: u32, near: [f32; 2], radius: f32) -> Option<[f32; 2]> {
    let (mut sum, mut count) = ([0.0_f32; 2], 0_u32);
    for (index, &[r, g, b, _]) in rgba.as_chunks::<4>().0.iter().enumerate() {
        let (x, y) = ((index as u32 % width) as f32, (index as u32 / width) as f32);
        let magenta = r > 200 && g < 120 && b > 200;
        if magenta && (x - near[0]).hypot(y - near[1]) <= radius {
            sum = [sum[0] + x, sum[1] + y];
            count = count.wrapping_add(1);
        }
    }
    (count > 10).then(|| [sum[0] / count as f32, sum[1] / count as f32])
}

/// Zoom of the sloping elevation tile.
const DEM_ZOOM: u8 = 10;

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn tilted_over_terrain_the_labels_stand_where_the_draped_fixes_are() {
    // Ground rising 1500 m from west to east across the z10 tile under the plan.
    let Some(mut efb) = sample_efb_on(
        |scenery| {
            let tile = project_to_tile(&Geodetic::from_degrees(40.70, -74.16, 0.0), DEM_ZOOM)
                .expect("tile")
                .tile;
            let heights = (0..16 * 16).map(|i| f64::from(i % 16) * 100.0).collect();
            let dem = TerrariumTile::from_heights(16, 16, heights).expect("dem");
            scenery.insert_elevation(tile, dem).expect("insert");
        },
        DEM_ZOOM,
    ) else {
        return;
    };
    // One fix flown toward and no leg into it: the only magenta is the fix's own circle.
    let fix = |id: u32, lat_deg: f64, lon_deg: f64| RouteFix {
        id: FixId(id),
        ident: format!("F{id}"),
        lat_deg,
        lon_deg,
        altitude_m: None,
    };
    let plan = FlightPlan {
        origin: None,
        fixes: vec![fix(0, 40.70, -74.16), fix(1, 40.72, -74.10)],
    };
    efb.set_route(&plan, RouteProgress::after_sequenced(None))
        .expect("route");
    efb.set_display_mode(EfbDisplayMode::ExpandedEfb);
    efb.set_follow_mode(EfbFollowMode::FollowNorthUp);
    efb.zoom_by(13.0 - efb.zoom());
    efb.pitch_by(45.0);
    let own = crate::ChartOwnship {
        lat_deg: 40.69,
        lon_deg: -74.17,
        true_heading_deg: 180.0,
        ground_velocity_en_mps: [0.0, -40.0],
    };
    let symbology = ChartSymbologyInput {
        camera: efb.camera().spec(),
        viewport_px: [1, 1],
        ownship: Some(own),
        runway: None,
        traffic: &[],
    };
    for _ in 0..200 {
        efb.render(&symbology).expect("frame");
        if !efb.needs_redraw() {
            break;
        }
        std::thread::yield_now();
    }
    let spec = efb
        .camera_spec(EfbDisplayMode::ExpandedEfb)
        .expect("camera");
    assert!((spec.pitch_deg - 45.0).abs() < 1e-9);
    let label = efb.fix_positions(EfbDisplayMode::ExpandedEfb)[0].expect("on screen");
    let size = efb.size_of(EfbDisplayMode::ExpandedEfb);
    let flat = screen_positions((&spec, size, efb.pixel_ratio()), None, &[[40.70, -74.16]])[0]
        .expect("on screen");
    let drawn = magenta_centroid(&efb.read_active_rgba().expect("rgba"), size[0], label, 14.0)
        .expect("the draped fix is drawn by its label");
    let off = |[x, y]: [f32; 2]| (x - drawn[0]).hypot(y - drawn[1]);
    assert!(off(label) < 3.0, "label {label:?}, drawn {drawn:?}");
    assert!(
        off(flat) > off(label) + 3.0,
        "standing it at sea level would miss: {flat:?}"
    );
}
