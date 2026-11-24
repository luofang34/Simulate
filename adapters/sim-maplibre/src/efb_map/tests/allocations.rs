//! Drawing the plan allocates nothing that outlives a frame. On the web a dropped wgpu
//! buffer or texture lives on until the page's garbage collector frees it, so a frame or a
//! plan update that allocated would grow the tab however promptly the Rust side dropped it.

use super::plan::sample_efb;
use crate::{
    ChartOwnship, ChartSymbologyInput, EfbDisplayMode, EfbFollowMode, EfbTracks, FixId, FlightPlan,
    RouteFix, RouteProgress, SigmaRing, TrackLine,
};

fn fix(id: u32, lat_deg: f64, lon_deg: f64) -> RouteFix {
    RouteFix {
        id: FixId(id),
        ident: format!("F{id}"),
        lat_deg,
        lon_deg,
        altitude_m: None,
    }
}

/// A track of `n` points and a ring that grows with it.
fn tracks(n: u32) -> EfbTracks {
    EfbTracks {
        lines: vec![TrackLine {
            colour: [0xff, 0xe0, 0x66],
            width_px: 2.0,
            points: (0..n)
                .map(|i| {
                    [
                        40.69 + 0.0002 * f64::from(i),
                        -74.17 + 0.0003 * f64::from(i),
                    ]
                })
                .collect(),
        }],
        rings: vec![SigmaRing {
            centre_deg: [40.70, -74.15],
            sigma_m: f64::from(n),
            colour: [0xd9, 0x59, 0x26],
        }],
        ..EfbTracks::default()
    }
}

/// Live buffers and textures on `device`.
fn live(device: &wgpu::Device) -> [isize; 2] {
    let hal = device.get_internal_counters().hal;
    [hal.buffers.read(), hal.textures.read()]
}

/// Frames every warm-up draws, enough for the pipelines and the largest track.
const WARM_UP_FRAMES: u32 = 120;
/// Frames the counters hold still before the warm-up counts as over.
const SETTLED_FRAMES: u32 = 60;
/// Frames after which a warm-up that never settles is measured anyway, and fails.
const WARM_UP_LIMIT: u32 = 3000;

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn drawing_both_views_and_updating_the_plan_allocates_nothing_lasting() {
    let Some(mut efb) = sample_efb() else {
        return;
    };
    let device = efb.runtime.host_gpu().device.clone();
    let plan = FlightPlan {
        origin: Some([40.69, -74.17]),
        fixes: vec![fix(0, 40.69, -74.14), fix(1, 40.71, -74.12)],
    };
    efb.set_all_views_shown(true);
    efb.set_display_mode(EfbDisplayMode::ExpandedEfb);
    efb.set_follow_mode(EfbFollowMode::FollowNorthUp);
    let own = ChartOwnship {
        lat_deg: 40.70,
        lon_deg: -74.15,
        true_heading_deg: 45.0,
        ground_velocity_en_mps: [40.0, 40.0],
    };
    let symbology = ChartSymbologyInput {
        camera: efb.camera().spec(),
        viewport_px: [1, 1],
        ownship: Some(own),
        runway: None,
        traffic: &[],
    };
    let frame = |efb: &mut super::EfbMapSurface, step: u32| {
        if step.is_multiple_of(5) {
            let progress = RouteProgress::after_sequenced(Some((0, step.is_multiple_of(10))));
            efb.set_route(&plan, progress).expect("route");
            // Keep geometry within the peak size warmed on step 0 so lazy small-slab
            // growth in `BufferPool` settles before the zero-growth measurement window.
            let n = if step == 0 { 120 } else { 2 + (step % 118) };
            efb.set_tracks(tracks(n)).expect("tracks");
        }
        efb.render(&symbology).expect("frame");
    };
    // Warm up: tiles, pipelines, the overlay's buffers and vector slabs reach their size.
    // Tiles arrive on the loader's clock, not the frame count, so the warm-up ends when
    // the counters have held still, however long a busy machine takes to get there.
    let mut step = 0;
    let mut settled = (live(&device), 0_u32);
    while step < WARM_UP_FRAMES || (settled.1 < SETTLED_FRAMES && step < WARM_UP_LIMIT) {
        frame(&mut efb, step);
        step += 1;
        let now = live(&device);
        settled = if now == settled.0 {
            (now, settled.1 + 1)
        } else {
            (now, 0)
        };
    }
    let before = live(&device);
    for step in step..step + 200 {
        frame(&mut efb, step);
    }
    let after = live(&device);
    assert!(before[0] > 0, "the counters count: {before:?}");
    assert!(
        after[0] <= before[0] && after[1] <= before[1],
        "live [buffers, textures] grew from {before:?} to {after:?} over 200 frames"
    );
}
