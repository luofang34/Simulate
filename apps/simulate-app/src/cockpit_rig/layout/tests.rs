//! Dock geometry on wide, narrow, tiny and phone-shaped windows.

use super::*;

#[test]
fn docked_panels_keep_four_by_three_pixels_on_a_wide_window() {
    let size = [1280_u32, 720_u32];
    let rects = dock_rects(size, 3, SafeAreaInsets::NONE);
    assert_eq!(rects.len(), 3);
    for rect in &rects {
        let aspect = (rect.width * 1280.0) / (rect.height * 720.0);
        assert!((aspect - PANEL_ASPECT).abs() < 1e-4, "{aspect}");
        assert!((rect.y + rect.height + GAP_BOTTOM - 1.0).abs() < 1e-6);
    }
    assert!((rects[0].width - 0.255).abs() < 1e-3);
    assert!((rects[1].x + rects[1].width * 0.5 - 0.5).abs() < 1e-4);
}

#[test]
fn a_narrow_window_shrinks_the_dock_instead_of_overlapping() {
    let rects = dock_rects([600, 900], 3, SafeAreaInsets::NONE);
    for pair in rects.windows(2) {
        assert!(pair[0].x + pair[0].width <= pair[1].x + 1e-5);
    }
    let r = rects[2];
    assert!(r.x + r.width <= 1.0 + 1e-5);
    assert!(((r.width * 600.0) / (r.height * 900.0) - PANEL_ASPECT).abs() < 1e-4);
}

#[test]
fn left_rail_stacks_pfd_above_hsi_without_overlap_or_aspect_distortion() {
    for size in [[1704_u32, 786_u32], [786_u32, 1704_u32]] {
        let layout = dock_layout(
            size,
            InstrumentDockStyle::LeftRail,
            false,
            DockClearance::NONE,
        );
        let pfd = viewport_to_px_rect(layout.pfd, size);
        let hsi = viewport_to_px_rect(layout.hsi, size);
        assert_eq!([pfd[2], pfd[3]], layout.panel_px);
        assert_eq!([hsi[2], hsi[3]], layout.panel_px);
        assert_eq!(pfd[0], hsi[0]);
        assert!(pfd[1] + pfd[3] < hsi[1], "PFD must sit cleanly above HSI");
        assert!(hsi[1] + hsi[3] <= size[1]);
        assert!(layout.right_edge_norm > 0.15 && layout.right_edge_norm < 0.45);
        let aspect = pfd[2] as f32 / pfd[3] as f32;
        assert!((aspect - PANEL_ASPECT).abs() < 0.02, "{aspect}");
    }
}

#[test]
fn docks_stay_inside_a_surface_of_any_size() {
    let insets = SafeAreaInsets::from_lengths([59.0, 59.0, 59.0, 21.0], [402.0, 402.0]);
    for size in [
        [1_u32, 1_u32],
        [60, 1],
        [1, 60],
        [47, 47],
        [300, 20],
        [20, 300],
    ] {
        for style in [
            InstrumentDockStyle::LeftRail,
            InstrumentDockStyle::BottomRow,
        ] {
            for system in [SafeAreaInsets::NONE, insets] {
                let insets = DockClearance {
                    system,
                    thumbs: system,
                };
                let layout = dock_layout(size, style, true, insets);
                for rect in [Some(layout.pfd), layout.mfd, Some(layout.hsi)]
                    .into_iter()
                    .flatten()
                {
                    let [x, y, w, h] = viewport_to_px_rect(rect, size);
                    assert!(w >= 1 && h >= 1, "{size:?} {style:?}");
                    assert!(x + w <= size[0].max(1), "{size:?} {style:?}");
                    assert!(y + h <= size[1].max(1), "{size:?} {style:?}");
                }
            }
        }
    }
}

#[test]
fn the_rail_clears_a_landscape_phone_sensor_housing() {
    let size = [1748_u32, 804_u32];
    let housing = SafeAreaInsets::from_lengths([59.0, 0.0, 59.0, 21.0], [874.0, 402.0]);
    let bare = dock_layout(
        size,
        InstrumentDockStyle::LeftRail,
        false,
        DockClearance::NONE,
    );
    let clear = dock_layout(
        size,
        InstrumentDockStyle::LeftRail,
        false,
        DockClearance {
            system: housing,
            thumbs: SafeAreaInsets::NONE,
        },
    );
    let [bare_x, ..] = viewport_to_px_rect(bare.pfd, size);
    let [x, y, w, h] = viewport_to_px_rect(clear.pfd, size);
    let [_, hsi_y, _, hsi_h] = viewport_to_px_rect(clear.hsi, size);
    assert!(
        bare_x < 118,
        "an uncovered window keeps the standard margin"
    );
    assert_eq!(x, 118, "the larger of margin and inset, not their sum");
    assert!(y + h < hsi_y);
    assert!(hsi_y + hsi_h <= 804 - 42);
    assert!((w as f32 / h as f32 - PANEL_ASPECT).abs() < 0.02);
}

#[test]
fn the_bottom_row_clears_both_side_insets_and_the_home_indicator() {
    let size = [1748_u32, 804_u32];
    let housing = SafeAreaInsets::from_lengths([59.0, 0.0, 59.0, 21.0], [874.0, 402.0]);
    let rects = dock_rects(size, 2, housing);
    let first = viewport_to_px_rect(rects[0], size);
    let last = viewport_to_px_rect(rects[1], size);
    assert!(first[0] >= 118);
    assert!(last[0] + last[2] <= 1748 - 118);
    assert!(last[1] + last[3] <= 804 - 42);
}

#[test]
fn a_tall_window_takes_the_rail_whatever_is_asked_for() {
    for style in [
        InstrumentDockStyle::LeftRail,
        InstrumentDockStyle::BottomRow,
    ] {
        assert_eq!(
            drawn_style(style, [804, 1748]),
            InstrumentDockStyle::LeftRail
        );
        assert_eq!(drawn_style(style, [1748, 804]), style);
    }
}

#[test]
fn a_landscape_phone_row_sits_between_the_thumb_controls() {
    let size = [1748_u32, 804_u32];
    let clearance = DockClearance {
        system: SafeAreaInsets::from_lengths([59.0, 0.0, 59.0, 21.0], [874.0, 402.0]),
        thumbs: SafeAreaInsets::from_lengths([250.0, 0.0, 205.0, 0.0], [874.0, 402.0]),
    };
    let layout = dock_layout(size, InstrumentDockStyle::BottomRow, false, clearance);
    let pfd = viewport_to_px_rect(layout.pfd, size);
    let hsi = viewport_to_px_rect(layout.hsi, size);
    assert!(pfd[0] >= 500, "clear of the left thumb: {pfd:?}");
    assert!(
        hsi[0] + hsi[2] <= 1748 - 410,
        "clear of the right thumb: {hsi:?}"
    );
    assert!(pfd[0] + pfd[2] <= hsi[0], "side by side: {pfd:?} {hsi:?}");
    assert!(pfd[3] >= 200, "large enough to read: {pfd:?}");
    assert!(pfd[1] + pfd[3] <= 804 - 42, "above the home indicator");
    assert!(layout.right_edge_norm.abs() < f32::EPSILON);
}
