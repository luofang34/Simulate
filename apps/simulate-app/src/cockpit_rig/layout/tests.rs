//! Dock geometry on wide, narrow, tiny and phone-shaped windows: a desktop's bottom row
//! at its fixed fractions, and a handheld's left column inside the insets and the left
//! thumb's reach.

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

/// A phone held sideways: the sensor housing covers the left, the home indicator the bottom.
const PHONE_WIDE: [f64; 2] = [874.0, 402.0];
const PHONE_WIDE_INSETS: [f64; 4] = [59.0, 0.0, 59.0, 21.0];
/// The same phone upright: `[left, top, right, bottom]`.
const PHONE_TALL: [f64; 2] = [402.0, 874.0];
const PHONE_TALL_INSETS: [f64; 4] = [0.0, 59.0, 0.0, 34.0];

fn rail(size: [u32; 2], insets: SafeAreaInsets) -> ([u32; 4], [u32; 4], PanelDockLayout) {
    let layout = dock_layout(size, InstrumentDockStyle::LeftRail, true, insets);
    (
        viewport_to_px_rect(layout.pfd, size),
        viewport_to_px_rect(layout.hsi, size),
        layout,
    )
}

#[test]
fn left_rail_stacks_pfd_above_hsi_without_overlap_or_aspect_distortion() {
    for size in [[1704_u32, 786_u32], [786_u32, 1704_u32]] {
        let (pfd, hsi, layout) = rail(size, SafeAreaInsets::NONE);
        assert_eq!([pfd[2], pfd[3]], layout.panel_px);
        assert_eq!([hsi[2], hsi[3]], layout.panel_px);
        assert_eq!(pfd[0], hsi[0]);
        assert!(pfd[1] + pfd[3] < hsi[1], "PFD must sit cleanly above HSI");
        assert!(hsi[1] + hsi[3] <= size[1]);
        assert!(layout.right_edge_norm > 0.15 && layout.right_edge_norm < 0.45);
        let aspect = pfd[2] as f32 / pfd[3] as f32;
        assert!((aspect - PANEL_ASPECT).abs() < 0.02, "{aspect}");
        assert!(layout.mfd.is_none(), "the rail has no place for a chart");
    }
}

#[test]
fn docks_stay_inside_a_surface_of_any_size() {
    let covered = SafeAreaInsets::from_lengths([59.0, 59.0, 59.0, 21.0], [402.0, 402.0]);
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
            for insets in [SafeAreaInsets::NONE, covered] {
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
fn a_landscape_phone_column_stays_inside_the_insets_and_the_left_thumbs_reach() {
    let insets = SafeAreaInsets::from_lengths(PHONE_WIDE_INSETS, PHONE_WIDE);
    for scale in [1_u32, 2, 3] {
        let size = [874 * scale, 402 * scale];
        let (pfd, hsi, layout) = rail(size, insets);
        assert_eq!(pfd[0], 59 * scale, "the larger of margin and inset");
        assert_eq!(pfd[0], hsi[0]);
        let right = pfd[0] + pfd[2];
        assert!(
            right as f32 <= RAIL_REACH_FRACTION * size[0] as f32,
            "{scale}x: {right} is past the reach"
        );
        assert!(right >= 230 * scale, "{scale}x: the reach is used: {right}");
        assert!(layout.right_edge_norm <= RAIL_REACH_FRACTION);
        assert!(pfd[1] + pfd[3] < hsi[1], "PFD above HSI, apart");
        assert!(hsi[1] + hsi[3] <= (402 - 21) * scale, "above the home bar");
        assert!(hsi[0] + hsi[2] <= size[0] && hsi[1] + hsi[3] <= size[1]);
        assert!((pfd[2] as f32 / pfd[3] as f32 - PANEL_ASPECT).abs() < 0.02);
        assert!(layout.mfd.is_none(), "a handheld shows one map");
    }
    let (pfd, hsi, _) = rail([874, 402], insets);
    assert_eq!(pfd, [59, 66, 176, 132]);
    assert_eq!(hsi, [59, 204, 176, 132]);
}

#[test]
fn the_column_is_sane_on_a_small_phone_and_on_an_upright_one() {
    let (pfd, hsi, _) = rail([667, 375], SafeAreaInsets::NONE);
    assert!(pfd[0] >= 4, "a standard margin: {pfd:?}");
    assert!((pfd[0] + pfd[2]) as f32 <= RAIL_REACH_FRACTION * 667.0);
    assert!(pfd[2] >= 150, "large enough to read: {pfd:?}");
    assert!(pfd[1] + pfd[3] < hsi[1] && hsi[1] + hsi[3] <= 375);

    let insets = SafeAreaInsets::from_lengths(PHONE_TALL_INSETS, PHONE_TALL);
    let (pfd, hsi, _) = rail([402, 874], insets);
    assert!(pfd[1] >= 59, "under the sensor housing: {pfd:?}");
    assert!(hsi[1] + hsi[3] <= 874 - 34, "above the home bar: {hsi:?}");
    assert!(pfd[1] + pfd[3] < hsi[1]);
    assert!(pfd[0] + pfd[2] <= 402);
    assert!((pfd[2] as f32 / pfd[3] as f32 - PANEL_ASPECT).abs() < 0.02);
}

#[test]
fn the_bottom_row_clears_both_side_insets_and_the_home_indicator() {
    let size = [1748_u32, 804_u32];
    let housing = SafeAreaInsets::from_lengths(PHONE_WIDE_INSETS, PHONE_WIDE);
    let rects = dock_rects(size, 2, housing);
    let first = viewport_to_px_rect(rects[0], size);
    let last = viewport_to_px_rect(rects[1], size);
    assert!(first[0] >= 118);
    assert!(last[0] + last[2] <= 1748 - 118);
    assert!(last[1] + last[3] <= 804 - 42);
}

/// The row a desktop window has always drawn: `0.34` of the height tall, `4:3`, `0.015`
/// of the width from each side and `0.008` of the height from the bottom, the chart centred.
#[test]
fn a_desktop_window_keeps_its_bottom_row() {
    let size = [1920_u32, 1080_u32];
    let row = dock_layout(
        size,
        InstrumentDockStyle::default(),
        true,
        SafeAreaInsets::NONE,
    );
    assert_eq!(row.panel_px, [490, 367]);
    assert_eq!(viewport_to_px_rect(row.pfd, size), [29, 704, 490, 367]);
    let chart = row.mfd.map(|mfd| viewport_to_px_rect(mfd, size));
    assert_eq!(chart, Some([715, 704, 490, 367]));
    assert_eq!(viewport_to_px_rect(row.hsi, size), [1402, 704, 490, 367]);
    assert!(row.right_edge_norm.abs() < f32::EPSILON);

    let bare = dock_layout(
        size,
        InstrumentDockStyle::BottomRow,
        false,
        SafeAreaInsets::NONE,
    );
    assert!(bare.mfd.is_none());
    assert_eq!((bare.pfd, bare.hsi), (row.pfd, row.hsi));
    let rects = dock_rects(size, 2, SafeAreaInsets::NONE);
    assert!((rects[0].x - GAP_X).abs() < 1e-6);
    assert!((rects[0].width - PANEL_HEIGHT_FRACTION * PANEL_ASPECT * 1080.0 / 1920.0).abs() < 1e-6);
    assert!((rects[0].y - (1.0 - PANEL_HEIGHT_FRACTION - GAP_BOTTOM)).abs() < 1e-6);
    assert!((rects[1].x + rects[1].width - (1.0 - GAP_X)).abs() < 1e-6);
}
