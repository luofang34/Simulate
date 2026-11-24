//! EFB placement and chrome: the corner map stays in the top-right corner clear of the
//! HUD and the dock, keeps `4:3` pixels, and every drawn button hit-tests to its command.

use sim_maplibre::EfbDisplayMode;
use sim_render::ViewportRect;

use super::{EfbChrome, EfbHit, EfbPlacement, ZOOM_STEP, efb_map_rect, rect_px};
use crate::cockpit_rig::SafeAreaInsets;
use crate::cockpit_rig::layout::{InstrumentDockStyle, dock_layout, dock_rects};
use crate::map_rig::{EfbCommand, EfbStatus};

const WINDOWS: [[u32; 2]; 5] = [
    [1280, 720],
    [2560, 1440],
    [1920, 1200],
    [800, 600],
    [1080, 1920],
];

fn status(mode: EfbDisplayMode) -> EfbStatus {
    EfbStatus {
        mode,
        collapsed: false,
        zoom: 10.2,
        north_up: false,
        custom_bearing: false,
        free_pan: false,
        plate_shown: true,
        plate_opacity_pct: 75,
        diagram_shown: true,
        airport: "KEWR".to_owned(),
        plate_title: Some("ILS OR LOC RWY 22L".to_owned()),
        satellite: false,
    }
}

fn overlaps(a: ViewportRect, b: ViewportRect) -> bool {
    a.x < b.x + b.width && b.x < a.x + a.width && a.y < b.y + b.height && b.y < a.y + a.height
}

/// The physical pixel at the centre of design rectangle `rect`.
fn centre_px(chrome: &EfbChrome, [x, y, w, h]: [f32; 4], [sw, sh]: [u32; 2]) -> [f64; 2] {
    let [dw, dh] = chrome.design;
    [
        f64::from((x + w * 0.5) / dw * sw as f32),
        f64::from((y + h * 0.5) / dh * sh as f32),
    ]
}

#[test]
fn the_corner_map_sits_top_right_clear_of_the_hud_and_the_dock() {
    let hud = ViewportRect {
        x: 0.25,
        y: 0.15,
        width: 0.5,
        height: 0.5,
    };
    for size in WINDOWS {
        let rect = efb_map_rect(size, EfbDisplayMode::CompactCorner, EfbPlacement::default());
        assert!(rect.x > 0.70, "{size:?} {rect:?}");
        assert!(
            rect.y > 0.0 && rect.y + rect.height < 0.35,
            "{size:?} {rect:?}"
        );
        assert!(rect.x + rect.width < 1.0, "{size:?}");
        assert!(!overlaps(rect, hud), "{size:?} covers the HUD");
        for dock in dock_rects(size, 2, SafeAreaInsets::NONE) {
            assert!(!overlaps(rect, dock), "{size:?} covers {dock:?}");
        }
    }
}

#[test]
fn both_views_keep_four_by_three_pixels_inside_the_window() {
    for size in WINDOWS {
        for mode in [EfbDisplayMode::CompactCorner, EfbDisplayMode::ExpandedEfb] {
            let rect = efb_map_rect(size, mode, EfbPlacement::default());
            let [w, h] = rect_px(rect, size);
            let aspect = f64::from(w) / f64::from(h);
            assert!(
                (aspect - 4.0 / 3.0).abs() < 0.02,
                "{size:?} {mode:?} {aspect}"
            );
            assert!(
                rect.x >= 0.0 && rect.x + rect.width <= 1.0,
                "{size:?} {mode:?}"
            );
            assert!(
                rect.y >= 0.0 && rect.y + rect.height <= 1.0,
                "{size:?} {mode:?}"
            );
        }
        let chrome = EfbChrome::new(
            size,
            &status(EfbDisplayMode::ExpandedEfb),
            EfbPlacement::default(),
        );
        let [_, by, _, bh] = chrome.bezel;
        assert!(
            by >= 0.0 && by + bh <= chrome.design[1],
            "{size:?} bezel {by} {bh}"
        );
    }
}

#[test]
fn compact_buttons_and_the_map_hit_test_to_what_is_drawn() {
    for size in WINDOWS {
        let chrome = EfbChrome::new(
            size,
            &status(EfbDisplayMode::CompactCorner),
            EfbPlacement::default(),
        );
        let commands: Vec<EfbCommand> = chrome.buttons.iter().map(|b| b.command).collect();
        assert_eq!(
            commands,
            [
                EfbCommand::ZoomBy(-ZOOM_STEP),
                EfbCommand::ZoomBy(ZOOM_STEP),
                EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb),
                EfbCommand::SetCollapsed(true),
            ]
        );
        for button in &chrome.buttons {
            let at = centre_px(&chrome, button.rect, size);
            assert_eq!(chrome.hit(at, size), Some(EfbHit::Button(button.command)));
            let [mx, my, mw, mh] = chrome.map;
            let [x, y, w, h] = button.rect;
            assert!(x >= mx && y >= my && x + w <= mx + mw && y + h <= my + mh);
        }
        let at = centre_px(&chrome, chrome.map, size);
        assert_eq!(chrome.hit(at, size), Some(EfbHit::Map));
        assert_eq!(
            chrome.hit([1.0, 1.0], size),
            None,
            "top-left is the windshield"
        );
        assert!(
            chrome.title.0.is_empty(),
            "compact corner omits header badge"
        );

        let mut folded_status = status(EfbDisplayMode::CompactCorner);
        folded_status.collapsed = true;
        let folded = EfbChrome::new(size, &folded_status, EfbPlacement::default());
        assert_eq!(folded.map, [0.0; 4]);
        assert_eq!(folded.buttons.len(), 1);
        let Some(unfold_btn) = folded.buttons.first() else {
            return;
        };
        assert_eq!(unfold_btn.command, EfbCommand::SetCollapsed(false));
        assert_eq!(unfold_btn.label, "MAP");
        let unfold_at = centre_px(&folded, unfold_btn.rect, size);
        assert_eq!(
            folded.hit(unfold_at, size),
            Some(EfbHit::Button(EfbCommand::SetCollapsed(false)))
        );
        assert_eq!(
            folded.hit(at, size),
            None,
            "collapsed mini-map passes clicks through to cockpit"
        );
    }
}

#[test]
fn expanded_toolbar_chips_read_in_full_and_hit_test_to_their_commands() {
    for size in WINDOWS {
        let chrome = EfbChrome::new(
            size,
            &status(EfbDisplayMode::ExpandedEfb),
            EfbPlacement::default(),
        );
        assert_eq!(chrome.buttons.len(), 9);
        let labels: Vec<&str> = chrome.buttons.iter().map(|b| b.label.as_str()).collect();
        assert_eq!(
            labels,
            [
                "-", "+", "TRK UP", "PLATE ON", "OPAC 75%", "DIAG ON", "SAT OFF", "CENTER",
                "MINIMIZE"
            ]
        );
        for (i, a) in chrome.buttons.iter().enumerate() {
            for b in chrome.buttons.iter().skip(i + 1) {
                let [ax, ay, aw, ah] = a.rect;
                let [bx, by, bw, bh] = b.rect;
                let apart = ax + aw <= bx || bx + bw <= ax || ay + ah <= by || by + bh <= ay;
                assert!(apart, "{size:?} {} overlaps {}", a.label, b.label);
            }
            let [mx, my, mw, mh] = chrome.map;
            let [x, y, w, h] = a.rect;
            assert!(
                y + h <= my || y >= my + mh,
                "{size:?} {} covers the map",
                a.label
            );
            assert!(x >= mx && x + w <= mx + mw + 0.01, "{size:?} {}", a.label);
        }
        for button in &chrome.buttons {
            let text_w = button.label.chars().count() as f32 * 9.5 * 6.0 / 7.0;
            assert!(
                text_w + 4.0 <= button.rect[2],
                "{size:?} {} clips",
                button.label
            );
            let at = centre_px(&chrome, button.rect, size);
            assert_eq!(chrome.hit(at, size), Some(EfbHit::Button(button.command)));
        }
        let at = centre_px(&chrome, chrome.map, size);
        assert_eq!(chrome.hit(at, size), Some(EfbHit::Map));
        let [bx, by, _, _] = chrome.bezel;
        let corner = centre_px(&chrome, [bx, by, 2.0, 2.0], size);
        assert_eq!(chrome.hit(corner, size), Some(EfbHit::Bezel));
        let viewport = chrome.map_viewport();
        let rect = efb_map_rect(size, EfbDisplayMode::ExpandedEfb, EfbPlacement::default());
        assert!((viewport.x - rect.x).abs() < 1e-5 && (viewport.width - rect.width).abs() < 1e-5);
    }
}

/// A handheld's cockpit on a `[width, height]` point window whose system covers `covered`
/// (`[left, top, right, bottom]`).
fn handheld(covered: [f64; 4], window: [f64; 2]) -> EfbPlacement {
    EfbPlacement {
        insets: SafeAreaInsets::from_lengths(covered, window),
        handheld: true,
    }
}

#[test]
fn a_desktop_corner_map_sits_where_it_always_has() {
    let rect = efb_map_rect(
        [1920, 1080],
        EfbDisplayMode::CompactCorner,
        EfbPlacement::default(),
    );
    let width = 0.235 * (4.0 / 3.0) * 1080.0 / 1920.0;
    assert!((rect.height - 0.235).abs() < 1e-6, "{rect:?}");
    assert!((rect.width - width).abs() < 1e-6, "{rect:?}");
    assert!((rect.x - (1.0 - 0.015 - width)).abs() < 1e-6, "{rect:?}");
    assert!((rect.y - 0.018).abs() < 1e-6, "{rect:?}");
    let chrome = EfbChrome::new(
        [1920, 1080],
        &status(EfbDisplayMode::CompactCorner),
        EfbPlacement::default(),
    );
    assert_eq!(chrome.buttons.len(), 4, "a pointer keeps its buttons");
}

#[test]
fn a_handheld_corner_map_is_larger_and_inside_the_covered_edges() {
    let wide = handheld([59.0, 0.0, 59.0, 21.0], [874.0, 402.0]);
    let rect = efb_map_rect([874, 402], EfbDisplayMode::CompactCorner, wide);
    let [x, y, w, h] = [
        rect.x * 874.0,
        rect.y * 402.0,
        rect.width * 874.0,
        rect.height * 402.0,
    ];
    assert!((h - 0.34 * 402.0).abs() < 0.01, "{h}");
    assert!((w / h - 4.0 / 3.0).abs() < 1e-3, "{w} x {h}");
    assert!(
        (x + w - (874.0 - 59.0)).abs() < 0.01,
        "ends at the housing: {x} {w}"
    );
    assert!((y - 0.018 * 402.0).abs() < 0.01, "the standard margin: {y}");
    assert!(
        (x - 632.8).abs() < 0.1 && (y + h - 143.9).abs() < 0.1,
        "{x} {y} {w} {h}"
    );

    let tall = handheld([0.0, 59.0, 0.0, 34.0], [402.0, 874.0]);
    let rect = efb_map_rect([402, 874], EfbDisplayMode::CompactCorner, tall);
    assert!(
        (rect.y * 874.0 - 59.0).abs() < 0.01,
        "under the housing: {rect:?}"
    );
    assert!(
        (rect.width - 0.28).abs() < 1e-6,
        "capped by the width: {rect:?}"
    );
    assert!(rect.x + rect.width <= 1.0 - 0.015 + 1e-6, "{rect:?}");
    let [w, h] = rect_px(rect, [1206, 2622]);
    assert!((f64::from(w) / f64::from(h) - 4.0 / 3.0).abs() < 0.02);
}

#[test]
fn covered_edges_move_the_corner_map_and_never_resize_it() {
    for size in [[874_u32, 402_u32], [402, 874], [667, 375]] {
        let bare = EfbPlacement {
            handheld: true,
            ..EfbPlacement::default()
        };
        let covered = handheld([59.0, 59.0, 59.0, 34.0], [402.0, 402.0]);
        let a = efb_map_rect(size, EfbDisplayMode::CompactCorner, bare);
        let b = efb_map_rect(size, EfbDisplayMode::CompactCorner, covered);
        assert_eq!([a.width, a.height], [b.width, b.height], "{size:?}");
        assert!(b.x < a.x && b.y > a.y, "{size:?}");
        let tablet = efb_map_rect(size, EfbDisplayMode::ExpandedEfb, covered);
        assert_eq!(
            tablet,
            efb_map_rect(size, EfbDisplayMode::ExpandedEfb, EfbPlacement::default())
        );
    }
}

#[test]
fn a_handheld_corner_map_has_no_vector_buttons_and_a_tap_lands_on_the_map() {
    let placement = handheld([59.0, 0.0, 59.0, 21.0], [874.0, 402.0]);
    let size = [1748_u32, 804_u32];
    let chrome = EfbChrome::new(size, &status(EfbDisplayMode::CompactCorner), placement);
    assert!(chrome.buttons.is_empty(), "the page's tab folds the map");
    let rect = efb_map_rect(size, EfbDisplayMode::CompactCorner, placement);
    let viewport = chrome.map_viewport();
    assert!((viewport.x - rect.x).abs() < 1e-5 && (viewport.y - rect.y).abs() < 1e-5);
    assert!((viewport.width - rect.width).abs() < 1e-5);
    for corner in [[0.02, 0.02], [0.5, 0.5], [0.98, 0.98]] {
        let at = [
            f64::from((rect.x + rect.width * corner[0]) * 1748.0),
            f64::from((rect.y + rect.height * corner[1]) * 804.0),
        ];
        assert_eq!(chrome.hit(at, size), Some(EfbHit::Map), "{corner:?}");
    }
    let tablet = EfbChrome::new(size, &status(EfbDisplayMode::ExpandedEfb), placement);
    assert_eq!(tablet.buttons.len(), 9, "the tablet keeps its toolbar");
}

#[test]
fn the_handheld_corner_map_and_the_instrument_column_never_overlap() {
    for (window, covered) in [
        ([874.0, 402.0], [59.0, 0.0, 59.0, 21.0]),
        ([667.0, 375.0], [0.0; 4]),
        ([402.0, 874.0], [0.0, 59.0, 0.0, 34.0]),
    ] {
        let placement = handheld(covered, window);
        for scale in [1.0, 2.0, 3.0] {
            let size = window.map(|v: f64| (v * scale) as u32);
            let map = efb_map_rect(size, EfbDisplayMode::CompactCorner, placement);
            let dock = dock_layout(size, InstrumentDockStyle::LeftRail, true, placement.insets);
            for panel in [dock.pfd, dock.hsi] {
                assert!(!overlaps(map, panel), "{size:?} {map:?} {panel:?}");
            }
            assert!(map.x >= 0.0 && map.x + map.width <= 1.0, "{size:?}");
            assert!(map.y >= 0.0 && map.y + map.height <= 1.0, "{size:?}");
        }
    }
}

#[test]
fn a_one_pixel_window_places_a_map_without_panicking() {
    for placement in [
        EfbPlacement::default(),
        handheld([59.0, 59.0, 59.0, 34.0], [100.0, 100.0]),
    ] {
        for mode in [EfbDisplayMode::CompactCorner, EfbDisplayMode::ExpandedEfb] {
            for size in [[1_u32, 1_u32], [0, 0], [1, 900], [900, 1]] {
                let rect = efb_map_rect(size, mode, placement);
                assert!(rect.width.is_finite() && rect.height.is_finite());
                assert!(rect.x.is_finite() && rect.y.is_finite(), "{size:?}");
                let _chrome = EfbChrome::new(size, &status(mode), placement);
            }
        }
    }
}
