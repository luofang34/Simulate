//! EFB placement and chrome: the corner map stays in the top-right corner clear of the
//! HUD and the dock, keeps `4:3` pixels, and every drawn button hit-tests to its command.

use sim_maplibre::EfbDisplayMode;
use sim_render::ViewportRect;

use super::{EfbChrome, EfbHit, ZOOM_STEP, efb_map_rect, rect_px};
use crate::cockpit_rig::layout::dock_rects;
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
        let rect = efb_map_rect(size, EfbDisplayMode::CompactCorner);
        assert!(rect.x > 0.70, "{size:?} {rect:?}");
        assert!(
            rect.y > 0.0 && rect.y + rect.height < 0.35,
            "{size:?} {rect:?}"
        );
        assert!(rect.x + rect.width < 1.0, "{size:?}");
        assert!(!overlaps(rect, hud), "{size:?} covers the HUD");
        for dock in dock_rects(size, 2) {
            assert!(!overlaps(rect, dock), "{size:?} covers {dock:?}");
        }
    }
}

#[test]
fn both_views_keep_four_by_three_pixels_inside_the_window() {
    for size in WINDOWS {
        for mode in [EfbDisplayMode::CompactCorner, EfbDisplayMode::ExpandedEfb] {
            let rect = efb_map_rect(size, mode);
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
        let chrome = EfbChrome::new(size, &status(EfbDisplayMode::ExpandedEfb));
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
        let chrome = EfbChrome::new(size, &status(EfbDisplayMode::CompactCorner));
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
        let folded = EfbChrome::new(size, &folded_status);
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
        let chrome = EfbChrome::new(size, &status(EfbDisplayMode::ExpandedEfb));
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
        let rect = efb_map_rect(size, EfbDisplayMode::ExpandedEfb);
        assert!((viewport.x - rect.x).abs() < 1e-5 && (viewport.width - rect.width).abs() < 1e-5);
    }
}
