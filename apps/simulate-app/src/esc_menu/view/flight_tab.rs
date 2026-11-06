//! `1 FLIGHT & TIMELINE`: session actions, the rewind timeline, rate, sky, avionics,
//! scenery, and live pacing.

use indicate_instrument_scene::HAlign;

use super::{HubView, MenuStatus, Palette, TimelineScrub, WidgetIcon, WidgetId, WidgetStyle};
use crate::esc_menu::Step;
use crate::esc_menu::catalog::{AvionicsLayout, RateChoice, SceneryMode};

/// Steps back offered on the timeline, seconds.
const BACK_STEPS_S: [u32; 3] = [30, 15, 5];
/// Steps forward offered on the timeline, seconds.
const FORWARD_STEPS_S: [u32; 2] = [5, 15];
/// Rewind offset below which the flight counts as live, seconds.
const LIVE_EPS_S: f64 = 0.05;
const BAR_H: f32 = 12.0;
const CHIP_H: f32 = 34.0;
const LEFT_W: f32 = 360.0;
const BUTTON_H: f32 = 44.0;
const GAP: f32 = 10.0;

pub(super) fn build(view: &mut HubView, status: &MenuStatus, [x, y, w, _h]: [f32; 4]) {
    session(view, status, [x, y]);
    timeline(view, status, [x, y + 4.0 * (BUTTON_H + GAP) + 16.0]);
    let rx = x + LEFT_W + 30.0;
    settings(view, status, [rx, y, w - LEFT_W - 30.0]);
}

fn session(view: &mut HubView, status: &MenuStatus, [x, y]: [f32; 2]) {
    let rect = |row: f32| [x, y + row * (BUTTON_H + GAP), LEFT_W, BUTTON_H];
    view.widget(
        WidgetId::Resume,
        rect(0.0),
        "Resume Flight   ESC",
        WidgetStyle::Primary,
        false,
    );
    let (label, style) = if status.scenario_active {
        ("Take Manual Control   M", WidgetStyle::Attention)
    } else {
        ("Manual Control Active", WidgetStyle::Button)
    };
    view.widget(
        WidgetId::TakeControl,
        rect(1.0),
        label,
        style,
        !status.scenario_active,
    );
    view.widget(
        WidgetId::Restart,
        rect(2.0),
        "Restart Flight",
        WidgetStyle::Button,
        false,
    );
    view.widget(
        WidgetId::Quit,
        rect(3.0),
        "Quit Simulate   CMD-Q",
        WidgetStyle::Button,
        false,
    );
}

fn timeline(view: &mut HubView, status: &MenuStatus, [x, y]: [f32; 2]) {
    view.text(
        [x, y],
        11.0,
        HAlign::Left,
        Palette::LABEL,
        "REWIND TIMELINE",
    );
    let history_s = status.history_s.max(0.0);
    let offset_s = status.rewind_offset_s.clamp(0.0, history_s);
    let live = offset_s <= LIVE_EPS_S;
    let (line, ink) = if live {
        (
            format!("LIVE   {history_s:.0} S BUFFERED"),
            Palette::READOUT,
        )
    } else {
        let line = format!("REWOUND -{offset_s:.1} S   {history_s:.0} S BUFFERED");
        (line, Palette::WARN)
    };
    view.text([x, y + 18.0], 10.0, HAlign::Left, ink, &line);
    let playhead = if history_s > 0.0 {
        1.0 - offset_s / history_s
    } else {
        1.0
    };
    let bar_y = y + 34.0;
    view.bar([x, bar_y, LEFT_W, BAR_H], playhead, false, Palette::READOUT);
    let thumb_x = x + LEFT_W * playhead as f32;
    let thumb = [thumb_x - 2.0, bar_y - 5.0, 4.0, BAR_H + 10.0];
    view.panel(thumb, Palette::TITLE, None);
    let track = [x, bar_y - 8.0, LEFT_W, BAR_H + 16.0];
    view.widget(WidgetId::TimelineBar, track, "", WidgetStyle::Track, false);
    let at_oldest = history_s - offset_s < 0.25;
    let chip_w = (LEFT_W - 2.0 * GAP) / 3.0;
    let chip = |row: f32, col: f32| {
        let top = bar_y + BAR_H + 14.0 + row * (CHIP_H + 8.0);
        [x + col * (chip_w + GAP), top, chip_w, CHIP_H]
    };
    for (i, seconds) in BACK_STEPS_S.into_iter().enumerate() {
        let style = if at_oldest {
            WidgetStyle::Muted
        } else {
            WidgetStyle::Chip
        };
        let label = format!("{seconds} S");
        let rect = chip(0.0, i as f32);
        let id = WidgetId::Rewind(seconds);
        view.icon_label_widget(id, rect, WidgetIcon::Minus, &label, style, false);
    }
    for (i, seconds) in FORWARD_STEPS_S.into_iter().enumerate() {
        let style = if live {
            WidgetStyle::Muted
        } else {
            WidgetStyle::Chip
        };
        let id = WidgetId::Scrub(TimelineScrub::Forward(seconds));
        let label = format!("{seconds} S");
        let rect = chip(1.0, i as f32);
        view.icon_label_widget(id, rect, WidgetIcon::Plus, &label, style, false);
    }
    let id = WidgetId::Scrub(TimelineScrub::Live);
    view.widget(id, chip(1.0, 2.0), "Live", WidgetStyle::Chip, live);
}

impl HubView {
    /// Thousandths along the rewind timeline bar (`0` oldest, `1000` live) under
    /// `cursor_px` (physical window pixels), when the bar is laid out.
    #[must_use]
    pub fn timeline_permille(&self, cursor_px: [f64; 2]) -> Option<u16> {
        let [bx, _, bw, _] = self.widget_rect(WidgetId::TimelineBar)?;
        let [cx, _] = self.to_design(cursor_px);
        let fraction = ((cx - bx) / bw.max(1.0)).clamp(0.0, 1.0);
        Some((fraction * 1000.0).round() as u16)
    }
}

fn settings(view: &mut HubView, status: &MenuStatus, [x, y, w]: [f32; 3]) {
    let row_h = 64.0;
    let chips = |view: &mut HubView, row: f32, title: &str, items: Vec<(WidgetId, &str, bool)>| {
        let top = y + row * row_h;
        view.text([x, top + 8.0], 11.0, HAlign::Left, Palette::LABEL, title);
        let chip_w = ((w - GAP * (items.len() as f32 - 1.0)) / items.len() as f32).min(204.0);
        for (i, (id, label, active)) in items.into_iter().enumerate() {
            let rect = [x + i as f32 * (chip_w + GAP), top + 20.0, chip_w, 34.0];
            view.widget(id, rect, label, WidgetStyle::Chip, active);
        }
    };
    let rates = RateChoice::ALL
        .into_iter()
        .map(|r| (WidgetId::Rate(r), r.label(), r == status.rate))
        .collect();
    chips(view, 0.0, "SIMULATION RATE", rates);
    sky(view, status, [x, y + row_h, w]);
    let layouts = AvionicsLayout::ALL
        .into_iter()
        .map(|l| (WidgetId::Layout(l), l.label(), l == status.layout))
        .collect();
    chips(view, 2.0, "AVIONICS LAYOUT", layouts);
    let scenery = [SceneryMode::Online, SceneryMode::Offline]
        .into_iter()
        .map(|m| (WidgetId::Scenery(m), m.label(), m == status.scenery))
        .collect();
    chips(view, 3.0, "SCENERY", scenery);
    readouts(view, status, [x, y + 4.0 * row_h + 8.0, w]);
}

fn sky(view: &mut HubView, status: &MenuStatus, [x, y, w]: [f32; 3]) {
    view.text(
        [x, y + 8.0],
        11.0,
        HAlign::Left,
        Palette::LABEL,
        "SKY EPHEMERIS",
    );
    view.icon_widget(
        WidgetId::Sky(Step::Prev),
        [x, y + 20.0, 44.0, 34.0],
        WidgetIcon::ChevronLeft,
        false,
    );
    view.panel([x + 52.0, y + 20.0, 260.0, 34.0], Palette::PANEL, None);
    view.text(
        [x + 182.0, y + 37.0],
        12.0,
        HAlign::Center,
        Palette::VALUE,
        &status.sky,
    );
    let next = [x + 320.0, y + 20.0, 44.0, 34.0];
    view.icon_widget(
        WidgetId::Sky(Step::Next),
        next,
        WidgetIcon::ChevronRight,
        false,
    );
    let _ = w;
}

fn readouts(view: &mut HubView, status: &MenuStatus, [x, y, w]: [f32; 3]) {
    view.panel([x, y, w, 120.0], Palette::PANEL, Some(Palette::EDGE));
    let p = &status.pacing;
    let lines = [
        format!(
            "FRAME RATE {:.0} FPS   MEAN {:.1} MS   MAX {:.1} MS",
            p.fps, p.mean_frame_ms, p.max_frame_ms
        ),
        format!(
            "REAL-TIME PACING {:.2}X   EPOCH {}",
            p.real_time_ratio, status.epoch
        ),
        match status.sensor_last_s {
            Some(at) => format!(
                "OPTICAL SENSOR 30 HZ   {} FRAMES   LAST {at:.2} S",
                status.sensor_frames
            ),
            None => format!("OPTICAL SENSOR 30 HZ   {} FRAMES", status.sensor_frames),
        },
        format!("FLYING {}   {}", status.aircraft, status.scenario),
    ];
    for (i, line) in lines.iter().enumerate() {
        let at = [x + 14.0, y + 22.0 + i as f32 * 26.0];
        view.text_fit(at, 11.0, Palette::READOUT, w - 28.0, line);
    }
}
