//! `5 CONTROLS & PRESETS`: preset selector, live four-axis telemetry, per-axis rebind /
//! invert / deadzone / expo, and keyboard rebinding.

use indicate_instrument_scene::HAlign;

use super::{HubView, MenuStatus, Palette, WidgetIcon, WidgetId, WidgetStyle};
use crate::controls::{CaptureTarget, InputSource, control_title, control_value};
use crate::esc_menu::Step;

/// Pitch of the axis rows.
const AXIS_ROW_H: f32 = 44.0;
/// Pitch of the keyboard rows.
const KEY_ROW_H: f32 = 36.0;
/// Top of the axis column headers below the content top.
const AXES_TOP: f32 = 82.0;
/// Top of the keyboard header below the content top.
const KEYS_TOP: f32 = 284.0;
const PRESET_AUTO_W: f32 = 176.0;
const DEVICE_X: f32 = 620.0;
/// Axis row column offsets: live bar, binding readout.
const BAR_X: f32 = 90.0;
const BINDING_X: f32 = 280.0;
/// Width of the tuning block at the right of each axis row.
const TUNING_W: f32 = 560.0;

pub(super) fn build(view: &mut HubView, status: &MenuStatus, [x, y, w, _h]: [f32; 4]) {
    preset_row(view, status, [x, y, w]);
    let axes_top = y + AXES_TOP;
    for (dx, title) in [
        (0.0, "AXIS"),
        (BAR_X, "LIVE"),
        (BINDING_X, "BINDING"),
        (w - TUNING_W, "TUNING"),
    ] {
        view.text(
            [x + dx, axes_top],
            11.0,
            HAlign::Left,
            Palette::LABEL,
            title,
        );
    }
    for (i, row) in status.controls.rows.iter().enumerate() {
        axis_row(
            view,
            status,
            row,
            [x, axes_top + 13.0 + i as f32 * AXIS_ROW_H, w],
        );
    }
    let keys_top = y + KEYS_TOP;
    view.text(
        [x, keys_top],
        11.0,
        HAlign::Left,
        Palette::LABEL,
        "KEYBOARD   CLICK A KEY THEN PRESS THE NEW KEY",
    );
    let col_w = (w - 20.0) / 2.0;
    for (i, (action, key)) in status.controls.keys.iter().enumerate() {
        let col = (i % 2) as f32;
        let row = (i / 2) as f32;
        let kx = x + col * (col_w + 20.0);
        let ky = keys_top + 14.0 + row * KEY_ROW_H;
        let chip_h = KEY_ROW_H - 6.0;
        view.text(
            [kx, ky + chip_h * 0.5],
            11.0,
            HAlign::Left,
            Palette::LABEL,
            action.title(),
        );
        let capturing = status.controls.capture == Some(CaptureTarget::Key(*action));
        let label = if capturing { "PRESS KEY" } else { key.as_str() };
        let rect = [kx + col_w - 150.0, ky, 150.0, chip_h];
        view.widget(
            WidgetId::KeyRebind(*action),
            rect,
            label,
            WidgetStyle::Chip,
            capturing,
        );
    }
}

fn preset_row(view: &mut HubView, status: &MenuStatus, [x, y, w]: [f32; 3]) {
    let c = &status.controls;
    view.text(
        [x, y + 6.0],
        11.0,
        HAlign::Left,
        Palette::LABEL,
        "CONTROL PRESET",
    );
    view.icon_widget(
        WidgetId::Preset(Step::Prev),
        [x, y + 18.0, 40.0, 32.0],
        WidgetIcon::ChevronLeft,
        false,
    );
    view.panel([x + 46.0, y + 18.0, 330.0, 32.0], Palette::PANEL, None);
    view.text(
        [x + 211.0, y + 34.0],
        12.0,
        HAlign::Center,
        Palette::VALUE,
        c.preset.title(),
    );
    view.icon_widget(
        WidgetId::Preset(Step::Next),
        [x + 382.0, y + 18.0, 40.0, 32.0],
        WidgetIcon::ChevronRight,
        false,
    );
    view.widget(
        WidgetId::PresetAuto,
        [x + 430.0, y + 18.0, PRESET_AUTO_W, 32.0],
        "Vehicle Default",
        WidgetStyle::Chip,
        c.automatic,
    );
    let device = c.controller.as_deref().unwrap_or("NO CONTROLLER");
    let source = match c.source {
        InputSource::Keyboard => "ACTIVE INPUT KEYBOARD",
        InputSource::Gamepad => "ACTIVE INPUT GAMEPAD",
    };
    view.text_fit(
        [x + DEVICE_X, y + 26.0],
        10.5,
        Palette::READOUT,
        w - DEVICE_X,
        device,
    );
    view.text_fit(
        [x + DEVICE_X, y + 44.0],
        10.0,
        Palette::HINT,
        w - DEVICE_X,
        source,
    );
    view.text_fit(
        [x, y + 60.0],
        9.5,
        Palette::HINT,
        DEVICE_X,
        c.preset.summary(),
    );
}

fn axis_row(
    view: &mut HubView,
    status: &MenuStatus,
    row: &crate::controls::AxisRow,
    [x, y, w]: [f32; 3],
) {
    let control = row.control;
    view.text(
        [x, y + 18.0],
        12.0,
        HAlign::Left,
        Palette::LABEL,
        control_title(control),
    );
    let value = control_value(&status.controls.live, control);
    let centered = control != sim_input::Control::Throttle;
    view.bar(
        [x + BAR_X, y + 12.0, 180.0, 12.0],
        value,
        centered,
        Palette::VALUE,
    );
    let bx = x + w - TUNING_W;
    view.text_fit(
        [x + BINDING_X, y + 18.0],
        10.5,
        Palette::READOUT,
        bx - x - BINDING_X - 10.0,
        &row.sources,
    );
    let capturing = status.controls.capture == Some(CaptureTarget::Axis(control));
    let rebind = if capturing { "Move Axis" } else { "Rebind" };
    view.widget(
        WidgetId::AxisRebind(control),
        [bx, y + 2.0, 110.0, 32.0],
        rebind,
        WidgetStyle::Chip,
        capturing,
    );
    view.widget(
        WidgetId::AxisInvert(control),
        [bx + 116.0, y + 2.0, 90.0, 32.0],
        "Invert",
        WidgetStyle::Chip,
        row.invert,
    );
    stepper(
        view,
        [bx + 214.0, y + 2.0],
        &format!("DZ {:.2}", row.deadzone),
        |s| WidgetId::Deadzone(control, s),
    );
    stepper(
        view,
        [bx + 390.0, y + 2.0],
        &format!("EXPO {:.1}", row.expo),
        |s| WidgetId::Expo(control, s),
    );
}

fn stepper(view: &mut HubView, [x, y]: [f32; 2], value: &str, id: impl Fn(Step) -> WidgetId) {
    view.icon_widget(id(Step::Prev), [x, y, 32.0, 32.0], WidgetIcon::Minus, false);
    view.text(
        [x + 83.0, y + 16.0],
        10.5,
        HAlign::Center,
        Palette::VALUE,
        value,
    );
    view.icon_widget(
        id(Step::Next),
        [x + 134.0, y, 32.0, 32.0],
        WidgetIcon::Plus,
        false,
    );
}
