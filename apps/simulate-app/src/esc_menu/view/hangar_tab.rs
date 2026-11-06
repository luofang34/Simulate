//! `2 AIRCRAFT HANGAR`: the scrollable vehicle registry and the live 3D preview card.

use indicate_instrument_scene::HAlign;

use super::{ADVANCE_PER_SIZE, HubView, MenuStatus, Palette, VEHICLE_ROWS, WidgetId, WidgetStyle};
use crate::controls::{CONTROLS, control_title, control_value};
use crate::esc_menu::EscMenu;
use crate::esc_menu::catalog::{HubCatalog, VehicleDescriptor, mode_label};
use crate::vehicles::FreeFlightVehicle;

const LIST_W: f32 = 380.0;
const ROW_H: f32 = 52.0;
const PREVIEW_H: f32 = 330.0;
const APPLY_W: f32 = 240.0;
const CHIP_H: f32 = 34.0;
/// Live control bar offset right of its title (clear of `THROTTLE`).
const BAR_DX: f32 = 76.0;

pub(super) fn build(
    view: &mut HubView,
    menu: &EscMenu,
    catalog: &HubCatalog,
    status: &MenuStatus,
    [x, y, w, h]: [f32; 4],
) {
    registry(view, menu, catalog, [x, y, h]);
    let px = x + LIST_W + 24.0;
    let pw = w - LIST_W - 24.0;
    let card = [px, y, pw, PREVIEW_H];
    view.panel(card, Palette::STUDIO, Some(Palette::EDGE));
    view.set_preview([px + 2.0, y + 2.0, pw - 4.0, PREVIEW_H - 4.0]);
    let controls_y = y + PREVIEW_H + 10.0;
    view.widget(
        WidgetId::AutoRotate,
        [px, controls_y, 150.0, 32.0],
        "Auto-Rotate",
        WidgetStyle::Chip,
        status.auto_rotate,
    );
    view.widget(
        WidgetId::ResetView,
        [px + 160.0, controls_y, 150.0, 32.0],
        "Reset View",
        WidgetStyle::Chip,
        false,
    );
    view.text(
        [px + pw, controls_y + 16.0],
        10.0,
        HAlign::Right,
        Palette::HINT,
        "DRAG TO ORBIT   SCROLL TO ZOOM",
    );
    let vehicle = catalog.vehicles.get(menu.vehicle());
    let mode =
        vehicle.and_then(|v| v.mode_or_default(menu.vehicle_mode().or(status.active_vehicle)));
    if let Some(vehicle) = vehicle {
        details(view, vehicle, mode, status, [px, controls_y + 48.0, pw]);
        if vehicle.modes().len() > 1 {
            mode_chips(view, vehicle, mode, [px, y + h - CHIP_H]);
        }
    }
    let apply = [px + pw - APPLY_W, y + h - 48.0, APPLY_W, 48.0];
    view.widget(
        WidgetId::ApplyVehicle,
        apply,
        "Apply / Fly Vehicle",
        WidgetStyle::Primary,
        false,
    );
}

/// The `FLIGHT DYNAMICS` chips of an airframe flown on several plants.
fn mode_chips(
    view: &mut HubView,
    vehicle: &VehicleDescriptor,
    active: Option<FreeFlightVehicle>,
    [x, y]: [f32; 2],
) {
    view.text(
        [x, y - 12.0],
        10.0,
        HAlign::Left,
        Palette::LABEL,
        "FLIGHT DYNAMICS",
    );
    let mut cx = x;
    for &mode in vehicle.modes() {
        let label = mode_label(mode);
        let width = label.len() as f32 * 11.0 * ADVANCE_PER_SIZE + 26.0;
        view.widget(
            WidgetId::VehicleMode(mode),
            [cx, y, width, CHIP_H],
            label,
            WidgetStyle::Chip,
            active == Some(mode),
        );
        cx += width + 8.0;
    }
}

fn registry(view: &mut HubView, menu: &EscMenu, catalog: &HubCatalog, [x, y, h]: [f32; 3]) {
    view.text(
        [x, y + 6.0],
        11.0,
        HAlign::Left,
        Palette::LABEL,
        "VEHICLE REGISTRY",
    );
    let count = catalog.vehicles.len();
    let shown = format!("{} OF {count}", count.min(VEHICLE_ROWS));
    view.text(
        [x + LIST_W, y + 6.0],
        10.0,
        HAlign::Right,
        Palette::HINT,
        &shown,
    );
    let top = y + 18.0;
    view.panel([x, top, LIST_W, h - 18.0], Palette::PANEL, None);
    let rows = catalog
        .vehicles
        .iter()
        .enumerate()
        .skip(menu.vehicle_scroll())
        .take(VEHICLE_ROWS);
    for (slot, (index, vehicle)) in rows.enumerate() {
        let ry = top + 4.0 + slot as f32 * ROW_H;
        let rect = [x + 4.0, ry, LIST_W - 8.0, ROW_H - 6.0];
        view.widget(
            WidgetId::VehicleRow(index),
            rect,
            &vehicle.title,
            WidgetStyle::Row,
            index == menu.vehicle(),
        );
        let sub = match vehicle.modes().len() {
            0 | 1 => format!("{}  {}", vehicle.role, vehicle.fdm(None).badge()),
            n => format!("{}  {n} FLIGHT MODELS", vehicle.role),
        };
        view.text_fit(
            [x + 14.0, ry + ROW_H - 16.0],
            9.5,
            Palette::HINT,
            LIST_W - 28.0,
            &sub,
        );
    }
    if count > VEHICLE_ROWS {
        let track = [x + LIST_W - 4.0, top, 3.0, h - 18.0];
        let fraction = menu.vehicle_scroll() as f64 / (count - VEHICLE_ROWS) as f64;
        view.bar(track, fraction, false, Palette::HINT);
    }
}

fn details(
    view: &mut HubView,
    vehicle: &VehicleDescriptor,
    mode: Option<FreeFlightVehicle>,
    status: &MenuStatus,
    [x, y, w]: [f32; 3],
) {
    let cruise = if vehicle.cruise_kt > 0.0 {
        format!("{:.0} KT", vehicle.cruise_kt)
    } else {
        "LUNAR DESCENT".to_owned()
    };
    let lines = [
        format!(
            "{}   {}   {}",
            vehicle.role,
            vehicle.fdm(mode).badge(),
            cruise
        ),
        format!(
            "MASS {:.*} KG   DEFAULT PRESET {}",
            usize::from(vehicle.mass_kg < 100.0),
            vehicle.mass_kg,
            vehicle.default_preset().title()
        ),
        "MOVE STICK OR KEYS TO TEST CONTROL SURFACES".to_owned(),
    ];
    for (i, line) in lines.iter().enumerate() {
        let ink = if i == 2 {
            Palette::HINT
        } else {
            Palette::LABEL
        };
        view.text_fit([x, y + i as f32 * 18.0], 11.0, ink, w, line);
    }
    let live = status.controls.live;
    let bar_w = (w - 340.0).max(120.0) / 2.0;
    for (i, control) in CONTROLS.into_iter().enumerate() {
        let col = (i % 2) as f32;
        let row = (i / 2) as f32;
        let bx = x + col * (bar_w + BAR_DX + 20.0);
        let by = y + 58.0 + row * 20.0;
        view.text(
            [bx, by],
            10.0,
            HAlign::Left,
            Palette::HINT,
            control_title(control),
        );
        let centered = control != sim_input::Control::Throttle;
        view.bar(
            [bx + BAR_DX, by - 5.0, bar_w, 10.0],
            control_value(&live, control),
            centered,
            Palette::VALUE,
        );
    }
}
