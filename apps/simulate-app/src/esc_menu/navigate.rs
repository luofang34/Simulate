//! Keyboard / gamepad focus movement: list rows step and scroll their list, everything
//! else moves to the nearest widget in the pressed direction.

use super::view::{AERODROME_ROWS, HubView, SCENARIO_ROWS, VEHICLE_ROWS, WidgetId};
use super::{EscMenu, HubCatalog, MenuOutcome};

/// A navigation direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Dir {
    Up,
    Down,
    Left,
    Right,
}

impl EscMenu {
    pub(super) fn navigate(
        &mut self,
        view: &HubView,
        catalog: &HubCatalog,
        dir: Dir,
    ) -> MenuOutcome {
        if let Some(next) = self.list_step(dir, catalog) {
            self.focus = Some(next);
            return MenuOutcome::Idle;
        }
        let current = self.focus.and_then(|id| view.widget_rect(id));
        self.focus = match current {
            Some(rect) => nearest(view, rect, dir).or(self.focus),
            None => view.first_focusable(),
        };
        MenuOutcome::Idle
    }

    /// Steps the focused list row within its list, scrolling to keep it shown.
    fn list_step(&mut self, dir: Dir, catalog: &HubCatalog) -> Option<WidgetId> {
        let down = match dir {
            Dir::Down => true,
            Dir::Up => false,
            Dir::Left | Dir::Right => return None,
        };
        match self.focus? {
            WidgetId::VehicleRow(i) => {
                let j = step_index(i, catalog.vehicles.len(), down)?;
                keep_visible(&mut self.vehicle_scroll, j, VEHICLE_ROWS);
                Some(WidgetId::VehicleRow(j))
            }
            WidgetId::ScenarioRow(i) => {
                let j = step_index(i, catalog.scenarios.len(), down)?;
                keep_visible(&mut self.scenario_scroll, j, SCENARIO_ROWS);
                Some(WidgetId::ScenarioRow(j))
            }
            WidgetId::AerodromeRow(i) => {
                let filtered = catalog.filter_aerodromes(&self.search);
                let at = filtered.iter().position(|&a| a == i)?;
                let to = step_index(at, filtered.len(), down)?;
                keep_visible(&mut self.aerodrome_scroll, to, AERODROME_ROWS);
                filtered.get(to).map(|&a| WidgetId::AerodromeRow(a))
            }
            _ => None,
        }
    }
}

fn step_index(index: usize, len: usize, down: bool) -> Option<usize> {
    if down {
        let next = index.checked_add(1)?;
        (next < len).then_some(next)
    } else {
        index.checked_sub(1)
    }
}

fn keep_visible(scroll: &mut usize, index: usize, visible: usize) {
    if index < *scroll {
        *scroll = index;
    } else if index >= scroll.saturating_add(visible) {
        *scroll = index.saturating_add(1).saturating_sub(visible);
    }
}

/// The focusable widget nearest `from` in `dir`, favouring alignment with it.
fn nearest(view: &HubView, from: [f32; 4], dir: Dir) -> Option<WidgetId> {
    let [fx, fy] = centre(from);
    view.widgets()
        .iter()
        .filter_map(|w| {
            let [x, y] = centre(w.rect);
            let (dx, dy) = (x - fx, y - fy);
            let (along, across) = match dir {
                Dir::Up => (-dy, dx),
                Dir::Down => (dy, dx),
                Dir::Left => (-dx, dy),
                Dir::Right => (dx, dy),
            };
            (along > 1.0).then_some((w.id, along + across.abs() * 2.0))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(id, _)| id)
}

fn centre([x, y, w, h]: [f32; 4]) -> [f32; 2] {
    [x + w * 0.5, y + h * 0.5]
}
