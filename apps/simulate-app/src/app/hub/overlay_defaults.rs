//! Which overlays a cockpit opens with when the flown airframe changes: a
//! period airframe opens on its own panel without the conformal HUD or glass
//! dock, and leaving it restores the fit flown before. Changes between modern
//! airframes keep whatever the pilot has toggled.

use crate::esc_menu::AvionicsLayout;
use crate::vehicles::FreeFlightVehicle;

/// The airframe the cockpit last took overlay defaults for (`None` inside: a
/// scenario), and the instrument fit to restore after a period airframe.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct OverlayDefaults {
    applied_for: Option<Option<FreeFlightVehicle>>,
    layout_before_period: Option<AvionicsLayout>,
}

const fn is_period(airframe: Option<FreeFlightVehicle>) -> bool {
    matches!(airframe, Some(vehicle) if vehicle.period_cockpit())
}

impl OverlayDefaults {
    /// Notes that `airframe` is now flown with the hub fit `layout`; returns the
    /// `[glass dock, conformal HUD]` to show when they change, restoring `layout`
    /// on leaving a period airframe.
    pub fn on_airframe(
        &mut self,
        airframe: Option<FreeFlightVehicle>,
        layout: &mut AvionicsLayout,
    ) -> Option<[bool; 2]> {
        let previous = self.applied_for.replace(airframe);
        if previous == Some(airframe) {
            return None;
        }
        let was_period = previous.is_some_and(is_period);
        if is_period(airframe) {
            if !was_period {
                self.layout_before_period = Some(*layout);
            }
            return Some([false, false]);
        }
        if let Some(before) = self.layout_before_period.take() {
            *layout = before;
        } else if previous.is_some() {
            return None;
        }
        Some([!matches!(layout, AvionicsLayout::HudOnly), true])
    }
}
