//! Where a handheld's touch overlay hangs its controls: the page lays its tabs and bars
//! out from the rectangles the window draws, so they sit on what they fold.

use sim_maplibre::EfbDisplayMode;
use sim_scenario::ScenarioWorld;

use super::CockpitRig;
use super::layout::{self, InstrumentDockStyle};
use super::safe_area::SafeAreaInsets;

/// The drawn rectangles a touch overlay attaches to, as fractions of the window's width
/// (left, right) and height (top, bottom).
#[cfg_attr(
    not(target_arch = "wasm32"),
    allow(dead_code, reason = "read by the wasm32 touch overlay and tests")
)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TouchAnchors {
    /// The instrument column's `[left, top, right]` edges, shown or folded; `None` when
    /// the flight has no instruments to put there.
    pub rail: Option<[f32; 3]>,
    /// Whether the instrument column is drawn.
    pub rail_shown: bool,
    /// The corner map's `[left, bottom]` edges while it is drawn.
    pub corner_map: Option<[f32; 2]>,
}

impl TouchAnchors {
    /// How far in from the left edge the drawn instrument column reaches; `0.0` when it
    /// is folded or absent, which gives the thumb controls the edge.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "read by the wasm32 touch overlay and tests")
    )]
    #[must_use]
    pub fn rail_reach(&self) -> f32 {
        self.rail
            .filter(|_| self.rail_shown)
            .map_or(0.0, |[_, _, right]| right)
    }
}

/// The instrument column's `[left, top, right]` edges on a `surface_size` window.
fn rail_edges(surface_size: [u32; 2], insets: SafeAreaInsets) -> [f32; 3] {
    let dock = layout::dock_layout(surface_size, InstrumentDockStyle::LeftRail, false, insets);
    [dock.pfd.x, dock.pfd.y, dock.right_edge_norm]
}

impl CockpitRig {
    /// What the touch overlay attaches to on a `surface_size` window over `world`.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "read by the wasm32 touch overlay and tests")
    )]
    #[must_use]
    pub fn touch_anchors(&self, surface_size: [u32; 2], world: ScenarioWorld) -> TouchAnchors {
        let has_rail = self.dock_style == InstrumentDockStyle::LeftRail
            && self.sources.avionics
            && self.panels_apart.is_none();
        let corner_map = self
            .efb_chrome(world)
            .filter(|chrome| chrome.mode == EfbDisplayMode::CompactCorner)
            .map(|chrome| chrome.map_viewport())
            .filter(|map| map.width > 0.0)
            .map(|map| [map.x, map.y + map.height]);
        TouchAnchors {
            rail: has_rail.then(|| rail_edges(surface_size, self.safe_area)),
            rail_shown: has_rail && self.show_instruments,
            corner_map,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{TouchAnchors, rail_edges};
    use crate::cockpit_rig::SafeAreaInsets;

    #[test]
    fn a_folded_column_keeps_its_place_and_gives_up_its_reach() {
        let housing = SafeAreaInsets::from_lengths([59.0, 0.0, 59.0, 21.0], [874.0, 402.0]);
        let [left, top, right] = rail_edges([1748, 804], housing);
        assert!((left - 59.0 / 874.0).abs() < 1e-3, "inside the housing");
        assert!(top > 0.0 && right > left && right <= 0.27);
        let shown = TouchAnchors {
            rail: Some([left, top, right]),
            rail_shown: true,
            corner_map: None,
        };
        assert!((shown.rail_reach() - right).abs() < f32::EPSILON);
        let folded = TouchAnchors {
            rail_shown: false,
            ..shown
        };
        assert_eq!(folded.rail, shown.rail, "the tab that unfolds it stays put");
        assert!(folded.rail_reach().abs() < f32::EPSILON);
        assert!(TouchAnchors::default().rail_reach().abs() < f32::EPSILON);
    }
}
