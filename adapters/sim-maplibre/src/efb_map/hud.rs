//! MapLibre built-in navigation HUD controls (`NavigationHudPlugin`) on the expanded EFB
//! and web page map, kept hidden on the compact cockpit minimap inset.

use std::time::Duration;

use cgmath::Vector2;
use maplibre::{
    headless::{ViewSlotId, map::HeadlessMap},
    hud::NavigationHudState,
    navigation::NavigationState,
    render::camera::EdgeInsets,
};

pub use maplibre::hud::HudButton;

use super::{EfbDisplayMode, EfbFollowMode, EfbMapSurface, mode_index};
use crate::{ChartCameraSpec, MapAdapterError};

/// Oblique tilt restored when the HUD `3D` toggle re-enters oblique relief view, degrees.
const DEFAULT_HUD_3D_PITCH_DEG: f64 = 45.0;

/// Configures `map` so [`EfbDisplayMode::CompactCorner`] hides the navigation HUD while
/// [`EfbDisplayMode::ExpandedEfb`] renders it.
pub(super) fn init_navigation_hud(
    map: &mut HeadlessMap,
    slots: [ViewSlotId; 2],
) -> Result<NavigationState, MapAdapterError> {
    let mut navigation = NavigationState::default();
    navigation.show_hud = true;
    let _animating = map.navigate(&mut navigation, Duration::ZERO);
    navigation.auto_tilt = false;
    let [compact_slot, expanded_slot] = slots;
    map.set_view_slot_navigation_hud(compact_slot, false)
        .map_err(|source| MapAdapterError::MultiView { source })?;
    map.set_view_slot_navigation_hud(expanded_slot, true)
        .map_err(|source| MapAdapterError::MultiView { source })?;
    Ok(navigation)
}

impl EfbMapSurface {
    /// Synchronises the HUD capsule flags (`3D/2D`, `North-Lock`) before rendering `mode`.
    pub(super) fn sync_navigation_hud(&mut self, mode: EfbDisplayMode, spec: &ChartCameraSpec) {
        if mode != EfbDisplayMode::ExpandedEfb {
            return;
        }
        let follow = self
            .cameras
            .get(mode_index(mode))
            .map_or(EfbFollowMode::FollowTrackUp, |camera| camera.follow);
        self.navigation.show_hud = true;
        self.navigation.auto_tilt = false;
        self.navigation.three_dimensional = spec.pitch_deg.abs() > 1.0;
        self.navigation.north_locked = follow == EfbFollowMode::FollowNorthUp;
        let _animating = self.map.navigate(&mut self.navigation, Duration::ZERO);
    }

    /// Hit-tests a physical texture coordinate on `mode` against the MapLibre navigation
    /// HUD capsule (`None` on [`EfbDisplayMode::CompactCorner`], where the HUD is hidden).
    #[must_use]
    pub fn hit_navigation_hud(
        &self,
        mode: EfbDisplayMode,
        physical_px: [f32; 2],
    ) -> Option<HudButton> {
        if mode != EfbDisplayMode::ExpandedEfb {
            return None;
        }
        let [px_x, px_y] = physical_px;
        if !px_x.is_finite() || !px_y.is_finite() {
            return None;
        }
        let ratio = self.pixel_ratio();
        let [width_px, height_px] = self.size_of(mode);
        let logical = Vector2::new(f64::from(px_x) / ratio, f64::from(px_y) / ratio);
        NavigationHudState::hit_test(
            logical,
            f64::from(width_px) / ratio,
            f64::from(height_px) / ratio,
            &EdgeInsets::default(),
        )
    }

    /// Hit-tests `physical_px` on `mode`'s navigation HUD and applies the matching camera
    /// action (`Compass`, `Mode3D`, `ZoomIn`, `ZoomOut`, `Home`) when hit.
    pub fn activate_navigation_hud(
        &mut self,
        mode: EfbDisplayMode,
        physical_px: [f32; 2],
    ) -> Option<HudButton> {
        let button = self.hit_navigation_hud(mode, physical_px)?;
        let camera = self.cameras.get_mut(mode_index(mode))?;
        match button {
            HudButton::Compass => {
                let next = match camera.follow {
                    EfbFollowMode::FollowNorthUp => EfbFollowMode::FollowTrackUp,
                    _ => EfbFollowMode::FollowNorthUp,
                };
                camera.set_follow(next);
            }
            HudButton::Mode3D => {
                let current = camera.spec().pitch_deg;
                let target = if current > 1.0 {
                    0.0
                } else {
                    DEFAULT_HUD_3D_PITCH_DEG
                };
                camera.pitch_by(target - current);
            }
            HudButton::ZoomIn => camera.zoom_by(1.0),
            HudButton::ZoomOut => camera.zoom_by(-1.0),
            HudButton::Home => camera.recenter(),
            HudButton::PointerMode => {}
        }
        Some(button)
    }
}
