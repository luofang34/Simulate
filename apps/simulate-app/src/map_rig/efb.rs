//! The electronic flight bag map of the cockpit: a compact moving map in the top-right
//! corner that expands into an EFB page, with the active airport's surface diagram and a
//! georeferenced approach plate for its active runway end.

use std::time::Duration;

use serde_json::Value;
use sim_maplibre::{
    ChartAssetStore, ChartSymbologyInput, EfbDisplayMode, EfbFollowMode, EfbLayerFlags,
    EfbMapConfig, EfbMapSurface, SceneryEndpoints, SharedMapRuntime, ViewSlotMask,
};
use sim_scenario::OperatorViewportMode;
use sim_scenery::AirportDatabase;
use tracing::warn;

use super::{AvionicsPreset, MapFrameInput, MapLibreRig, charts};
use crate::cockpit_rig::efb_chrome::{efb_map_rect, rect_px};
use crate::error::AppSimError;

mod plate;
#[cfg(test)]
mod tests;

/// Plate opacities the `OPAC` chip cycles through, the first being the default.
pub const PLATE_OPACITIES: [f32; 3] = [0.75, 0.45, 0.95];
/// Shortest interval between EFB frames while nothing but ownship moves.
const FRAME_INTERVAL: Duration = Duration::from_millis(66);

/// An operator command to the EFB.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EfbCommand {
    /// Change the shown view's zoom by this many levels.
    ZoomBy(f64),
    /// Rotate the shown view's bearing by this many degrees (`> 0` counter-clockwise).
    RotateBy(f64),
    /// Show the compact corner map or the expanded EFB page.
    SetMode(EfbDisplayMode),
    /// Collapse (`true`) or unfold (`false`) the compact corner map into a small button.
    SetCollapsed(bool),
    /// Switch between the compact and expanded views.
    ToggleExpanded,
    /// Switch between track-up and north-up following.
    ToggleNorthUp,
    /// Show or hide the approach plate in the shown view.
    TogglePlate,
    /// Step the plate opacity through [`PLATE_OPACITIES`].
    CycleOpacity,
    /// Show or hide the airport surface diagram.
    ToggleDiagram,
    /// Follow ownship again.
    Recenter,
    /// Pan the shown view by `[dx, dy]` texture pixels and stop following ownship.
    PanPixels([f32; 2]),
}

/// What the EFB shows, for its bezel and buttons.
#[derive(Debug, Clone, PartialEq)]
pub struct EfbStatus {
    /// The shown view.
    pub mode: EfbDisplayMode,
    /// Whether the compact corner map is collapsed into a small unfold button.
    pub collapsed: bool,
    /// The shown view's zoom.
    pub zoom: f64,
    /// Whether the camera keeps north up (else track up or custom bearing).
    pub north_up: bool,
    /// Whether the pilot twisted the map to a custom bearing while following ownship.
    pub custom_bearing: bool,
    /// Whether the pilot panned away from ownship.
    pub free_pan: bool,
    /// Whether the approach plate is drawn in the shown view.
    pub plate_shown: bool,
    /// Plate opacity, percent.
    pub plate_opacity_pct: u32,
    /// Whether the airport surface diagram is drawn.
    pub diagram_shown: bool,
    /// Active airport code, e.g. `KEWR`.
    pub airport: String,
    /// Title of the active approach plate, e.g. `ILS OR LOC RWY 22L`.
    pub plate_title: Option<String>,
}

/// The airport and runway end the EFB shows, `(code, designator)`.
pub type EfbAirport = (String, String);

/// One frame's inputs to the EFB.
pub struct EfbFrame<'a> {
    /// Simulation time of the frame.
    pub sim_time: Duration,
    /// Active airport and runway end.
    pub airport: Option<EfbAirport>,
    /// Ownship, runway, and traffic symbology (camera and viewport are the EFB's own).
    pub symbology: ChartSymbologyInput<'a>,
}

/// The EFB map surface and the state its chrome toggles.
pub struct EfbRig {
    surface: EfbMapSurface,
    view: Option<wgpu::TextureView>,
    collapsed: bool,
    airport: Option<EfbAirport>,
    plate_title: Option<String>,
    /// Whether the plate shows in `[compact, expanded]`.
    plate_shown: [bool; 2],
    diagram_shown: bool,
    opacity_step: usize,
    last_frame: Option<Duration>,
    dirty: bool,
    merged_diagram: Value,
    merged_diagram_shown: bool,
}

impl EfbRig {
    /// Builds the EFB over `runtime` with plates served from `charts`, whose compact view
    /// is `compact_px` and whose diagram starts as the merged built-in `aerodrome_geojson`.
    pub fn new(
        runtime: &SharedMapRuntime,
        charts: ChartAssetStore,
        endpoints: &SceneryEndpoints,
        aerodrome_geojson: &Value,
        compact_px: [u32; 2],
    ) -> Result<Self, AppSimError> {
        let config = EfbMapConfig {
            compact_size: compact_px.map(|v| v.max(64)),
            ..EfbMapConfig::default()
        };
        let surface = EfbMapSurface::new(
            runtime,
            charts,
            endpoints,
            aerodrome_geojson.clone(),
            config,
        )?;
        let mut rig = Self {
            surface,
            view: None,
            collapsed: false,
            airport: None,
            plate_title: None,
            plate_shown: [false, true],
            diagram_shown: true,
            opacity_step: 0,
            last_frame: None,
            dirty: true,
            merged_diagram: aerodrome_geojson.clone(),
            merged_diagram_shown: true,
        };
        rig.apply_layers()?;
        Ok(rig)
    }

    /// The shown view's last frame (`None` while collapsed).
    #[must_use]
    pub const fn view(&self) -> Option<&wgpu::TextureView> {
        if self.collapsed {
            None
        } else {
            self.view.as_ref()
        }
    }

    /// Whether the EFB has either a rendered frame or is collapsed into its unfold button.
    #[must_use]
    pub const fn ready(&self) -> bool {
        self.view.is_some() || self.collapsed
    }

    /// The shown view.
    #[must_use]
    pub const fn mode(&self) -> EfbDisplayMode {
        self.surface.display_mode()
    }

    /// Whether arrived tiles still need drawing.
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        if self.collapsed {
            self.dirty
        } else {
            self.dirty || self.surface.needs_redraw()
        }
    }

    /// What the EFB shows.
    #[must_use]
    pub fn status(&self) -> EfbStatus {
        let follow = self.surface.follow_mode();
        EfbStatus {
            mode: self.mode(),
            collapsed: self.collapsed && self.mode() == EfbDisplayMode::CompactCorner,
            zoom: self.surface.zoom(),
            north_up: follow == EfbFollowMode::FollowNorthUp,
            custom_bearing: self.surface.has_custom_bearing(),
            free_pan: follow == EfbFollowMode::FreePan,
            plate_shown: self.plate_visible(),
            plate_opacity_pct: (self.surface.plate_opacity() * 100.0).round() as u32,
            diagram_shown: self.diagram_shown,
            airport: self
                .airport
                .as_ref()
                .map_or_else(String::new, |a| a.0.clone()),
            plate_title: self.plate_title.clone(),
        }
    }

    /// Resizes the expanded EFB page to `size_px`, its on-screen physical size.
    pub fn resize_expanded(&mut self, size_px: [u32; 2]) -> Result<(), AppSimError> {
        Ok(self.surface.resize_expanded(size_px.map(|v| v.max(64)))?)
    }

    /// Draws the EFB at `ratio` physical pixels per logical pixel (`2.0` on Retina).
    pub fn set_pixel_ratio(&mut self, ratio: f64) {
        self.surface.set_pixel_ratio(ratio);
        self.dirty = true;
    }

    /// Applies an operator command; the next frame redraws.
    pub fn command(&mut self, command: EfbCommand) -> Result<(), AppSimError> {
        self.dirty = true;
        match command {
            EfbCommand::ZoomBy(delta) => self.surface.zoom_by(delta),
            EfbCommand::RotateBy(delta_deg) => self.surface.rotate_by_deg(delta_deg),
            EfbCommand::SetMode(mode) => {
                self.collapsed = false;
                self.surface.set_display_mode(mode);
                self.view = None;
                self.apply_layers()?;
            }
            EfbCommand::SetCollapsed(collapsed) => {
                self.collapsed = collapsed;
            }
            EfbCommand::ToggleExpanded => {
                self.collapsed = false;
                let _shown = self.surface.toggle_expanded();
                self.view = None;
                self.apply_layers()?;
            }
            EfbCommand::ToggleNorthUp => {
                let follow = match self.surface.follow_mode() {
                    EfbFollowMode::FollowNorthUp => EfbFollowMode::FollowTrackUp,
                    EfbFollowMode::FollowTrackUp
                    | EfbFollowMode::FollowBearing
                    | EfbFollowMode::FreePan => EfbFollowMode::FollowNorthUp,
                };
                self.surface.set_follow_mode(follow);
            }
            EfbCommand::TogglePlate => {
                let index = self.mode_index();
                if let Some(shown) = self.plate_shown.get_mut(index) {
                    *shown = !*shown;
                }
                self.apply_layers()?;
            }
            EfbCommand::CycleOpacity => {
                self.opacity_step = self.opacity_step.wrapping_add(1) % PLATE_OPACITIES.len();
                let opacity = PLATE_OPACITIES
                    .get(self.opacity_step)
                    .copied()
                    .unwrap_or(0.75);
                self.surface.set_plate_opacity(opacity)?;
            }
            EfbCommand::ToggleDiagram => {
                self.diagram_shown = !self.diagram_shown;
                self.apply_layers()?;
            }
            EfbCommand::Recenter => self.surface.recenter_ownship(),
            EfbCommand::PanPixels([dx, dy]) => self.surface.pan_by_pixels(dx, dy),
        }
        Ok(())
    }

    /// Follows the active airport and draws the shown view when it is due.
    pub fn step(&mut self, frame: &EfbFrame<'_>) -> Result<(), AppSimError> {
        if frame.airport != self.airport {
            self.show_airport(frame.airport.clone())?;
        }
        if self.collapsed && self.mode() == EfbDisplayMode::CompactCorner {
            self.dirty = false;
            return Ok(());
        }
        let due = self.last_frame.is_none_or(|last| {
            frame.sim_time < last || frame.sim_time.saturating_sub(last) >= FRAME_INTERVAL
        });
        if !(due || self.view.is_none() || self.needs_redraw()) {
            return Ok(());
        }
        self.view = self.surface.render(&frame.symbology)?.cloned();
        self.last_frame = Some(frame.sim_time);
        self.dirty = false;
        Ok(())
    }

    fn mode_index(&self) -> usize {
        match self.mode() {
            EfbDisplayMode::CompactCorner => 0,
            EfbDisplayMode::ExpandedEfb => 1,
        }
    }

    fn plate_visible(&self) -> bool {
        self.surface.approach_plate().is_some()
            && self.plate_shown.get(self.mode_index()).copied() == Some(true)
    }

    fn apply_layers(&mut self) -> Result<(), AppSimError> {
        let layers = EfbLayerFlags {
            approach_plate: self.plate_shown.get(self.mode_index()).copied() == Some(true),
            airport_diagram: self.diagram_shown,
            ..EfbLayerFlags::default()
        };
        if layers != self.surface.layers() {
            self.surface.set_layers(layers)?;
        }
        Ok(())
    }

    /// Shows `airport`'s surface diagram and the approach plate of its runway end.
    fn show_airport(&mut self, airport: Option<EfbAirport>) -> Result<(), AppSimError> {
        self.dirty = true;
        let record = airport
            .as_ref()
            .and_then(|(code, _)| AirportDatabase::builtin().by_code(code))
            .map(|(_, record)| record);
        let mapped = record.is_none_or(|r| r.osm_aerodrome().is_some());
        if mapped && !self.merged_diagram_shown {
            self.surface
                .set_surface_diagram_geojson(&self.merged_diagram.to_string())?;
        } else if let (false, Some(record)) = (mapped, record) {
            // GeoJSON vector tiles (`geojson-vt`) clamp latitudes beyond Web Mercator's
            // ±85.0511° limit onto the parallel; polar runways render via `OverlayMesh`.
            let diagram = if record.lat_deg.abs() > 85.0 {
                r#"{"type":"FeatureCollection","features":[]}"#.to_owned()
            } else {
                record.to_surface_geojson()
            };
            self.surface.set_surface_diagram_geojson(&diagram)?;
        }
        self.merged_diagram_shown = mapped;
        let plate = match (record, &airport) {
            (Some(record), Some((_, designator))) => plate::plate_for(record, designator)
                .inspect_err(|err| warn!(error = %err, "approach plate unavailable"))
                .ok()
                .flatten(),
            _ => None,
        };
        self.plate_title = plate.as_ref().map(|p| p.title.clone());
        self.surface.set_approach_plate(plate)?;
        self.airport = airport;
        self.apply_layers()
    }
}

impl MapLibreRig {
    /// Builds the EFB the first time a map preset needs it; a failure leaves the cockpit
    /// without one.
    pub(super) fn ensure_efb(&mut self) {
        if self.preset == AvionicsPreset::NoMapGauges || self.efb.is_some() {
            return;
        }
        let compact = rect_px(
            efb_map_rect(self.window_px, EfbDisplayMode::CompactCorner),
            self.window_px,
        );
        self.efb = EfbRig::new(
            &self.runtime,
            self.charts.clone(),
            &self.endpoints,
            &self.aerodrome_geojson,
            compact,
        )
        .inspect_err(|err| warn!(error = %err, "EFB map unavailable"))
        .ok()
        .map(|mut efb| {
            efb.set_pixel_ratio(self.pixel_ratio);
            efb
        });
    }

    /// The EFB, when the preset shows one.
    fn shown_efb(&self) -> Option<&EfbRig> {
        self.efb
            .as_ref()
            .filter(|_| self.preset != AvionicsPreset::NoMapGauges)
    }

    /// Whether the preset shows the EFB.
    #[must_use]
    pub fn efb_shown(&self) -> bool {
        self.shown_efb().is_some()
    }

    /// What the EFB shows, when the preset shows one.
    #[must_use]
    pub fn efb_status(&self) -> Option<EfbStatus> {
        self.shown_efb().map(EfbRig::status)
    }

    /// The EFB's last frame, when the preset shows one and it is not collapsed.
    #[must_use]
    pub fn efb_view(&self) -> Option<&wgpu::TextureView> {
        self.shown_efb().and_then(EfbRig::view)
    }

    /// Whether the EFB has chrome to display (either a rendered frame or its unfold button).
    #[must_use]
    pub fn efb_ready(&self) -> bool {
        self.shown_efb().is_some_and(EfbRig::ready)
    }

    /// Whether the shown EFB has tile arrivals or a command waiting to be drawn.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn efb_needs_redraw(&self) -> bool {
        self.shown_efb().is_some_and(EfbRig::needs_redraw)
    }

    /// Applies an operator command to the EFB, if there is one.
    pub fn efb_command(&mut self, command: EfbCommand) -> Result<(), AppSimError> {
        match self.efb.as_mut() {
            Some(efb) => efb.command(command),
            None => Ok(()),
        }
    }

    /// Sizes the expanded EFB page to its on-screen `size_px`.
    pub fn set_efb_expanded_px(&mut self, size_px: [u32; 2]) -> Result<(), AppSimError> {
        match self.efb.as_mut() {
            Some(efb) => efb.resize_expanded(size_px),
            None => Ok(()),
        }
    }

    /// The airport the EFB shows instead of the supervisor's, if one was set.
    #[must_use]
    pub fn efb_airport(&self) -> Option<EfbAirport> {
        self.efb_airport.clone()
    }

    /// Shows `airport` on the EFB instead of the supervisor's active aerodrome (`None`
    /// follows the supervisor again).
    pub fn set_efb_airport(&mut self, airport: Option<EfbAirport>) {
        self.efb_airport = airport;
    }

    /// Draws the EFB when it is shown and due, on the navigation solution.
    pub(super) fn step_efb(
        &mut self,
        input: &MapFrameInput<'_>,
        sim_time: Duration,
    ) -> Result<(), AppSimError> {
        if self.preset == AvionicsPreset::NoMapGauges
            || input.supervisor.viewport_mode() == OperatorViewportMode::ChartMap
        {
            return Ok(());
        }
        let airport = self.efb_airport.clone().or_else(|| {
            let code = input.supervisor.effective_airport_code(input.profile);
            let runway = input
                .supervisor
                .active_runway(input.profile)
                .map_or_else(String::new, |r| r.designator.as_str().to_owned());
            Some((code.to_owned(), runway))
        });
        let landing_runway = self.active_onboard_landing_runway(input);
        let Some(efb) = self.efb.as_mut() else {
            return Ok(());
        };
        let own = charts::chart_ownship(&input.estimated_vehicle);
        let traffic = input.supervisor.belief_entities(ViewSlotMask::ALL);
        efb.step(&EfbFrame {
            sim_time,
            airport,
            symbology: ChartSymbologyInput {
                camera: charts::moving_map_camera(&own, input.supervisor),
                viewport_px: [1, 1],
                ownship: Some(own),
                runway: landing_runway.as_ref(),
                traffic: &traffic,
            },
        })
    }
}
