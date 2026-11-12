//! The electronic flight bag map of the cockpit: a compact moving map in the top-right
//! corner that expands into an EFB page, with the active airport's surface diagram and a
//! georeferenced approach plate for its active runway end.

use std::collections::BTreeMap;
use std::time::Duration;

use serde_json::Value;
use sim_maplibre::{
    ChartAssetStore, ChartSymbologyInput, EfbBasemap, EfbDisplayMode, EfbFollowMode, EfbLayerFlags,
    EfbMapConfig, EfbMapSurface, EfbPacer, EfbRefreshPolicy, FixPick, FlightPlan, RouteProgress,
    SceneryEndpoints, SharedMapRuntime,
};
use sim_scenery::{AirportDatabase, AirportRecord};
use tracing::warn;
use web_time::Instant;

use crate::error::AppSimError;

mod cockpit;
mod page;
mod plate;
#[cfg(test)]
mod tests;

/// Plate opacities the `OPAC` chip cycles through, the first being the default.
pub const PLATE_OPACITIES: [f32; 3] = [0.75, 0.45, 0.95];

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
    /// Pick the plan's fix nearest this point of the shown view, texture pixels, or
    /// none when no fix is near. Only a host that shows the picked fix, the page, picks.
    #[cfg(any(target_arch = "wasm32", test))]
    Pick([f32; 2]),
    /// Draw the plan over this basemap.
    SetBasemap(EfbBasemap),
    /// Dispatch a mission-planner / ad-hoc runway action to [`crate::efb_plan::EfbWorkspace`].
    Plan(crate::efb_plan::EfbPlanAction),
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
    /// Whether satellite imagery lies under the plan instead of the chart.
    pub satellite: bool,
}

/// The airport and runway end the EFB shows, `(code, designator)`.
pub type EfbAirport = (String, String);

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
    pacer: EfbPacer,
    dirty: bool,
    merged_diagram: Value,
    merged_diagram_shown: bool,
    extra_airports: BTreeMap<String, AirportRecord>,
    picked: Option<FixPick>,
    /// Which of `[compact, expanded]` a page's host draws, once it chose.
    views_shown: Option<[bool; 2]>,
    /// Frames drawn, so a host shows each new one once.
    frames: u64,
    /// The colour a page judging the flight from outside draws ownship's ring in; a
    /// cockpit, which has no such page, has none and marks no truth.
    page_ownship: Option<sim_maplibre::Rgb>,
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
            pacer: EfbPacer::new(EfbRefreshPolicy::default()),
            dirty: true,
            merged_diagram: aerodrome_geojson.clone(),
            merged_diagram_shown: true,
            extra_airports: BTreeMap::new(),
            picked: None,
            views_shown: None,
            frames: 0,
            page_ownship: None,
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
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        self.dirty || (!self.collapsed && self.surface.needs_redraw())
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
            satellite: self.surface.basemap() == EfbBasemap::Satellite,
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
            #[cfg(any(target_arch = "wasm32", test))]
            EfbCommand::Pick(at_px) => self.pick(at_px),
            EfbCommand::SetBasemap(basemap) => self.surface.set_basemap(basemap)?,
            EfbCommand::Plan(_) => {}
        }
        Ok(())
    }

    fn prepare_step(
        &mut self,
        now: Instant,
        sim_time: Duration,
        airport: Option<EfbAirport>,
    ) -> Result<bool, AppSimError> {
        if airport != self.airport {
            self.show_airport(airport)?;
        }
        if self.collapsed && self.mode() == EfbDisplayMode::CompactCorner {
            self.dirty = false;
            return Ok(false);
        }
        let ready = self.surface.has_ready_work();
        Ok(self.pacer.should_render(
            now,
            sim_time,
            self.mode(),
            self.view.is_some(),
            self.dirty,
            ready,
        ))
    }

    fn render_prepared(
        &mut self,
        now: Instant,
        sim_time: Duration,
        symbology: &ChartSymbologyInput<'_>,
    ) -> Result<(), AppSimError> {
        self.view = self.surface.render(symbology)?.cloned();
        self.pacer.record_rendered(now, sim_time);
        self.frames = self.frames.wrapping_add(1);
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

    /// Registers an ad-hoc or supplemental [`AirportRecord`] (`ZMRS`, `EFHF`, `ZZ01`, ...)
    /// and merges its surface `diagram_geojson` features into the EFB surface diagram, so
    /// [`Self::show_airport`] renders both its surface diagram and georeferenced approach plate.
    pub fn register_extra_airport(&mut self, record: AirportRecord, diagram_geojson: &Value) {
        let code = record.ids.primary.as_str().to_owned();
        if let (Some(merged_features), Some(new_features)) = (
            self.merged_diagram
                .get_mut("features")
                .and_then(Value::as_array_mut),
            diagram_geojson.get("features").and_then(Value::as_array),
        ) {
            merged_features.retain(|f| {
                f.get("properties")
                    .and_then(|p| p.get("aerodrome"))
                    .and_then(Value::as_str)
                    != Some(code.as_str())
            });
            merged_features.extend(new_features.iter().cloned());
        }
        self.extra_airports.insert(code.clone(), record);
        self.merged_diagram_shown = false;
        if self.airport.as_ref().is_some_and(|(a, _)| a == &code) {
            self.airport = None;
        }
        self.dirty = true;
    }

    /// Draws `plan` at `progress` on the EFB map.
    pub fn set_route(
        &mut self,
        plan: &FlightPlan,
        progress: RouteProgress,
    ) -> Result<(), AppSimError> {
        self.surface.set_route(plan, progress)?;
        self.dirty = true;
        Ok(())
    }

    /// Centres the shown EFB view on `(lat_deg, lon_deg)`, optionally setting its zoom.
    pub fn jump_to(&mut self, lat_deg: f64, lon_deg: f64, zoom: Option<f64>) {
        self.surface.jump_to(lat_deg, lon_deg, zoom);
        self.dirty = true;
    }

    /// Unprojects physical texture pixel `at_px` on the shown EFB view to `[lat_deg, lon_deg]`.
    #[must_use]
    pub fn screen_to_geodetic(&self, at_px: [f32; 2]) -> Option<[f64; 2]> {
        self.surface.screen_to_geodetic(self.mode(), at_px)
    }

    /// Picks the nearest route fix to physical texture pixel `at_px` within `radius_css_px`.
    #[must_use]
    pub fn pick_fix_at(&self, at_px: [f32; 2], radius_css_px: f32) -> Option<FixPick> {
        let radius_px = radius_css_px * self.surface.pixel_ratio() as f32;
        self.surface.pick_fix(self.mode(), at_px, radius_px)
    }

    /// Shows `airport`'s surface diagram and the approach plate of its runway end.
    fn show_airport(&mut self, airport: Option<EfbAirport>) -> Result<(), AppSimError> {
        self.dirty = true;
        let extra = airport
            .as_ref()
            .and_then(|(code, _)| self.extra_airports.get(code))
            .cloned();
        let builtin = airport
            .as_ref()
            .and_then(|(code, _)| AirportDatabase::builtin().by_code(code))
            .map(|(_, record)| record);
        let record = extra.as_ref().or(builtin);
        let mapped = extra.is_some() || record.is_none_or(|r| r.osm_aerodrome().is_some());
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
