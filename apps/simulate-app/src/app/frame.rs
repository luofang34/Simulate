//! One window frame: advance the flight, place the camera, evaluate the sky, draw the
//! cockpit and the menu, and present.

use std::collections::hash_map::Entry;

use sim_geo::BodyEllipsoid;
use sim_render::{Camera, EnvironmentFrame};
use sim_scenario::{OperatorViewportMode, ScenarioWorld, SimTimeScale};
use winit::event_loop::ActiveEventLoop;

use super::{FlightApp, TITLE_INTERVAL_S};
use crate::sky_rig::wall_clock_utc_s;
use crate::telemetry;

impl FlightApp {
    /// Uploads the fallback `GlobeScene` mesh for `world` when needed (e.g. `LunarTranquility`
    /// or `WebGL2` fallback).
    fn ensure_globe_world(&mut self, event_loop: &ActiveEventLoop, world: ScenarioWorld) {
        let Some(renderer) = self.renderer.as_mut() else {
            return;
        };
        if self.drawn_world == Some(world) {
            return;
        }
        let scene = match self.scenes.entry(world) {
            Entry::Occupied(scene) => scene.into_mut(),
            Entry::Vacant(slot) => match world.scene() {
                Ok(scene) => slot.insert(scene),
                Err(err) => return self.fail(event_loop, "building the terrain failed", &err),
            },
        };
        if let Err(err) = renderer.set_scene(&scene.mesh, scene.atlas.as_ref()) {
            return self.fail(event_loop, "uploading the terrain failed", &err);
        }
        self.drawn_world = Some(world);
    }

    /// Viewport mode, and the time scale the flight advances at: held while the menu is open.
    pub(super) fn view_and_time_scale(&self) -> (OperatorViewportMode, SimTimeScale) {
        let (mode, scale) = self.cockpit_rig.as_ref().map_or(
            (OperatorViewportMode::Cockpit, SimTimeScale::RealTime),
            |rig| {
                (
                    rig.supervisor().viewport_mode(),
                    rig.supervisor().time_scale(),
                )
            },
        );
        let scale = if self.esc_menu.is_open() {
            SimTimeScale::Paused
        } else {
            scale
        };
        (mode, scale)
    }

    /// Logs cockpit state transitions and refreshes the window title periodically.
    fn observe_cockpit(&mut self, elapsed_s: f64) {
        let cockpit = self.flight.cockpit();
        telemetry::log_changes(&self.cockpit, &cockpit);
        self.cockpit = cockpit;
        self.title_age_s += elapsed_s;
        if self.title_age_s < TITLE_INTERVAL_S {
            return;
        }
        let Some(window) = &self.window else {
            return;
        };
        self.title_age_s = 0.0;
        let profile = self.flight.profile();
        let earth = self.flight.world().body() == BodyEllipsoid::WGS84;
        let (viewport_mode, cam_title, aero_code, time_scale) = self.cockpit_rig.as_ref().map_or(
            (
                OperatorViewportMode::Cockpit,
                None,
                None,
                SimTimeScale::RealTime,
            ),
            |rig| {
                let sup = rig.supervisor();
                (
                    sup.viewport_mode(),
                    Some(sup.active_virtual_camera(profile).title),
                    // A link's page flies its own route; the app's default aerodrome is
                    // not on it.
                    (earth && !self.flight.shows_page())
                        .then(|| sup.effective_airport_code(profile)),
                    sup.time_scale(),
                )
            },
        );
        self.host.set_title(
            window,
            &telemetry::title(
                &self.flight,
                viewport_mode,
                cam_title.as_deref(),
                aero_code,
                time_scale,
                &cockpit,
            ),
        );
    }

    /// The window camera on the interpolated display state.
    fn frame_camera(
        &mut self,
        event_loop: &ActiveEventLoop,
        mode: OperatorViewportMode,
    ) -> Option<Camera> {
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.supervisor_mut()
                .set_vehicle_cameras(self.flight.vehicle_cameras());
        }
        let body = self.flight.world().body();
        let camera = match self.flight.display_camera(mode) {
            Ok(camera) => {
                self.look
                    .apply_on(body, mode, camera, &self.flight.display_vehicle_state())
            }
            Err(err) => {
                self.fail(event_loop, "the view could not be placed", &err);
                return None;
            }
        };
        if mode == OperatorViewportMode::SensorAperture
            && let Some(rig) = self.cockpit_rig.as_ref()
        {
            let (cam_body, spec) = rig
                .supervisor()
                .active_virtual_camera_on_body(self.flight.profile());
            if let Some(vcam) = self.flight.virtual_camera(cam_body, &spec) {
                return Some(vcam);
            }
        }
        Some(camera)
    }

    /// The pilot's keys, throttle and look over `elapsed_s` of wall time.
    fn pilot_inputs(&mut self, elapsed_s: f64, viewport_mode: OperatorViewportMode) {
        self.keyboard.update(elapsed_s, &mut self.mapper);
        let throttle = self.mapper.controls().throttle;
        if let Some(t) = self.controls.integrate_throttle(elapsed_s, throttle) {
            self.mapper.set(sim_input::Control::Throttle, t);
        }
        self.controls.sync_live(self.mapper.controls());
        if self.controls.take_look_reset() {
            self.look.reset();
        }
        let look_axes = self.controls.camera_look_axes();
        if viewport_mode == OperatorViewportMode::Cockpit {
            self.look.follow_stick(look_axes, elapsed_s);
        } else {
            self.look.integrate(look_axes, elapsed_s);
        }
    }

    /// Sizes the panels drawn apart to their canvases, and the instruments to the panels.
    fn follow_panel_layout(&mut self) {
        if let (Some(panels), Some(renderer), Some(rig)) = (
            self.panels.as_mut(),
            self.renderer.as_ref(),
            self.cockpit_rig.as_mut(),
        ) {
            rig.set_panels_apart(Some(panels.follow_layout(renderer)));
        }
    }

    pub(super) fn redraw(&mut self, event_loop: &ActiveEventLoop) {
        let (viewport_mode, time_scale) = self.view_and_time_scale();
        let controls = self.mapper.controls();
        let wall_s;
        match self.flight.advance_to_now(controls, time_scale) {
            Ok(elapsed_s) => {
                if !self.esc_menu.is_open() {
                    self.pilot_inputs(elapsed_s, viewport_mode);
                }
                self.update_takeover_hint(elapsed_s);
                wall_s = elapsed_s;
                self.observe_cockpit(elapsed_s);
            }
            Err(err) => return self.fail(event_loop, "the flight model stopped", &err),
        }
        let Some(camera) = self.frame_camera(event_loop, viewport_mode) else {
            return;
        };
        self.follow_panel_layout();
        let exposures = self.flight.drain_sensor_exposures();
        let world = self.flight.world();
        let sky = self
            .sky
            .evaluate(&self.flight, camera.pose.position_ecef, wall_clock_utc_s());
        let environment = sky.environment;
        if let Some(rig) = self.cockpit_rig.as_mut()
            && page_map_frame(
                (rig, self.page_map.as_mut()),
                &self.flight,
                self.renderer.as_ref(),
                self.occluded,
                (&camera, &sky),
            )
        {
            return self.present_occluded();
        }
        let need_globe = match (&self.renderer, self.cockpit_rig.as_mut()) {
            (Some(renderer), Some(rig)) => match rig.prepare_frame(
                renderer.queue(),
                renderer.surface_size(),
                &self.flight,
                &camera,
                &sky,
                &exposures,
            ) {
                Ok(need_globe) => need_globe,
                Err(err) => return self.fail(event_loop, "cockpit avionics update failed", &err),
            },
            (Some(_), None) => true,
            (None, _) => return,
        };
        if need_globe {
            self.ensure_globe_world(event_loop, world);
            if self.failed {
                return;
            }
        }
        self.render_menu(event_loop, wall_s);
        if self.failed {
            return;
        }
        if self.occluded {
            return self.present_occluded();
        }
        self.present(
            event_loop,
            world,
            need_globe.then_some((&camera, &environment)),
        );
    }

    fn present(
        &mut self,
        event_loop: &ActiveEventLoop,
        world: ScenarioWorld,
        globe: Option<(&Camera, &EnvironmentFrame)>,
    ) {
        let (Some(renderer), Some(rig)) = (self.renderer.as_mut(), self.cockpit_rig.as_ref())
        else {
            return;
        };
        let mut quads = rig.overlay_quads(world);
        let preview_view =
            super::hub::hub_preview_view(self.esc_menu.tab(), self.hangar.as_ref(), rig.maps());
        super::hub::hub_quads(
            self.menu_overlay.as_ref(),
            self.hub_preview,
            preview_view,
            &mut quads,
        );
        if let Err(err) = renderer.render_composite(globe, &quads) {
            return self.fail(event_loop, "rendering failed", &err);
        }
        let apart = match (self.panels.as_mut(), rig.apart_panel_quads(world)) {
            (Some(panels), Some(quads)) => panels.present(renderer, &quads),
            _ => Ok(()),
        };
        if let Err(err) = apart {
            return self.fail(event_loop, "the instrument panels could not be drawn", &err);
        }
        self.present_page_map();
    }

    /// While the window is occluded: no window frame, but the GPU's work goes on, and the
    /// page's map, a surface of its own, may be on screen while the world view is not.
    fn present_occluded(&mut self) {
        if let Some(renderer) = &self.renderer
            && let Err(err) = renderer.device().poll(wgpu::PollType::Poll)
        {
            tracing::debug!(?err, "occluded GPU poll failed");
        }
        self.present_page_map();
    }

    /// Shows the EFB's expanded view on the page's map canvas, when there is one.
    fn present_page_map(&mut self) {
        if let (Some(renderer), Some(surface), Some(rig), Some(link)) = (
            self.renderer.as_mut(),
            self.page_map.as_mut(),
            self.cockpit_rig.as_mut(),
            self.flight.page_map(),
        ) && let Err(err) = rig.present_page_map(renderer, surface, link)
        {
            tracing::warn!(error = %err, "the page map was not drawn");
        }
    }
}

/// Hands the page's input to its map; with the window `occluded` and a page's map to
/// show, draws the page's map alone and returns `true`.
fn page_map_frame(
    (rig, presenter): (
        &mut crate::cockpit_rig::CockpitRig,
        Option<&mut sim_render::PanelSurface>,
    ),
    flight: &crate::flight::Flight,
    renderer: Option<&sim_render::Renderer>,
    occluded: bool,
    view: (&Camera, &crate::sky_rig::SkyFrame<'_>),
) -> bool {
    let Some(link) = flight.page_map() else {
        return false;
    };
    let page_only = occluded && presenter.is_some();
    if let (Some([width, height]), Some(presenter), Some(renderer)) =
        (rig.take_page_map(link, !occluded), presenter, renderer)
    {
        renderer.resize_panel(presenter, width, height);
    }
    let Some(renderer) = renderer.filter(|_| page_only) else {
        return false;
    };
    rig.prepare_page_map_only(renderer.surface_size(), flight, view);
    true
}
