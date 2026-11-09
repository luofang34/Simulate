//! Headless `1280×720` visual verification shots covering the top-right compact EFB,
//! expanded EFB approach plate, pure avionics SVS sky, KEWR runway threshold spawn, and
//! every tab of the `Esc` Flight Hub.

use std::path::{Path, PathBuf};

use sim_core::ControlInputs;
use sim_maplibre::{EfbDisplayMode, HostGpu};
use sim_render::{
    CompositorPass, CompositorTarget, HeadlessGpu, OverlayQuad, VectorScenePass, ViewportRect,
};
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::{AerodromeId, AirportDatabase};

use super::CockpitRig;
use crate::controls::{ControlScheme, KeyAction, VehicleClass, key_label};
use crate::error::AppSimError;
use crate::esc_menu::{
    AvionicsLayout, ControlsStatus, EscMenu, HubCatalog, HubTab, HubView, MenuInput, MenuOverlay,
    MenuStatus, OverlayContent, RateChoice, SceneryMode, SpawnMode,
};
use crate::flight::Flight;
use crate::gallery::{create_target_texture, readback_target_rgba, save_png};
use crate::hangar::HangarRig;
use crate::map_rig::{AvionicsPreset, EfbCommand, GlobeSelection, ScenerySource};
use crate::sky_rig::SkyFrame;
use crate::vehicles::FreeFlightVehicle;
use crate::world_surface::WorldSurfaceHandle;

mod hud_audit;
mod n51_audit;
mod p38g;
mod polar;

const SHOT_SIZE: [u32; 2] = [1280, 720];

struct VerifyHarness {
    gpu: HeadlessGpu,
    compositor: CompositorPass,
    target: wgpu::Texture,
    target_view: wgpu::TextureView,
    rig: CockpitRig,
    flight: Flight,
    overlay: MenuOverlay,
    hangar: HangarRig,
    catalog: HubCatalog,
    menu: EscMenu,
    controls: ControlScheme,
}

fn poll_gpu(gpu: &HeadlessGpu) -> Result<(), AppSimError> {
    gpu.device
        .poll(wgpu::PollType::wait_indefinitely())
        .map(|_| ())
        .map_err(|e| AppSimError::Screenshot {
            detail: e.to_string(),
        })
}

impl VerifyHarness {
    fn new() -> Result<Option<Self>, AppSimError> {
        let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
            return Ok(None);
        };
        if gpu.backend == wgpu::Backend::Gl {
            return Ok(None);
        }
        let host_gpu = HostGpu {
            instance: gpu.instance.clone(),
            adapter: gpu.adapter.clone(),
            device: gpu.device.clone(),
            queue: gpu.queue.clone(),
        };
        let hud_pass = pollster::block_on(VectorScenePass::new(&gpu.device))?;
        let menu_pass = pollster::block_on(VectorScenePass::new(&gpu.device))?;
        let compositor = pollster::block_on(CompositorPass::new(
            &gpu.device,
            wgpu::TextureFormat::Rgba8UnormSrgb,
        ))?;
        let target = create_target_texture(&gpu.device, SHOT_SIZE);
        let target_view = target.create_view(&wgpu::TextureViewDescriptor::default());
        let surface = WorldSurfaceHandle::new();
        let scenery = ScenerySource::from_environment();
        let rig = CockpitRig::new_with_gpu(
            host_gpu,
            SHOT_SIZE,
            Some(scenery),
            hud_pass,
            surface.clone(),
        )?;
        let flight = Flight::with_surface(surface)?;
        let overlay = MenuOverlay::new(menu_pass);
        let hangar = HangarRig::new(&gpu.device)?;
        Ok(Some(Self {
            gpu,
            compositor,
            target,
            target_view,
            rig,
            flight,
            overlay,
            hangar,
            catalog: HubCatalog::with_global_airports(),
            menu: EscMenu::default(),
            controls: ControlScheme::new(VehicleClass::FixedWing),
        }))
    }

    fn settle_and_draw_flight(&mut self, out: &Path) -> Result<(), AppSimError> {
        let sun = self.flight.world().sun_direction_ecef();
        self.settle_and_draw_flight_with_sun(sun, out)
    }

    fn settle_and_draw_flight_with_sun(
        &mut self,
        sun_ecef: glam::DVec3,
        out: &Path,
    ) -> Result<(), AppSimError> {
        let sky = SkyFrame::fixed_sun(self.flight.world().body(), sun_ecef);
        for _ in 0..35 {
            let mode = self.rig.supervisor().viewport_mode();
            let cam = self.flight.display_camera(mode)?;
            self.rig.invalidate_chart();
            let _globe = self.rig.prepare_frame(
                &self.gpu.queue,
                SHOT_SIZE,
                &self.flight,
                &cam,
                &sky,
                &[],
            )?;
            poll_gpu(&self.gpu)?;
            std::thread::sleep(std::time::Duration::from_millis(25));
        }
        let chrome = self.rig.efb_chrome(self.flight.world());
        self.overlay.render(
            &self.gpu.device,
            &self.gpu.queue,
            OverlayContent::FlightChrome {
                banner: None,
                efb: chrome.as_ref(),
            },
            SHOT_SIZE,
        )?;
        let mut quads = self.rig.overlay_quads(self.flight.world());
        if let Some(menu) = self.overlay.view() {
            quads.push(OverlayQuad::vector_overlay(ViewportRect::FULL, menu));
        }
        self.composite_to_png(&quads, out)
    }

    fn draw_hub_tab(
        &mut self,
        tab: HubTab,
        globe_sel: Option<GlobeSelection>,
        out: &Path,
    ) -> Result<(), AppSimError> {
        let status = self.menu_status();
        let dummy = HubView::build(&self.menu, &self.catalog, &status, SHOT_SIZE);
        if !self.menu.is_open() {
            let _ = self.menu.handle(MenuInput::Toggle, &dummy, &self.catalog);
        }
        let _ = self.menu.handle(MenuInput::Tab(tab), &dummy, &self.catalog);
        let view = HubView::build(&self.menu, &self.catalog, &status, SHOT_SIZE);
        self.overlay.render(
            &self.gpu.device,
            &self.gpu.queue,
            OverlayContent::Hub {
                view: &view,
                highlights: self.menu.highlights(),
            },
            SHOT_SIZE,
        )?;
        self.hangar.hide();
        if tab == HubTab::Hangar
            && let Some(px) = view.preview_px()
            && let Some(veh) = self.catalog.vehicles.get(self.menu.vehicle())
        {
            self.hangar.render(
                &self.gpu.device,
                &self.gpu.queue,
                veh.model,
                px,
                ControlInputs::default(),
                0.5,
            )?;
        }
        if tab == HubTab::World
            && let Some(px) = view.preview_px()
            && let Some(maps) = self.rig.maps_mut()
        {
            for _ in 0..95 {
                maps.render_globe_picker(px, globe_sel.clone())?;
                poll_gpu(&self.gpu)?;
                std::thread::sleep(std::time::Duration::from_millis(25));
            }
        }
        let mut quads = self.rig.overlay_quads(self.flight.world());
        if let Some(rect) = view.preview_viewport() {
            if tab == HubTab::World {
                if let Some(globe) = self.rig.maps().and_then(|m| m.globe_view()) {
                    quads.push(OverlayQuad::inset_map(rect, globe, false));
                }
            } else if let Some(hview) = self.hangar.view() {
                quads.push(OverlayQuad::panel(rect, hview, true, false));
            }
        }
        if let Some(menu) = self.overlay.view() {
            quads.push(OverlayQuad::vector_overlay(ViewportRect::FULL, menu));
        }
        self.composite_to_png(&quads, out)
    }

    fn composite_to_png(
        &self,
        quads: &[sim_render::OverlayQuad<'_>],
        out: &Path,
    ) -> Result<(), AppSimError> {
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("verify shot composite"),
            });
        self.compositor.encode(
            &self.gpu.device,
            &self.gpu.queue,
            &mut encoder,
            CompositorTarget {
                view: &self.target_view,
                clear: Some(wgpu::Color::BLACK),
                size_px: SHOT_SIZE,
            },
            quads,
        );
        self.gpu.queue.submit([encoder.finish()]);
        let rgba = readback_target_rgba(&self.gpu.device, &self.gpu.queue, &self.target)?;
        save_png(out, SHOT_SIZE, rgba)
    }

    fn menu_status(&self) -> MenuStatus {
        let sup = self.rig.supervisor();
        MenuStatus {
            pacing: self.flight.pacing_stats(),
            history_s: self.flight.history_span_s(),
            rewind_offset_s: self.flight.rewind_offset_s(),
            epoch: self.rig.guidance_epoch(),
            sensor_frames: self.rig.maps().map_or(0, |m| m.sensor_frame_count()),
            sensor_last_s: None,
            scenario: "FREE FLIGHT".to_owned(),
            scenario_active: false,
            airport: sup.effective_airport_code(None).to_owned(),
            aircraft: self.flight.preferred_vehicle().title().to_owned(),
            active_vehicle: self.flight.free_vehicle(),
            sky: "14:00 LOCAL".to_owned(),
            paused: true,
            rate: RateChoice::Real,
            layout: AvionicsLayout::FullGlass,
            scenery: SceneryMode::Online,
            auto_rotate: true,
            controls: ControlsStatus {
                preset: self.controls.preset(),
                automatic: self.controls.is_automatic(),
                rows: self.controls.axis_rows(),
                keys: KeyAction::ALL
                    .into_iter()
                    .map(|a| (a, key_label(self.controls.keys().key_for(a))))
                    .collect(),
                capture: None,
                live: ControlInputs {
                    roll: 0.25,
                    pitch: -0.15,
                    yaw: 0.40,
                    throttle: 0.65,
                },
                source: self.controls.source(),
                controller: Some("Xbox Wireless Controller".to_owned()),
            },
        }
    }
}

#[test]
#[ignore = "writes 1280x720 verification PNGs into $SIMULATE_VERIFY_OUT"]
fn render_ui_and_efb_verification_shots() -> Result<(), AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    std::fs::create_dir_all(&dir).map_err(|e| AppSimError::Screenshot {
        detail: e.to_string(),
    })?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.rig.set_avionics_preset(AvionicsPreset::GlassCockpit)?;
    for _ in 0..240 {
        h.flight
            .advance_by(0.10, ControlInputs::default(), SimTimeScale::RealTime)?;
    }
    h.settle_and_draw_flight(&dir.join("01_cockpit_compact_efb_svs.png"))?;

    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb))?;
    h.settle_and_draw_flight(&dir.join("02_cockpit_expanded_efb_plate.png"))?;
    h.rig
        .efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner))?;

    let _ = h.flight.scrub_by_seconds(-10.0)?;
    h.draw_hub_tab(HubTab::Flight, None, &dir.join("04_hub_tab1_timeline.png"))?;
    let _ = h.flight.scrub_to_live()?;

    h.menu
        .sync_vehicle(&h.catalog, FreeFlightVehicle::Kinematic);
    h.draw_hub_tab(HubTab::Hangar, None, &dir.join("05_hub_tab2_hangar.png"))?;

    let (kewr_idx, kewr_rec) =
        AirportDatabase::builtin()
            .by_code("KEWR")
            .ok_or_else(|| AppSimError::Screenshot {
                detail: "missing KEWR in AirportDatabase".to_owned(),
            })?;
    if let Some(cat_idx) = h.catalog.index_of_airport(kewr_idx) {
        h.menu.select_aerodrome(cat_idx);
    }
    let globe_sel = Some(GlobeSelection {
        lat_deg: kewr_rec.lat_deg,
        lon_deg: kewr_rec.lon_deg,
        airport: Some(kewr_idx),
        runway: Some("22L".to_owned()),
    });
    h.draw_hub_tab(
        HubTab::World,
        globe_sel,
        &dir.join("06_hub_tab3_world_globe.png"),
    )?;
    h.draw_hub_tab(
        HubTab::Scenarios,
        None,
        &dir.join("07_hub_tab4_scenarios.png"),
    )?;
    h.draw_hub_tab(
        HubTab::Controls,
        None,
        &dir.join("08_hub_tab5_controls.png"),
    )?;

    h.rig
        .supervisor_mut()
        .set_free_aerodrome(AerodromeId::NewarkKewr);
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::NewarkKewr).cloned() {
        h.flight
            .spawn_at_aerodrome(&diag, 0, SpawnMode::Threshold)?;
    }
    for _ in 0..30 {
        h.flight
            .advance_by(0.10, ControlInputs::default(), SimTimeScale::RealTime)?;
    }
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    h.settle_and_draw_flight(&dir.join("03_kewr_threshold_chase.png"))?;
    h.gpu.check_device()?;
    Ok(())
}

#[test]
#[ignore = "writes 1280x720 landed-cockpit and sideslip HUD PNGs into $SIMULATE_VERIFY_OUT"]
fn render_runway_cockpit_and_hud_verification_shots() -> Result<(), AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    std::fs::create_dir_all(&dir).map_err(|e| AppSimError::Screenshot {
        detail: e.to_string(),
    })?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Cockpit);
    h.rig.set_show_conformal_hud(true);

    for (code, file) in [
        ("KEWR", "09_kewr_runway_cockpit_hud.png"),
        ("47N", "10_47n_runway_cockpit_hud.png"),
    ] {
        if let Some((idx, rec)) = AirportDatabase::builtin().by_code(code) {
            h.rig.supervisor_mut().set_free_airport(idx, 0);
            h.flight
                .spawn_at_airport_record(rec, 0, SpawnMode::Threshold)?;
            // Stream DEM/ortho tiles first, then respawn and settle onto resident DEM.
            h.settle_and_draw_flight(&dir.join(file))?;
            h.flight
                .spawn_at_airport_record(rec, 0, SpawnMode::Threshold)?;
            for _ in 0..30 {
                h.flight
                    .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
            }
            h.settle_and_draw_flight(&dir.join(file))?;
        }
    }

    h.rig
        .supervisor_mut()
        .set_free_aerodrome(AerodromeId::DoverKdov);
    if let Some(diag) = h.rig.supervisor().diagram(AerodromeId::DoverKdov).cloned() {
        h.flight
            .spawn_at_aerodrome(&diag, 0, SpawnMode::ShortFinal)?;
    }
    let slip = ControlInputs {
        roll: -0.16,
        pitch: 0.03,
        yaw: 0.85,
        throttle: 0.72,
    };
    for _ in 0..40 {
        h.flight.advance_by(0.10, slip, SimTimeScale::RealTime)?;
    }
    h.settle_and_draw_flight(&dir.join("11_sideslip_hud_centred.png"))?;
    h.gpu.check_device()?;
    Ok(())
}
