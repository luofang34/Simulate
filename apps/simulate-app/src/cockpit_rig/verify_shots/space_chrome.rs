//! Headless GPU verification that spaceflight `FlightChrome` renders the gimbal-free
//! `FDAI / Navball`, 3D `Orrery`, and `AerospikePlumeVisual` + spacecraft bus telemetry
//! onto `MenuOverlay`.

use sim_core::ControlInputs;
use sim_render::{
    CompositorPass, CompositorTarget, HeadlessGpu, OverlayQuad, VectorScenePass, ViewportRect,
};
use sim_scenario::SimTimeScale;

use super::SHOT_SIZE;
use crate::error::AppSimError;
use crate::esc_menu::{MenuOverlay, OverlayContent};
use crate::flight::Flight;
use crate::gallery::{create_target_texture, readback_target_rgba};

fn count_lit_pixels(rgba: &[u8], [x0, y0, x1, y1]: [u32; 4]) -> usize {
    let [width, _] = SHOT_SIZE;
    let mut lit = 0_usize;
    for y in y0..y1 {
        for x in x0..x1 {
            let idx = ((y * width + x) * 4) as usize;
            if let Some(px) = rgba.get(idx..idx + 3)
                && px.iter().any(|&c| c > 20)
            {
                lit = lit.wrapping_add(1);
            }
        }
    }
    lit
}

#[test]
fn space_flight_chrome_renders_fdai_orrery_and_plume_pixels_on_gpu() -> Result<(), AppSimError> {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return Ok(());
    };
    if gpu.backend == wgpu::Backend::Gl {
        return Ok(());
    }
    let menu_pass = pollster::block_on(VectorScenePass::new(&gpu.device))?;
    let compositor = pollster::block_on(CompositorPass::new(
        &gpu.device,
        wgpu::TextureFormat::Rgba8UnormSrgb,
    ))?;
    let target = create_target_texture(&gpu.device, SHOT_SIZE);
    let target_view = target.create_view(&wgpu::TextureViewDescriptor::default());
    let mut overlay = MenuOverlay::new(menu_pass);
    let mut flight = Flight::new()?;
    flight.fly_raketa()?;
    for _ in 0..40 {
        flight.advance_by(0.25, ControlInputs::default(), SimTimeScale::RealTime)?;
    }
    let space = flight
        .space_hud_chrome()
        .ok_or_else(|| AppSimError::Screenshot {
            detail: "missing space_hud_chrome in spaceflight mode".to_owned(),
        })?;
    assert!(
        space.plume.visible,
        "ascending Raketa plume must be visible"
    );
    assert_eq!(space.orrery.len(), 6, "orrery must project 6 nodes");

    overlay.render(
        &gpu.device,
        &gpu.queue,
        OverlayContent::FlightChrome {
            banner: None,
            efb: None,
            space: Some(&space),
        },
        SHOT_SIZE,
    )?;
    let view = overlay.view().ok_or_else(|| AppSimError::Screenshot {
        detail: "MenuOverlay must produce a texture view for space FlightChrome".to_owned(),
    })?;
    let quads = [OverlayQuad::vector_overlay(ViewportRect::FULL, view)];
    let mut encoder = gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("verify space flight chrome"),
        });
    compositor.encode(
        &gpu.device,
        &gpu.queue,
        &mut encoder,
        CompositorTarget {
            view: &target_view,
            clear: Some(wgpu::Color::BLACK),
            size_px: SHOT_SIZE,
        },
        &quads,
    );
    gpu.queue.submit([encoder.finish()]);
    let rgba = readback_target_rgba(&gpu.device, &gpu.queue, &target)?;
    assert!(count_lit_pixels(&rgba, [18, 520, 210, 705]) > 1_000);
    assert!(count_lit_pixels(&rgba, [1050, 520, 1265, 705]) > 1_000);
    assert!(count_lit_pixels(&rgba, [18, 18, 290, 150]) > 1_000);
    gpu.check_device()?;
    Ok(())
}
