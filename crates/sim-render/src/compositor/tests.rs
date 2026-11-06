use super::{CompositorPass, CompositorTarget, OverlayQuad, RgbaPanelTexture, ViewportRect};
use crate::{HeadlessGpu, RenderError};

#[test]
fn rgba_panel_rejects_mismatched_upload_length() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let Ok(panel) = RgbaPanelTexture::new(&gpu.device, "test panel", 4, 4) else {
        return;
    };
    let bad = [0_u8; 12];
    let res = panel.upload(&gpu.queue, &bad);
    assert!(matches!(
        res,
        Err(RenderError::InvalidUploadBytes {
            expected: 64,
            actual: 12
        })
    ));
}

fn create_test_unorm_map_texture(gpu: &HeadlessGpu) -> wgpu::TextureView {
    let map_tex = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("map unorm"),
        size: wgpu::Extent3d {
            width: 32,
            height: 32,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let map_bytes = [100_u8; 32 * 32 * 4];
    gpu.queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &map_tex,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &map_bytes,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(32 * 4),
            rows_per_image: Some(32),
        },
        wgpu::Extent3d {
            width: 32,
            height: 32,
            depth_or_array_layers: 1,
        },
    );
    map_tex.create_view(&wgpu::TextureViewDescriptor::default())
}

#[test]
fn compositor_composites_unorm_map_and_srgb_panel_without_gpu_errors() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let target_format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let Ok(compositor) = pollster::block_on(CompositorPass::new(&gpu.device, target_format)) else {
        return;
    };

    let target = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("composite target"),
        size: wgpu::Extent3d {
            width: 64,
            height: 64,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: target_format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let target_view = target.create_view(&wgpu::TextureViewDescriptor::default());
    let map_view = create_test_unorm_map_texture(&gpu);

    let Ok(panel) = RgbaPanelTexture::new(&gpu.device, "pfd", 16, 16) else {
        return;
    };
    assert!(panel.upload(&gpu.queue, &[200_u8; 16 * 16 * 4]).is_ok());

    let quads = [
        OverlayQuad::full_map(&map_view),
        OverlayQuad::panel(
            ViewportRect {
                x: 0.1,
                y: 0.5,
                width: 0.4,
                height: 0.4,
            },
            panel.view(),
            false,
            true,
        ),
        OverlayQuad::vector_overlay(ViewportRect::FULL, panel.view()),
    ];

    let mut encoder = gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("test composite"),
        });
    compositor.encode(
        &gpu.device,
        &gpu.queue,
        &mut encoder,
        CompositorTarget {
            view: &target_view,
            clear: Some(wgpu::Color::BLACK),
            size_px: [64, 64],
        },
        &quads,
    );
    gpu.queue.submit([encoder.finish()]);
    assert!(gpu.device.poll(wgpu::PollType::wait_indefinitely()).is_ok());
    assert!(gpu.check_device().is_ok());
}
