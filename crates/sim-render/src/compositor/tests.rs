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
    solid_unorm_map_texture(gpu, 100)
}

/// A `maplibre`-style `Rgba8Unorm` texture with every channel of every texel `value`.
pub(crate) fn solid_unorm_map_texture(gpu: &HeadlessGpu, value: u8) -> wgpu::TextureView {
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
    let map_bytes = [value; 32 * 32 * 4];
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

pub(crate) const TARGET_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
pub(crate) const TARGET_PX: u32 = 64;

pub(crate) fn composite_target(gpu: &HeadlessGpu) -> wgpu::Texture {
    gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("composite target"),
        size: wgpu::Extent3d {
            width: TARGET_PX,
            height: TARGET_PX,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: TARGET_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    })
}

/// The red channel of `target`'s texel at `[x, y]`.
pub(crate) fn read_red(gpu: &HeadlessGpu, target: &wgpu::Texture, [x, y]: [u32; 2]) -> u8 {
    let row_bytes = TARGET_PX * 4;
    let staging = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("composite readback"),
        size: u64::from(row_bytes) * u64::from(TARGET_PX),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        target.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(row_bytes),
                rows_per_image: Some(TARGET_PX),
            },
        },
        target.size(),
    );
    gpu.queue.submit([encoder.finish()]);
    staging.map_async(wgpu::MapMode::Read, .., |r| r.unwrap());
    gpu.device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    staging.get_mapped_range(..).unwrap()[((y * TARGET_PX + x) * 4) as usize]
}

#[test]
fn a_later_layer_of_a_submission_leaves_an_earlier_layers_quads_where_they_were() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let Ok(compositor) = pollster::block_on(CompositorPass::new(&gpu.device, TARGET_FORMAT)) else {
        return;
    };
    let target = composite_target(&gpu);
    let target_view = target.create_view(&wgpu::TextureViewDescriptor::default());
    let world = solid_unorm_map_texture(&gpu, 200);
    let inset = solid_unorm_map_texture(&gpu, 50);
    let corner = ViewportRect {
        x: 0.5,
        y: 0.0,
        width: 0.5,
        height: 0.5,
    };

    // A full-window world under a corner map, as a window's frame layers them: two
    // passes of one submission, with other passes free to draw between them.
    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    let mut layers = compositor.layers(&gpu.device, &gpu.queue);
    layers.encode(
        &mut encoder,
        CompositorTarget {
            view: &target_view,
            clear: Some(wgpu::Color::BLACK),
            size_px: [TARGET_PX; 2],
        },
        &[OverlayQuad::full_map(&world)],
    );
    layers.encode(
        &mut encoder,
        CompositorTarget {
            view: &target_view,
            clear: None,
            size_px: [TARGET_PX; 2],
        },
        &[OverlayQuad::inset_map(corner, &inset, false)],
    );
    gpu.queue.submit([encoder.finish()]);

    let outside_the_corner = read_red(&gpu, &target, [8, 56]);
    let inside_the_corner = read_red(&gpu, &target, [56, 8]);
    assert!(
        outside_the_corner.abs_diff(200) <= 2,
        "the world fills the window beside the corner map, not the clear colour: {outside_the_corner}"
    );
    assert!(
        inside_the_corner.abs_diff(50) <= 2,
        "the corner map covers its corner: {inside_the_corner}"
    );
    assert!(gpu.check_device().is_ok());
}

#[test]
fn compositor_composites_unorm_map_and_srgb_panel_without_gpu_errors() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let Ok(compositor) = pollster::block_on(CompositorPass::new(&gpu.device, TARGET_FORMAT)) else {
        return;
    };
    let target = composite_target(&gpu);
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
            size_px: [TARGET_PX; 2],
        },
        &quads,
    );
    gpu.queue.submit([encoder.finish()]);
    assert!(gpu.device.poll(wgpu::PollType::wait_indefinitely()).is_ok());
    assert!(gpu.check_device().is_ok());
}
