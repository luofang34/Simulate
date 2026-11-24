use super::layers::{FrameLayers, encode_frame_layers};
use crate::compositor::tests::{
    TARGET_FORMAT, TARGET_PX, composite_target, read_red, solid_unorm_map_texture,
};
use crate::vector_pass::{DirectVectorPass, DirectVectorTarget};
use crate::{CompositorPass, HeadlessGpu, OverlayQuad, ViewportRect};

#[test]
fn a_frames_corner_map_leaves_the_world_under_it_full_window() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let (Ok(compositor), Ok(mut direct_vector)) = (
        pollster::block_on(CompositorPass::new(&gpu.device, TARGET_FORMAT)),
        pollster::block_on(DirectVectorPass::new(&gpu.device, TARGET_FORMAT)),
    ) else {
        return;
    };
    let target = composite_target(&gpu);
    let color_view = target.create_view(&wgpu::TextureViewDescriptor::default());
    let depth_view = gpu
        .device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("frame depth"),
            size: target.size(),
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: crate::gpu::DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default());
    let world = solid_unorm_map_texture(&gpu, 200);
    let corner_map = solid_unorm_map_texture(&gpu, 50);
    let corner = ViewportRect {
        x: 0.5,
        y: 0.0,
        width: 0.5,
        height: 0.5,
    };

    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    encode_frame_layers(
        &compositor,
        &mut direct_vector,
        (&gpu.device, &gpu.queue),
        &mut encoder,
        DirectVectorTarget {
            color_view: &color_view,
            depth_view: &depth_view,
            size_px: [TARGET_PX; 2],
        },
        &FrameLayers {
            clear: Some(wgpu::Color::BLACK),
            under_quads: &[OverlayQuad::full_map(&world)],
            direct_vectors: &[],
            over_quads: &[OverlayQuad::inset_map(corner, &corner_map, false)],
        },
    )
    .unwrap();
    gpu.queue.submit([encoder.finish()]);

    let beside_the_map = read_red(&gpu, &target, [8, 56]);
    let on_the_map = read_red(&gpu, &target, [56, 8]);
    assert!(
        beside_the_map.abs_diff(200) <= 2,
        "the world, not the clear colour, fills the window beside the corner map: {beside_the_map}"
    );
    assert!(
        on_the_map.abs_diff(50) <= 2,
        "the corner map covers its corner: {on_the_map}"
    );
    assert!(gpu.check_device().is_ok());
}
