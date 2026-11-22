use super::{
    BACKDROP_DEPTH, BEZEL_INNER_DEPTH, BEZEL_OUTER_DEPTH, DirectVectorDraw, DirectVectorPass,
    DirectVectorTarget, clamp_rect_px, pack_draws,
};
use crate::{HeadlessGpu, VectorVertex, padded_bytes_per_row};

const SIZE: u32 = 32;

fn vert(pos_px: [f32; 2], color_premul: [f32; 4], clip: [f32; 4], depth: f32) -> VectorVertex {
    VectorVertex {
        pos_px,
        color_premul,
        clip_rect_px: clip,
        depth,
    }
}

fn quad(
    out: &mut Vec<VectorVertex>,
    [x0, y0, x1, y1]: [f32; 4],
    color: [f32; 4],
    clip: [f32; 4],
    depth: f32,
) {
    for p in [[x0, y0], [x1, y0], [x0, y1], [x0, y1], [x1, y0], [x1, y1]] {
        out.push(vert(p, color, clip, depth));
    }
}

fn readback_rgba(gpu: &HeadlessGpu, tex: &wgpu::Texture, size: u32) -> Vec<u8> {
    let padded = padded_bytes_per_row(size, 4).unwrap();
    let staging = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("direct readback"),
        size: u64::from(padded) * u64::from(size),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut enc = gpu.device.create_command_encoder(&Default::default());
    enc.copy_texture_to_buffer(
        tex.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(size),
            },
        },
        wgpu::Extent3d {
            width: size,
            height: size,
            depth_or_array_layers: 1,
        },
    );
    gpu.queue.submit([enc.finish()]);
    staging.map_async(wgpu::MapMode::Read, .., |r| r.unwrap());
    gpu.device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    staging
        .get_mapped_range(..)
        .unwrap()
        .chunks_exact(padded as usize)
        .flat_map(|row| row.iter().take((size * 4) as usize).copied())
        .collect()
}

fn make_target_textures(
    device: &wgpu::Device,
    fmt: wgpu::TextureFormat,
) -> (wgpu::Texture, wgpu::TextureView, wgpu::TextureView) {
    let extent = wgpu::Extent3d {
        width: SIZE,
        height: SIZE,
        depth_or_array_layers: 1,
    };
    let color = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("direct color"),
        size: extent,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: fmt,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let depth = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("direct depth"),
        size: extent,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: crate::gpu::DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let cv = color.create_view(&wgpu::TextureViewDescriptor::default());
    let dv = depth.create_view(&wgpu::TextureViewDescriptor::default());
    (color, cv, dv)
}

#[test]
fn direct_vector_packing_enforces_depth_order_and_clamps_rects() {
    assert_eq!(
        clamp_rect_px([10, 20, 100, 80], [90, 90]),
        Some([10, 20, 80, 70])
    );
    assert_eq!(clamp_rect_px([90, 20, 40, 40], [90, 90]), None);
    assert_eq!(clamp_rect_px([10, 20, 0, 40], [90, 90]), None);

    // Indicate paint ops start at 1 / 2^24; backdrop sits below and bezel sits above.
    const FIRST_INDICATE_DEPTH: f32 = 1.0 / 16_777_216.0;
    const {
        assert!(BACKDROP_DEPTH > 0.0);
        assert!(BACKDROP_DEPTH < FIRST_INDICATE_DEPTH);
        assert!(BEZEL_OUTER_DEPTH > 0.99);
        assert!(BEZEL_INNER_DEPTH > BEZEL_OUTER_DEPTH);
    }

    let op = [
        vert(
            [4.0, 4.0],
            [1.0, 1.0, 1.0, 1.0],
            [0.0, 0.0, 40.0, 30.0],
            FIRST_INDICATE_DEPTH,
        ),
        vert(
            [24.0, 4.0],
            [1.0, 1.0, 1.0, 1.0],
            [0.0, 0.0, 40.0, 30.0],
            FIRST_INDICATE_DEPTH,
        ),
        vert(
            [4.0, 24.0],
            [1.0, 1.0, 1.0, 1.0],
            [0.0, 0.0, 40.0, 30.0],
            FIRST_INDICATE_DEPTH,
        ),
    ];
    let draws = [
        DirectVectorDraw {
            rect_px: [100, 100, 20, 20],
            vertices: &[],
            opaque_backdrop: None,
            bezel: false,
        },
        DirectVectorDraw {
            rect_px: [20, 30, 50, 40],
            vertices: &op,
            opaque_backdrop: Some([0.0, 0.0, 0.0, 1.0]),
            bezel: true,
        },
    ];
    let mut scratch = Vec::new();
    let prepared = pack_draws(&draws, [60, 60], &mut scratch);
    assert_eq!(prepared.len(), 1, "empty-draw and out-of-bounds skipped");
    assert_eq!(prepared[0].rect_px, [20, 30, 40, 30]);
    assert_eq!(prepared[0].count, 6 + 3 + 48);
    assert_eq!(scratch[0].depth, BACKDROP_DEPTH);
    assert_eq!(scratch[6].depth, FIRST_INDICATE_DEPTH);
    assert_eq!(scratch[scratch.len() - 1].depth, BEZEL_INNER_DEPTH);
}

#[test]
fn direct_vector_pass_draws_clipped_offset_subrect_on_gpu() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let fmt = wgpu::TextureFormat::Rgba8UnormSrgb;
    let mut direct =
        pollster::block_on(DirectVectorPass::new(&gpu.device, fmt)).expect("direct pass");
    let (color, color_view, depth_view) = make_target_textures(&gpu.device, fmt);
    let mut v = Vec::new();
    quad(
        &mut v,
        [2.0, 2.0, 12.0, 12.0],
        [1.0; 4],
        [0.0, 0.0, 10.0, 10.0],
        0.1,
    );
    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    let _clear = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some("clear"),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view: &color_view,
            depth_slice: None,
            resolve_target: None,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                store: wgpu::StoreOp::Store,
            },
        })],
        depth_stencil_attachment: None,
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    });
    drop(_clear);
    direct
        .encode(
            &gpu.device,
            &gpu.queue,
            &mut encoder,
            DirectVectorTarget {
                color_view: &color_view,
                depth_view: &depth_view,
                size_px: [SIZE, SIZE],
            },
            &[DirectVectorDraw {
                rect_px: [10, 10, 16, 16],
                vertices: &v,
                opaque_backdrop: Some([0.0, 0.0, 0.0, 1.0]),
                bezel: false,
            }],
        )
        .unwrap();
    gpu.queue.submit([encoder.finish()]);
    let px = readback_rgba(&gpu, &color, SIZE);
    gpu.check_device().unwrap();
    let rgb = |x: u32, y: u32| {
        let i = ((y * SIZE + x) * 4) as usize;
        [px[i], px[i + 1], px[i + 2], px[i + 3]]
    };
    assert_eq!(rgb(2, 2)[3], 0, "outside subrect stays untouched");
    assert_eq!(rgb(11, 11), [0, 0, 0, 255], "backdrop covers subrect");
    assert_eq!(
        rgb(15, 15),
        [255, 255, 255, 255],
        "clipped interior draws white"
    );
    assert_eq!(
        rgb(21, 15),
        [0, 0, 0, 255],
        "panel-local clip discards >= 10"
    );
}
