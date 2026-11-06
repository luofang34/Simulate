use super::{VectorScenePass, VectorTarget, VectorVertex};
use crate::{HeadlessGpu, RenderError, padded_bytes_per_row};

const SIZE: u32 = 32;
const FULL_CLIP: [f32; 4] = [0.0, 0.0, SIZE as f32, SIZE as f32];

fn vertex(pos_px: [f32; 2], color_premul: [f32; 4], clip: [f32; 4], depth: f32) -> VectorVertex {
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
        out.push(vertex(p, color, clip, depth));
    }
}

fn read_back(gpu: &HeadlessGpu, target: &VectorTarget) -> Vec<u8> {
    let [w, h] = target.size_px();
    let padded = padded_bytes_per_row(w, 4).unwrap();
    let staging = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("vector readback"),
        size: u64::from(padded) * u64::from(h),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        target.texture().as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(h),
            },
        },
        wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
    );
    gpu.queue.submit([encoder.finish()]);
    staging.map_async(wgpu::MapMode::Read, .., |r| r.unwrap());
    gpu.device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    let bytes = staging.get_mapped_range(..).unwrap().to_vec();
    bytes
        .chunks_exact(padded as usize)
        .flat_map(|row| row.iter().take((w * 4) as usize).copied())
        .collect()
}

fn alpha(pixels: &[u8], x: u32, y: u32) -> u8 {
    pixels[((y * SIZE + x) * 4 + 3) as usize]
}

fn draw(gpu: &HeadlessGpu, pass: &mut VectorScenePass, target: &VectorTarget, v: &[VectorVertex]) {
    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    pass.encode(&gpu.device, &gpu.queue, &mut encoder, target, v)
        .unwrap();
    gpu.queue.submit([encoder.finish()]);
}

#[test]
fn target_rejects_zero_size() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let res = VectorTarget::new(&gpu.device, "zero", 0, 8);
    assert!(matches!(
        res,
        Err(RenderError::EmptyTexture { width: 0, .. })
    ));
    let mut target = VectorTarget::new(&gpu.device, "ok", 8, 8).unwrap();
    assert!(target.ensure_size(&gpu.device, 8, 0).is_err());
    target.ensure_size(&gpu.device, 12, 6).unwrap();
    assert_eq!(target.size_px(), [12, 6]);
}

#[test]
fn vector_pass_antialiases_clips_blends_once_and_clears() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let mut pass = pollster::block_on(VectorScenePass::new(&gpu.device)).unwrap();
    let target = VectorTarget::new(&gpu.device, "vector test", SIZE, SIZE).unwrap();
    let white = [1.0, 1.0, 1.0, 1.0];
    let half = [0.5, 0.5, 0.5, 0.5];
    let mut v = vec![
        vertex([0.0, 0.0], white, FULL_CLIP, 0.1),
        vertex([16.0, 0.0], white, FULL_CLIP, 0.1),
        vertex([0.0, 16.0], white, FULL_CLIP, 0.1),
    ];
    quad(
        &mut v,
        [20.0, 0.0, 30.0, 10.0],
        white,
        [20.0, 0.0, 25.0, 10.0],
        0.2,
    );
    // The same operation covering its area twice must blend once...
    quad(&mut v, [0.0, 20.0, 10.0, 30.0], half, FULL_CLIP, 0.3);
    quad(&mut v, [0.0, 20.0, 10.0, 30.0], half, FULL_CLIP, 0.3);
    // ...while a later operation still composites over it.
    quad(&mut v, [5.0, 20.0, 10.0, 30.0], half, FULL_CLIP, 0.4);
    draw(&gpu, &mut pass, &target, &v);
    let px = read_back(&gpu, &target);
    gpu.check_device().unwrap();

    assert_eq!(alpha(&px, 2, 2), 255);
    assert_eq!(alpha(&px, 14, 14), 0);
    let edge: Vec<u8> = (0..16).map(|x| alpha(&px, x, 15 - x)).collect();
    assert!(
        edge.iter().any(|&a| (40..=215).contains(&a)),
        "diagonal edge has no partial coverage: {edge:?}"
    );
    assert_eq!(alpha(&px, 22, 5), 255);
    assert_eq!(alpha(&px, 27, 5), 0, "clip rect must discard");
    assert!(
        (alpha(&px, 2, 25) as i32 - 128).abs() <= 2,
        "double cover blended twice"
    );
    assert!(
        (alpha(&px, 7, 25) as i32 - 191).abs() <= 2,
        "later operation lost"
    );

    draw(&gpu, &mut pass, &target, &[]);
    let cleared = read_back(&gpu, &target);
    assert!(cleared.iter().all(|&b| b == 0), "empty draw must clear");
}
