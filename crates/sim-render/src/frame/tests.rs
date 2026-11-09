use super::*;

/// WebGL2's guaranteed uniform-buffer size.
const WEBGL2_UNIFORM_BYTES: usize = 16 * 1024;

#[test]
fn the_uniform_block_fits_webgl2() {
    assert!(FRAME_UNIFORM_FLOATS * size_of::<f32>() <= WEBGL2_UNIFORM_BYTES);
}

#[test]
fn offsets_keep_their_precision_far_from_the_origin() {
    let eye = DVec3::new(6_378_137.25, 0.0, 0.0);
    let centre = DVec3::new(6_378_137.75, 1.5, -2.5);
    let params = FrameParams {
        view_proj: Mat4::IDENTITY,
        sun_direction: DVec3::Z * 3.0,
        thermal: [1.0, 2.0, 3.0, 4.0],
        exposure: 0.5,
        fog: [5.0, 6.0, 7.0, 8.0],
    };
    let mut block = Box::new([9.0_f32; FRAME_UNIFORM_FLOATS]);
    write_frame_uniform(&mut block, &params, eye, &[centre]);
    assert_eq!(
        block[16..28],
        [0.0, 0.0, 1.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    );
    assert_eq!(block[28..32], [0.5, 1.5, -2.5, 0.0]);
    assert_eq!(block[32..36], [0.0; 4], "unused slots are cleared");
}
