use sim_geo::{BodyEllipsoid, LocalFrame};
use sim_scenery::tile_point_to_geodetic;

use super::{
    HEIGHT, NEAR_M, WIDTH, draped_mesh, gpu, intrinsics, nadir, sun_overhead, tile_scene, visible,
};
use crate::aperture::{OffscreenAperture, SensorAperturePass};
use crate::{ApertureImage, CameraPose, OrthoAtlas, PinholeProjection};

#[test]
fn sub_frames_across_a_moving_exposure_blur_the_tile_edge() {
    let Some(gpu) = gpu() else { return };
    let (_, east, _, catalog, footprint) = tile_scene();
    let atlas = OrthoAtlas::from_source(&catalog, footprint, 4).unwrap();
    let body = BodyEllipsoid::WGS84;
    let edge = tile_point_to_geodetic(east, 0.0, 0.5).unwrap();
    let mesh = draped_mesh(body, &edge, &footprint);
    // From 20 km at fx = 800 a pixel is 25 m; flying 1 km east during the
    // exposure sweeps the edge across 40 pixels.
    let middle = nadir(&body, &edge, 20_000.0);
    let east_m = LocalFrame::at(&edge).east() * 500.0;
    let shifted = |offset| CameraPose {
        position_ecef: middle.position_ecef + offset,
        ..middle
    };
    let (start, end) = (shifted(-east_m), shifted(east_m));
    let projection = PinholeProjection::new(intrinsics(800.0), NEAR_M).unwrap();
    let target = OffscreenAperture::new(&gpu.device, projection).unwrap();
    let mut pass = pollster::block_on(SensorAperturePass::new(
        &gpu.device,
        &gpu.queue,
        &mesh,
        Some(&atlas),
    ))
    .unwrap();
    let sub_frames: Vec<ApertureImage> = start
        .exposure_sub_poses(&end, 40)
        .into_iter()
        .map(|pose| {
            pass.render(
                &gpu.device,
                &gpu.queue,
                &target,
                &visible(pose, sun_overhead(&edge)),
            );
            target.read_blocking(&gpu.device, &gpu.queue).unwrap()
        })
        .collect();
    gpu.check_device().unwrap();
    let blurred = ApertureImage::blend_exposure(&sub_frames).unwrap();
    let at = |dx: i32| {
        blurred
            .pixel((WIDTH as i32 / 2 + dx) as u32, HEIGHT / 2)
            .unwrap()
    };
    let (red, blue) = (|p: [u8; 4]| p[0], |p: [u8; 4]| p[2]);
    // Far from the edge the tiles keep their colours; across the swept band
    // red gives way to blue gradually instead of in one step.
    assert!(red(at(-40)) > 180 && blue(at(-40)) < 90, "{:?}", at(-40));
    assert!(blue(at(40)) > 180 && red(at(40)) < 90, "{:?}", at(40));
    let reds: Vec<u8> = (-15..=15).step_by(5).map(|dx| red(at(dx))).collect();
    assert!(reds.windows(2).all(|pair| pair[0] >= pair[1]), "{reds:?}");
    assert!(
        reds.iter().filter(|&&r| (90..180).contains(&r)).count() >= 3,
        "{reds:?}"
    );
}
