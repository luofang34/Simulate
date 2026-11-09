use glam::DVec3;

use super::*;
use crate::CameraPose;

fn grey(width: u32, levels: &[u8], depth: f32) -> ApertureImage {
    ApertureImage {
        width,
        height: 1,
        rgba8: levels.iter().flat_map(|&g| [g, g, g, 255]).collect(),
        depth: vec![depth; levels.len()],
    }
}

#[test]
fn one_sub_frame_is_returned_unchanged() {
    let frame = grey(4, &[0, 17, 128, 255], 0.25);
    assert_eq!(
        ApertureImage::blend_exposure(std::slice::from_ref(&frame)).unwrap(),
        frame
    );
}

#[test]
fn sub_frames_average_in_linear_light() {
    let black = grey(1, &[0], 0.1);
    let white = grey(1, &[255], 0.3);
    let blended = ApertureImage::blend_exposure(&[black, white.clone(), white]).unwrap();
    // Two thirds linear white encodes to sRGB 0.836, level 213; averaging the
    // encoded bytes would give 170.
    assert_eq!(blended.pixel(0, 0), Some([213, 213, 213, 255]));
    assert_eq!(blended.depth, vec![0.3], "middle sub-frame's depth");
}

#[test]
fn a_moving_edge_blurs_into_a_ramp() {
    // A bright edge steps one pixel right in each of four sub-frames.
    let frames: Vec<_> = (0..4)
        .map(|shift| {
            let levels: Vec<u8> = (0..8)
                .map(|x| if x < 2 + shift { 255 } else { 0 })
                .collect();
            grey(8, &levels, 0.0)
        })
        .collect();
    let blended = ApertureImage::blend_exposure(&frames).unwrap();
    let row: Vec<u8> = (0..8).map(|x| blended.pixel(x, 0).unwrap()[0]).collect();
    assert_eq!(row[..2], [255, 255]);
    assert!(row[2] > row[3] && row[3] > row[4] && row[4] > row[5]);
    assert_eq!(row[6..], [0, 0]);
}

#[test]
fn mismatched_or_missing_sub_frames_are_refused() {
    assert!(ApertureImage::blend_exposure(&[]).is_err());
    let small = grey(2, &[0, 0], 0.0);
    let large = grey(3, &[0, 0, 0], 0.0);
    assert!(ApertureImage::blend_exposure(&[small, large]).is_err());
}

#[test]
fn sub_poses_sit_at_the_middles_of_equal_slices() {
    let start = CameraPose::looking(DVec3::new(1.0, 0.0, 0.0), DVec3::X, DVec3::Z);
    let end = CameraPose::looking(DVec3::new(5.0, 0.0, 0.0), DVec3::Y, DVec3::Z);
    let poses = start.exposure_sub_poses(&end, 4);
    let xs: Vec<f64> = poses.iter().map(|pose| pose.position_ecef.x).collect();
    assert_eq!(xs, [1.5, 2.5, 3.5, 4.5]);
    let turn = |pose: &CameraPose| pose.orientation.angle_between(start.orientation);
    let total = end.orientation.angle_between(start.orientation);
    assert!((turn(&poses[0]) - total / 8.0).abs() < 1e-12);
    assert!((turn(&poses[3]) - total * 7.0 / 8.0).abs() < 1e-12);
}
