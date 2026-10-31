use super::*;

#[test]
fn rows_pad_to_the_copy_alignment() {
    assert_eq!(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT, 256);
    assert_eq!(padded_bytes_per_row(800, 4), Some(3328));
    assert_eq!(padded_bytes_per_row(1600, 4), Some(6400));
    assert_eq!(padded_bytes_per_row(1, 4), Some(256));
    assert_eq!(padded_bytes_per_row(64, 4), Some(256));
    assert_eq!(padded_bytes_per_row(65, 4), Some(512));
    assert_eq!(padded_bytes_per_row(u32::MAX, 4), None);
}

#[test]
fn padding_is_removed_from_every_row() {
    let padded = [1, 2, 0, 0, 3, 4, 0, 0];
    assert_eq!(strip_row_padding(&padded, 4, 2), [1, 2, 3, 4]);
}

#[test]
fn images_address_their_pixels() {
    let image = ApertureImage {
        width: 2,
        height: 2,
        rgba8: (0..16).collect(),
        depth: vec![0.0, 0.25, 0.5, 0.75],
    };
    assert_eq!(image.pixel(1, 1), Some([12, 13, 14, 15]));
    assert_eq!(image.pixel(2, 0), None);
    assert_eq!(image.depth_at(0, 1), Some(0.5));
    assert_eq!(image.depth_at(0, 2), None);
}
