use super::*;

#[test]
fn decodes_the_terrarium_formula() {
    assert_eq!(decode_terrarium([128, 0, 0]), 0.0);
    assert_eq!(decode_terrarium([128, 1, 128]), 1.5);
    assert_eq!(decode_terrarium([0, 0, 0]), -32_768.0);
    assert_eq!(
        decode_terrarium([117, 72, 128]),
        117.0 * 256.0 + 72.5 - 32_768.0
    );
    assert_eq!(decode_terrarium([255, 255, 255]), 32_767.0 + 255.0 / 256.0);
}

#[test]
fn encoding_round_trips_to_the_format_step() {
    for height in [-430.25, 0.0, 1.5, 8_848.86, 31.337] {
        let decoded = decode_terrarium(encode_terrarium(height));
        assert!(
            (decoded - height).abs() <= 1.0 / 512.0,
            "{height} {decoded}"
        );
    }
}

#[test]
fn encoding_saturates_outside_the_format() {
    assert_eq!(encode_terrarium(-40_000.0), [0, 0, 0]);
    assert_eq!(encode_terrarium(40_000.0), [255, 255, 255]);
}

#[test]
fn rgb_and_rgba_decode_alike() {
    let rgb = [128, 1, 128, 128, 0, 0];
    let rgba = [128, 1, 128, 7, 128, 0, 0, 9];
    let a = TerrariumTile::from_rgb(2, 1, &rgb).unwrap();
    let b = TerrariumTile::from_rgba(2, 1, &rgba).unwrap();
    assert_eq!(a, b);
    assert_eq!(a.texel(0, 0), Some(1.5));
    assert_eq!(a.texel(1, 0), Some(0.0));
    assert_eq!(a.texel(2, 0), None);
}

#[test]
fn rejects_pixels_that_do_not_fill_the_raster() {
    let short = TerrariumTile::from_rgb(2, 2, &[0; 11]).unwrap_err();
    assert_eq!(
        short,
        SceneryError::InvalidRaster {
            width: 2,
            height: 2,
            expected: 12,
            actual: 11
        }
    );
    assert!(TerrariumTile::from_heights(0, 0, Vec::new()).is_err());
}

#[test]
fn bilinear_interpolates_between_pixel_centres() {
    let tile = TerrariumTile::from_heights(2, 2, vec![0.0, 10.0, 20.0, 30.0]).unwrap();
    // Pixel centres are at fractions 0.25 and 0.75.
    assert_eq!(tile.sample_bilinear(0.25, 0.25), Some(0.0));
    assert_eq!(tile.sample_bilinear(0.75, 0.75), Some(30.0));
    assert_eq!(tile.sample_bilinear(0.5, 0.25), Some(5.0));
    assert_eq!(tile.sample_bilinear(0.25, 0.5), Some(10.0));
    assert_eq!(tile.sample_bilinear(0.5, 0.5), Some(15.0));
}

#[test]
fn bilinear_holds_the_edge_beyond_the_outer_centres() {
    let tile = TerrariumTile::from_heights(2, 2, vec![0.0, 10.0, 20.0, 30.0]).unwrap();
    assert_eq!(tile.sample_bilinear(0.0, 0.0), Some(0.0));
    assert_eq!(tile.sample_bilinear(1.0, 1.0), Some(30.0));
}

#[test]
fn f32_raster_storage_preserves_every_terrarium_step_bit_exact() {
    let samples = [
        [0, 0, 0],
        [0, 0, 1],
        [117, 72, 128],
        [127, 255, 255],
        [128, 0, 0],
        [128, 0, 1],
        [162, 144, 220],
        [255, 255, 254],
        [255, 255, 255],
    ];
    let mut rgb = Vec::with_capacity(samples.len() * 3);
    for px in samples {
        rgb.extend_from_slice(&px);
    }
    let tile = TerrariumTile::from_rgb(samples.len() as u32, 1, &rgb).unwrap();
    for (idx, px) in samples.iter().enumerate() {
        assert_eq!(tile.texel(idx as u32, 0), Some(decode_terrarium(*px)));
    }
}
