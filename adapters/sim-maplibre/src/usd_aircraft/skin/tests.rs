use super::{SkinFormat, SkinMap};

#[test]
fn mips_run_down_to_one_texel() {
    let map = SkinMap::with_mips(SkinFormat::Srgba8, 8, 4, vec![200; 8 * 4 * 4]);
    let sizes: Vec<usize> = map.levels().iter().map(Vec::len).collect();
    assert_eq!(sizes, [128, 32, 8, 4]);
    assert_eq!(map.byte_len(), 172);
}

#[test]
fn srgb_mips_average_in_linear_light() {
    // Black and white texels average to half the light, which sRGB encodes
    // near 188, not to the byte midpoint.
    let pixels = [[0, 0, 0, 255], [255, 255, 255, 255]].repeat(2).concat();
    let map = SkinMap::with_mips(SkinFormat::Srgba8, 2, 2, pixels);
    let coarse = map.levels().last().cloned().unwrap_or_default();
    assert_eq!(coarse, [188, 188, 188, 255]);
}

#[test]
fn normal_mips_shorten_toward_flat() {
    // Two opposite tilts average to a flat normal.
    let map = SkinMap::with_mips(SkinFormat::Normal8, 2, 1, vec![255, 128, 0, 128]);
    assert_eq!(map.levels().last().cloned().unwrap_or_default(), [128, 128]);
}

#[test]
fn oversized_maps_start_at_a_mip_that_fits() {
    use crate::usd_pass::first_fitting_level;
    assert_eq!(first_fitting_level(4096, 4096, 8192), 0);
    assert_eq!(first_fitting_level(4096, 4096, 2048), 1);
    assert_eq!(first_fitting_level(4096, 1024, 1024), 2);
}
