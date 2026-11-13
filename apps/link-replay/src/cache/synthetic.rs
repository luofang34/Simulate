//! Deterministic procedural DEM and orthoimagery tile generator for offline
//! cache tests without network access.

use std::io::Cursor;

use image::{ImageFormat, Rgb, RgbImage};
use sim_navdata::HttpAnswer;

use super::{parse_esri_zyx, parse_s2cloudless, parse_terrarium_zxy};

const TILE_SIDE_PX: u32 = 256;

/// Synthesizes a valid `256 x 256` Mapzen Terrarium elevation PNG or textured
/// orthoimagery JPEG matching `url` so offline tests can seal and fly complete
/// visual packages deterministically without touching the network.
///
/// # Errors
///
/// Returns an error string if PNG/JPEG encoding fails.
pub fn synthetic_tile_answer(url: &str) -> Result<HttpAnswer, String> {
    let clean = url.split('?').next().unwrap_or(url);
    if let Some((z, x, y)) = parse_terrarium_zxy(clean) {
        let body = encode_terrarium_png(z, x, y)?;
        return Ok(HttpAnswer { status: 200, body });
    }
    let (z, x, y) = parse_esri_zyx(clean)
        .or_else(|| parse_s2cloudless(clean))
        .unwrap_or((12, 0, 0));
    let body = encode_textured_jpeg(z, x, y)?;
    Ok(HttpAnswer { status: 200, body })
}

fn encode_terrarium_png(z: u8, tile_x: u32, tile_y: u32) -> Result<Vec<u8>, String> {
    let mut img = RgbImage::new(TILE_SIDE_PX, TILE_SIDE_PX);
    let scale = f64::from(TILE_SIDE_PX);
    for py in 0..TILE_SIDE_PX {
        for px in 0..TILE_SIDE_PX {
            let gx = (f64::from(tile_x) + f64::from(px) / scale) / f64::from(1_u32 << z.min(20));
            let gy = (f64::from(tile_y) + f64::from(py) / scale) / f64::from(1_u32 << z.min(20));
            let height_m = 220.0
                + 85.0 * (gx * 900.0).sin() * (gy * 900.0).cos()
                + 35.0 * (gx * 2_700.0 + gy * 1_900.0).sin();
            let shifted = (height_m + 32_768.0).clamp(0.0, 65_535.0);
            let r = (shifted / 256.0).floor() as u8;
            let rem = shifted - f64::from(r) * 256.0;
            let g = rem.floor() as u8;
            let b = ((rem - f64::from(g)) * 256.0).round().clamp(0.0, 255.0) as u8;
            img.put_pixel(px, py, Rgb([r, g, b]));
        }
    }
    let mut bytes = Vec::new();
    img.write_to(&mut Cursor::new(&mut bytes), ImageFormat::Png)
        .map_err(|e| format!("synthetic Terrarium PNG encode failed: {e}"))?;
    Ok(bytes)
}

fn encode_textured_jpeg(z: u8, tile_x: u32, tile_y: u32) -> Result<Vec<u8>, String> {
    let mut img = RgbImage::new(TILE_SIDE_PX, TILE_SIDE_PX);
    let base_x = u64::from(tile_x).wrapping_mul(u64::from(TILE_SIDE_PX));
    let base_y = u64::from(tile_y).wrapping_mul(u64::from(TILE_SIDE_PX));
    for py in 0..TILE_SIDE_PX {
        for px in 0..TILE_SIDE_PX {
            let wx = base_x.wrapping_add(u64::from(px)) as f64;
            let wy = base_y.wrapping_add(u64::from(py)) as f64;
            // Multi-scale non-repeating spatial features for visual correlation.
            let coarse = ((wx * 0.041 + f64::from(z)).sin() * (wy * 0.037).cos()) * 55.0;
            let medium = ((wx * 0.173 - wy * 0.129).sin()) * 38.0;
            let fine = ((wx * 0.61 + wy * 0.47).cos()) * 24.0;
            let hash = mix_u64(
                base_x
                    .wrapping_add(u64::from(px) / 4)
                    .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                    ^ base_y.wrapping_add(u64::from(py) / 4),
            );
            let patch = f64::from((hash & 0x3F) as u8) - 32.0;
            let luma = (128.0 + coarse + medium + fine + patch).clamp(18.0, 242.0);
            let r = (luma * 1.03).clamp(0.0, 255.0) as u8;
            let g = (luma * 0.99).clamp(0.0, 255.0) as u8;
            let b = (luma * 0.92).clamp(0.0, 255.0) as u8;
            img.put_pixel(px, py, Rgb([r, g, b]));
        }
    }
    let mut bytes = Vec::new();
    img.write_to(&mut Cursor::new(&mut bytes), ImageFormat::Jpeg)
        .map_err(|e| format!("synthetic imagery JPEG encode failed: {e}"))?;
    Ok(bytes)
}

const fn mix_u64(mut x: u64) -> u64 {
    x ^= x >> 30;
    x = x.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x ^= x >> 27;
    x = x.wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}
