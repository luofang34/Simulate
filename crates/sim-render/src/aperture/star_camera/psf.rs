use std::f64::consts::SQRT_2;

/// Half-width of a star's stamp in standard deviations: beyond 4σ a Gaussian
/// carries 6e-5 of its light per axis, below one electron for any star under
/// the full well.
const STAMP_HALF_WIDTH_SIGMA: f64 = 4.0;

/// The error function, Abramowitz and Stegun 7.1.26: absolute error under
/// 1.5e-7, far below the centroid tolerance it serves.
pub(super) fn erf(x: f64) -> f64 {
    const P: f64 = 0.327_591_1;
    const A: [f64; 5] = [
        0.254_829_592,
        -0.284_496_736,
        1.421_413_741,
        -1.453_152_027,
        1.061_405_429,
    ];
    let magnitude = x.abs();
    let t = 1.0 / (1.0 + P * magnitude);
    let polynomial = A.iter().rev().fold(0.0, |sum, a| sum * t + a) * t;
    let value = 1.0 - polynomial * (-magnitude * magnitude).exp();
    value.copysign(x)
}

/// The fractions of a Gaussian of `sigma` centred at `centre` that land in
/// each pixel along one axis, pixel `i` spanning `[i − 0.5, i + 0.5]`. Gives
/// the first pixel and the fractions, clipped to `0..extent`; `None` when the
/// stamp misses the detector.
pub(super) fn axis_weights(centre: f64, sigma: f64, extent: u32) -> Option<(u32, Vec<f64>)> {
    if !centre.is_finite() {
        return None;
    }
    let reach = STAMP_HALF_WIDTH_SIGMA * sigma;
    let first = (centre - reach).round().max(0.0);
    let last = (centre + reach).round().min(f64::from(extent) - 1.0);
    if first > last {
        return None;
    }
    let scale = 1.0 / (sigma * SQRT_2);
    let cumulative = |edge: f64| 0.5 * erf((edge - centre) * scale);
    let weights = (first as u32..=last as u32)
        .map(|pixel| {
            let middle = f64::from(pixel);
            cumulative(middle + 0.5) - cumulative(middle - 0.5)
        })
        .collect();
    Some((first as u32, weights))
}

/// Adds `electrons` spread by a Gaussian PSF centred at pixel coordinate
/// `centre` into a row-major `image` of `width` × `height`.
pub(super) fn add_star(
    image: &mut [f64],
    [width, height]: [u32; 2],
    centre: [f64; 2],
    sigma: f64,
    electrons: f64,
) {
    let [x, y] = centre;
    let (Some((first_x, columns)), Some((first_y, rows))) = (
        axis_weights(x, sigma, width),
        axis_weights(y, sigma, height),
    ) else {
        return;
    };
    let width = width as usize;
    let first_x = first_x as usize;
    for (row, row_weight) in image
        .chunks_exact_mut(width)
        .skip(first_y as usize)
        .zip(rows)
    {
        let row_electrons = electrons * row_weight;
        for (pixel, column_weight) in row.iter_mut().skip(first_x).zip(&columns) {
            *pixel += row_electrons * column_weight;
        }
    }
}
