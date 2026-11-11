//! The flight seen from above: a Web Mercator box over the route, and the
//! camera's world drawn into it as a grey picture for the page's track map.
//! Web Mercator is the imagery's own projection, so the picture is the
//! tiles as they are, and north is up.

use std::f64::consts::PI;

use sim_geo::Geodetic;

use super::CameraWorld;

/// How much the picture's grey is dimmed, so the tracks drawn over it
/// stand out.
const DIM: f64 = 0.6;
/// The colour where the package has no imagery, RGBA.
const UNCOVERED: [u8; 4] = [11, 14, 23, 255];

/// A box on the Web Mercator plane, in units of the whole plane: `x` from
/// 0 at 180° west eastward, `y` from 0 at the north edge southward.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapView {
    /// West edge.
    pub x0: f64,
    /// North edge.
    pub y0: f64,
    /// East edge.
    pub x1: f64,
    /// South edge.
    pub y1: f64,
}

/// The Web Mercator point of `lat_deg`, `lon_deg`.
pub fn mercator(lat_deg: f64, lon_deg: f64) -> (f64, f64) {
    let lat = lat_deg.clamp(-85.0, 85.0).to_radians();
    let x = (lon_deg + 180.0) / 360.0;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / PI) / 2.0;
    (x, y)
}

/// The latitude and longitude, degrees, of the Web Mercator point `x`, `y`.
pub fn unmercator(x: f64, y: f64) -> (f64, f64) {
    let lat = (PI * (1.0 - 2.0 * y)).sinh().atan();
    (lat.to_degrees(), x * 360.0 - 180.0)
}

impl MapView {
    /// The smallest box holding every point, grown by `margin` of its
    /// larger side on each edge; `None` without a point.
    pub fn around(points: impl IntoIterator<Item = (f64, f64)>, margin: f64) -> Option<Self> {
        let mut view: Option<Self> = None;
        for (lat, lon) in points {
            let (x, y) = mercator(lat, lon);
            if !(x.is_finite() && y.is_finite()) {
                continue;
            }
            view = Some(view.map_or(
                Self {
                    x0: x,
                    y0: y,
                    x1: x,
                    y1: y,
                },
                |v| Self {
                    x0: v.x0.min(x),
                    y0: v.y0.min(y),
                    x1: v.x1.max(x),
                    y1: v.y1.max(y),
                },
            ));
        }
        let view = view?;
        let pad = margin * (view.x1 - view.x0).max(view.y1 - view.y0).max(1e-6);
        Some(Self {
            x0: view.x0 - pad,
            y0: view.y0 - pad,
            x1: view.x1 + pad,
            y1: view.y1 + pad,
        })
    }

    /// This box grown about its centre to `height / width` of
    /// `aspect`, so a canvas of that shape shows it unstretched.
    pub fn with_aspect(self, aspect: f64) -> Self {
        let (w, h) = (self.x1 - self.x0, self.y1 - self.y0);
        let (cx, cy) = ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0);
        let (w, h) = if h / w > aspect {
            (h / aspect, h)
        } else {
            (w, w * aspect)
        };
        Self {
            x0: cx - w / 2.0,
            y0: cy - h / 2.0,
            x1: cx + w / 2.0,
            y1: cy + h / 2.0,
        }
    }

    /// Whether `other` lies wholly inside this box.
    pub fn holds(&self, other: &Self) -> bool {
        other.x0 >= self.x0 && other.x1 <= self.x1 && other.y0 >= self.y0 && other.y1 <= self.y1
    }

    /// Where `lat_deg`, `lon_deg` falls on a canvas of `width` × `height`
    /// pixels showing this box.
    pub fn pixel(&self, (lat_deg, lon_deg): (f64, f64), width: f64, height: f64) -> (f64, f64) {
        let (x, y) = mercator(lat_deg, lon_deg);
        (
            (x - self.x0) / (self.x1 - self.x0) * width,
            (y - self.y0) / (self.y1 - self.y0) * height,
        )
    }

    /// Ground metres per pixel at the box's centre on a canvas `width`
    /// pixels wide.
    pub fn metres_per_pixel(&self, width: f64) -> f64 {
        const EQUATOR_M: f64 = 40_075_016.686;
        let (lat, _) = unmercator(0.5 * (self.x0 + self.x1), 0.5 * (self.y0 + self.y1));
        (self.x1 - self.x0) * EQUATOR_M * lat.to_radians().cos() / width.max(1.0)
    }
}

impl CameraWorld {
    /// The world's imagery under `view` as `width` × `height` RGBA pixels,
    /// row-major, top row first: grey and dimmed where the package has
    /// imagery, a dark fill where it has none.
    pub fn overview_rgba(&self, view: &MapView, width: u32, height: u32) -> Vec<u8> {
        let scenery = self.scenery();
        let mut rgba = Vec::with_capacity(width as usize * height as usize * 4);
        for row in 0..height {
            let y = view.y0 + (f64::from(row) + 0.5) / f64::from(height) * (view.y1 - view.y0);
            for column in 0..width {
                let x =
                    view.x0 + (f64::from(column) + 0.5) / f64::from(width) * (view.x1 - view.x0);
                let (lat, lon) = unmercator(x, y);
                let pixel = scenery
                    .imagery_luma_at(&Geodetic::from_degrees(lat, lon, 0.0))
                    .map_or(UNCOVERED, |luma| {
                        let grey = (luma * DIM).clamp(0.0, 255.0) as u8;
                        [grey, grey, grey, 255]
                    });
                rgba.extend_from_slice(&pixel);
            }
        }
        rgba
    }
}
