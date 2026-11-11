//! What a host draws over the plan besides it: flown tracks, each with a ring of its
//! stated one-sigma at its latest point, marks at chosen points, and the outlines of
//! areas such as where an onboard map has coverage.
//!
//! A cockpit leaves these empty: they are for a host that judges the flight from outside,
//! such as a page comparing navigators against the simulator's truth.

use serde_json::{Value, json};

use super::route::lon_lat;

/// A colour, `[r, g, b]` in sRGB bytes.
pub type Rgb = [u8; 3];

/// One flown track.
#[derive(Clone, Debug, PartialEq)]
pub struct TrackLine {
    /// Its colour.
    pub colour: Rgb,
    /// Line width, CSS pixels.
    pub width_px: f32,
    /// `[lat, lon]` points in time order, degrees.
    pub points: Vec<[f64; 2]>,
}

/// A ring of a stated horizontal one-sigma about a point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SigmaRing {
    /// `[lat, lon]` of the centre, degrees.
    pub centre_deg: [f64; 2],
    /// The one-sigma radius, metres.
    pub sigma_m: f64,
    /// Its colour.
    pub colour: Rgb,
}

/// A mark at a point: a dot when `filled`, else a ring.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointMark {
    /// `[lat, lon]`, degrees.
    pub at_deg: [f64; 2],
    /// Its colour.
    pub colour: Rgb,
    /// A filled dot rather than a hollow ring.
    pub filled: bool,
}

/// Everything drawn over the plan besides it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EfbTracks {
    /// Flown tracks, drawn in order.
    pub lines: Vec<TrackLine>,
    /// Rings at least [`MIN_RING_PX`] across, drawn over the tracks.
    pub rings: Vec<SigmaRing>,
    /// Marks at points.
    pub marks: Vec<PointMark>,
    /// Closed outlines, each `[lat, lon]` vertices in order, degrees.
    pub outlines: Vec<Vec<[f64; 2]>>,
}

/// Smallest ring radius drawn, CSS pixels: a sigma of a few metres is under a pixel at
/// most zooms.
pub const MIN_RING_PX: f32 = 4.0;

/// `#rrggbb` of `colour`.
#[must_use]
pub fn css_colour([r, g, b]: Rgb) -> String {
    format!("#{r:02x}{g:02x}{b:02x}")
}

impl EfbTracks {
    /// The tracks and marks as one GeoJSON document: lines carry `colour` and `width`,
    /// marks `colour` and `filled`.
    #[must_use]
    pub fn geojson(&self) -> Value {
        let lines = self
            .lines
            .iter()
            .filter(|line| line.points.len() >= 2)
            .map(|line| {
                let coordinates: Vec<[f64; 2]> = line.points.iter().copied().map(lon_lat).collect();
                json!({
                    "type": "Feature",
                    "properties": { "colour": css_colour(line.colour), "width": line.width_px },
                    "geometry": { "type": "LineString", "coordinates": coordinates },
                })
            });
        let marks = self.marks.iter().map(|mark| {
            json!({
                "type": "Feature",
                "properties": { "colour": css_colour(mark.colour), "filled": mark.filled },
                "geometry": { "type": "Point", "coordinates": lon_lat(mark.at_deg) },
            })
        });
        json!({ "type": "FeatureCollection", "features": lines.chain(marks).collect::<Vec<_>>() })
    }

    /// The outlines as GeoJSON closed lines.
    #[must_use]
    pub fn outlines_geojson(&self) -> Value {
        let features: Vec<Value> = self
            .outlines
            .iter()
            .filter(|outline| outline.len() >= 3)
            .map(|outline| {
                let mut coordinates: Vec<[f64; 2]> = outline.iter().copied().map(lon_lat).collect();
                if let Some(first) = coordinates.first().copied() {
                    coordinates.push(first);
                }
                json!({
                    "type": "Feature",
                    "properties": {},
                    "geometry": { "type": "LineString", "coordinates": coordinates },
                })
            })
            .collect();
        json!({ "type": "FeatureCollection", "features": features })
    }
}
