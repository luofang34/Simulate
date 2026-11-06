//! Georeferenced instrument approach plates for the EFB map: published charts placed by
//! their Flightbag footprint, and a procedural ILS plate drawn for any runway end.
//!
//! The procedural plate is a north-up `12 NM × 12 NM` raster centred on the final approach
//! course, from 9 NM before the threshold to 3 NM past it, so it drapes over the moving
//! map exactly where its fixes and courses lie.

use std::{io::Cursor, sync::Arc};

use image::{DynamicImage, ImageFormat};

use crate::{MapAdapterError, geodesy::METRES_PER_NM};

mod canvas;
mod glyphs;
mod symbols;

use canvas::Canvas;

/// Side of the procedural plate raster, pixels.
pub(crate) const PLATE_PX: u32 = 512;
/// Side of the procedural plate footprint, nautical miles.
const PLATE_NM: f64 = 12.0;
/// How far before the threshold the plate reaches along the final approach course.
const PLATE_NM_BEFORE_THRESHOLD: f64 = 9.0;
/// Mean Earth radius for the plate's footprint, metres.
const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// Georeferenced instrument approach plate raster overlay.
#[derive(Clone, Debug)]
pub struct ApproachPlateOverlay {
    /// Stable chart identifier (`kewr-ils-22l`).
    pub chart_id: String,
    /// Human-readable procedure title (`ILS OR LOC RWY 22L`).
    pub title: String,
    /// Four geographic corners `[lon_deg, lat_deg]` ordered `[NW, NE, SE, SW]` (`top_left, top_right, bottom_right, bottom_left`).
    pub corners_lon_lat: [[f64; 2]; 4],
    /// Encoded RGBA PNG image bytes.
    pub png_bytes: Arc<[u8]>,
}

impl ApproachPlateOverlay {
    /// A published chart placed by its Flightbag footprint, north-up over its bounds.
    #[must_use]
    pub fn from_flightbag_georef(chart: &flightbag::GeorefChartRef, png_bytes: Arc<[u8]>) -> Self {
        let b = &chart.bounds;
        Self {
            chart_id: chart.chart_id.clone(),
            title: chart.title.clone(),
            corners_lon_lat: [
                [b.west_lon_deg, b.north_lat_deg],
                [b.east_lon_deg, b.north_lat_deg],
                [b.east_lon_deg, b.south_lat_deg],
                [b.west_lon_deg, b.south_lat_deg],
            ],
            png_bytes,
        }
    }
}

/// What a procedural ILS approach plate shows.
#[derive(Clone, Debug, PartialEq)]
pub struct ProceduralPlateSpec {
    /// Aerodrome identifier (`KEWR`).
    pub airport_code: String,
    /// Aerodrome name (`NEWARK LIBERTY INTL`).
    pub airport_name: String,
    /// Runway end designator (`22L`).
    pub runway_designator: String,
    /// Threshold latitude, degrees.
    pub threshold_lat_deg: f64,
    /// Threshold longitude, degrees.
    pub threshold_lon_deg: f64,
    /// Final approach course (runway true heading), degrees.
    pub true_heading_deg: f64,
    /// Threshold elevation, feet MSL.
    pub elevation_ft: f64,
    /// Localizer frequency, kHz (`110750` for 110.75 MHz).
    pub ils_freq_khz: u32,
    /// Glidepath angle, degrees.
    pub glidepath_deg: f64,
}

/// Draws the procedural ILS plate of `spec` and georeferences it.
pub fn generate_procedural_approach_plate(
    spec: &ProceduralPlateSpec,
) -> Result<ApproachPlateOverlay, MapAdapterError> {
    let chart_id = format!(
        "{}-ils-{}",
        spec.airport_code.to_ascii_lowercase(),
        spec.runway_designator.to_ascii_lowercase()
    );
    let finite = [
        spec.threshold_lat_deg,
        spec.threshold_lon_deg,
        spec.true_heading_deg,
        spec.elevation_ft,
        spec.glidepath_deg,
    ]
    .iter()
    .all(|v| v.is_finite());
    if !finite || spec.threshold_lat_deg.abs() > 84.0 {
        return Err(MapAdapterError::ChartRaster {
            chart_id,
            detail: "threshold, heading, elevation and glidepath must be finite, |lat| <= 84"
                .to_owned(),
        });
    }
    let frame = PlateFrame::new(spec);
    let mut canvas = Canvas::new(PLATE_PX, PLATE_PX);
    symbols::draw_plate(&mut canvas, &frame, spec);
    let png_bytes = encode_png(canvas, &chart_id)?;
    Ok(ApproachPlateOverlay {
        title: format!("ILS OR LOC RWY {}", spec.runway_designator),
        corners_lon_lat: frame.corners_lon_lat(),
        chart_id,
        png_bytes,
    })
}

fn encode_png(canvas: Canvas, chart_id: &str) -> Result<Arc<[u8]>, MapAdapterError> {
    let mut bytes = Vec::new();
    DynamicImage::ImageRgba8(canvas.image)
        .write_to(&mut Cursor::new(&mut bytes), ImageFormat::Png)
        .map_err(|error| MapAdapterError::ChartRaster {
            chart_id: chart_id.to_owned(),
            detail: error.to_string(),
        })?;
    Ok(Arc::from(bytes))
}

/// The plate's north-up pixel frame: its centre on the course and the threshold in pixels.
pub(crate) struct PlateFrame {
    center_lat_deg: f64,
    center_lon_deg: f64,
    /// Threshold in raster pixels.
    pub(crate) threshold_px: [f64; 2],
    /// Final approach course, degrees true.
    pub(crate) course_deg: f64,
}

impl PlateFrame {
    fn new(spec: &ProceduralPlateSpec) -> Self {
        let course_deg = spec.true_heading_deg.rem_euclid(360.0);
        // The plate spans 9 NM before to 3 NM past the threshold: its centre lies
        // 3 NM before the threshold, so the threshold sits 3 NM along the course from it.
        let ahead_nm = PLATE_NM_BEFORE_THRESHOLD - PLATE_NM * 0.5;
        let center = crate::geodesy::destination(
            [spec.threshold_lon_deg, spec.threshold_lat_deg],
            course_deg + 180.0,
            ahead_nm * METRES_PER_NM,
        );
        let half = f64::from(PLATE_PX) * 0.5;
        let [east, north] = direction(course_deg);
        let ahead_px = ahead_nm * px_per_nm();
        Self {
            center_lat_deg: center[1],
            center_lon_deg: center[0],
            threshold_px: [half + east * ahead_px, half - north * ahead_px],
            course_deg,
        }
    }

    /// `[lon, lat]` of the raster's NW, NE, SE and SW corners.
    fn corners_lon_lat(&self) -> [[f64; 2]; 4] {
        let half_m = PLATE_NM * 0.5 * METRES_PER_NM;
        let dlat = (half_m / EARTH_RADIUS_M).to_degrees();
        let dlon = dlat / self.center_lat_deg.to_radians().cos();
        let (lat, lon) = (self.center_lat_deg, self.center_lon_deg);
        [
            [lon - dlon, lat + dlat],
            [lon + dlon, lat + dlat],
            [lon + dlon, lat - dlat],
            [lon - dlon, lat - dlat],
        ]
    }

    /// The point `nm` from the threshold along `bearing_deg`, in raster pixels.
    pub(crate) fn point_from_threshold(&self, bearing_deg: f64, nm: f64) -> [f64; 2] {
        let [east, north] = direction(bearing_deg);
        let d = nm * px_per_nm();
        [
            self.threshold_px[0] + east * d,
            self.threshold_px[1] - north * d,
        ]
    }

    /// The point `nm` before the threshold on the final approach course.
    pub(crate) fn on_final(&self, nm: f64) -> [f64; 2] {
        self.point_from_threshold(self.course_deg + 180.0, nm)
    }
}

/// Raster pixels per nautical mile.
pub(crate) fn px_per_nm() -> f64 {
    f64::from(PLATE_PX) / PLATE_NM
}

/// `[east, north]` unit vector of a true bearing.
pub(crate) fn direction(bearing_deg: f64) -> [f64; 2] {
    let (sin, cos) = bearing_deg.to_radians().sin_cos();
    [sin, cos]
}
