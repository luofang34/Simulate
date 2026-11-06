//! The procedural ILS plate's symbology: chart tint and border, range rings around the
//! threshold, the final approach course with its localizer feather, the IF and FAF fixes,
//! the runway, the dashed missed approach, and the header and footer banners.

use super::{
    PLATE_PX, PlateFrame, ProceduralPlateSpec,
    canvas::{Canvas, Px},
    direction,
    glyphs::{draw_text, text_height, text_width},
    px_per_nm,
};

const IVORY_TINT: [u8; 4] = [246, 241, 226, 168];
const SLATE: [u8; 4] = [30, 41, 59, 255];
const INK: [u8; 4] = [15, 23, 42, 240];
const RING: [u8; 4] = [71, 85, 105, 190];
const MAGENTA: [u8; 4] = [190, 24, 93, 235];
const BANNER: [u8; 4] = [30, 41, 59, 232];
const BANNER_TEXT: [u8; 4] = [248, 250, 252, 255];
/// Final approach fix distance before the threshold, NM.
const FAF_NM: f64 = 5.2;
/// Intermediate fix distance before the threshold, NM.
const IF_NM: f64 = 8.5;
/// Banner height, pixels.
const BANNER_PX: f64 = 34.0;
/// Font scale of banner text.
const BANNER_SCALE: u32 = 2;

/// Draws every element of the plate.
pub(super) fn draw_plate(canvas: &mut Canvas, frame: &PlateFrame, spec: &ProceduralPlateSpec) {
    let size = f64::from(PLATE_PX);
    canvas.fill_rect([0.0, 0.0], [size, size], IVORY_TINT);
    for nm in [5.0, 10.0] {
        canvas.ring(frame.threshold_px, nm * px_per_nm(), 1.5, RING);
    }
    draw_final_course(canvas, frame);
    draw_runway(canvas, frame);
    draw_missed_approach(canvas, frame);
    maltese_cross(canvas, frame.on_final(FAF_NM), 9.0, frame.course_deg);
    fix_triangle(canvas, frame.on_final(IF_NM), 7.0);
    label(canvas, frame.on_final(FAF_NM), "FAF");
    label(canvas, frame.on_final(IF_NM), "IF");
    draw_banners(canvas, spec);
    for inset in [4.0, 5.0, 6.0] {
        let far = size - inset;
        canvas.polyline(
            &[
                [inset, inset],
                [far, inset],
                [far, far],
                [inset, far],
                [inset, inset],
            ],
            1.0,
            SLATE,
        );
    }
}

/// The bold course into the threshold with an arrowhead, and the localizer feather: a
/// half-filled wedge widening away from the threshold.
fn draw_final_course(canvas: &mut Canvas, frame: &PlateFrame) {
    canvas.line(frame.on_final(9.0), frame.threshold_px, 4.0, INK);
    let tip = frame.on_final(1.6);
    let [east, north] = direction(frame.course_deg);
    let normal = [north, east];
    let base = frame.on_final(2.4);
    let wing = 8.0;
    canvas.triangle(
        [
            tip,
            [base[0] + normal[0] * wing, base[1] + normal[1] * wing],
            [base[0] - normal[0] * wing, base[1] - normal[1] * wing],
        ],
        INK,
    );
    let (near, far) = (frame.on_final(3.5), frame.on_final(9.0));
    let spread = 0.45 * px_per_nm();
    let left = [far[0] + normal[0] * spread, far[1] + normal[1] * spread];
    let right = [far[0] - normal[0] * spread, far[1] - normal[1] * spread];
    canvas.triangle([near, far, left], [15, 23, 42, 150]);
    canvas.polyline(&[near, right, far], 1.5, INK);
}

/// The runway: a broad bar from the threshold along the course.
fn draw_runway(canvas: &mut Canvas, frame: &PlateFrame) {
    let end = frame.point_from_threshold(frame.course_deg, 0.8);
    canvas.line(frame.threshold_px, end, 7.0, SLATE);
}

/// Missed approach: straight ahead 2 NM, then a right turn for 1 NM, dashed.
fn draw_missed_approach(canvas: &mut Canvas, frame: &PlateFrame) {
    let climb = frame.point_from_threshold(frame.course_deg, 2.0);
    let [east, north] = direction(frame.course_deg + 90.0);
    let d = px_per_nm();
    let turn = [climb[0] + east * d, climb[1] - north * d];
    let start = frame.point_from_threshold(frame.course_deg, 0.9);
    canvas.dashed_line(start, climb, 2.5, [9.0, 6.0], MAGENTA);
    canvas.dashed_line(climb, turn, 2.5, [9.0, 6.0], MAGENTA);
}

/// The final approach fix: four triangles meeting at their tips.
fn maltese_cross(canvas: &mut Canvas, center: Px, size: f64, course_deg: f64) {
    for quarter in 0..4 {
        let bearing = course_deg + 45.0 + 90.0 * f64::from(quarter);
        let [e1, n1] = direction(bearing - 20.0);
        let [e2, n2] = direction(bearing + 20.0);
        canvas.triangle(
            [
                center,
                [center[0] + e1 * size, center[1] - n1 * size],
                [center[0] + e2 * size, center[1] - n2 * size],
            ],
            INK,
        );
    }
}

/// An intermediate fix: an upward triangle.
fn fix_triangle(canvas: &mut Canvas, [x, y]: Px, size: f64) {
    canvas.triangle(
        [
            [x, y - size],
            [x + size * 0.87, y + size * 0.5],
            [x - size * 0.87, y + size * 0.5],
        ],
        INK,
    );
}

fn label(canvas: &mut Canvas, [x, y]: Px, text: &str) {
    draw_text(canvas, x as i64 + 10, y as i64 - 4, text, 1, INK);
}

/// Header with the aerodrome, procedure and elevation; footer with the localizer
/// frequency, glidepath, course and the aerodrome name.
fn draw_banners(canvas: &mut Canvas, spec: &ProceduralPlateSpec) {
    let size = f64::from(PLATE_PX);
    canvas.fill_rect([0.0, 0.0], [size, BANNER_PX], BANNER);
    canvas.fill_rect([0.0, size - BANNER_PX], [size, size], BANNER);
    let header = format!(
        "{}  ILS/RNAV RWY {}  ELEV {:.0} FT",
        spec.airport_code, spec.runway_designator, spec.elevation_ft
    );
    let footer = format!(
        "LOC {:.2}  GS {:.2}  CRS {:03.0}  {}",
        f64::from(spec.ils_freq_khz) / 1000.0,
        spec.glidepath_deg,
        spec.true_heading_deg.rem_euclid(360.0),
        spec.airport_name
    );
    let text_y = |banner_top: f64| {
        banner_top as i64 + (BANNER_PX as i64 - i64::from(text_height(BANNER_SCALE))) / 2
    };
    for (text, top) in [(header, 0.0), (footer, size - BANNER_PX)] {
        let fitted = fit(&text, PLATE_PX - 16);
        let x = (i64::from(PLATE_PX) - i64::from(text_width(&fitted, BANNER_SCALE))) / 2;
        draw_text(
            canvas,
            x.max(8),
            text_y(top),
            &fitted,
            BANNER_SCALE,
            BANNER_TEXT,
        );
    }
}

/// `text` cut to the characters that fit `width` pixels at the banner scale.
fn fit(text: &str, width: u32) -> String {
    let mut fitted = String::new();
    for c in text.chars() {
        fitted.push(c);
        if text_width(&fitted, BANNER_SCALE) > width {
            fitted.pop();
            break;
        }
    }
    fitted
}
