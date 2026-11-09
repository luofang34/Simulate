use sim_geo::{Geodetic, ecef_to_geodetic, geodetic_to_ecef};

use super::{MIN_STAMP_ZOOM, RunwayDatum};
use crate::{AirportDatabase, AirportRecord, TerrariumTile, project_to_tile};

fn airport(code: &str) -> &'static AirportRecord {
    AirportDatabase::builtin().by_code(code).unwrap().1
}

/// Each runway end's point `along` of the way to the other end, with the charted height
/// there.
fn runway_points(record: &AirportRecord, along: f64) -> Vec<(Geodetic, f64)> {
    record
        .runways
        .iter()
        .flat_map(|runway| {
            let [a, b] = &runway.ends;
            [(a, b), (b, a)]
        })
        .map(|(end, other)| {
            // Interpolated in ECEF so a runway across a pole stays straight.
            let from = geodetic_to_ecef(&Geodetic::from_degrees(end.lat_deg, end.lon_deg, 0.0));
            let to = geodetic_to_ecef(&Geodetic::from_degrees(other.lat_deg, other.lon_deg, 0.0));
            let point = Geodetic {
                height_m: 0.0,
                ..ecef_to_geodetic(from + (to - from) * along)
            };
            let charted = f64::from(end.elevation_msl_m)
                + f64::from(other.elevation_msl_m - end.elevation_msl_m) * along;
            (point, charted)
        })
        .collect()
}

/// Whether `point` lies on the pavement of more than one runway: at an intersection,
/// where the longer runway's height holds.
fn on_crossing(datum: &RunwayDatum, point: &Geodetic) -> bool {
    let surface = crate::world_query::surface_ecef(point);
    let key = super::fine_key(point.lat_deg(), point.lon_deg());
    let runways: std::collections::BTreeSet<_> = datum
        .fine_cell(key)
        .iter()
        .filter(|strip| {
            strip
                .strip
                .reach_ecef(surface)
                .is_some_and(|reach| reach.off_pavement_m == 0.0)
        })
        .map(|strip| (strip.runway.airport, strip.runway.runway))
        .collect();
    runways.len() > 1
}

fn flat_tile(height_m: f64) -> TerrariumTile {
    TerrariumTile::from_heights(256, 256, vec![height_m; 256 * 256]).unwrap()
}

#[test]
fn every_runway_point_reads_its_charted_height_at_full_weight() {
    let datum = RunwayDatum::builtin();
    for code in ["KEWR", "47N", "KDEN", "VCBI", "EGLL", "NZSP", "RJTT"] {
        for along in [0.0, 0.3, 0.5, 0.9] {
            for (point, charted) in runway_points(airport(code), along) {
                if on_crossing(&datum, &point) {
                    continue;
                }
                let (height, weight) = datum
                    .height_and_weight(&point)
                    .unwrap_or_else(|| panic!("{code} {along} {point:?} off every strip"));
                assert_eq!(weight, 1.0, "{code} {along}");
                assert!(
                    (height - charted).abs() < 0.02,
                    "{code} {along}: {height} vs {charted}"
                );
            }
        }
    }
}

#[test]
fn ground_far_from_any_airport_is_left_alone() {
    let datum = RunwayDatum::builtin();
    let pacific = Geodetic::from_degrees(-30.0, -140.0, 0.0);
    assert_eq!(datum.height_and_weight(&pacific), None);
    assert_eq!(datum.blend(&pacific, 12.5), 12.5);
    let tile = project_to_tile(&pacific, 12).unwrap().tile;
    assert!(!datum.touches_tile(tile));
    let mut dem = flat_tile(-4_000.0);
    assert!(!datum.stamp_terrarium(tile, &mut dem));
}

/// A stamped tile reads the charted height on the runway whether a reader puts a
/// pixel's height at its centre (the scenery crate) or at its corner (maplibre's GL JS
/// convention): pavement and shoulders are flat across more than a pixel.
#[test]
fn stamped_tiles_carry_the_charted_runway_in_either_pixel_convention() {
    let datum = RunwayDatum::builtin();
    for code in ["KEWR", "KDEN", "VCBI"] {
        for (point, charted) in runway_points(airport(code), 0.5)
            .into_iter()
            .chain(runway_points(airport(code), 0.05))
        {
            for zoom in [MIN_STAMP_ZOOM + 4, 15] {
                let coord = project_to_tile(&point, zoom).unwrap();
                let mut dem = flat_tile(charted + 3.0);
                assert!(datum.touches_tile(coord.tile), "{code} z{zoom}");
                assert!(
                    datum.stamp_terrarium(coord.tile, &mut dem),
                    "{code} z{zoom}"
                );
                let centre = dem.sample_bilinear(coord.u, coord.v).unwrap();
                let corner_u = (coord.u * 256.0 + 0.5) / 256.0;
                let corner_v = (coord.v * 256.0 + 0.5) / 256.0;
                let corner = dem.sample_bilinear(corner_u, corner_v).unwrap();
                for (convention, height) in [("centre", centre), ("corner", corner)] {
                    assert!(
                        (height - charted).abs() < 0.05,
                        "{code} z{zoom} {convention}: {height} vs {charted}"
                    );
                }
            }
        }
    }
}

#[test]
fn coarse_tiles_are_not_stamped() {
    let datum = RunwayDatum::builtin();
    let (point, _) = runway_points(airport("KEWR"), 0.5)[0];
    let tile = project_to_tile(&point, MIN_STAMP_ZOOM - 1).unwrap().tile;
    assert!(!datum.touches_tile(tile));
}

/// The straight line through a runway's charted ends at `point`, extended past them,
/// and how far along the runway `point` is.
fn charted_line(record: &AirportRecord, runway: usize, point: &Geodetic) -> (f64, f64) {
    let [a, b] = &record.runways[runway].ends;
    let a_ecef = geodetic_to_ecef(&Geodetic::from_degrees(a.lat_deg, a.lon_deg, 0.0));
    let b_ecef = geodetic_to_ecef(&Geodetic::from_degrees(b.lat_deg, b.lon_deg, 0.0));
    let t = (geodetic_to_ecef(point) - a_ecef).dot(b_ecef - a_ecef)
        / (b_ecef - a_ecef).length_squared();
    (
        f64::from(a.elevation_msl_m) + f64::from(b.elevation_msl_m - a.elevation_msl_m) * t,
        t,
    )
}

/// Heights every 15 m along runway 0's centreline from its first end.
fn centreline(datum: &RunwayDatum, record: &AirportRecord) -> Vec<(Geodetic, f64)> {
    let [a, b] = &record.runways[0].ends;
    let a_ecef = geodetic_to_ecef(&Geodetic::from_degrees(a.lat_deg, a.lon_deg, 0.0));
    let b_ecef = geodetic_to_ecef(&Geodetic::from_degrees(b.lat_deg, b.lon_deg, 0.0));
    let steps = ((b_ecef - a_ecef).length() / 15.0).ceil() as usize;
    (0..=steps)
        .map(|i| {
            let point = ecef_to_geodetic(a_ecef + (b_ecef - a_ecef) * (i as f64 / steps as f64));
            let point = Geodetic {
                height_m: 0.0,
                ..point
            };
            (point, datum.height_and_weight(&point).unwrap().0)
        })
        .collect()
}

fn assert_graded(code: &str, heights: &[(Geodetic, f64)]) {
    for pair in heights.windows(2) {
        let run = (geodetic_to_ecef(&pair[1].0) - geodetic_to_ecef(&pair[0].0)).length();
        let grade = (pair[1].1 - pair[0].1).abs() / run;
        assert!(grade <= 0.0201, "{code}: {:.2}% grade", grade * 100.0);
    }
}

/// The charted line of whichever runway of `record` `point` lies nearest, with how far
/// along that runway it is and which runway.
fn nearest_line(record: &AirportRecord, point: &Geodetic) -> (f64, f64, usize) {
    (0..record.runways.len())
        .map(|runway| {
            let (line, t) = charted_line(record, runway, point);
            let [a, b] = &record.runways[runway].ends;
            let a = geodetic_to_ecef(&Geodetic::from_degrees(a.lat_deg, a.lon_deg, 0.0));
            let b = geodetic_to_ecef(&Geodetic::from_degrees(b.lat_deg, b.lon_deg, 0.0));
            let off = (geodetic_to_ecef(point) - (a + (b - a) * t)).length();
            (off, line, t, runway)
        })
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, line, t, runway)| (line, t, runway))
        .unwrap()
}

fn bump(t: f64) -> f64 {
    (1.0 - (2.0 * t - 1.0).powi(2)).max(0.0)
}

/// N51 is crowned: its field elevation stands above all its runway ends. Runway 04/22's
/// terrain rises 4 m above its charted line mid-runway, under metre-scale noise and a
/// 4 m datum offset. Its profile follows that crown, ends on the charted ends, never
/// rises past the field elevation and stays within a 2% grade: no trench cut to the
/// straight line.
#[test]
fn a_crowned_runway_follows_its_terrain_crown() {
    let datum = RunwayDatum::builtin();
    let (idx, n51) = AirportDatabase::builtin().by_code("N51").unwrap();
    let field = f64::from(n51.elevation_msl_m);
    let terrain = |point: &Geodetic| {
        let (line, t, runway) = nearest_line(n51, point);
        let crown = if runway == 0 { 4.0 * bump(t) } else { 0.0 };
        Some(line + 4.0 + crown + 0.4 * (point.lon_rad * 4.0e5).sin())
    };
    assert!(!datum.is_resolved(idx));
    let before = datum.generation();
    assert!(datum.resolve_airport(idx, &terrain));
    assert!(!datum.resolve_airport(idx, &|_| None));
    assert_eq!(datum.generation(), before + 1);

    let heights = centreline(&datum, n51);
    let [a, b] = &n51.runways[0].ends;
    let (first, last) = (heights.first().unwrap().1, heights.last().unwrap().1);
    assert!(
        (first - f64::from(a.elevation_msl_m)).abs() < 0.02,
        "{first}"
    );
    assert!((last - f64::from(b.elevation_msl_m)).abs() < 0.02, "{last}");
    let (mid_point, mid) = &heights[heights.len() / 2];
    let crowned = charted_line(n51, 0, mid_point).0 + 4.0;
    assert!(
        (mid - crowned).abs() < 0.3,
        "mid {mid} vs crowned {crowned}"
    );
    // Where 04/22 crosses another runway the longer one's height holds.
    let own: Vec<_> = heights
        .iter()
        .filter(|(point, _)| !on_crossing(&datum, point))
        .collect();
    for (point, height) in &own {
        assert!(*height <= field + 0.01, "{height} above field {field}");
        assert!(
            *height >= charted_line(n51, 0, point).0 - 0.05,
            "a trench at {point:?}"
        );
    }
    assert!(own.len() > heights.len() / 2);
    assert_graded("N51", &own.into_iter().copied().collect::<Vec<_>>());
}

/// Colombo's one runway is the highest point of its field, 0.9 m above its higher end.
/// Terrain showing a smaller crown is raised so the runway reaches the field elevation.
#[test]
fn the_runway_that_defines_the_field_elevation_reaches_it() {
    let datum = RunwayDatum::builtin();
    let (idx, vcbi) = AirportDatabase::builtin().by_code("VCBI").unwrap();
    let field = f64::from(vcbi.elevation_msl_m);
    let terrain = |point: &Geodetic| {
        let (line, t, _) = nearest_line(vcbi, point);
        Some(line + 2.7 + 0.5 * bump(t))
    };
    assert!(datum.resolve_airport(idx, &terrain));
    let heights = centreline(&datum, vcbi);
    let peak = heights
        .iter()
        .map(|h| h.1)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!((peak - field).abs() < 0.05, "peak {peak} vs field {field}");
    assert_graded("VCBI", &heights);
}

/// Terrain that only sits off the charted runway by a constant, as at 47N and KDEN,
/// leaves the runway on its charted line; terrain that cannot be read leaves it straight;
/// a crown above the field elevation is held to it.
#[test]
fn offset_missing_and_overtall_terrain_keep_the_runway_charted() {
    let datum = RunwayDatum::builtin();
    let db = AirportDatabase::builtin();
    for (code, offset) in [("47N", -4.5), ("KDEN", 1.0)] {
        let (idx, record) = db.by_code(code).unwrap();
        let shifted = |point: &Geodetic| Some(nearest_line(record, point).0 + offset);
        assert!(datum.resolve_airport(idx, &shifted));
        let heights = centreline(&datum, record);
        for (point, height) in &heights {
            let line = charted_line(record, 0, point).0;
            assert!((height - line).abs() < 0.05, "{code}: {height} vs {line}");
        }
        assert_graded(code, &heights);
    }

    let (vcbi_idx, vcbi) = db.by_code("VCBI").unwrap();
    assert!(datum.resolve_airport(vcbi_idx, &|_| None));
    for (point, height) in centreline(&datum, vcbi) {
        assert!((height - charted_line(vcbi, 0, &point).0).abs() < 0.02);
    }

    let (egll_idx, egll) = db.by_code("EGLL").unwrap();
    let field = f64::from(egll.elevation_msl_m);
    let mountain = |point: &Geodetic| {
        let (line, t, _) = nearest_line(egll, point);
        Some(line + 30.0 * bump(t))
    };
    assert!(datum.resolve_airport(egll_idx, &mountain));
    let heights = centreline(&datum, egll);
    let peak = heights
        .iter()
        .map(|h| h.1)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(peak <= field + 0.01, "peak {peak} above field {field}");
    assert_graded("EGLL", &heights);
}
