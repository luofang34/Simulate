use glam::DVec3;
use sim_geo::{Geodetic, LocalFrame, geodetic_to_ecef};

use crate::aerodrome::geojson::{buffer_polyline, offset_lonlat};
use crate::airport_db::geometry::{bearing_and_distance, offset_deg};
use crate::{
    AerodromeRecord, AerodromeTier, AirportDatabase, CatalogVerticalDatum, FrequencyRole,
    ICONIC_AIRPORTS,
};

#[test]
fn nzsp_is_in_builtin_pack_and_visible_on_antarctic_globe() {
    assert_eq!(ICONIC_AIRPORTS.len(), 13);
    assert!(ICONIC_AIRPORTS.contains(&"NZSP"));

    let db = AirportDatabase::builtin();
    let (nzsp_idx, nzsp) = db.by_code("NZSP").expect("NZSP in builtin pack");
    assert_eq!(db.by_code("NPX").map(|(idx, _)| idx), Some(nzsp_idx));
    assert_eq!(&*nzsp.name, "Amundsen-Scott South Pole Station");
    assert_eq!(&*nzsp.municipality, "South Pole");
    assert_eq!(nzsp.country(), "AQ");
    assert_eq!(nzsp.region_code.as_str(), "AQ");
    assert!((nzsp.lat_deg - (-89.9975)).abs() < 1e-7);
    assert!((nzsp.lon_deg - 139.2729).abs() < 1e-7);
    assert!((nzsp.elevation_msl_m - 2_835.0).abs() < 1e-3);
    assert_eq!(nzsp.mag_var_deg, 0.0);
    assert!(nzsp.towered);
    assert_eq!(nzsp.tier, AerodromeTier::Tier1Major);
    assert_eq!(
        nzsp.primary_frequency_mhz(),
        Some((FrequencyRole::Tower, 126.2))
    );

    assert_eq!(db.search("NZSP", 5).first().copied(), Some(nzsp_idx));
    assert_eq!(db.search("South Pole", 5).first().copied(), Some(nzsp_idx));
    assert_eq!(
        db.nearest(-90.0, 0.0, 10.0, AerodromeTier::Tier1Major),
        Some(nzsp_idx)
    );
    assert!(
        db.query_globe_visible(-85.0, 0.0, 2.6, 200)
            .contains(&nzsp_idx)
    );
    assert!(
        db.query_globe_visible(-90.0, 0.0, 2.0, 200)
            .contains(&nzsp_idx)
    );
}

#[test]
fn nzsp_runway_02t_20t_and_flightbag_graph_are_polar_exact() {
    let (_, nzsp) = AirportDatabase::builtin().by_code("NZSP").unwrap();
    assert_eq!(nzsp.runways.len(), 1);
    let rwy = &nzsp.runways[0];
    assert_eq!(rwy.id.as_str(), "02T/20T");
    assert!((rwy.length_m - 3_658.0).abs() < 1e-3);
    assert!((rwy.width_m - 46.0).abs() < 1e-3);

    let [r02t, r20t] = &rwy.ends;
    assert_eq!(r02t.designator.as_str(), "02T");
    assert_eq!(r20t.designator.as_str(), "20T");
    assert!((r02t.true_heading_deg - 20.0).abs() < 1e-4);
    assert!((r20t.lon_deg - r02t.lon_deg).abs() > 15.0);
    let (brg_ab, len_m) =
        bearing_and_distance((r02t.lat_deg, r02t.lon_deg), (r20t.lat_deg, r20t.lon_deg));
    let (brg_ba, _) =
        bearing_and_distance((r20t.lat_deg, r20t.lon_deg), (r02t.lat_deg, r02t.lon_deg));
    assert!((brg_ab.to_degrees() - 20.0).abs() < 1e-3);
    assert!((len_m - 3_658.0).abs() < 0.05, "{len_m}");
    assert!((f64::from(r20t.true_heading_deg) - brg_ba.to_degrees()).abs() < 1e-3);

    let graph = nzsp.to_flightbag_graph(CatalogVerticalDatum::OrthometricMslEgm96);
    assert_eq!(graph.facility.as_str(), "NZSP");
    assert_eq!(graph.runway_ends.len(), 2);
    assert_eq!(graph.runway_ends[0].designator.as_str(), "02T");
    assert_eq!(graph.runway_ends[1].designator.as_str(), "20T");

    let summary = AerodromeRecord::from_surface_graph(graph, "South Pole", "AQ", 2_835.0).summary();
    let sf = &summary.runways[0].short_final;
    let thr = &summary.runways[0].threshold;
    let to_thr = geodetic_to_ecef(&thr.position) - geodetic_to_ecef(&sf.position);
    let sf_ned = LocalFrame::at(&sf.position).ecef_to_ned_vector(to_thr);
    let expected_hdg = sf_ned.y.atan2(sf_ned.x).rem_euclid(std::f64::consts::TAU);
    assert!((sf.heading_rad - expected_hdg).abs().to_degrees() < 0.05);
}

#[test]
fn nzsp_surface_geojson_and_tangent_geodesy_have_zero_polar_skew() {
    let (_, nzsp) = AirportDatabase::builtin().by_code("NZSP").unwrap();
    let value: serde_json::Value = serde_json::from_str(&nzsp.to_surface_geojson()).unwrap();
    let features = value["features"].as_array().unwrap();
    let pavement = features
        .iter()
        .find(|f| {
            matches!(
                f["properties"]["kind"].as_str(),
                Some("runway_pavement" | "runway_turf")
            )
        })
        .unwrap();
    let ring = pavement["geometry"]["coordinates"][0].as_array().unwrap();
    let ecef = |idx: usize| -> DVec3 {
        let lon = ring[idx][0].as_f64().unwrap();
        let lat = ring[idx][1].as_f64().unwrap();
        geodetic_to_ecef(&Geodetic::from_degrees(lat, lon, 0.0))
    };
    let (p0, p1, p2, p3) = (ecef(0), ecef(1), ecef(2), ecef(3));
    assert!(((p1 - p0).length() - 46.0).abs() < 0.05);
    assert!(((p2 - p3).length() - 46.0).abs() < 0.05);
    assert!(((p2 - p1).length() - 3_658.0).abs() < 0.1);
    assert!(((p3 - p0).length() - 3_658.0).abs() < 0.1);
    let cos_corner = (p1 - p0).normalize().dot((p2 - p1).normalize()).abs();
    assert!(cos_corner < 1e-4, "skewed corner cos={cos_corner}");

    // Step 3,658 m along 200° true right across 90° S: lon flips ~179°.
    let across_pole = offset_deg((-89.9975, 139.2729), 200.0_f64.to_radians(), 3_658.0);
    let (brg, dist) = bearing_and_distance((-89.9975, 139.2729), across_pole);
    assert!((brg.to_degrees() - 200.0).abs() < 1e-4);
    assert!((dist - 3_658.0).abs() < 0.05, "{dist}");

    let buffered = buffer_polyline(
        &[[139.2729, -89.9975], [across_pole.1, across_pole.0]],
        23.0,
    );
    assert_eq!(buffered.len(), 1);
    let moved = offset_lonlat([0.0, -90.0], 90.0_f64.to_radians(), 100.0);
    let (_, pole_step) = bearing_and_distance((-90.0, 0.0), (moved[1], moved[0]));
    assert!((pole_step - 100.0).abs() < 0.01);
}
