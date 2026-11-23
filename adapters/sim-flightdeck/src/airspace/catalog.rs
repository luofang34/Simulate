//! Built-in NY/NJ corridor `Flightbag` 3D airspace volumes and `world-notices` TFR / SUA fixtures.

use core::f64::consts::TAU;
use flightbag_core::{
    AirspaceClassification, AirspacePolygon, AirspaceVolume, ControlledAirspaceClass,
    ControlledLocalType, SuaKind as StaticSuaKind, VerticalLimit,
};
use world_core::{DecimalYear, GeodeticPosition, LocalTangentPlane, Ned, ValidityWindow};
use world_notices::{
    AirspaceKey, LateralShape, NoticeAltitude, NoticeId, NoticesStore, SuaActivationNotice,
    SuaKind as LiveSuaKind, TfrKind, TfrNotice, VerticalBand,
};

use crate::error::FlightDeckError;

/// Default evaluation epoch (`2026.86`, matching the `2026-10-29` AIRAC cycle and WMM2025 window).
pub const DEFAULT_EPOCH: DecimalYear = DecimalYear(2026.86);

/// Builds a validated set of `Flightbag` 3D airspace shelves covering `47N`, `KMMU`, `KEWR`, `KJFK`, and `R-5001`.
///
/// # Errors
///
/// Returns [`FlightDeckError::Airspace`] if any volume fails [`AirspaceVolume::validate`].
pub fn builtin_ny_nj_airspaces() -> Result<Vec<AirspaceVolume>, FlightDeckError> {
    let mut volumes = newark_class_b_shelves()?;
    volumes.extend(regional_ny_nj_shelves()?);
    for vol in &volumes {
        vol.validate().map_err(|e| FlightDeckError::Airspace {
            reason: format!("invalid builtin volume {}: {e:?}", vol.airspace_key),
        })?;
    }
    Ok(volumes)
}

fn newark_class_b_shelves() -> Result<Vec<AirspaceVolume>, FlightDeckError> {
    let kewr = GeodeticPosition::wgs84(40.6895, -74.1745, 0.0);
    Ok(vec![
        AirspaceVolume {
            airspace_key: "KEWR-B-SFC".to_owned(),
            ident: "EWR".to_owned(),
            name: "NEWARK CLASS B CORE".to_owned(),
            sector: Some("SFC-70".to_owned()),
            classification: AirspaceClassification::Controlled {
                class: ControlledAirspaceClass::ClassB,
                local_type: ControlledLocalType::ClassB,
            },
            lower: VerticalLimit::SURFACE,
            upper: VerticalLimit::msl_ft(7_000),
            hours_of_use: Some("CONT".to_owned()),
            controlling_agency: Some("NEW YORK TRACON".to_owned()),
            polygons: vec![circle_polygon(kewr, 11_112.0, 28)?],
        },
        AirspaceVolume {
            airspace_key: "KEWR-B-15".to_owned(),
            ident: "EWR".to_owned(),
            name: "NEWARK CLASS B 15-70".to_owned(),
            sector: Some("15-70".to_owned()),
            classification: AirspaceClassification::Controlled {
                class: ControlledAirspaceClass::ClassB,
                local_type: ControlledLocalType::ClassB,
            },
            lower: VerticalLimit::msl_ft(1_500),
            upper: VerticalLimit::msl_ft(7_000),
            hours_of_use: Some("CONT".to_owned()),
            controlling_agency: Some("NEW YORK TRACON".to_owned()),
            polygons: vec![annulus_polygon(kewr, 20_372.0, 11_112.0, 32)?],
        },
        AirspaceVolume {
            airspace_key: "KEWR-B-30".to_owned(),
            ident: "EWR".to_owned(),
            name: "NEWARK CLASS B 30-70".to_owned(),
            sector: Some("30-70".to_owned()),
            classification: AirspaceClassification::Controlled {
                class: ControlledAirspaceClass::ClassB,
                local_type: ControlledLocalType::ClassB,
            },
            lower: VerticalLimit::msl_ft(3_000),
            upper: VerticalLimit::msl_ft(7_000),
            hours_of_use: Some("CONT".to_owned()),
            controlling_agency: Some("NEW YORK TRACON".to_owned()),
            polygons: vec![annulus_polygon(kewr, 33_336.0, 20_372.0, 36)?],
        },
    ])
}

fn regional_ny_nj_shelves() -> Result<Vec<AirspaceVolume>, FlightDeckError> {
    let kjfk = GeodeticPosition::wgs84(40.6413, -73.7781, 0.0);
    let kmmu = GeodeticPosition::wgs84(40.7994, -74.4149, 0.0);
    let c47n = GeodeticPosition::wgs84(40.5243, -74.5984, 0.0);
    let r5001 = GeodeticPosition::wgs84(40.3950, -74.5200, 0.0);
    Ok(vec![
        AirspaceVolume {
            airspace_key: "KJFK-B-30".to_owned(),
            ident: "JFK".to_owned(),
            name: "KENNEDY CLASS B WEST".to_owned(),
            sector: Some("30-70".to_owned()),
            classification: AirspaceClassification::Controlled {
                class: ControlledAirspaceClass::ClassB,
                local_type: ControlledLocalType::ClassB,
            },
            lower: VerticalLimit::msl_ft(3_000),
            upper: VerticalLimit::msl_ft(7_000),
            hours_of_use: Some("CONT".to_owned()),
            controlling_agency: Some("NEW YORK TRACON".to_owned()),
            polygons: vec![circle_polygon(kjfk, 27_780.0, 28)?],
        },
        AirspaceVolume {
            airspace_key: "KMMU-D".to_owned(),
            ident: "MMU".to_owned(),
            name: "MORRISTOWN CLASS D".to_owned(),
            sector: None,
            classification: AirspaceClassification::Controlled {
                class: ControlledAirspaceClass::ClassD,
                local_type: ControlledLocalType::ClassD,
            },
            lower: VerticalLimit::SURFACE,
            upper: VerticalLimit::msl_ft(2_700),
            hours_of_use: Some("0645-2230L".to_owned()),
            controlling_agency: Some("MORRISTOWN TOWER".to_owned()),
            polygons: vec![circle_polygon(kmmu, 7_800.0, 24)?],
        },
        AirspaceVolume {
            airspace_key: "47N-E5".to_owned(),
            ident: "47N".to_owned(),
            name: "CENTRAL JERSEY CLASS E".to_owned(),
            sector: None,
            classification: AirspaceClassification::Controlled {
                class: ControlledAirspaceClass::ClassE,
                local_type: ControlledLocalType::ClassE5Transition,
            },
            lower: VerticalLimit::agl_ft(700),
            upper: VerticalLimit::msl_ft(2_900),
            hours_of_use: Some("CONT".to_owned()),
            controlling_agency: Some("NEW YORK TRACON".to_owned()),
            polygons: vec![circle_polygon(c47n, 10_186.0, 24)?],
        },
        AirspaceVolume {
            airspace_key: "R-5001".to_owned(),
            ident: "R-5001".to_owned(),
            name: "R-5001 CENTRAL NJ SUA".to_owned(),
            sector: Some("A".to_owned()),
            classification: AirspaceClassification::SpecialUse(StaticSuaKind::Restricted),
            lower: VerticalLimit::msl_ft(500),
            upper: VerticalLimit::msl_ft(5_000),
            hours_of_use: Some("BY NOTAM".to_owned()),
            controlling_agency: Some("MCGUIRE APP".to_owned()),
            polygons: vec![rect_polygon(r5001, 6_500.0, 5_000.0)?],
        },
    ])
}

/// Seeds [`NoticesStore`] with an active Bedminster/Somerset VIP TFR and `R-5001` SUA hot window.
///
/// # Errors
///
/// Returns [`FlightDeckError::Airspace`] if notice identifiers or geometries fail validation.
pub fn builtin_ny_nj_notices() -> Result<NoticesStore, FlightDeckError> {
    let mut store = NoticesStore::default();
    let valid = ValidityWindow {
        from: DecimalYear(2026.0),
        until: DecimalYear(2028.0),
    };

    let tfr_id = NoticeId::parse("FDC 6/9141").map_err(|e| FlightDeckError::Airspace {
        reason: e.to_string(),
    })?;
    let tfr = TfrNotice {
        id: tfr_id,
        kind: TfrKind::Vip,
        lateral: LateralShape::Circle {
            center: GeodeticPosition::wgs84(40.6350, -74.6450, 0.0),
            radius_m: 5_556.0, // 3 NM inner VIP ring
        },
        vertical: VerticalBand {
            floor: NoticeAltitude::Surface,
            ceiling: NoticeAltitude::GeometricMsl(2_438.4), // 8,000 ft MSL
        },
        valid,
        cancelled: false,
        provenance: None,
    };
    store
        .upsert_tfr(tfr)
        .map_err(|e| FlightDeckError::Airspace {
            reason: e.to_string(),
        })?;

    let sua_id = NoticeId::parse("SUA 6/5001").map_err(|e| FlightDeckError::Airspace {
        reason: e.to_string(),
    })?;
    let sua_key = AirspaceKey::parse("R-5001").map_err(|e| FlightDeckError::Airspace {
        reason: e.to_string(),
    })?;
    let sua = SuaActivationNotice {
        id: sua_id,
        airspace: sua_key,
        kind: LiveSuaKind::Restricted,
        vertical: VerticalBand {
            floor: NoticeAltitude::GeometricMsl(152.4),
            ceiling: NoticeAltitude::GeometricMsl(1_524.0),
        },
        valid,
        cancelled: false,
    };
    store
        .upsert_sua_activation(sua)
        .map_err(|e| FlightDeckError::Airspace {
            reason: e.to_string(),
        })?;

    Ok(store)
}

/// Approximates a geodesic circle on WGS-84 as an `n`-gon [`AirspacePolygon`].
///
/// # Errors
///
/// Returns [`FlightDeckError::Airspace`] if tangent-plane projection fails.
pub fn circle_polygon(
    center: GeodeticPosition,
    radius_m: f64,
    segments: usize,
) -> Result<AirspacePolygon, FlightDeckError> {
    let exterior = circle_ring(center, radius_m, segments, false)?;
    Ok(AirspacePolygon {
        exterior,
        interiors: Vec::new(),
    })
}

fn annulus_polygon(
    center: GeodeticPosition,
    outer_radius_m: f64,
    inner_radius_m: f64,
    segments: usize,
) -> Result<AirspacePolygon, FlightDeckError> {
    let exterior = circle_ring(center, outer_radius_m, segments, false)?;
    let hole = circle_ring(center, inner_radius_m, segments, true)?;
    Ok(AirspacePolygon {
        exterior,
        interiors: vec![hole],
    })
}

fn rect_polygon(
    center: GeodeticPosition,
    half_east_m: f64,
    half_north_m: f64,
) -> Result<AirspacePolygon, FlightDeckError> {
    let plane = LocalTangentPlane::new(center).map_err(|e| FlightDeckError::Airspace {
        reason: format!("LTP error: {e:?}"),
    })?;
    let corners = [
        (-half_north_m, -half_east_m),
        (-half_north_m, half_east_m),
        (half_north_m, half_east_m),
        (half_north_m, -half_east_m),
    ];
    let exterior = corners
        .iter()
        .map(|&(n, e)| {
            plane.to_geodetic(Ned {
                north_m: n,
                east_m: e,
                down_m: 0.0,
            })
        })
        .collect();
    Ok(AirspacePolygon {
        exterior,
        interiors: Vec::new(),
    })
}

fn circle_ring(
    center: GeodeticPosition,
    radius_m: f64,
    segments: usize,
    clockwise: bool,
) -> Result<Vec<GeodeticPosition>, FlightDeckError> {
    let count = segments.max(8);
    let plane = LocalTangentPlane::new(center).map_err(|e| FlightDeckError::Airspace {
        reason: format!("LTP error: {e:?}"),
    })?;
    let mut ring = Vec::with_capacity(count);
    for idx in 0..count {
        let frac = idx as f64 / count as f64;
        let angle = if clockwise { frac * TAU } else { -frac * TAU };
        let north_m = radius_m * libm::cos(angle);
        let east_m = radius_m * libm::sin(angle);
        ring.push(plane.to_geodetic(Ned {
            north_m,
            east_m,
            down_m: 0.0,
        }));
    }
    Ok(ring)
}
