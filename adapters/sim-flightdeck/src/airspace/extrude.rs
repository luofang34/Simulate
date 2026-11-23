//! Local East-North-Up polygon curtain, horizontal shelf cap, and rim extrusion for 3D airspace
//! volumes and TFRs (supporting `Flightbag` manifold `full` / `walls` / `floor` / `roof` primitives
//! and suppressing Web Mercator MVT tile-boundary seam edges).

use core::f64::consts::PI;

use crate::airspace::catalog::circle_polygon;
use crate::airspace::{AirspaceClassStyle, AirspaceRenderVolume};
use crate::telemetry::FEET_TO_METRES;
use flightbag_core::{AirspacePolygon, AirspaceVolume, AltitudeDatum, VerticalLimit};
use world_core::{DecimalYear, GeodeticPosition, LocalTangentPlane};
use world_notices::{LateralShape, NoticeAltitude, TfrNotice};

#[derive(Clone, Copy, PartialEq, Eq)]
enum ExtrudePart {
    Full,
    Walls,
    Floor,
    Roof,
}

#[derive(Clone, Copy)]
struct TileSeamBounds {
    west_lon: f64,
    south_lat: f64,
    east_lon: f64,
    north_lat: f64,
}

pub(super) struct VolumeExtrudeSpec<'a> {
    pub airspace_key: &'a str,
    pub label: &'a str,
    pub sector_tag: Option<&'a str>,
    pub style: AirspaceClassStyle,
    pub active: bool,
    pub is_sfc_floor: bool,
    pub has_floating_floor: bool,
    pub polygons: &'a [AirspacePolygon],
    pub floor_msl_m: f64,
    pub ceil_msl_m: f64,
    pub own_alt_msl_m: f64,
}

struct CurtainGeometry {
    wall_positions: Vec<[f32; 3]>,
    wall_indices: Vec<u32>,
    rim_segments: Vec<[[f32; 3]; 2]>,
    best_anchor: [f32; 3],
    best_dist: f64,
}

impl CurtainGeometry {
    fn new(z_hi: f32) -> Self {
        Self {
            wall_positions: Vec::new(),
            wall_indices: Vec::new(),
            rim_segments: Vec::new(),
            best_anchor: [0.0, 1_000.0, z_hi],
            best_dist: f64::INFINITY,
        }
    }

    fn append_walls_and_rims(
        &mut self,
        ring_geo: &[GeodeticPosition],
        ring_en: &[(f64, f64)],
        (z_lo, z_hi): (f32, f32),
        (emit_walls, emit_lo_rim, emit_hi_rim): (bool, bool, bool),
        seam: Option<TileSeamBounds>,
    ) {
        let count = ring_en.len();
        for (idx, &(e0, n0)) in ring_en.iter().enumerate() {
            let d = libm::hypot(e0, n0);
            if d < self.best_dist {
                self.best_dist = d;
                self.best_anchor = [e0 as f32, n0 as f32, (z_lo + z_hi) * 0.5];
            }
            let next = if idx.wrapping_add(1) == count {
                0
            } else {
                idx.wrapping_add(1)
            };
            let Some(&(e1, n1)) = ring_en.get(next) else {
                continue;
            };
            if let (Some(bounds), Some(g0), Some(g1)) =
                (seam, ring_geo.get(idx), ring_geo.get(next))
                && is_on_tile_seam(*g0, *g1, bounds)
            {
                continue;
            }
            let p00 = [e0 as f32, n0 as f32, z_lo];
            let p10 = [e1 as f32, n1 as f32, z_lo];
            let p11 = [e1 as f32, n1 as f32, z_hi];
            let p01 = [e0 as f32, n0 as f32, z_hi];
            if emit_walls {
                let b0 = self.wall_positions.len() as u32;
                let (b1, b2, b3) = (b0.wrapping_add(1), b0.wrapping_add(2), b0.wrapping_add(3));
                self.wall_positions.extend_from_slice(&[p00, p10, p11, p01]);
                self.wall_indices
                    .extend_from_slice(&[b0, b1, b2, b0, b2, b3, b0, b2, b1, b0, b3, b2]);
            }
            if emit_lo_rim {
                self.rim_segments.push([p00, p10]);
            }
            if emit_hi_rim {
                self.rim_segments.push([p01, p11]);
            }
        }
    }

    fn append_horizontal_cap(&mut self, ext_en: &[(f64, f64)], z_cap: f32) {
        if ext_en.len() < 3 {
            return;
        }
        let base = self.wall_positions.len() as u32;
        for &(e, n) in ext_en {
            self.wall_positions.push([e as f32, n as f32, z_cap]);
        }
        let tris = ear_clip_ring(ext_en);
        for (a, b, c) in tris {
            let ia = base.wrapping_add(a);
            let ib = base.wrapping_add(b);
            let ic = base.wrapping_add(c);
            self.wall_indices
                .extend_from_slice(&[ia, ib, ic, ia, ic, ib]);
        }
    }
}

pub(super) fn extrude_polygons_enu(
    spec: &VolumeExtrudeSpec<'_>,
    plane: &LocalTangentPlane,
    cull_radius_m: f64,
) -> Option<AirspaceRenderVolume> {
    let (part, seam) = parse_mvt_sector_tag(spec.sector_tag);
    let z_lo = (spec.floor_msl_m - spec.own_alt_msl_m) as f32;
    let z_hi = (spec.ceil_msl_m.max(spec.floor_msl_m + 30.0) - spec.own_alt_msl_m) as f32;
    let mut geom = CurtainGeometry::new(z_hi);
    let mut ownship_inside_lateral = false;

    for poly in spec.polygons {
        if append_polygon_primitive(
            poly,
            plane,
            cull_radius_m,
            (z_lo, z_hi),
            (part, spec.is_sfc_floor, spec.has_floating_floor),
            seam,
            &mut geom,
        ) {
            ownship_inside_lateral = true;
        }
    }

    if geom.wall_positions.is_empty() && geom.rim_segments.is_empty() {
        return None;
    }
    let tracks_containment = matches!(part, ExtrudePart::Full | ExtrudePart::Walls);
    let ownship_inside = tracks_containment
        && ownship_inside_lateral
        && spec.own_alt_msl_m >= spec.floor_msl_m
        && spec.own_alt_msl_m <= spec.ceil_msl_m;

    Some(AirspaceRenderVolume {
        airspace_key: spec.airspace_key.to_owned(),
        label: spec.label.to_owned(),
        style: spec.style,
        rgb: spec.style.rgb(),
        active: spec.active,
        ownship_inside,
        wall_positions_enu_m: geom.wall_positions,
        wall_indices: geom.wall_indices,
        rim_segments_enu_m: geom.rim_segments,
        label_anchor_enu_m: geom.best_anchor,
    })
}

fn append_polygon_primitive(
    poly: &AirspacePolygon,
    plane: &LocalTangentPlane,
    cull_radius_m: f64,
    (z_lo, z_hi): (f32, f32),
    (part, is_sfc_floor, has_floating_floor): (ExtrudePart, bool, bool),
    seam: Option<TileSeamBounds>,
    geom: &mut CurtainGeometry,
) -> bool {
    let ext_en = project_ring_en(&poly.exterior, plane);
    if ext_en.len() < 3 {
        return false;
    }
    let min_d = ext_en
        .iter()
        .fold(f64::INFINITY, |acc, &(e, n)| acc.min(libm::hypot(e, n)));
    let inside_ext = origin_in_ring_en(&ext_en);
    if !inside_ext && min_d > cull_radius_m {
        return false;
    }
    let in_hole = poly
        .interiors
        .iter()
        .any(|h| origin_in_ring_en(&project_ring_en(h, plane)));

    let flags = match part {
        ExtrudePart::Full => (true, true, true),
        ExtrudePart::Walls => (true, is_sfc_floor, false),
        ExtrudePart::Floor => (false, true, false),
        ExtrudePart::Roof => (false, false, true),
    };
    geom.append_walls_and_rims(&poly.exterior, &ext_en, (z_lo, z_hi), flags, seam);
    for hole in &poly.interiors {
        let hole_en = project_ring_en(hole, plane);
        if hole_en.len() >= 3 {
            geom.append_walls_and_rims(hole, &hole_en, (z_lo, z_hi), flags, seam);
        }
    }
    if poly.interiors.is_empty() {
        match part {
            ExtrudePart::Floor => geom.append_horizontal_cap(&ext_en, z_lo),
            ExtrudePart::Roof => geom.append_horizontal_cap(&ext_en, z_hi),
            ExtrudePart::Full => {
                if has_floating_floor {
                    geom.append_horizontal_cap(&ext_en, z_lo);
                }
                geom.append_horizontal_cap(&ext_en, z_hi);
            }
            ExtrudePart::Walls => {}
        }
    }
    inside_ext && !in_hole
}

pub(super) fn extrude_tfr_enu(
    tfr: &TfrNotice,
    own_geo: GeodeticPosition,
    own_alt_msl_m: f64,
    ground_elev_msl_m: f64,
    epoch: DecimalYear,
    plane: &LocalTangentPlane,
    cull_radius_m: f64,
) -> Option<AirspaceRenderVolume> {
    let poly = match &tfr.lateral {
        LateralShape::Circle { center, radius_m } => circle_polygon(*center, *radius_m, 28).ok()?,
        LateralShape::Polygon { vertices } => AirspacePolygon {
            exterior: vertices.clone(),
            interiors: Vec::new(),
        },
    };
    let floor_m = match tfr.vertical.floor {
        NoticeAltitude::Surface => ground_elev_msl_m,
        other => other.approximate_msl_m(),
    };
    let ceil_m = match tfr.vertical.ceiling {
        NoticeAltitude::Surface => ground_elev_msl_m + 300.0,
        other => other.approximate_msl_m(),
    };
    let label = format!(
        "TFR {} • {:.0}–{:.0} FT",
        tfr.id,
        floor_m / FEET_TO_METRES,
        ceil_m / FEET_TO_METRES
    );
    let mut rendered = extrude_polygons_enu(
        &VolumeExtrudeSpec {
            airspace_key: tfr.id.as_str(),
            label: &label,
            sector_tag: None,
            style: AirspaceClassStyle::TemporaryFlightRestriction,
            active: true,
            is_sfc_floor: matches!(tfr.vertical.floor, NoticeAltitude::Surface),
            has_floating_floor: !matches!(tfr.vertical.floor, NoticeAltitude::Surface),
            polygons: &[poly],
            floor_msl_m: floor_m,
            ceil_msl_m: ceil_m,
            own_alt_msl_m,
        },
        plane,
        cull_radius_m,
    )?;
    rendered.ownship_inside = tfr.contains(own_geo, own_alt_msl_m, epoch);
    Some(rendered)
}

fn parse_mvt_sector_tag(sector: Option<&str>) -> (ExtrudePart, Option<TileSeamBounds>) {
    let Some(raw) = sector else {
        return (ExtrudePart::Full, None);
    };
    let mut it = raw.split(':');
    if it.next() != Some("mvt") {
        return (ExtrudePart::Full, None);
    }
    let part = match it.next() {
        Some("walls") => ExtrudePart::Walls,
        Some("floor") => ExtrudePart::Floor,
        Some("roof") => ExtrudePart::Roof,
        _ => ExtrudePart::Full,
    };
    let seam = match (
        it.next().and_then(|s| s.parse::<u8>().ok()),
        it.next().and_then(|s| s.parse::<u32>().ok()),
        it.next().and_then(|s| s.parse::<u32>().ok()),
    ) {
        (Some(z), Some(tx), Some(ty)) => Some(tile_seam_bounds(z, tx, ty)),
        _ => None,
    };
    (part, seam)
}

fn tile_seam_bounds(zoom: u8, tile_x: u32, tile_y: u32) -> TileSeamBounds {
    let n = f64::from(1u32 << zoom.min(24));
    let lon_of = |x: u32| (f64::from(x) / n) * 360.0 - 180.0;
    let lat_of = |y: u32| {
        let v = f64::from(y) / n;
        libm::atan(libm::sinh(PI * (1.0 - 2.0 * v))).to_degrees()
    };
    TileSeamBounds {
        west_lon: lon_of(tile_x),
        east_lon: lon_of(tile_x.saturating_add(1)),
        north_lat: lat_of(tile_y),
        south_lat: lat_of(tile_y.saturating_add(1)),
    }
}

fn is_on_tile_seam(a: GeodeticPosition, b: GeodeticPosition, s: TileSeamBounds) -> bool {
    let tol = 2.0e-5;
    let on_west =
        (a.longitude_deg - s.west_lon).abs() <= tol && (b.longitude_deg - s.west_lon).abs() <= tol;
    let on_east =
        (a.longitude_deg - s.east_lon).abs() <= tol && (b.longitude_deg - s.east_lon).abs() <= tol;
    let on_north =
        (a.latitude_deg - s.north_lat).abs() <= tol && (b.latitude_deg - s.north_lat).abs() <= tol;
    let on_south =
        (a.latitude_deg - s.south_lat).abs() <= tol && (b.latitude_deg - s.south_lat).abs() <= tol;
    on_west || on_east || on_north || on_south
}

fn ear_clip_ring(pts: &[(f64, f64)]) -> Vec<(u32, u32, u32)> {
    let mut indices: Vec<usize> = (0..pts.len()).collect();
    if indices.len() >= 2
        && let (Some(&f), Some(&l)) = (indices.first(), indices.last())
        && let (Some(&(fx, fy)), Some(&(lx, ly))) = (pts.get(f), pts.get(l))
        && libm::hypot(fx - lx, fy - ly) < 1e-4
    {
        indices.pop();
    }
    let mut area2 = 0.0;
    for (pos, &i0) in indices.iter().enumerate() {
        let next = pos.wrapping_add(1) % indices.len().max(1);
        if let (Some(&i1), Some(&(x0, y0))) = (indices.get(next), pts.get(i0))
            && let Some(&(x1, y1)) = pts.get(i1)
        {
            area2 += x0 * y1 - x1 * y0;
        }
    }
    if area2 < 0.0 {
        indices.reverse();
    }

    let mut tris = Vec::with_capacity(indices.len().saturating_sub(2));
    let mut guard = indices
        .len()
        .saturating_mul(indices.len())
        .saturating_add(4);
    while indices.len() > 3 && guard > 0 {
        guard = guard.saturating_sub(1);
        let len = indices.len();
        let mut clipped = false;
        for pos in 0..len {
            let prev = pos.checked_sub(1).unwrap_or(len.saturating_sub(1));
            let next = pos.wrapping_add(1) % len;
            let (Some(&ia), Some(&ib), Some(&ic)) =
                (indices.get(prev), indices.get(pos), indices.get(next))
            else {
                continue;
            };
            if is_valid_ear(pts, &indices, (ia, ib, ic)) {
                tris.push((ia as u32, ib as u32, ic as u32));
                indices.remove(pos);
                clipped = true;
                break;
            }
        }
        if !clipped {
            break;
        }
    }
    if let (Some(&ia), Some(&ib), Some(&ic)) = (indices.first(), indices.get(1), indices.get(2))
        && indices.len() == 3
    {
        tris.push((ia as u32, ib as u32, ic as u32));
    }
    tris
}

fn is_valid_ear(pts: &[(f64, f64)], active: &[usize], (ia, ib, ic): (usize, usize, usize)) -> bool {
    let (Some(&a), Some(&b), Some(&c)) = (pts.get(ia), pts.get(ib), pts.get(ic)) else {
        return false;
    };
    let cross = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    if cross <= 1e-8 {
        return false;
    }
    for &ip in active {
        if ip == ia || ip == ib || ip == ic {
            continue;
        }
        if let Some(&p) = pts.get(ip) {
            let c1 = (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0);
            let c2 = (c.0 - b.0) * (p.1 - b.1) - (c.1 - b.1) * (p.0 - b.0);
            let c3 = (a.0 - c.0) * (p.1 - c.1) - (a.1 - c.1) * (p.0 - c.0);
            if c1 >= -1e-8 && c2 >= -1e-8 && c3 >= -1e-8 {
                return false;
            }
        }
    }
    true
}

pub(super) fn min_rim_horiz_m(vol: &AirspaceRenderVolume) -> f64 {
    vol.wall_positions_enu_m
        .iter()
        .fold(f64::INFINITY, |acc, p| {
            let e = f64::from(*p.first().unwrap_or(&0.0));
            let n = f64::from(*p.get(1).unwrap_or(&0.0));
            acc.min(libm::hypot(e, n))
        })
}

pub(super) fn resolve_limit_msl_m(limit: VerticalLimit, ground_elev_msl_m: f64) -> f64 {
    match limit.datum {
        AltitudeDatum::Surface => ground_elev_msl_m,
        AltitudeDatum::Agl => ground_elev_msl_m + limit.metres(),
        AltitudeDatum::Msl | AltitudeDatum::FlightLevel | AltitudeDatum::Unlimited => {
            limit.metres()
        }
    }
}

pub(super) fn format_static_label(vol: &AirspaceVolume) -> String {
    let lo = match vol.lower.datum {
        AltitudeDatum::Surface => "SFC".to_owned(),
        AltitudeDatum::Agl => format!("{} AGL", vol.lower.value_ft),
        _ => format!("{}", vol.lower.value_ft),
    };
    format!("{} ({lo}–{} FT)", vol.name, vol.upper.value_ft)
}

fn project_ring_en(ring: &[GeodeticPosition], plane: &LocalTangentPlane) -> Vec<(f64, f64)> {
    ring.iter()
        .filter_map(|pt| {
            plane
                .to_ned(GeodeticPosition::wgs84(
                    pt.latitude_deg,
                    pt.longitude_deg,
                    0.0,
                ))
                .ok()
                .map(|ned| (ned.east_m, ned.north_m))
        })
        .collect()
}

fn origin_in_ring_en(ring: &[(f64, f64)]) -> bool {
    let count = ring.len();
    if count < 3 {
        return false;
    }
    let mut inside = false;
    for (idx, &(xi, yi)) in ring.iter().enumerate() {
        let prev = idx.checked_sub(1).unwrap_or(count.saturating_sub(1));
        let Some(&(xj, yj)) = ring.get(prev) else {
            return false;
        };
        if (yi > 0.0) != (yj > 0.0) {
            let denom = yj - yi;
            if denom.abs() > 1e-12 {
                let x_int = xi - yi * (xj - xi) / denom;
                if x_int >= 0.0 {
                    inside = !inside;
                }
            }
        }
    }
    inside
}
