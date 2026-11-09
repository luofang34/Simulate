use glam::{DQuat, DVec3, Vec4};
use maplibre::{
    coords::LatLon,
    headless::{AltitudeReference, ViewSlotMask},
};
use sim_core::{AirframeArticulation, CameraPoseEcef, SimInstant};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

use super::*;
use crate::{
    UsdAircraftAsset, conformal_stamp_from_pose, orientation_from_ned_body_quat, symmetric_frustum,
};

pub(super) fn slot(index: u32) -> ViewSlotId {
    ViewSlotId::from_raw(index)
}

/// A camera 40 m south of `target`, level, looking north.
pub(super) fn stamp_looking_north_at(target: &Geodetic) -> ConformalFrameStamp {
    let local = LocalFrame::at(target);
    let target_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(target);
    let eye = target_ecef - local.north() * 40.0;
    let orientation =
        DQuat::from_mat3(&DMat3::from_cols(local.east(), local.up(), -local.north())).normalize();
    let frustum = symmetric_frustum(1.0, 800, 600, 0.25, 350_000.0).unwrap();
    conformal_stamp_from_pose(
        SimInstant::START,
        &CameraPoseEcef {
            position_ecef: eye,
            orientation,
        },
        frustum,
        0.0,
    )
    .unwrap()
}

pub(super) fn library(asset: UsdAircraftAsset) -> AircraftLibrary {
    AircraftLibrary::new(std::sync::Arc::new(asset))
}

pub(super) fn aircraft_at(target: &Geodetic, ownship: bool) -> AircraftInstance {
    AircraftInstance {
        placement: Placement3d {
            position: LatLon::new(target.lat_rad.to_degrees(), target.lon_rad.to_degrees()),
            altitude_meters: target.height_m,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: orientation_from_ned_body_quat(DQuat::IDENTITY),
            view_mask: ViewSlotMask::ALL,
        },
        articulation: AirframeArticulation::default(),
        ownship,
        ground_height_m: None,
    }
}

#[test]
fn the_draw_block_packs_little_endian_in_shader_order() {
    let block = DrawBlock {
        clip_from_part: Mat4::from_diagonal(Vec4::new(1.0, 2.0, 3.0, 4.0)),
        enu_from_part: Mat4::IDENTITY,
        base_color: [0.5, 0.25, 0.125, 1.0],
        emissive: [0.0, 0.0, 0.0, 0.7],
        params: [0.3, 1.0, 0.2, 2.0],
        sun: [0.0, 0.0, 1.0, 1.0],
        detail: [[0.5, 0.0, 0.3, 1.0], [2.0, -1.0, 2.5, 0.7]],
    };
    let bytes = block.to_le_bytes();
    let read = |i: usize| f32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap());
    assert_eq!(read(0), 1.0);
    assert_eq!(read(5), 2.0);
    assert_eq!(read(15), 4.0);
    assert_eq!(read(16), 1.0);
    assert_eq!(read(32), 0.5);
    assert_eq!(read(39), 0.7);
    assert_eq!(read(43), 2.0);
    assert_eq!(read(46), 1.0);
    assert_eq!(read(50), 0.3);
    assert_eq!(read(53), -1.0);
    assert_eq!(read(55), 0.7);
}

#[test]
fn an_aircraft_ahead_lands_mid_screen_inside_the_reversed_depth_range() {
    let asset = UsdAircraftAsset::embedded_c172().unwrap();
    let target = Geodetic::from_degrees(40.69, -74.17, 300.0);
    let stamp = stamp_looking_north_at(&target);
    let view = AircraftSlotView {
        stamp: &stamp,
        sun_enu: DVec3::Z,
        daylight: 1.0,
        ownship_role: SlotCameraRole::External,
        slot: slot(0),
    };
    let draws = pack_slot_draws(&library(asset), &[aircraft_at(&target, false)], None, &view);
    assert!(draws.len() > 10);
    let clip_origin = |draw: &PackedDraw| {
        let m = Mat4::from_cols_slice(
            &draw.uniform[..64]
                .as_chunks::<4>()
                .0
                .iter()
                .map(|c| f32::from_le_bytes(*c))
                .collect::<Vec<_>>(),
        );
        let clip = m * Vec4::new(0.0, 0.0, 0.0, 1.0);
        (clip.x / clip.w, clip.y / clip.w, clip.z / clip.w)
    };
    let fuselage = draws.iter().find(|d| !d.translucent).unwrap();
    let (x, y, z) = clip_origin(fuselage);
    assert!(x.abs() < 0.3 && y.abs() < 0.3, "{x} {y}");
    // About near / distance under reversed Z.
    assert!(z > 0.25 / 60.0 && z < 0.25 / 20.0, "{z}");
    let first_translucent = draws.iter().position(|d| d.translucent).unwrap();
    assert!(draws[first_translucent..].iter().all(|d| d.translucent));
}

#[test]
fn ownship_shows_only_its_nose_from_the_cockpit_and_nothing_from_the_belly() {
    let asset = UsdAircraftAsset::embedded_c172().unwrap();
    let target = Geodetic::from_degrees(40.69, -74.17, 300.0);
    let stamp = stamp_looking_north_at(&target);
    let mut view = AircraftSlotView {
        stamp: &stamp,
        sun_enu: DVec3::Z,
        daylight: 1.0,
        ownship_role: SlotCameraRole::CockpitInterior,
        slot: slot(0),
    };
    let own = [aircraft_at(&target, true)];
    let asset = library(asset);
    let cockpit = pack_slot_draws(&asset, &own, None, &view);
    let external = {
        view.ownship_role = SlotCameraRole::External;
        pack_slot_draws(&asset, &own, None, &view)
    };
    assert!(!cockpit.is_empty() && cockpit.len() < external.len());
    view.ownship_role = SlotCameraRole::BellyNadir;
    assert!(pack_slot_draws(&asset, &own, None, &view).is_empty());
}

#[test]
fn ownship_draws_the_selected_airframe_and_traffic_keeps_the_scene_asset() {
    let target = Geodetic::from_degrees(40.69, -74.17, 300.0);
    let stamp = stamp_looking_north_at(&target);
    let view = AircraftSlotView {
        stamp: &stamp,
        sun_enu: DVec3::Z,
        daylight: 1.0,
        ownship_role: SlotCameraRole::External,
        slot: slot(0),
    };
    let mut library = library(UsdAircraftAsset::embedded_c172().unwrap());
    let own = [aircraft_at(&target, true)];
    let traffic = [aircraft_at(&target, false)];
    let c172 = pack_slot_draws(&library, &own, None, &view);
    assert!(c172.iter().all(|draw| draw.asset == AssetKey::Scene));

    library
        .select_ownship(OwnshipVisualKind::QuadrotorX500)
        .unwrap();
    let quad = pack_slot_draws(&library, &own, None, &view);
    assert!(!quad.is_empty());
    assert!(
        quad.iter()
            .all(|draw| draw.asset == AssetKey::QuadrotorX500)
    );
    assert_ne!(quad.len(), c172.len());
    let traffic_draws = pack_slot_draws(&library, &traffic, None, &view);
    assert_eq!(traffic_draws.len(), c172.len());
    assert!(
        traffic_draws
            .iter()
            .all(|draw| draw.asset == AssetKey::Scene)
    );

    library.select_ownship(OwnshipVisualKind::Hidden).unwrap();
    assert!(pack_slot_draws(&library, &own, None, &view).is_empty());
    assert_eq!(
        pack_slot_draws(&library, &traffic, None, &view).len(),
        c172.len()
    );
}

#[test]
fn a_procedural_airframe_is_built_once_across_switches() {
    let mut library = library(UsdAircraftAsset::embedded_c172().unwrap());
    assert_eq!(library.resident().count(), 1);
    for kind in [
        OwnshipVisualKind::LunarLander,
        OwnshipVisualKind::C172,
        OwnshipVisualKind::FixedWingUav,
        OwnshipVisualKind::LunarLander,
    ] {
        library.select_ownship(kind).unwrap();
        assert_eq!(library.ownship_kind(), kind);
    }
    let first: *const UsdAircraftAsset = library.get(AssetKey::LunarLander).unwrap();
    library
        .select_ownship(OwnshipVisualKind::LunarLander)
        .unwrap();
    let again: *const UsdAircraftAsset = library.get(AssetKey::LunarLander).unwrap();
    assert_eq!(first, again);
    assert_eq!(library.resident().count(), 3);
}

#[test]
fn polar_skiway_at_nzsp_packs_opaque_groomed_deck_and_flags() {
    let spec = flightbag::RunwayEndSpec {
        designator: flightbag::RunwayDesignator::parse("02").unwrap(),
        threshold: world::GeodeticPosition::wgs84(-89.9975, 139.2729, 2_835.0),
        true_bearing_rad: 20.0_f64.to_radians(),
        length_m: 3_658.0,
        width_m: 61.0,
    };
    let rwy = sim_scenery::LandingRunwayGeometry::from_runway_end_spec(&spec);
    let target = Geodetic::from_degrees(-89.9975, 139.2729, 2_845.0);
    let stamp = stamp_looking_north_at(&target);
    let mut library = library(UsdAircraftAsset::embedded_c172().unwrap());
    let ski_key = PolarSkiwayKey::from_runway(&rwy, stamp.render_body);
    // On Body::EARTH (R = 6,371,008.8 m vs WGS84 polar radius c = 6,399,593.6 m), a 3,658.0 m
    // WGS84 polar skiway spans ~3,643.28 m on the rendered sphere.
    assert!(
        (f64::from(ski_key.after_threshold_cm) * 0.01 - 3_643.28).abs() < 0.25,
        "expected spherical NZSP skiway length ~3643.28 m, got {} cm",
        ski_key.after_threshold_cm,
    );
    let key = library.ensure_polar_skiway(ski_key);
    let view = AircraftSlotView {
        stamp: &stamp,
        sun_enu: DVec3::Z,
        daylight: 1.0,
        ownship_role: SlotCameraRole::CockpitInterior,
        slot: slot(0),
    };
    let draws = pack_slot_draws(&library, &[], Some((key, &rwy)), &view);
    assert_eq!(draws.len(), 4);
    assert!(draws.iter().all(|d| !d.translucent && d.asset == key));
}

fn nzsp_overhead_stamp(vp: [u32; 2]) -> ConformalFrameStamp {
    // Overhead pitched -12 deg down along 020 deg true (reproducing the user's NZSP view),
    // with an intentionally mismatched ownship geoid undulation (-29.8 m vs threshold's) to
    // prove zero vertical datum leak.
    let eye = Geodetic::from_degrees(-89.9975, 139.2729, 2_835.0 - 29.8 + 260.0);
    let local = sim_geo::LocalFrame::at(&eye);
    let brg = 20.0_f64.to_radians();
    let pitch = -12.0_f64.to_radians();
    let horiz_fwd = local.north() * brg.cos() + local.east() * brg.sin();
    let fwd = (horiz_fwd * pitch.cos() + local.up() * pitch.sin()).normalize();
    let right = fwd.cross(local.up()).normalize();
    let cam_up = right.cross(fwd).normalize();
    let pose = sim_core::CameraPoseEcef {
        position_ecef: sim_geo::BodyEllipsoid::WGS84.geodetic_to_fixed(&eye),
        orientation: DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -fwd)).normalize(),
    };
    let frustum = crate::symmetric_frustum(
        50.0_f64.to_radians(),
        vp[0],
        vp[1],
        crate::DEFAULT_NEAR_CLIP_M,
        crate::DEFAULT_FAR_CLIP_M,
    )
    .expect("frustum");
    crate::conformal_stamp_from_pose(sim_core::SimInstant::START, &pose, frustum, -29.8)
        .expect("stamp")
}

#[test]
fn polar_skiway_deck_registers_pixel_for_pixel_with_sphere_and_svs_at_nzsp() {
    let db = sim_scenery::AirportDatabase::builtin();
    let (_, nzsp) = db.by_code("NZSP").expect("NZSP");
    let rwy = nzsp
        .landing_runway(
            "02T",
            sim_scenery::CatalogVerticalDatum::OrthometricMslEgm96,
        )
        .expect("NZSP 02T");
    let vp = [916_u32, 876_u32];
    let stamp = nzsp_overhead_stamp(vp);

    let mut library = library(UsdAircraftAsset::embedded_c172().unwrap());
    let ski_key = PolarSkiwayKey::from_runway(&rwy, stamp.render_body);
    let key = library.ensure_polar_skiway(ski_key);
    let asset = library.get(key).expect("skiway asset");
    let deck_idx = asset
        .parts()
        .iter()
        .position(|p| p.prim_path == "/PolarSkiway/GroomedDeck")
        .expect("GroomedDeck");
    let draws = pack_slot_draws(
        &library,
        &[],
        Some((key, &rwy)),
        &AircraftSlotView {
            stamp: &stamp,
            sun_enu: DVec3::Z,
            daylight: 1.0,
            ownship_role: SlotCameraRole::CockpitInterior,
            slot: slot(0),
        },
    );
    let deck_draw = draws
        .iter()
        .find(|d| d.part_index == deck_idx)
        .expect("deck draw");
    let mut cols = [0.0_f32; 16];
    for (dst, bytes) in cols
        .iter_mut()
        .zip(deck_draw.uniform[..64].as_chunks::<4>().0)
    {
        *dst = f32::from_le_bytes(*bytes);
    }
    let clip_from_part = glam::Mat4::from_cols_array(&cols);

    let deck_verts = &asset.parts()[deck_idx].vertices;
    // Far-left and far-right corners of the final quad of `/PolarSkiway/GroomedDeck`.
    let n = deck_verts.len();
    let far_right_v = deck_verts[n - 2].position;
    let far_left_v = deck_verts[n - 1].position;
    let project_part = |p: [f32; 3]| -> [f32; 2] {
        let clip = clip_from_part * glam::Vec4::new(p[0], p[1], p[2], 1.0);
        let ndc = clip.truncate() / clip.w;
        [
            (ndc.x * 0.5 + 0.5) * vp[0] as f32,
            (0.5 - ndc.y * 0.5) * vp[1] as f32,
        ]
    };
    let deck_far_mid = {
        let l = project_part(far_left_v);
        let r = project_part(far_right_v);
        [(l[0] + r[0]) * 0.5, (l[1] + r[1]) * 0.5]
    };
    let far_msl_geo = Geodetic {
        height_m: rwy.far_elevation_msl_m + 0.20,
        ..rwy.far_end
    };
    let (sphere_far_px, _) = crate::project_geodetic_to_screen(&stamp, &far_msl_geo, 0.0, vp)
        .expect("far_end on screen");
    let err_px = (deck_far_mid[0] - sphere_far_px[0]).hypot(deck_far_mid[1] - sphere_far_px[1]);
    assert!(
        err_px < 0.25,
        "NZSP polar skiway far end {deck_far_mid:?} must register with Body::EARTH far_end {sphere_far_px:?} (<0.25 px, got {err_px:.3} px)"
    );
}
