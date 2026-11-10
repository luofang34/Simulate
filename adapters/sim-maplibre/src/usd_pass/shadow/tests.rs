use glam::{DVec3, Vec4};
use sim_geo::Geodetic;

use super::{SHADOW_BYTES, ShadowFrame, ownship_shadow};
use crate::usd_pass::tests::{aircraft_at, library, slot, stamp_looking_north_at};
use crate::{SlotCameraRole, UsdAircraftAsset, usd_pass::AircraftSlotView};

const SUN_SOUTH_WEST: DVec3 = DVec3::new(-0.5, -0.6, 0.62);

fn view<'a>(stamp: &'a crate::ConformalFrameStamp, daylight: f32) -> AircraftSlotView<'a> {
    AircraftSlotView {
        stamp,
        sun_enu: SUN_SOUTH_WEST.normalize(),
        daylight,
        ownship_role: SlotCameraRole::External,
        slot: slot(0),
        shutter_s: crate::UsdAircraftAsset::DISPLAY_SHUTTER_S,
    }
}

#[test]
fn the_light_volume_holds_the_airframe_and_its_ground_shadow_falls_down_sun() {
    let target = Geodetic::from_degrees(40.69, -74.17, 30.0);
    let stamp = stamp_looking_north_at(&target);
    let library = library(UsdAircraftAsset::embedded_c172().unwrap());
    let mut own = aircraft_at(&target, true);
    own.ground_height_m = Some(20.0);
    let frame = ownship_shadow(&library, &[own], &view(&stamp, 1.0)).unwrap();

    let centre = crate::usd_pass::enu_from_aircraft(&stamp, &own.placement)
        .unwrap()
        .w_axis
        .truncate()
        .as_vec3();
    let light = frame.light_from_enu * centre.extend(1.0);
    assert!(light.x.abs() < 1e-3 && light.y.abs() < 1e-3, "{light}");
    assert!((0.0..1.0).contains(&light.z), "{light}");

    let ground = frame.ground.unwrap();
    let offset = (ground.centre - centre).as_dvec3();
    // 10 m above the ground, the shadow centre lies 10 m lower and away from the Sun.
    assert!(
        (offset.z + 10.0 - super::GROUND_LIFT_M).abs() < 0.05,
        "{offset}"
    );
    assert!(offset.x > 0.0 && offset.y > 0.0, "{offset}");
    assert!(ground.strength > 0.4);
}

#[test]
fn an_aircraft_on_its_gear_shadows_the_ground_it_reports() {
    let target = Geodetic::from_degrees(40.69, -74.17, 30.0);
    let stamp = stamp_looking_north_at(&target);
    let library = library(UsdAircraftAsset::embedded_c172().unwrap());
    let mut own = aircraft_at(&target, true);
    own.ground_height_m = Some(own.placement.altitude_meters - 1.3);
    own.articulation.gear_wow = [true, true, true];
    let frame = ownship_shadow(&library, &[own], &view(&stamp, 1.0)).unwrap();
    let centre = crate::usd_pass::enu_from_aircraft(&stamp, &own.placement)
        .unwrap()
        .w_axis
        .truncate();
    let drop = f64::from(frame.ground.unwrap().centre.z) - centre.z;
    assert!((drop + 1.3 - super::GROUND_LIFT_M).abs() < 0.05, "{drop}");
}

#[test]
fn night_unknown_ground_and_disabled_blocks_turn_lookups_off() {
    let target = Geodetic::from_degrees(40.69, -74.17, 30.0);
    let stamp = stamp_looking_north_at(&target);
    let library = library(UsdAircraftAsset::embedded_c172().unwrap());
    let own = aircraft_at(&target, true);
    assert!(ownship_shadow(&library, &[own], &view(&stamp, 0.0)).is_none());
    let traffic = aircraft_at(&target, false);
    assert!(ownship_shadow(&library, &[traffic], &view(&stamp, 1.0)).is_none());
    let airframe_only = ownship_shadow(&library, &[own], &view(&stamp, 1.0)).unwrap();
    assert!(airframe_only.ground.is_none());

    let read = |bytes: &[u8; SHADOW_BYTES], i: usize| {
        f32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap())
    };
    let off = ShadowFrame::block(None);
    assert_eq!(read(&off, 43), 0.0, "enabled flag");
    let on = ShadowFrame::block(Some(&airframe_only));
    assert_eq!(read(&on, 43), 1.0);
    assert_eq!(read(&on, 35), 0.0, "no ground receiver strength");
    let column = |i: usize| {
        Vec4::new(
            read(&on, i),
            read(&on, i + 1),
            read(&on, i + 2),
            read(&on, i + 3),
        )
    };
    assert_eq!(column(16), airframe_only.light_from_enu.x_axis);
}
