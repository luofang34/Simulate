use glam::DVec3;
use sim_geo::Geodetic;

use super::{aircraft_at, library, slot, stamp_looking_north_at};
use crate::{
    PartRole, SlotCameraRole, UsdAircraftAsset, usd_pass::AircraftSlotView,
    usd_pass::pack_slot_draws,
};

#[test]
fn c172_ownship_cabin_shell_is_double_sided_in_cockpit_and_single_sided_outside() {
    let target = Geodetic::from_degrees(40.69, -74.17, 300.0);
    let stamp = stamp_looking_north_at(&target);
    let asset = UsdAircraftAsset::embedded_c172s_n172sp().unwrap();
    let lib = library(asset);
    let own = [aircraft_at(&target, true)];

    let cockpit_view = AircraftSlotView {
        stamp: &stamp,
        sun_enu: DVec3::Z,
        daylight: 1.0,
        ownship_role: SlotCameraRole::CockpitInterior,
        slot: slot(0),
        shutter_s: UsdAircraftAsset::DISPLAY_SHUTTER_S,
        hide_cockpit_controls: false,
    };
    let cockpit_draws = pack_slot_draws(&lib, &own, None, &cockpit_view);
    let own_asset = lib.get(cockpit_draws.first().unwrap().asset).unwrap();
    let part_draw = |suffix: &str| {
        cockpit_draws
            .iter()
            .find(|d| own_asset.parts()[d.part_index].prim_path.ends_with(suffix))
            .unwrap_or_else(|| panic!("missing cockpit draw for {suffix}"))
    };
    for suffix in ["/CabinWhite", "/CabinInterior", "/CabinDark", "/CabinTrim"] {
        assert!(
            part_draw(suffix).double_sided,
            "{suffix} must be double-sided inside CockpitInterior"
        );
    }
    assert!(
        !part_draw("/LiveryRegistrationPort").double_sided,
        "exterior livery decals stay single-sided"
    );

    let external_view = AircraftSlotView {
        ownship_role: SlotCameraRole::External,
        ..cockpit_view
    };
    let ext_draws = pack_slot_draws(&lib, &own, None, &external_view);
    let ext_white = ext_draws
        .iter()
        .find(|d| {
            own_asset.parts()[d.part_index]
                .prim_path
                .ends_with("/CabinWhite")
        })
        .expect("CabinWhite");
    assert!(
        !ext_white.double_sided,
        "external view keeps back-face culling"
    );
}

#[test]
fn hiding_cockpit_controls_omits_ownship_yoke_in_interior_views() {
    let target = Geodetic::from_degrees(40.69, -74.17, 300.0);
    let stamp = stamp_looking_north_at(&target);
    let asset = UsdAircraftAsset::p38g().unwrap();
    let lib = library(asset);
    let own = [aircraft_at(&target, true)];

    let mut view = AircraftSlotView {
        stamp: &stamp,
        sun_enu: DVec3::Z,
        daylight: 1.0,
        ownship_role: SlotCameraRole::PilotEye,
        slot: slot(0),
        shutter_s: UsdAircraftAsset::DISPLAY_SHUTTER_S,
        hide_cockpit_controls: false,
    };
    let shown = pack_slot_draws(&lib, &own, None, &view);
    let p38 = lib.get(shown.first().unwrap().asset).unwrap();
    let has_control = |draws: &[crate::usd_pass::PackedDraw]| {
        draws.iter().any(|d| {
            let p = &p38.parts()[d.part_index];
            p.role == PartRole::CockpitControl && p.prim_path.ends_with("/ControlWheel")
        })
    };
    assert!(has_control(&shown), "control wheel drawn by default");

    view.hide_cockpit_controls = true;
    let hidden = pack_slot_draws(&lib, &own, None, &view);
    assert!(
        !has_control(&hidden),
        "control wheel omitted when hide_cockpit_controls is true"
    );
}
