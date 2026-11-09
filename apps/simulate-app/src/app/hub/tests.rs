use super::overlay_defaults::OverlayDefaults;
use super::shown_layout;
use crate::esc_menu::AvionicsLayout;
use crate::map_rig::AvionicsPreset;
use crate::vehicles::FreeFlightVehicle;

#[test]
fn hotkey_fits_map_onto_the_hub_layout() {
    use AvionicsLayout::{BasicSix, FullGlass, HudOnly};
    let glass = Some(AvionicsPreset::GlassCockpit);
    let gauges = Some(AvionicsPreset::NoMapGauges);
    assert_eq!(
        shown_layout(false, glass, FullGlass),
        HudOnly,
        "dock hidden"
    );
    assert_eq!(shown_layout(true, glass, HudOnly), FullGlass, "dock back");
    assert_eq!(
        shown_layout(true, gauges, FullGlass),
        BasicSix,
        "preset cycled"
    );
    assert_eq!(
        shown_layout(true, Some(AvionicsPreset::SingleMovingMap), BasicSix),
        FullGlass
    );
    assert_eq!(shown_layout(true, None, BasicSix), BasicSix, "no maps");
    assert_eq!(shown_layout(true, None, HudOnly), FullGlass);
}

#[test]
fn every_listed_runway_end_of_a_database_airport_resolves_in_its_surface_graph() {
    use sim_scenery::{AirportDatabase, CatalogVerticalDatum};

    use super::spawn::record_runway_end_index;
    use crate::esc_menu::catalog::airports::runway_entries;

    let database = AirportDatabase::builtin();
    for code in ["EGLL", "RJTT", "KSFO", "VNLK", "KEWR"] {
        let (_, record) = database.by_code(code).expect("airport");
        for datum in [
            CatalogVerticalDatum::OrthometricMslEgm96,
            CatalogVerticalDatum::EllipsoidWgs84,
        ] {
            let graph = record.to_flightbag_graph(datum);
            assert!(!graph.runway_ends.is_empty(), "{code}");
            for entry in runway_entries(record) {
                let end_idx = record_runway_end_index(record, datum, Some(&entry.designator));
                assert_eq!(
                    graph.runway_ends[end_idx].designator.as_str(),
                    entry.designator,
                    "{code} {}",
                    entry.designator
                );
            }
        }
    }
}

#[test]
fn a_period_fighter_opens_without_hud_or_glass_and_leaving_it_restores_the_fit() {
    use AvionicsLayout::{FullGlass, HudOnly};
    use FreeFlightVehicle::{JsbsimC172, JsbsimP38g};
    let mut defaults = OverlayDefaults::default();
    let mut layout = FullGlass;
    assert_eq!(
        defaults.on_airframe(Some(JsbsimC172), &mut layout),
        Some([true, true])
    );
    assert_eq!(defaults.on_airframe(Some(JsbsimC172), &mut layout), None);
    assert_eq!(
        defaults.on_airframe(Some(JsbsimP38g), &mut layout),
        Some([false, false])
    );
    // A hotkey in the P-38 mirrors its hidden dock into the hub as HUD only.
    layout = HudOnly;
    assert_eq!(
        defaults.on_airframe(None, &mut layout),
        Some([true, true]),
        "a scenario after the P-38 gets the fit flown before it"
    );
    assert_eq!(layout, FullGlass);
    layout = HudOnly;
    assert_eq!(
        defaults.on_airframe(Some(JsbsimC172), &mut layout),
        None,
        "a pilot's own fit carries between modern airframes"
    );
}
