use super::shown_layout;
use crate::esc_menu::AvionicsLayout;
use crate::map_rig::AvionicsPreset;

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
