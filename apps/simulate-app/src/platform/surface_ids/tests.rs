use super::{BAR, DETAIL, FILL, PHASE, ROOT};

const PAGE: &str = include_str!("../../../index.html");
const LOADER: &str = include_str!("../../../boot.mjs");

#[test]
fn the_page_and_its_loader_carry_every_surface_element() {
    for id in [ROOT, PHASE, BAR, FILL, DETAIL] {
        assert_eq!(
            PAGE.matches(&format!("id=\"{id}\"")).count(),
            1,
            "{id} in index.html"
        );
        assert!(
            LOADER.contains(&format!("getElementById(\"{id}\")")),
            "{id} in boot.mjs"
        );
    }
}

#[test]
fn the_page_starts_trunk_with_the_loader() {
    assert!(PAGE.contains("data-initializer=\"boot.mjs\""));
}
