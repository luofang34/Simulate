use super::{BAR, CARD, DETAIL, FILL, LINES, PHASE, ROOT};

const PAGE: &str = include_str!("../../../index.html");
const LOADER: &str = include_str!("../../../boot.mjs");

#[test]
fn the_page_carries_every_surface_element_once() {
    for id in [ROOT, CARD, PHASE, BAR, FILL, DETAIL, LINES] {
        assert_eq!(
            PAGE.matches(&format!("id=\"{id}\"")).count(),
            1,
            "{id} in index.html"
        );
    }
}

#[test]
fn the_loader_drives_the_elements_of_the_download() {
    // The labelled lines are the app's alone.
    for id in [ROOT, PHASE, BAR, FILL, DETAIL] {
        assert!(
            LOADER.contains(&format!("getElementById(\"{id}\")")),
            "{id} in boot.mjs"
        );
    }
}

#[test]
fn the_page_styles_what_the_app_builds_by_class() {
    for class in [
        ".section",
        ".bar",
        ".fill",
        ".detail",
        ".lines",
        ".plot",
        ".legend",
        ".switches",
        ".series",
        ".swatch",
        ".value",
        ".speeds",
    ] {
        let styled = [" ", ","]
            .iter()
            .any(|end| PAGE.contains(&format!("{class}{end}")));
        assert!(styled, "{class} in index.html");
    }
}

#[test]
fn the_page_starts_trunk_with_the_loader() {
    assert!(PAGE.contains("data-initializer=\"boot.mjs\""));
}
