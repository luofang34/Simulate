use super::{BAR, CARD, DETAIL, FILL, LINES, PHASE, ROOT, WRITER_SCRIPT};

const PAGE: &str = include_str!("../../../index.html");
const LOADER: &str = include_str!("../../../boot.mjs");
const WRITER: &str = include_str!("../../../opfs-writer.js");

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
        ".series",
        ".swatch",
        ".role",
        ".value",
        ".speeds",
        ".steering",
        ".steering-head",
        ".steering-others",
        ".sensor-controls",
        ".sensor-rows",
        ".fault-switch",
        ".gnss-checks",
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

#[test]
fn the_page_ships_the_store_writer_and_it_writes_through_sync_handles() {
    assert!(
        PAGE.contains(&format!(r#"rel="copy-file" href="{WRITER_SCRIPT}""#)),
        "index.html copies {WRITER_SCRIPT}"
    );
    // The protocol OpfsStore's writer speaks: an id per write, and an
    // answer that is ok, an error, or unsupported.
    for part in [
        "const { id, root, path, bytes } = event.data",
        "createSyncAccessHandle",
        "postMessage({ id, ok: true })",
        "unsupported: true",
    ] {
        assert!(WRITER.contains(part), "{WRITER_SCRIPT} has {part}");
    }
}
