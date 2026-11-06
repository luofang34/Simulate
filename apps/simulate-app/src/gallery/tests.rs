use std::path::PathBuf;

use super::*;

/// Renders the airport-pattern gallery into `$SIMULATE_GALLERY_OUT` with real tiles.
#[test]
#[ignore = "downloads real-world tiles; set SIMULATE_GALLERY_OUT and run with --ignored"]
fn render_airport_pattern_gallery() -> Result<(), AppSimError> {
    let Some(out) = std::env::var_os("SIMULATE_GALLERY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    assert_eq!(render_gallery(&out)?, 9);
    assert!(out.join("06_kewr_base_to_final_scene.usda").is_file());
    Ok(())
}
