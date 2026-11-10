use super::*;

#[test]
fn embedded_file_matches_pinned_digest_and_script_crop() {
    assert_eq!(
        ContentHash::of_bytes(MOLA_JEZERO_BYTES).to_hex(),
        MOLA_JEZERO_SHA256
    );
    assert_eq!(MOLA_JEZERO_BYTES.len(), HEADER_LEN + 256 * 256 * 2);

    let grid = DemElevationGrid::mars_jezero_mola().expect("load");
    grid.validate().expect("valid grid");
    assert_eq!((grid.rows, grid.cols), (256, 256));
    assert_eq!(grid.body, NaifId::MARS);
    // Crop lines [3144, 3400) and samples [9786, 10042) of the 128 px/deg source; bounds are
    // bin centres.
    let lon_min_deg = grid.lon_min_rad.to_degrees();
    let lon_max_deg = grid.lon_max_rad.to_degrees();
    assert!((lon_min_deg - (9786.5 / 128.0)).abs() < 1e-12);
    assert!((lon_max_deg - (10041.5 / 128.0)).abs() < 1e-12);
    // Geodetic bounds sit a fifth of a degree north of the planetocentric 17.4531..19.4453.
    let lat_min_deg = grid.lat_min_rad.to_degrees();
    let lat_max_deg = grid.lat_max_rad.to_degrees();
    assert!((lat_min_deg - 17.4531 - 0.2).abs() < 0.02, "{lat_min_deg}");
    assert!((lat_max_deg - 19.4453 - 0.2).abs() < 0.02, "{lat_max_deg}");
}

#[test]
fn rows_run_south_to_north_with_source_samples_unaltered() {
    let grid = DemElevationGrid::mars_jezero_mola().expect("load");
    let source_sample = |line: usize, col: usize| -> f64 {
        let at = HEADER_LEN + (line * 256 + col) * 2;
        f64::from(i16::from_be_bytes([
            MOLA_JEZERO_BYTES[at],
            MOLA_JEZERO_BYTES[at + 1],
        ])) + MARS_AREOID_ABOVE_IAU_ELLIPSOID_M
    };
    assert_eq!(grid.cell_at(0, 0), Some(source_sample(255, 0)));
    assert_eq!(grid.cell_at(255, 255), Some(source_sample(0, 255)));
    assert_eq!(grid.cell_at(100, 7), Some(source_sample(155, 7)));
}

#[test]
fn corrupt_payload_is_rejected() {
    let mut bytes = MOLA_JEZERO_BYTES.to_vec();
    bytes.pop();
    assert!(matches!(
        parse_window(&bytes),
        Err(MolaLoadError::Malformed { .. })
    ));
    assert!(matches!(
        parse_window(b"NOTMOLA1"),
        Err(MolaLoadError::Malformed { .. })
    ));
}
