use super::*;

#[test]
fn empty_pixels_read_the_callers_sky() {
    assert_eq!(sky_where_empty(301.25, 220.0), 301.25);
    assert_eq!(sky_where_empty(NO_RADIOMETRY_K as f32, 220.0), 220.0);
    assert_eq!(sky_where_empty(f32::NAN, 220.0), 220.0);
    assert_eq!(sky_where_empty(0.0, 220.0), 0.0);
}
