use tracing::Level;

use super::console_shows;

#[test]
fn the_console_takes_events_from_info_up_and_no_span() {
    assert!(console_shows(true, Level::ERROR));
    assert!(console_shows(true, Level::INFO));
    assert!(!console_shows(true, Level::DEBUG));
    assert!(!console_shows(true, Level::TRACE));
    for level in [Level::ERROR, Level::INFO, Level::TRACE] {
        assert!(!console_shows(false, level), "a {level} span");
    }
}
