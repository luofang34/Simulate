//! C-ABI staticlib bridge exposing [`sim_flightdeck`] to Swift, iPadOS, and Apple Vision Pro (`visionOS`).

#![deny(missing_docs)]

/// C-ABI functions and owned byte-buffer types exported to `FlightDeckKit`.
pub mod bridge;

pub use bridge::{
    FlightDeckBodyInfo, FlightDeckByteBuffer, FlightDeckKmlStatus, FlightDeckSkyState, flightdeck_byte_buffer_free,
    flightdeck_session_body_info, flightdeck_session_clear_kml, flightdeck_session_emit_hud, flightdeck_session_evaluate_sky,
    flightdeck_session_free, flightdeck_session_ingest_connext, flightdeck_session_ingest_gdl90, flightdeck_session_kml_status,
    flightdeck_session_kml_track_json, flightdeck_session_load_kml, flightdeck_session_new_on, flightdeck_session_render_frame,
    flightdeck_session_runways_geojson, flightdeck_session_seek_kml, flightdeck_session_set_body,
    flightdeck_session_set_camera_view, flightdeck_session_set_crown_turn, flightdeck_session_set_designated_track,
    flightdeck_session_set_head_pose, flightdeck_session_set_hud_fov, flightdeck_session_set_kml_playback,
    flightdeck_session_set_source_mode, flightdeck_session_stars_f32, flightdeck_session_tick_connext,
    flightdeck_verified_glyph_pack,
};
