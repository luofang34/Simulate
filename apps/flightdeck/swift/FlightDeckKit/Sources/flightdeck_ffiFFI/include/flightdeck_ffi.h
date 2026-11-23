#ifndef FLIGHTDECK_FFI_H
#define FLIGHTDECK_FFI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FlightDeckSession FlightDeckSession;

typedef struct FlightDeckByteBuffer {
    uint8_t *ptr;
    size_t len;
    size_t cap;
} FlightDeckByteBuffer;

typedef struct FlightDeckBodyInfo {
    uint8_t body_code;
    double render_radius_m;
    double highest_ground_m;
    double lowest_ground_m;
    double semi_major_m;
    double semi_minor_m;
    double surface_gravity_mps2;
    bool has_atmosphere;
    bool is_earth_wgs84;
} FlightDeckBodyInfo;

typedef struct FlightDeckSkyState {
    bool has_sun;
    double sun_fixed_x;
    double sun_fixed_y;
    double sun_fixed_z;
    double sun_angular_radius_deg;
    bool has_moon;
    double moon_fixed_x;
    double moon_fixed_y;
    double moon_fixed_z;
    double moon_angular_radius_deg;
    float night_brightness;
    bool atmosphere;
    bool stars_updated;
    uint32_t star_count;
} FlightDeckSkyState;

typedef struct FlightDeckKmlStatus {
    bool has_track;
    bool paused;
    float speed_multiplier;
    double elapsed_s;
    double duration_s;
    uint32_t point_count;
} FlightDeckKmlStatus;

FlightDeckSession *flightdeck_session_new_on(uint8_t body_code);
void flightdeck_session_free(FlightDeckSession *session);

bool flightdeck_session_set_body(FlightDeckSession *session, uint8_t body_code, uint64_t now_ms);
FlightDeckBodyInfo flightdeck_session_body_info(const FlightDeckSession *session);

void flightdeck_session_set_source_mode(FlightDeckSession *session, uint8_t mode_code);
void flightdeck_session_set_crown_turn(FlightDeckSession *session, float turn);
void flightdeck_session_set_head_pose(
    FlightDeckSession *session,
    float yaw_rad,
    float pitch_rad,
    float roll_rad
);
void flightdeck_session_set_hud_fov(FlightDeckSession *session, float vertical_fov_rad);
void flightdeck_session_set_camera_view(
    FlightDeckSession *session,
    bool active,
    double latitude_deg,
    double longitude_deg,
    double altitude_datum_m,
    double true_heading_deg,
    double pitch_deg,
    double roll_deg,
    bool has_dem,
    double dem_datum_m
);
FlightDeckByteBuffer flightdeck_session_emit_hud(FlightDeckSession *session, float width, float height);
FlightDeckByteBuffer flightdeck_session_runways_geojson(
    const FlightDeckSession *session,
    double center_lat_deg,
    double center_lon_deg,
    double radius_km
);
void flightdeck_session_set_flight_plan_route(
    FlightDeckSession *session,
    const uint8_t *route_utf8,
    size_t route_len
);
FlightDeckByteBuffer flightdeck_session_lookup_airport_json(
    const FlightDeckSession *session,
    const uint8_t *code_utf8,
    size_t code_len
);
void flightdeck_session_set_designated_track(
    FlightDeckSession *session,
    bool has_track,
    uint64_t track_id
);

FlightDeckSkyState flightdeck_session_evaluate_sky(
    FlightDeckSession *session,
    double utc_unix_s,
    double fallback_lat_deg,
    double fallback_lon_deg,
    double fallback_alt_datum_m
);
const float *flightdeck_session_stars_f32(const FlightDeckSession *session, uint32_t *out_star_count);

FlightDeckByteBuffer flightdeck_session_ingest_connext(
    FlightDeckSession *session,
    const uint8_t *bytes,
    size_t len,
    uint64_t now_ms
);
FlightDeckByteBuffer flightdeck_session_tick_connext(FlightDeckSession *session, uint64_t now_ms);

void flightdeck_session_ingest_gdl90(
    FlightDeckSession *session,
    const uint8_t *bytes,
    size_t len,
    uint64_t now_ms
);

bool flightdeck_session_load_kml(
    FlightDeckSession *session,
    const uint8_t *bytes,
    size_t len,
    uint64_t now_ms
);
void flightdeck_session_clear_kml(FlightDeckSession *session);
void flightdeck_session_set_kml_playback(
    FlightDeckSession *session,
    bool paused,
    float speed_multiplier,
    uint64_t now_ms
);
void flightdeck_session_seek_kml(
    FlightDeckSession *session,
    double fraction,
    uint64_t now_ms
);
FlightDeckKmlStatus flightdeck_session_kml_status(const FlightDeckSession *session);
FlightDeckByteBuffer flightdeck_session_kml_track_json(const FlightDeckSession *session);

FlightDeckByteBuffer flightdeck_session_render_frame(FlightDeckSession *session, uint64_t now_ms);
FlightDeckByteBuffer flightdeck_verified_glyph_pack(void);
void flightdeck_byte_buffer_free(FlightDeckByteBuffer buf);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHTDECK_FFI_H */
