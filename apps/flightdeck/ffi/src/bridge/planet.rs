//! Planetary reference, sky ephemeris, starfield, and conformal HUD FOV C-ABI entry points.

use super::FlightDeckByteBuffer;
use core::ptr;
use sim_flightdeck::{CameraViewPose, PlanetaryBodyId, PlanetaryReference, FlightDeckSession};

/// Zero-allocation C-ABI descriptor of an explicitly declared planetary body and vertical datum.
#[repr(C)]
pub struct FlightDeckBodyInfo {
    /// Explicit `PlanetaryBodyId` code (`0` = Earth WGS-84/EGM96 MSL, `1` = Earth WGS-84 HAE,
    /// `2` = Moon IAU 1737.4 km, `3` = Mars IAU spheroid).
    pub body_code: u8,
    /// Mean spherical render radius (`m`) passed to the globe renderer.
    pub render_radius_m: f64,
    /// Highest terrain bound (`m`) above the mean radius for horizon culling.
    pub highest_ground_m: f64,
    /// Lowest terrain bound (`m`) relative to the mean radius.
    pub lowest_ground_m: f64,
    /// Equatorial semi-major axis (`m`) from `sim-geo`.
    pub semi_major_m: f64,
    /// Polar semi-minor axis (`m`) from `sim-geo`.
    pub semi_minor_m: f64,
    /// Equatorial surface gravity magnitude (`m/s²`) from `sim-geo`'s `J2` gravity model.
    pub surface_gravity_mps2: f64,
    /// Whether the body has an atmosphere that scatters sunlight at the limb.
    pub has_atmosphere: bool,
    /// Whether the body is `WGS-84` Earth (allowing `FAA NASR`, `WMM2025`, and `ADS-B`).
    pub is_earth_wgs84: bool,
}

/// Zero-allocation C-ABI snapshot of ephemeris-driven Sun, Moon, atmosphere, and starfield state.
#[repr(C)]
pub struct FlightDeckSkyState {
    /// Whether a valid body-fixed Sun direction was resolved.
    pub has_sun: bool,
    /// Body-fixed unit Sun `X` component.
    pub sun_fixed_x: f64,
    /// Body-fixed unit Sun `Y` component.
    pub sun_fixed_y: f64,
    /// Body-fixed unit Sun `Z` component.
    pub sun_fixed_z: f64,
    /// Apparent solar angular radius (`deg`).
    pub sun_angular_radius_deg: f64,
    /// Whether a valid topocentric Moon disc direction was resolved (`false` when on the Moon).
    pub has_moon: bool,
    /// Body-fixed unit Moon `X` component.
    pub moon_fixed_x: f64,
    /// Body-fixed unit Moon `Y` component.
    pub moon_fixed_y: f64,
    /// Body-fixed unit Moon `Z` component.
    pub moon_fixed_z: f64,
    /// Apparent lunar angular radius (`deg`).
    pub moon_angular_radius_deg: f64,
    /// Night-side terrain ambient brightness (`0..=1`).
    pub night_brightness: f32,
    /// Whether atmospheric limb scattering should be rendered.
    pub atmosphere: bool,
    /// Whether the starfield buffer changed and should be re-uploaded to GPU.
    pub stars_updated: bool,
    /// Number of stars currently stored in the session's packed `f32` buffer.
    pub star_count: u32,
}

impl FlightDeckBodyInfo {
    fn from_planet(planet: PlanetaryReference) -> Self {
        let g0 = planet.gravity_mps2(&planet.geodetic_from_datum(0.0, 0.0, 0.0));
        Self {
            body_code: planet.id.code(),
            render_radius_m: planet.render_radius_m,
            highest_ground_m: planet.highest_ground_m,
            lowest_ground_m: planet.lowest_ground_m,
            semi_major_m: planet.ellipsoid.semi_major_m,
            semi_minor_m: planet.ellipsoid.semi_minor_m(),
            surface_gravity_mps2: g0,
            has_atmosphere: planet.has_atmosphere,
            is_earth_wgs84: planet.is_earth_wgs84(),
        }
    }
}

/// Allocates and initializes an [`FlightDeckSession`] on an explicitly declared `body_code`
/// (`0..=3`), returning `NULL` if `body_code` is unknown or fixture validation fails.
///
/// Callers must declare the celestial body and vertical datum explicitly — there is
/// intentionally no parameterless constructor that silently assumes Earth or `WGS-84`.
#[unsafe(no_mangle)]
pub extern "C" fn flightdeck_session_new_on(body_code: u8) -> *mut FlightDeckSession {
    let Some(body_id) = PlanetaryBodyId::from_code(body_code) else {
        return ptr::null_mut();
    };
    let planet = PlanetaryReference::from_id(body_id);
    match FlightDeckSession::new_on(planet) {
        Ok(session) => Box::into_raw(Box::new(session)),
        Err(_) => ptr::null_mut(),
    }
}

/// Re-declares the active planetary body (`body_code` in `0..=3`) at `now_ms`, returning `true`
/// when `body_code` is valid.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_set_body(
    session: *mut FlightDeckSession,
    body_code: u8,
    now_ms: u64,
) -> bool {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return false;
    };
    let Some(body_id) = PlanetaryBodyId::from_code(body_code) else {
        return false;
    };
    sess.set_planet(PlanetaryReference::from_id(body_id), now_ms);
    true
}

/// Returns physical ellipsoid, gravity, atmosphere, and render-bound parameters for `session`'s
/// currently declared planetary reference.
///
/// # Safety
///
/// `session` must be `NULL` or a valid [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_body_info(session: *const FlightDeckSession) -> FlightDeckBodyInfo {
    let Some(sess) = (unsafe { session.as_ref() }) else {
        return FlightDeckBodyInfo {
            body_code: u8::MAX,
            render_radius_m: 0.0,
            highest_ground_m: 0.0,
            lowest_ground_m: 0.0,
            semi_major_m: 0.0,
            semi_minor_m: 0.0,
            surface_gravity_mps2: 0.0,
            has_atmosphere: false,
            is_earth_wgs84: false,
        };
    };
    FlightDeckBodyInfo::from_planet(sess.planet())
}

/// Updates the conformal HUD vertical field of view (`radians`, clamped to `5°..=120°`) so
/// pitch ladder bars and flight-path vector align 1:1 with the outside 3D scene viewport.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_set_hud_fov(session: *mut FlightDeckSession, vertical_fov_rad: f32) {
    if let Some(sess) = unsafe { session.as_mut() } {
        sess.set_hud_vertical_fov_rad(vertical_fov_rad);
    }
}

/// Closes any active KML/GPX track preview and returns `session` to default scenario mode.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_clear_kml(session: *mut FlightDeckSession) {
    if let Some(sess) = unsafe { session.as_mut() } {
        sess.clear_kml_replay();
    }
}

/// Evaluates ephemeris-driven Sun, Moon, atmosphere, and Johnson-V starfield on `session`'s
/// declared planetary body at `utc_unix_s`.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_evaluate_sky(
    session: *mut FlightDeckSession,
    utc_unix_s: f64,
    fallback_lat_deg: f64,
    fallback_lon_deg: f64,
    fallback_alt_datum_m: f64,
) -> FlightDeckSkyState {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return FlightDeckSkyState {
            has_sun: false,
            sun_fixed_x: 1.0,
            sun_fixed_y: 0.0,
            sun_fixed_z: 0.0,
            sun_angular_radius_deg: 0.266,
            has_moon: false,
            moon_fixed_x: 0.0,
            moon_fixed_y: 1.0,
            moon_fixed_z: 0.0,
            moon_angular_radius_deg: 0.0,
            night_brightness: 0.24,
            atmosphere: false,
            stars_updated: false,
            star_count: 0,
        };
    };
    let snap = sess.evaluate_sky(
        utc_unix_s,
        fallback_lat_deg,
        fallback_lon_deg,
        fallback_alt_datum_m,
    );
    let [sx, sy, sz] = snap.sun_direction_fixed.unwrap_or([1.0, 0.0, 0.0]);
    let [mx, my, mz] = snap.moon_direction_fixed.unwrap_or([0.0, 1.0, 0.0]);
    FlightDeckSkyState {
        has_sun: snap.sun_direction_fixed.is_some(),
        sun_fixed_x: sx,
        sun_fixed_y: sy,
        sun_fixed_z: sz,
        sun_angular_radius_deg: snap.sun_angular_radius_deg,
        has_moon: snap.moon_direction_fixed.is_some(),
        moon_fixed_x: mx,
        moon_fixed_y: my,
        moon_fixed_z: mz,
        moon_angular_radius_deg: snap.moon_angular_radius_deg,
        night_brightness: snap.night_brightness,
        atmosphere: snap.atmosphere,
        stars_updated: snap.stars_updated,
        star_count: sess.sky_star_count(),
    }
}

/// Borrows the packed `[dir_x, dir_y, dir_z, v_mag, r, g, b]` `f32` star buffer computed by the
/// most recent [`flightdeck_session_evaluate_sky`] call, valid until the next mutable call on `session`.
///
/// # Safety
///
/// `session` must be `NULL` or a valid [`FlightDeckSession`] pointer; `out_star_count` may be `NULL` or
/// point to a writable `u32`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_stars_f32(
    session: *const FlightDeckSession,
    out_star_count: *mut u32,
) -> *const f32 {
    if !out_star_count.is_null() {
        // SAFETY: Caller guarantees `out_star_count` is writable when non-null.
        unsafe {
            *out_star_count = 0;
        }
    }
    let Some(sess) = (unsafe { session.as_ref() }) else {
        return ptr::null();
    };
    let count = sess.sky_star_count();
    let buf = sess.sky_stars_f32();
    if count == 0 || buf.is_empty() {
        return ptr::null();
    }
    if !out_star_count.is_null() {
        // SAFETY: Caller guarantees `out_star_count` is writable when non-null.
        unsafe {
            *out_star_count = count;
        }
    }
    buf.as_ptr()
}

/// Updates the active outside-scene camera pose (`active = true` in `FPV`/`Chase`, `false` in `Globe`
/// orbit) and optional onboard terrain DEM sample (`has_dem`, `dem_datum_m`) for conformal HUD & runway
/// alignment.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn flightdeck_session_set_camera_view(
    session: *mut FlightDeckSession,
    active: bool,
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_datum_m: f64,
    true_heading_deg: f64,
    pitch_deg: f64,
    roll_deg: f64,
    has_dem: bool,
    dem_datum_m: f64,
) {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return;
    };
    let cam = active.then_some(CameraViewPose {
        latitude_deg,
        longitude_deg,
        altitude_datum_m,
        true_heading_deg,
        pitch_deg,
        roll_deg,
    });
    sess.set_camera_view_pose(cam, has_dem.then_some(dem_datum_m));
}

/// Re-emits the conformal HUD scene into the exact `(width, height)` `DesignFrame` requested by
/// `IndicateAppleDisplay`'s `PanelDisplay`, guaranteeing `[width * 0.5, height * 0.5]` centering.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_emit_hud(
    session: *mut FlightDeckSession,
    width: f32,
    height: f32,
) -> FlightDeckByteBuffer {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return FlightDeckByteBuffer::empty();
    };
    match sess.emit_hud_scene(width, height) {
        Ok(bytes) => FlightDeckByteBuffer::from_vec(bytes),
        Err(_) => FlightDeckByteBuffer::empty(),
    }
}

/// Returns UTF-8 GeoJSON `FeatureCollection` of surveyed airport runways from `AirportDatabase::builtin()`.
///
/// # Safety
///
/// `session` must be `NULL` or a valid [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_runways_geojson(
    session: *const FlightDeckSession,
    center_lat_deg: f64,
    center_lon_deg: f64,
    radius_km: f64,
) -> FlightDeckByteBuffer {
    let Some(sess) = (unsafe { session.as_ref() }) else {
        return FlightDeckByteBuffer::empty();
    };
    FlightDeckByteBuffer::from_vec(
        sess.airport_runways_geojson(center_lat_deg, center_lon_deg, radius_km)
            .into_bytes(),
    )
}

/// Updates the pilot's active EFB/GCS flight-plan route string (`route_utf8` of length `route_len`,
/// or `NULL` / `0` to clear) so destination airport runways are prioritized on map, SVS & HUD.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer; when `route_len > 0` and
/// `route_utf8` is non-null, it must address `route_len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_set_flight_plan_route(
    session: *mut FlightDeckSession,
    route_utf8: *const u8,
    route_len: usize,
) {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return;
    };
    if route_utf8.is_null() || route_len == 0 {
        sess.set_flight_plan_route(None);
        return;
    }
    // SAFETY: Caller guarantees `route_utf8` addresses `route_len` readable bytes.
    let bytes = unsafe { core::slice::from_raw_parts(route_utf8, route_len) };
    let text = core::str::from_utf8(bytes).ok();
    sess.set_flight_plan_route(text);
}

/// Resolves an airport identifier (`code_utf8` of length `code_len`) from `AirportDatabase::builtin()`
/// on `session`'s declared body, returning UTF-8 JSON metadata (`"{}"` when not found or non-Earth).
///
/// # Safety
///
/// `session` must be `NULL` or a valid [`FlightDeckSession`] pointer; when `code_len > 0` and `code_utf8`
/// is non-null, it must address `code_len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_lookup_airport_json(
    session: *const FlightDeckSession,
    code_utf8: *const u8,
    code_len: usize,
) -> FlightDeckByteBuffer {
    let Some(sess) = (unsafe { session.as_ref() }) else {
        return FlightDeckByteBuffer::empty();
    };
    if code_utf8.is_null() || code_len == 0 {
        return FlightDeckByteBuffer::from_vec(b"{}".to_vec());
    }
    // SAFETY: Caller guarantees `code_utf8` addresses `code_len` readable bytes.
    let bytes = unsafe { core::slice::from_raw_parts(code_utf8, code_len) };
    let Ok(code) = core::str::from_utf8(bytes) else {
        return FlightDeckByteBuffer::from_vec(b"{}".to_vec());
    };
    FlightDeckByteBuffer::from_vec(sess.lookup_airport(code).into_bytes())
}
