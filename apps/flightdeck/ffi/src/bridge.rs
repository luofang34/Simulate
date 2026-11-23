//! C-ABI entry points wrapping [`sim_flightdeck::FlightDeckSession`] and `FDKF`/`FDKG` wire buffers.

/// Planetary reference, sky ephemeris, starfield, and conformal HUD FOV C-ABI entry points.
pub mod planet;

use core::mem::ManuallyDrop;
use core::ptr;
use core::slice;
use sim_flightdeck::{
    HeadPose, FlightDeckSession, TelemetrySourceMode, encode_frame_snapshot, encode_glyph_pack,
    verified_glyph_pack,
};

pub use planet::{
    FlightDeckBodyInfo, FlightDeckSkyState, flightdeck_session_body_info, flightdeck_session_clear_kml, flightdeck_session_emit_hud,
    flightdeck_session_evaluate_sky, flightdeck_session_lookup_airport_json, flightdeck_session_new_on,
    flightdeck_session_runways_geojson, flightdeck_session_set_body, flightdeck_session_set_camera_view,
    flightdeck_session_set_flight_plan_route, flightdeck_session_set_hud_fov, flightdeck_session_stars_f32,
};

/// Owned Rust byte buffer transferred across the C ABI and freed via [`flightdeck_byte_buffer_free`].
#[repr(C)]
pub struct FlightDeckByteBuffer {
    /// Heap allocation pointer (`null` when empty).
    pub ptr: *mut u8,
    /// Initialized byte length.
    pub len: usize,
    /// Heap allocation capacity.
    pub cap: usize,
}

/// Zero-allocation C-ABI status snapshot of active KML/GPX flight track replay.
#[repr(C)]
pub struct FlightDeckKmlStatus {
    /// Whether a KML or GPX flight track is currently loaded.
    pub has_track: bool,
    /// Whether replay progression is paused.
    pub paused: bool,
    /// Playback speed multiplier (`1.0` = real-time).
    pub speed_multiplier: f32,
    /// Current playback offset in seconds (`0.0..=duration_s`).
    pub elapsed_s: f64,
    /// Total track duration in seconds.
    pub duration_s: f64,
    /// Number of decoded track vertices.
    pub point_count: u32,
}

impl FlightDeckByteBuffer {
    const fn empty() -> Self {
        Self {
            ptr: ptr::null_mut(),
            len: 0,
            cap: 0,
        }
    }

    fn from_vec(bytes: Vec<u8>) -> Self {
        if bytes.is_empty() {
            return Self::empty();
        }
        let mut manual = ManuallyDrop::new(bytes);
        Self {
            ptr: manual.as_mut_ptr(),
            len: manual.len(),
            cap: manual.capacity(),
        }
    }
}

/// Releases an [`FlightDeckSession`] previously returned by [`flightdeck_session_new_on`].
///
/// # Safety
///
/// `session` must be either `NULL` or a valid pointer from [`flightdeck_session_new_on`] not yet freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_free(session: *mut FlightDeckSession) {
    if !session.is_null() {
        // SAFETY: Caller guarantees `session` was allocated by `flightdeck_session_new_on` and is unique.
        unsafe {
            drop(Box::from_raw(session));
        }
    }
}

/// Switches the active [`TelemetrySourceMode`] (`0..=2` scenarios, `3` Connext replay,
/// `4` Connext live TCP/accessory, `5` GDL-90 UDP `:4000`).
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_set_source_mode(session: *mut FlightDeckSession, mode_code: u8) {
    // SAFETY: Checked for non-null by `as_mut`.
    if let Some(sess) = unsafe { session.as_mut() } {
        sess.set_source_mode(TelemetrySourceMode::from_code(mode_code));
    }
}

/// Updates the normalized Digital Crown virtual/reality blend (`0.0..=1.0`, passthrough `>= 15%`).
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_set_crown_turn(session: *mut FlightDeckSession, turn: f32) {
    // SAFETY: Checked for non-null by `as_mut`.
    if let Some(sess) = unsafe { session.as_mut() } {
        sess.set_crown_turn(turn);
    }
}

/// Updates pilot headset orientation (`radians`) relative to forward cockpit axes.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_set_head_pose(
    session: *mut FlightDeckSession,
    yaw_rad: f32,
    pitch_rad: f32,
    roll_rad: f32,
) {
    // SAFETY: Checked for non-null by `as_mut`.
    if let Some(sess) = unsafe { session.as_mut() } {
        sess.set_head_pose(HeadPose {
            yaw_rad,
            pitch_rad,
            roll_rad,
        });
    }
}

/// Designates or clears an ADS-B traffic `track_id` for HUD/XR target lock cues.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_set_designated_track(
    session: *mut FlightDeckSession,
    has_track: bool,
    track_id: u64,
) {
    // SAFETY: Checked for non-null by `as_mut`.
    if let Some(sess) = unsafe { session.as_mut() } {
        sess.set_designated_track(has_track.then_some(track_id));
    }
}

/// Ingests raw Connext stream bytes at `now_ms`, returning framed outbound transport writes
/// (`u32_le count` followed by `count` `[u32_le len, bytes]`).
///
/// # Safety
///
/// `session` must be `NULL` or valid; if `len > 0`, `bytes` must reference `len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_ingest_connext(
    session: *mut FlightDeckSession,
    bytes: *const u8,
    len: usize,
    now_ms: u64,
) -> FlightDeckByteBuffer {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return FlightDeckByteBuffer::empty();
    };
    let slice = if len == 0 || bytes.is_null() {
        &[]
    } else {
        // SAFETY: Caller guarantees `bytes` points to `len` valid bytes.
        unsafe { slice::from_raw_parts(bytes, len) }
    };
    encode_write_batch(&sess.ingest_connext_bytes(slice, now_ms))
}

/// Runs the periodic Connext keepalive and `FPN/RP` route-poll timer at `now_ms`.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_tick_connext(
    session: *mut FlightDeckSession,
    now_ms: u64,
) -> FlightDeckByteBuffer {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return FlightDeckByteBuffer::empty();
    };
    encode_write_batch(&sess.tick_connext(now_ms))
}

/// Ingests raw GDL-90 UDP datagram bytes (`:4000`) at `now_ms`.
///
/// # Safety
///
/// `session` must be `NULL` or valid; if `len > 0`, `bytes` must reference `len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_ingest_gdl90(
    session: *mut FlightDeckSession,
    bytes: *const u8,
    len: usize,
    now_ms: u64,
) {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return;
    };
    if len == 0 || bytes.is_null() {
        return;
    }
    // SAFETY: Caller guarantees `bytes` points to `len` valid bytes.
    let slice = unsafe { slice::from_raw_parts(bytes, len) };
    sess.ingest_gdl90_bytes(slice, now_ms);
}

/// Loads UTF-8 KML (`<gx:Track>`, `<LineString>`) or GPX (`<trkpt>`) flight track bytes and
/// switches [`FlightDeckSession`] into `TelemetrySourceMode::KmlReplay` (`6`), returning `true` on success.
///
/// # Safety
///
/// `session` must be `NULL` or valid; if `len > 0`, `bytes` must reference `len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_load_kml(
    session: *mut FlightDeckSession,
    bytes: *const u8,
    len: usize,
    now_ms: u64,
) -> bool {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return false;
    };
    if len == 0 || bytes.is_null() {
        return false;
    }
    // SAFETY: Caller guarantees `bytes` points to `len` valid bytes.
    let slice = unsafe { slice::from_raw_parts(bytes, len) };
    sess.load_kml_bytes(slice, now_ms).is_ok()
}

/// Updates KML replay pause state and speed multiplier (`0.25x..=30.0x`) at `now_ms`.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_set_kml_playback(
    session: *mut FlightDeckSession,
    paused: bool,
    speed_multiplier: f32,
    now_ms: u64,
) {
    // SAFETY: Checked for non-null by `as_mut`.
    if let Some(sess) = unsafe { session.as_mut() } {
        sess.set_kml_playback(paused, f64::from(speed_multiplier), now_ms);
    }
}

/// Seeks KML replay cursor to normalized `fraction` (`0.0..=1.0`) at `now_ms`.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_seek_kml(
    session: *mut FlightDeckSession,
    fraction: f64,
    now_ms: u64,
) {
    // SAFETY: Checked for non-null by `as_mut`.
    if let Some(sess) = unsafe { session.as_mut() } {
        sess.seek_kml_fraction(fraction, now_ms);
    }
}

/// Returns current KML replay progress, speed, and point count.
///
/// # Safety
///
/// `session` must be `NULL` or a valid [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_kml_status(session: *const FlightDeckSession) -> FlightDeckKmlStatus {
    let Some(sess) = (unsafe { session.as_ref() }) else {
        return FlightDeckKmlStatus {
            has_track: false,
            paused: true,
            speed_multiplier: 1.0,
            elapsed_s: 0.0,
            duration_s: 0.0,
            point_count: 0,
        };
    };
    let st = sess.kml_status();
    FlightDeckKmlStatus {
        has_track: st.has_track,
        paused: st.paused,
        speed_multiplier: st.speed_multiplier as f32,
        elapsed_s: st.elapsed_s,
        duration_s: st.duration_s,
        point_count: st.point_count as u32,
    }
}

/// Returns UTF-8 JSON containing loaded KML track metadata and `[lon, lat, alt_m]` coordinates.
///
/// # Safety
///
/// `session` must be `NULL` or a valid [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_kml_track_json(session: *const FlightDeckSession) -> FlightDeckByteBuffer {
    let Some(sess) = (unsafe { session.as_ref() }) else {
        return FlightDeckByteBuffer::empty();
    };
    FlightDeckByteBuffer::from_vec(sess.kml_track_json().into_bytes())
}

/// Advances simulation/replay to `now_ms` and returns an `FDKF` v1 encoded frame snapshot.
///
/// # Safety
///
/// `session` must be `NULL` or a valid mutable [`FlightDeckSession`] pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_session_render_frame(
    session: *mut FlightDeckSession,
    now_ms: u64,
) -> FlightDeckByteBuffer {
    let Some(sess) = (unsafe { session.as_mut() }) else {
        return FlightDeckByteBuffer::empty();
    };
    match sess.render_frame(now_ms) {
        Ok(snap) => FlightDeckByteBuffer::from_vec(encode_frame_snapshot(&snap)),
        Err(_) => FlightDeckByteBuffer::empty(),
    }
}

/// Returns the verified `Indicate` `PANEL_GLYPHS` font pack encoded in `FDKG` v1 format.
#[unsafe(no_mangle)]
pub extern "C" fn flightdeck_verified_glyph_pack() -> FlightDeckByteBuffer {
    FlightDeckByteBuffer::from_vec(encode_glyph_pack(&verified_glyph_pack()))
}

/// Releases an [`FlightDeckByteBuffer`] previously returned by this library.
///
/// # Safety
///
/// `buf` must have been returned by a `flightdeck_*` function and not yet freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flightdeck_byte_buffer_free(buf: FlightDeckByteBuffer) {
    if !buf.ptr.is_null() && buf.cap > 0 {
        // SAFETY: Reconstructs the `Vec<u8>` originally forgotten in `FlightDeckByteBuffer::from_vec`.
        unsafe {
            drop(Vec::from_raw_parts(buf.ptr, buf.len, buf.cap));
        }
    }
}

fn encode_write_batch(writes: &[Vec<u8>]) -> FlightDeckByteBuffer {
    if writes.is_empty() {
        return FlightDeckByteBuffer::empty();
    }
    let mut out = Vec::with_capacity(64);
    out.extend_from_slice(&(writes.len() as u32).to_le_bytes());
    for w in writes {
        out.extend_from_slice(&(w.len() as u32).to_le_bytes());
        out.extend_from_slice(w);
    }
    FlightDeckByteBuffer::from_vec(out)
}
