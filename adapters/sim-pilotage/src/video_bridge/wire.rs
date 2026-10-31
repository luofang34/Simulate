use crate::PilotageBridgeError;

mod primitives;

use primitives::{
    WIRE_BYTES, WIRE_FIXED32, WIRE_FIXED64, WIRE_VARINT, put_f32_nz, put_f64_nz, put_nested,
    put_u32_nz, put_u64_nz, put_varint, skip_field, take_bytes, take_f32, take_f64, take_tag,
    take_u32, take_varint, try_take_varint, wire_err,
};

/// Maximum `pilotage.bridge.v1` length-delimited envelope size (`16 MiB`),
/// matching `pilotage-sim-video`.
pub const MAX_ENVELOPE_BYTES: usize = 16 * 1024 * 1024;

/// Differential-drive or planar motion command (`pilotage.bridge.v1.BridgeControl`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BridgeControl {
    /// Forward linear command (`m/s` or normalized).
    pub linear_x: f64,
    /// Yaw angular command (`rad/s` or normalized).
    pub angular_z: f64,
}

/// Planar odometry sample (`pilotage.bridge.v1.BridgeOdometry`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BridgeOdometry {
    /// Local X coordinate (`m`).
    pub x: f64,
    /// Local Y coordinate (`m`).
    pub y: f64,
    /// Heading (`rad`).
    pub heading: f64,
    /// Ground speed (`m/s`).
    pub speed: f64,
    /// Simulation timestamp (`ns`).
    pub sim_time_ns: u64,
}

/// Satellite-navigation position sample (`pilotage.bridge.v1.BridgeNavSat`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BridgeNavSat {
    /// Geodetic latitude (`deg`).
    pub latitude_deg: f64,
    /// Geodetic longitude (`deg`).
    pub longitude_deg: f64,
    /// Reported altitude (`m`).
    pub altitude_m: f64,
    /// Simulation timestamp (`ns`).
    pub sim_time_ns: u64,
}

/// Raw camera frame (`pilotage.bridge.v1.BridgeFrame`).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BridgeFrame {
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
    /// Pixel format identifier (must be `"RGB_INT8"` for Pilotage's media encoder).
    pub pixel_format: String,
    /// Capture timestamp (`ns`).
    pub sim_time_ns: u64,
    /// Tightly packed RGB pixel buffer (`width * height * 3` bytes).
    pub rgb: Vec<u8>,
    /// Video source identifier (`0 = FPV`, `1 = chase`, `2 = gimbal`).
    pub camera_id: u32,
}

/// Host-to-producer camera command (`pilotage.bridge.v1.BridgeCameraCommand`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BridgeCameraCommand {
    /// 1-based camera mode (`0 = unchanged`, `1 = FPV`, `2 = gimbal`, `3 = free`).
    pub mode: u32,
    /// Gimbal pan (`rad`, positive right relative to vehicle heading).
    pub pan_rad: f32,
    /// Gimbal tilt (`rad`, positive up, world-stabilized).
    pub tilt_rad: f32,
    /// Index into the producer's published zoom-detent calibration table.
    pub zoom_detent: u32,
}

/// Gimbal angles attached to an external [`BridgeViewPose`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BridgeGimbalPose {
    /// Gimbal pan (`rad`, positive right relative to vehicle heading).
    pub pan_rad: f32,
    /// Gimbal tilt (`rad`, positive up, world-stabilized).
    pub tilt_rad: f32,
    /// Active zoom detent index.
    pub zoom_detent: u32,
}

/// External vehicle view pose (`pilotage.bridge.v1.BridgeViewPose`, envelope tag `6`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BridgeViewPose {
    /// Simulation timestamp (`ns`).
    pub sim_time_ns: u64,
    /// Geodetic latitude (`deg`).
    pub latitude_deg: f64,
    /// Geodetic longitude (`deg`).
    pub longitude_deg: f64,
    /// Altitude (`m`) in `vertical_datum`.
    pub altitude_m: f64,
    /// Vertical datum (`1 = Ellipsoid`, `2 = MSL`, `3 = AGL`).
    pub vertical_datum: u32,
    /// FRD-to-NED Hamilton quaternion scalar part `w`.
    pub q_w: f64,
    /// FRD-to-NED Hamilton quaternion `x`.
    pub q_x: f64,
    /// FRD-to-NED Hamilton quaternion `y`.
    pub q_y: f64,
    /// FRD-to-NED Hamilton quaternion `z`.
    pub q_z: f64,
    /// Optional gimbal orientation override.
    pub gimbal: Option<BridgeGimbalPose>,
}

/// `oneof payload` inside [`BridgeEnvelope`].
#[derive(Debug, Clone, PartialEq)]
pub enum BridgePayload {
    /// Motion control (`1`).
    Control(BridgeControl),
    /// Planar odometry (`2`).
    Odometry(BridgeOdometry),
    /// Raw camera frame (`3`).
    Frame(BridgeFrame),
    /// Camera mode / gimbal / zoom command (`4`).
    CameraCommand(BridgeCameraCommand),
    /// GNSS fix (`5`).
    NavSat(BridgeNavSat),
    /// Full 6-DoF view pose (`6`).
    ViewPose(BridgeViewPose),
}

/// Top-level `pilotage.bridge.v1.BridgeEnvelope` wire message.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BridgeEnvelope {
    /// Enclosed payload, if any.
    pub payload: Option<BridgePayload>,
}

/// Serializes `envelope` as raw protobuf v3 bytes (without length prefix).
#[must_use]
pub fn encode_envelope(envelope: &BridgeEnvelope) -> Vec<u8> {
    let mut out = Vec::new();
    match &envelope.payload {
        None => {}
        Some(BridgePayload::Control(c)) => put_nested(&mut out, 1, &encode_control(c)),
        Some(BridgePayload::Odometry(o)) => put_nested(&mut out, 2, &encode_odometry(o)),
        Some(BridgePayload::Frame(f)) => put_nested(&mut out, 3, &encode_frame(f)),
        Some(BridgePayload::CameraCommand(c)) => put_nested(&mut out, 4, &encode_camera_cmd(c)),
        Some(BridgePayload::NavSat(n)) => put_nested(&mut out, 5, &encode_navsat(n)),
        Some(BridgePayload::ViewPose(v)) => put_nested(&mut out, 6, &encode_view_pose(v)),
    }
    out
}

/// Serializes `envelope` with a protobuf base-128 varint length prefix,
/// matching `prost::Message::encode_length_delimited`.
#[must_use]
pub fn encode_length_delimited_envelope(envelope: &BridgeEnvelope) -> Vec<u8> {
    let body = encode_envelope(envelope);
    let mut out = Vec::with_capacity(10 + body.len());
    put_varint(&mut out, body.len() as u64);
    out.extend_from_slice(&body);
    out
}

/// Decodes one length-delimited `BridgeEnvelope` from the front of `bytes`,
/// returning `(envelope, bytes_consumed)` or `Ok(None)` if `bytes` does not yet
/// hold the full frame.
pub fn decode_length_delimited_envelope(
    bytes: &[u8],
) -> Result<Option<(BridgeEnvelope, usize)>, PilotageBridgeError> {
    let Some((len_u64, prefix_len)) = try_take_varint(bytes)? else {
        return Ok(None);
    };
    let body_len = usize::try_from(len_u64).map_err(|_| wire_err("envelope length overflow"))?;
    if body_len > MAX_ENVELOPE_BYTES {
        return Err(wire_err("envelope length exceeds 16 MiB cap"));
    }
    let total = prefix_len.saturating_add(body_len);
    let Some(body) = bytes.get(prefix_len..total) else {
        return Ok(None);
    };
    let envelope = decode_envelope(body)?;
    Ok(Some((envelope, total)))
}

/// Decodes a raw protobuf v3 `BridgeEnvelope` body (without length prefix).
pub fn decode_envelope(mut buf: &[u8]) -> Result<BridgeEnvelope, PilotageBridgeError> {
    let mut payload = None;
    while !buf.is_empty() {
        let (field_num, wire_type) = take_tag(&mut buf)?;
        match (field_num, wire_type) {
            (1, WIRE_BYTES) => {
                payload = Some(BridgePayload::Control(decode_control(take_bytes(
                    &mut buf,
                )?)?));
            }
            (2, WIRE_BYTES) => {
                payload = Some(BridgePayload::Odometry(decode_odometry(take_bytes(
                    &mut buf,
                )?)?));
            }
            (3, WIRE_BYTES) => {
                payload = Some(BridgePayload::Frame(decode_frame(take_bytes(&mut buf)?)?));
            }
            (4, WIRE_BYTES) => {
                let b = take_bytes(&mut buf)?;
                payload = Some(BridgePayload::CameraCommand(decode_camera_cmd(b)?));
            }
            (5, WIRE_BYTES) => {
                payload = Some(BridgePayload::NavSat(decode_navsat(take_bytes(&mut buf)?)?));
            }
            (6, WIRE_BYTES) => {
                payload = Some(BridgePayload::ViewPose(decode_view_pose(take_bytes(
                    &mut buf,
                )?)?));
            }
            _ => skip_field(&mut buf, wire_type)?,
        }
    }
    Ok(BridgeEnvelope { payload })
}

fn encode_control(c: &BridgeControl) -> Vec<u8> {
    let mut out = Vec::new();
    put_f64_nz(&mut out, 1, c.linear_x);
    put_f64_nz(&mut out, 2, c.angular_z);
    out
}

fn decode_control(mut buf: &[u8]) -> Result<BridgeControl, PilotageBridgeError> {
    let mut out = BridgeControl::default();
    while !buf.is_empty() {
        match take_tag(&mut buf)? {
            (1, WIRE_FIXED64) => out.linear_x = take_f64(&mut buf)?,
            (2, WIRE_FIXED64) => out.angular_z = take_f64(&mut buf)?,
            (_, wt) => skip_field(&mut buf, wt)?,
        }
    }
    Ok(out)
}

fn encode_odometry(o: &BridgeOdometry) -> Vec<u8> {
    let mut out = Vec::new();
    put_f64_nz(&mut out, 1, o.x);
    put_f64_nz(&mut out, 2, o.y);
    put_f64_nz(&mut out, 3, o.heading);
    put_f64_nz(&mut out, 4, o.speed);
    put_u64_nz(&mut out, 5, o.sim_time_ns);
    out
}

fn decode_odometry(mut buf: &[u8]) -> Result<BridgeOdometry, PilotageBridgeError> {
    let mut out = BridgeOdometry::default();
    while !buf.is_empty() {
        match take_tag(&mut buf)? {
            (1, WIRE_FIXED64) => out.x = take_f64(&mut buf)?,
            (2, WIRE_FIXED64) => out.y = take_f64(&mut buf)?,
            (3, WIRE_FIXED64) => out.heading = take_f64(&mut buf)?,
            (4, WIRE_FIXED64) => out.speed = take_f64(&mut buf)?,
            (5, WIRE_VARINT) => out.sim_time_ns = take_varint(&mut buf)?,
            (_, wt) => skip_field(&mut buf, wt)?,
        }
    }
    Ok(out)
}

fn encode_navsat(n: &BridgeNavSat) -> Vec<u8> {
    let mut out = Vec::new();
    put_f64_nz(&mut out, 1, n.latitude_deg);
    put_f64_nz(&mut out, 2, n.longitude_deg);
    put_f64_nz(&mut out, 3, n.altitude_m);
    put_u64_nz(&mut out, 4, n.sim_time_ns);
    out
}

fn decode_navsat(mut buf: &[u8]) -> Result<BridgeNavSat, PilotageBridgeError> {
    let mut out = BridgeNavSat::default();
    while !buf.is_empty() {
        match take_tag(&mut buf)? {
            (1, WIRE_FIXED64) => out.latitude_deg = take_f64(&mut buf)?,
            (2, WIRE_FIXED64) => out.longitude_deg = take_f64(&mut buf)?,
            (3, WIRE_FIXED64) => out.altitude_m = take_f64(&mut buf)?,
            (4, WIRE_VARINT) => out.sim_time_ns = take_varint(&mut buf)?,
            (_, wt) => skip_field(&mut buf, wt)?,
        }
    }
    Ok(out)
}

fn encode_frame(f: &BridgeFrame) -> Vec<u8> {
    let mut out = Vec::with_capacity(32 + f.pixel_format.len() + f.rgb.len());
    put_u32_nz(&mut out, 1, f.width);
    put_u32_nz(&mut out, 2, f.height);
    if !f.pixel_format.is_empty() {
        put_nested(&mut out, 3, f.pixel_format.as_bytes());
    }
    put_u64_nz(&mut out, 4, f.sim_time_ns);
    if !f.rgb.is_empty() {
        put_nested(&mut out, 5, &f.rgb);
    }
    put_u32_nz(&mut out, 6, f.camera_id);
    out
}

fn decode_frame(mut buf: &[u8]) -> Result<BridgeFrame, PilotageBridgeError> {
    let mut out = BridgeFrame::default();
    while !buf.is_empty() {
        match take_tag(&mut buf)? {
            (1, WIRE_VARINT) => out.width = take_u32(&mut buf)?,
            (2, WIRE_VARINT) => out.height = take_u32(&mut buf)?,
            (3, WIRE_BYTES) => {
                let raw = take_bytes(&mut buf)?;
                out.pixel_format = std::str::from_utf8(raw)
                    .map_err(|_| wire_err("invalid UTF-8 in BridgeFrame.pixel_format"))?
                    .to_owned();
            }
            (4, WIRE_VARINT) => out.sim_time_ns = take_varint(&mut buf)?,
            (5, WIRE_BYTES) => out.rgb = take_bytes(&mut buf)?.to_vec(),
            (6, WIRE_VARINT) => out.camera_id = take_u32(&mut buf)?,
            (_, wt) => skip_field(&mut buf, wt)?,
        }
    }
    Ok(out)
}

fn encode_camera_cmd(c: &BridgeCameraCommand) -> Vec<u8> {
    let mut out = Vec::new();
    put_u32_nz(&mut out, 1, c.mode);
    put_f32_nz(&mut out, 2, c.pan_rad);
    put_f32_nz(&mut out, 3, c.tilt_rad);
    put_u32_nz(&mut out, 4, c.zoom_detent);
    out
}

fn decode_camera_cmd(mut buf: &[u8]) -> Result<BridgeCameraCommand, PilotageBridgeError> {
    let mut out = BridgeCameraCommand::default();
    while !buf.is_empty() {
        match take_tag(&mut buf)? {
            (1, WIRE_VARINT) => out.mode = take_u32(&mut buf)?,
            (2, WIRE_FIXED32) => out.pan_rad = take_f32(&mut buf)?,
            (3, WIRE_FIXED32) => out.tilt_rad = take_f32(&mut buf)?,
            (4, WIRE_VARINT) => out.zoom_detent = take_u32(&mut buf)?,
            (_, wt) => skip_field(&mut buf, wt)?,
        }
    }
    Ok(out)
}

fn encode_gimbal_pose(g: &BridgeGimbalPose) -> Vec<u8> {
    let mut out = Vec::new();
    put_f32_nz(&mut out, 1, g.pan_rad);
    put_f32_nz(&mut out, 2, g.tilt_rad);
    put_u32_nz(&mut out, 3, g.zoom_detent);
    out
}

fn decode_gimbal_pose(mut buf: &[u8]) -> Result<BridgeGimbalPose, PilotageBridgeError> {
    let mut out = BridgeGimbalPose::default();
    while !buf.is_empty() {
        match take_tag(&mut buf)? {
            (1, WIRE_FIXED32) => out.pan_rad = take_f32(&mut buf)?,
            (2, WIRE_FIXED32) => out.tilt_rad = take_f32(&mut buf)?,
            (3, WIRE_VARINT) => out.zoom_detent = take_u32(&mut buf)?,
            (_, wt) => skip_field(&mut buf, wt)?,
        }
    }
    Ok(out)
}

fn encode_view_pose(v: &BridgeViewPose) -> Vec<u8> {
    let mut out = Vec::new();
    put_u64_nz(&mut out, 1, v.sim_time_ns);
    put_f64_nz(&mut out, 2, v.latitude_deg);
    put_f64_nz(&mut out, 3, v.longitude_deg);
    put_f64_nz(&mut out, 4, v.altitude_m);
    put_u32_nz(&mut out, 5, v.vertical_datum);
    put_f64_nz(&mut out, 6, v.q_w);
    put_f64_nz(&mut out, 7, v.q_x);
    put_f64_nz(&mut out, 8, v.q_y);
    put_f64_nz(&mut out, 9, v.q_z);
    if let Some(g) = &v.gimbal {
        put_nested(&mut out, 10, &encode_gimbal_pose(g));
    }
    out
}

fn decode_view_pose(mut buf: &[u8]) -> Result<BridgeViewPose, PilotageBridgeError> {
    let mut out = BridgeViewPose::default();
    while !buf.is_empty() {
        match take_tag(&mut buf)? {
            (1, WIRE_VARINT) => out.sim_time_ns = take_varint(&mut buf)?,
            (2, WIRE_FIXED64) => out.latitude_deg = take_f64(&mut buf)?,
            (3, WIRE_FIXED64) => out.longitude_deg = take_f64(&mut buf)?,
            (4, WIRE_FIXED64) => out.altitude_m = take_f64(&mut buf)?,
            (5, WIRE_VARINT) => out.vertical_datum = take_u32(&mut buf)?,
            (6, WIRE_FIXED64) => out.q_w = take_f64(&mut buf)?,
            (7, WIRE_FIXED64) => out.q_x = take_f64(&mut buf)?,
            (8, WIRE_FIXED64) => out.q_y = take_f64(&mut buf)?,
            (9, WIRE_FIXED64) => out.q_z = take_f64(&mut buf)?,
            (10, WIRE_BYTES) => out.gimbal = Some(decode_gimbal_pose(take_bytes(&mut buf)?)?),
            (_, wt) => skip_field(&mut buf, wt)?,
        }
    }
    Ok(out)
}
