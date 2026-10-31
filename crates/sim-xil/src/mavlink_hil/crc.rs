use crate::XilError;

/// MAVLink v2 start-of-frame marker.
pub const MAVLINK_V2_MAGIC: u8 = 0xFD;

/// MAVLink v2 header byte length (including `MAVLINK_V2_MAGIC`).
pub const MAVLINK_V2_HEADER_LEN: usize = 10;

/// Incompatibility flag indicating a 13-byte v2 link signature suffix.
pub const MAVLINK_IFLAG_SIGNED: u8 = 0x01;

/// MAVLink v2 signature tail length when [`MAVLINK_IFLAG_SIGNED`] is set.
pub const MAVLINK_V2_SIGNATURE_LEN: usize = 13;

/// `HEARTBEAT` message identifier (`0`).
pub const HEARTBEAT_MSG_ID: u32 = 0;
/// `HEARTBEAT` `CRC_EXTRA` seed (`50`).
pub const CRC_EXTRA_HEARTBEAT: u8 = 50;

/// `HIL_ACTUATOR_CONTROLS` message identifier (`93`).
pub const HIL_ACTUATOR_CONTROLS_MSG_ID: u32 = 93;
/// `HIL_ACTUATOR_CONTROLS` `CRC_EXTRA` seed (`47`).
pub const CRC_EXTRA_HIL_ACTUATOR_CONTROLS: u8 = 47;

/// `HIL_SENSOR` message identifier (`107`).
pub const HIL_SENSOR_MSG_ID: u32 = 107;
/// `HIL_SENSOR` `CRC_EXTRA` seed (`108`).
pub const CRC_EXTRA_HIL_SENSOR: u8 = 108;

/// `HIL_GPS` message identifier (`113`).
pub const HIL_GPS_MSG_ID: u32 = 113;
/// `HIL_GPS` `CRC_EXTRA` seed (`124`).
pub const CRC_EXTRA_HIL_GPS: u8 = 124;

/// `HIL_STATE_QUATERNION` message identifier (`115`).
pub const HIL_STATE_QUATERNION_MSG_ID: u32 = 115;
/// `HIL_STATE_QUATERNION` `CRC_EXTRA` seed (`4`).
pub const CRC_EXTRA_HIL_STATE_QUATERNION: u8 = 4;

/// Looks up the MAVLink `CRC_EXTRA` byte for supported HIL messages.
#[must_use]
pub const fn crc_extra_for(msg_id: u32) -> Option<u8> {
    match msg_id {
        HEARTBEAT_MSG_ID => Some(CRC_EXTRA_HEARTBEAT),
        HIL_ACTUATOR_CONTROLS_MSG_ID => Some(CRC_EXTRA_HIL_ACTUATOR_CONTROLS),
        HIL_SENSOR_MSG_ID => Some(CRC_EXTRA_HIL_SENSOR),
        HIL_GPS_MSG_ID => Some(CRC_EXTRA_HIL_GPS),
        HIL_STATE_QUATERNION_MSG_ID => Some(CRC_EXTRA_HIL_STATE_QUATERNION),
        _ => None,
    }
}

/// Computes the MAVLink X.25 (`CRC-16/MCRF4XX`) checksum over `header_and_body`
/// (bytes `1..10+len`, excluding `0xFD`) followed by `crc_extra`.
#[must_use]
pub fn x25_crc(header_and_body: &[u8], crc_extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in header_and_body {
        crc = x25_accumulate(crc, byte);
    }
    x25_accumulate(crc, crc_extra)
}

const fn x25_accumulate(crc: u16, byte: u8) -> u16 {
    let mut tmp = byte ^ ((crc & 0xFF) as u8);
    tmp ^= tmp << 4;
    let wide = tmp as u16;
    (crc >> 8) ^ (wide << 8) ^ (wide << 3) ^ (wide >> 4)
}

/// Encodes a MAVLink v2 frame into `out`, trimming trailing zero payload bytes
/// per the MAVLink 2 wire specification (preserving at least 1 byte when the
/// payload is empty or all-zero if `payload` is non-empty, matching reference C
/// ` _mav_trim_payload`). Returns the written frame byte count.
pub fn encode_v2_frame(
    seq: u8,
    system_id: u8,
    component_id: u8,
    msg_id: u32,
    crc_extra: u8,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, XilError> {
    if payload.len() > 255 || msg_id > 0x00FF_FFFF {
        return Err(XilError::ProtocolViolation {
            protocol: "MAVLink v2",
            detail: format!(
                "payload len {} or msg_id {msg_id} out of range",
                payload.len()
            ),
        });
    }
    let trimmed_len = trimmed_payload_len(payload);
    let total_len = MAVLINK_V2_HEADER_LEN
        .saturating_add(trimmed_len)
        .saturating_add(2);
    let Some(frame_buf) = out.get_mut(..total_len) else {
        return Err(XilError::InvalidPacketLength {
            protocol: "MAVLink v2",
            expected: total_len,
            actual: out.len(),
        });
    };

    let header = [
        MAVLINK_V2_MAGIC,
        trimmed_len as u8,
        0,
        0,
        seq,
        system_id,
        component_id,
        (msg_id & 0xFF) as u8,
        ((msg_id >> 8) & 0xFF) as u8,
        ((msg_id >> 16) & 0xFF) as u8,
    ];
    if let Some(dst) = frame_buf.get_mut(..MAVLINK_V2_HEADER_LEN) {
        dst.copy_from_slice(&header);
    }
    if let (Some(src), Some(dst)) = (
        payload.get(..trimmed_len),
        frame_buf.get_mut(MAVLINK_V2_HEADER_LEN..MAVLINK_V2_HEADER_LEN.saturating_add(trimmed_len)),
    ) {
        dst.copy_from_slice(src);
    }
    let crc_end = MAVLINK_V2_HEADER_LEN.saturating_add(trimmed_len);
    let crc = frame_buf
        .get(1..crc_end)
        .map_or(0, |slice| x25_crc(slice, crc_extra));
    if let Some(crc_dst) = frame_buf.get_mut(crc_end..total_len) {
        crc_dst.copy_from_slice(&crc.to_le_bytes());
    }
    Ok(total_len)
}

fn trimmed_payload_len(payload: &[u8]) -> usize {
    let mut len = payload.len();
    while len > 1 {
        let prev = len.saturating_sub(1);
        if payload.get(prev).copied() == Some(0) {
            len = prev;
        } else {
            break;
        }
    }
    len
}
