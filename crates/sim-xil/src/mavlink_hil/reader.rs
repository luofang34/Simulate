use std::collections::VecDeque;

use crate::mavlink_hil::crc::{
    HEARTBEAT_MSG_ID, HIL_ACTUATOR_CONTROLS_MSG_ID, MAVLINK_IFLAG_SIGNED, MAVLINK_V2_HEADER_LEN,
    MAVLINK_V2_MAGIC, MAVLINK_V2_SIGNATURE_LEN, crc_extra_for, x25_crc,
};
use crate::mavlink_hil::messages::{
    HilActuatorControls, MAV_MODE_FLAG_SAFETY_ARMED, decode_hil_actuator_controls,
};

/// Decoded inbound MAVLink v2 HIL event from an external flight controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MavlinkHilFrame {
    /// Flight controller heartbeat (`#0`).
    Heartbeat {
        /// Sender MAVLink system ID.
        system_id: u8,
        /// Sender MAVLink component ID.
        component_id: u8,
        /// Whether `MAV_MODE_FLAG_SAFETY_ARMED` is set.
        armed: bool,
    },
    /// Flight controller actuator output (`#93`).
    ActuatorControls {
        /// Sender MAVLink system ID.
        system_id: u8,
        /// Sender MAVLink component ID.
        component_id: u8,
        /// Decoded actuator payload.
        controls: HilActuatorControls,
    },
}

/// Sans-IO streaming MAVLink v2 parser for inbound HIL frames.
///
/// Resynchronizes byte-by-byte on corrupt frames so line noise or partial
/// startup reads never wedge a TCP or UART stream.
#[derive(Debug, Default)]
pub struct MavlinkHilReader {
    buf: Vec<u8>,
    ready: VecDeque<MavlinkHilFrame>,
    crc_errors: u64,
    unknown_frames: u64,
    decoded_frames: u64,
}

impl MavlinkHilReader {
    /// Creates an empty streaming reader.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Feeds arbitrary stream bytes into the parser and extracts any complete
    /// valid frames.
    pub fn push_bytes(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
        let mut cursor: usize = 0;
        while let Some(rel_magic) = self
            .buf
            .get(cursor..)
            .and_then(|tail| tail.iter().position(|&b| b == MAVLINK_V2_MAGIC))
        {
            cursor = cursor.saturating_add(rel_magic);
            let remaining = self.buf.len().saturating_sub(cursor);
            if remaining < MAVLINK_V2_HEADER_LEN + 2 {
                break;
            }
            let payload_len = usize::from(self.byte_at(cursor + 1));
            let incompat_flags = self.byte_at(cursor + 2);
            if (incompat_flags & !MAVLINK_IFLAG_SIGNED) != 0 {
                cursor = cursor.wrapping_add(1);
                continue;
            }
            let sig_len = if (incompat_flags & MAVLINK_IFLAG_SIGNED) != 0 {
                MAVLINK_V2_SIGNATURE_LEN
            } else {
                0
            };
            let frame_len = MAVLINK_V2_HEADER_LEN + payload_len + 2 + sig_len;
            if remaining < frame_len {
                break;
            }
            if let Some(consumed) = self.try_decode_at(cursor, payload_len, frame_len) {
                cursor = cursor.saturating_add(consumed);
            } else {
                cursor = cursor.wrapping_add(1);
            }
        }
        if cursor > 0 {
            self.buf.drain(..cursor);
        }
    }

    /// Pops the next decoded HIL frame in arrival order.
    pub fn pop_frame(&mut self) -> Option<MavlinkHilFrame> {
        self.ready.pop_front()
    }

    /// Count of frames rejected due to X.25 CRC mismatch.
    #[must_use]
    pub const fn crc_errors(&self) -> u64 {
        self.crc_errors
    }

    /// Count of syntactically valid frames with message IDs outside the HIL subset.
    #[must_use]
    pub const fn unknown_frames(&self) -> u64 {
        self.unknown_frames
    }

    /// Count of valid HIL frames decoded so far.
    #[must_use]
    pub const fn decoded_frames(&self) -> u64 {
        self.decoded_frames
    }

    fn try_decode_at(
        &mut self,
        cursor: usize,
        payload_len: usize,
        frame_len: usize,
    ) -> Option<usize> {
        let system_id = self.byte_at(cursor + 5);
        let component_id = self.byte_at(cursor + 6);
        let msg_id = u32::from(self.byte_at(cursor + 7))
            | (u32::from(self.byte_at(cursor + 8)) << 8)
            | (u32::from(self.byte_at(cursor + 9)) << 16);

        let Some(crc_extra) = crc_extra_for(msg_id) else {
            self.unknown_frames = self.unknown_frames.wrapping_add(1);
            return Some(frame_len);
        };

        let body_end = cursor + MAVLINK_V2_HEADER_LEN + payload_len;
        let expected_crc = self
            .buf
            .get((cursor + 1)..body_end)
            .map_or(0, |slice| x25_crc(slice, crc_extra));
        let wire_crc = u16::from_le_bytes([self.byte_at(body_end), self.byte_at(body_end + 1)]);
        if expected_crc != wire_crc {
            self.crc_errors = self.crc_errors.wrapping_add(1);
            return None;
        }

        let payload = self
            .buf
            .get((cursor + MAVLINK_V2_HEADER_LEN)..body_end)
            .unwrap_or(&[]);
        match msg_id {
            HEARTBEAT_MSG_ID => {
                let base_mode = payload.get(6).copied().unwrap_or(0);
                self.ready.push_back(MavlinkHilFrame::Heartbeat {
                    system_id,
                    component_id,
                    armed: (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0,
                });
                self.decoded_frames = self.decoded_frames.wrapping_add(1);
            }
            HIL_ACTUATOR_CONTROLS_MSG_ID => {
                let controls = decode_hil_actuator_controls(payload);
                self.ready.push_back(MavlinkHilFrame::ActuatorControls {
                    system_id,
                    component_id,
                    controls,
                });
                self.decoded_frames = self.decoded_frames.wrapping_add(1);
            }
            _ => {
                self.unknown_frames = self.unknown_frames.wrapping_add(1);
            }
        }
        Some(frame_len)
    }

    fn byte_at(&self, index: usize) -> u8 {
        self.buf.get(index).copied().unwrap_or(0)
    }
}
