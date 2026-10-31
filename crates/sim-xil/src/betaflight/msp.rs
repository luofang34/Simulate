use std::collections::VecDeque;

use crate::XilError;

/// `MSP_STATUS` command code (`101`).
pub const MSP_STATUS: u16 = 101;
/// `MSP_RAW_IMU` command code (`102`).
pub const MSP_RAW_IMU: u16 = 102;
/// `MSP_ATTITUDE` command code (`108`).
pub const MSP_ATTITUDE: u16 = 108;
/// `MSP_ALTITUDE` command code (`109`).
pub const MSP_ALTITUDE: u16 = 109;
/// `MSP_SET_RAW_RC` command code (`200`).
pub const MSP_SET_RAW_RC: u16 = 200;

const DVB_S2_POLY: u8 = 0xD5;
const MAX_MSP_V2_PAYLOAD: usize = 4_096;

/// MSP wire protocol version.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MspVersion {
    /// Classic MultiWii `$M` framing (`u8` command & length, XOR checksum).
    V1,
    /// Extended `$X` framing (`u16` command & length, CRC-8 DVB-S2 `0xD5`).
    V2,
}

/// MSP message direction marker (`<`, `>`, or `!`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MspDirection {
    /// Request or command from host to flight controller (`<`).
    ToFc,
    /// Response from flight controller to host (`>`).
    FromFc,
    /// Error notification (`!`).
    Error,
}

impl MspDirection {
    const fn to_byte(self) -> u8 {
        match self {
            Self::ToFc => b'<',
            Self::FromFc => b'>',
            Self::Error => b'!',
        }
    }

    const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            b'<' => Some(Self::ToFc),
            b'>' => Some(Self::FromFc),
            b'!' => Some(Self::Error),
            _ => None,
        }
    }
}

/// One decoded or ready-to-encode MSP frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MspFrame {
    /// Framing version (`V1` or `V2`).
    pub version: MspVersion,
    /// Direction byte (`<`, `>`, `!`).
    pub direction: MspDirection,
    /// MSP v2 flag byte (`0` for MSP v1).
    pub flag: u8,
    /// MSP command identifier.
    pub command: u16,
    /// Command payload bytes.
    pub payload: Vec<u8>,
}

/// Computes the MSP v2 CRC-8 DVB-S2 checksum (`poly = 0xD5`, seed `0x00`).
#[must_use]
pub fn crc8_dvb_s2(bytes: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if (crc & 0x80) != 0 {
                (crc << 1) ^ DVB_S2_POLY
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Encodes an MSP v1 (`$M`) frame. Fails if `command > 255` or `payload.len() > 255`.
pub fn encode_msp_v1(
    direction: MspDirection,
    command: u8,
    payload: &[u8],
) -> Result<Vec<u8>, XilError> {
    let Ok(size) = u8::try_from(payload.len()) else {
        return Err(XilError::InvalidPacketLength {
            protocol: "MSP v1",
            expected: 255,
            actual: payload.len(),
        });
    };
    let mut out = Vec::with_capacity(6 + payload.len());
    out.extend_from_slice(&[b'$', b'M', direction.to_byte(), size, command]);
    out.extend_from_slice(payload);
    let mut xor = size ^ command;
    for &b in payload {
        xor ^= b;
    }
    out.push(xor);
    Ok(out)
}

/// Encodes an MSP v2 (`$X`) frame with CRC-8 DVB-S2 checksum.
pub fn encode_msp_v2(
    direction: MspDirection,
    flag: u8,
    command: u16,
    payload: &[u8],
) -> Result<Vec<u8>, XilError> {
    let Ok(size) = u16::try_from(payload.len()) else {
        return Err(XilError::InvalidPacketLength {
            protocol: "MSP v2",
            expected: usize::from(u16::MAX),
            actual: payload.len(),
        });
    };
    let mut out = Vec::with_capacity(9 + payload.len());
    out.extend_from_slice(&[b'$', b'X', direction.to_byte(), flag]);
    out.extend_from_slice(&command.to_le_bytes());
    out.extend_from_slice(&size.to_le_bytes());
    out.extend_from_slice(payload);
    let crc = out.get(3..).map_or(0, crc8_dvb_s2);
    out.push(crc);
    Ok(out)
}

/// Sans-IO streaming parser for MSP v1 (`$M`) and MSP v2 (`$X`) byte streams.
#[derive(Debug, Default)]
pub struct MspStreamDecoder {
    buf: Vec<u8>,
    ready: VecDeque<MspFrame>,
    checksum_errors: u64,
}

impl MspStreamDecoder {
    /// Creates an empty MSP stream decoder.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends incoming stream bytes and parses any complete MSP v1/v2 frames.
    pub fn push_bytes(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
        let mut cursor: usize = 0;
        while let Some(rel) = self
            .buf
            .get(cursor..)
            .and_then(|tail| tail.iter().position(|&b| b == b'$'))
        {
            cursor = cursor.saturating_add(rel);
            let remaining = self.buf.len().saturating_sub(cursor);
            if remaining < 6 {
                break;
            }
            match self.byte_at(cursor + 1) {
                b'M' => match self.try_v1(cursor, remaining) {
                    Step::NeedMore => break,
                    Step::Advance(n) => cursor = cursor.saturating_add(n),
                },
                b'X' => match self.try_v2(cursor, remaining) {
                    Step::NeedMore => break,
                    Step::Advance(n) => cursor = cursor.saturating_add(n),
                },
                _ => cursor = cursor.wrapping_add(1),
            }
        }
        if cursor > 0 {
            self.buf.drain(..cursor);
        }
    }

    /// Pops the next decoded MSP frame.
    pub fn pop_frame(&mut self) -> Option<MspFrame> {
        self.ready.pop_front()
    }

    /// Count of frames rejected due to XOR or CRC-8 mismatch.
    #[must_use]
    pub const fn checksum_errors(&self) -> u64 {
        self.checksum_errors
    }

    fn try_v1(&mut self, cursor: usize, remaining: usize) -> Step {
        let Some(direction) = MspDirection::from_byte(self.byte_at(cursor + 2)) else {
            return Step::Advance(1);
        };
        let size = usize::from(self.byte_at(cursor + 3));
        let frame_len = 6 + size;
        if remaining < frame_len {
            return Step::NeedMore;
        }
        let cmd = self.byte_at(cursor + 4);
        let payload = self
            .buf
            .get((cursor + 5)..(cursor + 5 + size))
            .unwrap_or(&[]);
        let mut xor = (size as u8) ^ cmd;
        for &b in payload {
            xor ^= b;
        }
        if xor != self.byte_at(cursor + 5 + size) {
            self.checksum_errors = self.checksum_errors.wrapping_add(1);
            return Step::Advance(1);
        }
        self.ready.push_back(MspFrame {
            version: MspVersion::V1,
            direction,
            flag: 0,
            command: u16::from(cmd),
            payload: payload.to_vec(),
        });
        Step::Advance(frame_len)
    }

    fn try_v2(&mut self, cursor: usize, remaining: usize) -> Step {
        if remaining < 9 {
            return Step::NeedMore;
        }
        let Some(direction) = MspDirection::from_byte(self.byte_at(cursor + 2)) else {
            return Step::Advance(1);
        };
        let flag = self.byte_at(cursor + 3);
        let command = u16::from_le_bytes([self.byte_at(cursor + 4), self.byte_at(cursor + 5)]);
        let size = usize::from(u16::from_le_bytes([
            self.byte_at(cursor + 6),
            self.byte_at(cursor + 7),
        ]));
        if size > MAX_MSP_V2_PAYLOAD {
            self.checksum_errors = self.checksum_errors.wrapping_add(1);
            return Step::Advance(1);
        }
        let frame_len = 9 + size;
        if remaining < frame_len {
            return Step::NeedMore;
        }
        let crc_region = self
            .buf
            .get((cursor + 3)..(cursor + 8 + size))
            .unwrap_or(&[]);
        if crc8_dvb_s2(crc_region) != self.byte_at(cursor + 8 + size) {
            self.checksum_errors = self.checksum_errors.wrapping_add(1);
            return Step::Advance(1);
        }
        let payload = self
            .buf
            .get((cursor + 8)..(cursor + 8 + size))
            .unwrap_or(&[])
            .to_vec();
        self.ready.push_back(MspFrame {
            version: MspVersion::V2,
            direction,
            flag,
            command,
            payload,
        });
        Step::Advance(frame_len)
    }

    fn byte_at(&self, index: usize) -> u8 {
        self.buf.get(index).copied().unwrap_or(0)
    }
}

enum Step {
    NeedMore,
    Advance(usize),
}

mod payloads;

pub use payloads::{MspAltitude, MspAttitude, MspRawImu, MspRawRc, MspStatus};
