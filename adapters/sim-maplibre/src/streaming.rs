//! Binary WebTransport / QUIC companion display session framing (`WS-E2`).
//!
//! Streams validated `Indicate` 2D/3D volumetric scenes, echoed [`ViewPose3D`] state,
//! and optional [`TerrainPatchAux`] reliefs to remote companion displays (iPad EFB,
//! Apple Vision Pro, WebXR), while accepting inbound [`InteractionFrame`] spatial
//! gestures (`Drag3D`, `Transform3D` pinch/rotate, `Activate`, `ScrubTime`) with
//! fail-closed decoding and stale-frame drop protection.

use indicate_instrument_volumes::TerrainPatchAux;
use indicate_interaction::{
    InteractionFrame, InteractionOutcome, MAX_INTERACTION_BYTES, TargetId, VIEW_POSE_BYTES,
    ViewPose3D, decode_interaction, decode_view_pose, encode_interaction, encode_view_pose,
};
use sim_indicate::{VOLUME_SCENE_BYTES, VolumeInstrument, apply_volume_gesture};

use crate::MapAdapterError;

const PACKET_MAGIC: [u8; 4] = *b"SMLP";
const PACKET_VERSION: u8 = 1;
/// Fixed header size: 4B magic + 1B version + 1B instrument + 8B seq + 48B pose + 4B scene_len + 4B aux_len.
pub const STREAM_HEADER_BYTES: usize = 4 + 1 + 1 + 8 + VIEW_POSE_BYTES + 4 + 4;
const MAX_AUX_WIRE_BYTES: usize = 4096;

/// Decoded outbound display packet received by a remote iPad / Vision Pro companion client.
#[derive(Clone, Debug, PartialEq)]
pub struct OutboundDisplayPacket {
    /// Monotonic packet sequence number (`> 0`).
    pub sequence: u64,
    /// Target volumetric instrument.
    pub instrument: VolumeInstrument,
    /// Authoritative bounded [`ViewPose3D`] applied to this scene frame.
    pub pose: ViewPose3D,
    /// Validated `Indicate` 3D volumetric scene bytes.
    pub scene_bytes: Vec<u8>,
    /// Optional `AuxChannelId::TERRAIN_PATCH_3D` relief payload.
    pub terrain_aux_bytes: Vec<u8>,
}

/// Summary of applying one inbound [`InteractionFrame`] from a remote companion display.
#[derive(Clone, Debug, PartialEq)]
pub struct InboundInteractionResult {
    /// Device sequence stamp from the decoded [`InteractionFrame`].
    pub device_sequence: u32,
    /// Updated bounded [`ViewPose3D`] after applying the spatial gesture.
    pub updated_pose: ViewPose3D,
    /// Raw [`InteractionOutcome`] reported by `Indicate`.
    pub outcome: InteractionOutcome,
    /// Target handle activated by an `Activate` gesture, if any (for explicit operator intent routing).
    pub activated_target: Option<TargetId>,
}

/// Stateful host-side WebTransport / QUIC stream session managing monotonic outbound
/// sequence numbers, bounded [`ViewPose3D`] evolution, and stale-packet rejection.
#[derive(Clone, Debug)]
pub struct CompanionStreamSession {
    instrument: VolumeInstrument,
    pose: ViewPose3D,
    next_outbound_seq: u64,
    last_inbound_seq: Option<u32>,
}

impl CompanionStreamSession {
    /// Starts a new companion stream session for `instrument` at `ViewPose3D::IDENTITY`.
    #[must_use]
    pub const fn new(instrument: VolumeInstrument) -> Self {
        Self {
            instrument,
            pose: ViewPose3D::IDENTITY,
            next_outbound_seq: 1,
            last_inbound_seq: None,
        }
    }

    /// Active volumetric instrument streamed on this session.
    #[must_use]
    pub const fn instrument(&self) -> VolumeInstrument {
        self.instrument
    }

    /// Current bounded [`ViewPose3D`] on the host.
    #[must_use]
    pub const fn view_pose(&self) -> ViewPose3D {
        self.pose
    }

    /// Encodes a datagram/stream frame carrying the current [`ViewPose3D`], validated
    /// `scene_bytes`, and optional `terrain_aux_bytes`.
    pub fn encode_outbound_packet(
        &mut self,
        scene_bytes: &[u8],
        terrain_aux_bytes: Option<&[u8]>,
    ) -> Result<Vec<u8>, MapAdapterError> {
        let aux = terrain_aux_bytes.unwrap_or(&[]);
        if scene_bytes.len() > VOLUME_SCENE_BYTES || aux.len() > MAX_AUX_WIRE_BYTES {
            return Err(MapAdapterError::StreamWire {
                detail: "scene or terrain aux payload exceeds maximum stream frame size",
            });
        }
        if !aux.is_empty() {
            let _ = TerrainPatchAux::decode(aux).map_err(|_| MapAdapterError::StreamWire {
                detail: "outbound terrain aux payload failed Indicate validation",
            })?;
        }
        let Ok(scene_len) = u32::try_from(scene_bytes.len()) else {
            return Err(MapAdapterError::StreamWire {
                detail: "scene length overflows u32",
            });
        };
        let Ok(aux_len) = u32::try_from(aux.len()) else {
            return Err(MapAdapterError::StreamWire {
                detail: "aux length overflows u32",
            });
        };

        let seq = self.next_outbound_seq;
        self.next_outbound_seq = self.next_outbound_seq.wrapping_add(1);
        let mut pose_wire = [0_u8; VIEW_POSE_BYTES];
        encode_view_pose(&self.pose, &mut pose_wire)
            .map_err(|source| MapAdapterError::InteractionWire { source })?;

        let total = STREAM_HEADER_BYTES
            .saturating_add(scene_bytes.len())
            .saturating_add(aux.len());
        let mut buf = Vec::with_capacity(total);
        buf.extend_from_slice(&PACKET_MAGIC);
        buf.push(PACKET_VERSION);
        buf.push(instrument_to_byte(self.instrument));
        buf.extend_from_slice(&seq.to_le_bytes());
        buf.extend_from_slice(&pose_wire);
        buf.extend_from_slice(&scene_len.to_le_bytes());
        buf.extend_from_slice(&aux_len.to_le_bytes());
        buf.extend_from_slice(scene_bytes);
        buf.extend_from_slice(aux);
        Ok(buf)
    }

    /// Decodes and validates an inbound [`InteractionFrame`] wire payload, drops out-of-order
    /// device frames (`Ok(None)`), and evolves `self.pose` within `instrument`'s
    /// [`indicate_interaction::ViewLimits`].
    pub fn apply_inbound_interaction_bytes(
        &mut self,
        wire: &[u8],
    ) -> Result<Option<InboundInteractionResult>, MapAdapterError> {
        let frame = decode_interaction(wire)
            .map_err(|source| MapAdapterError::InteractionWire { source })?;
        if let Some(prev) = self.last_inbound_seq
            && frame.stamp.sequence <= prev
        {
            return Ok(None);
        }
        self.last_inbound_seq = Some(frame.stamp.sequence);

        let outcome = apply_volume_gesture(self.instrument, &mut self.pose, &frame.event)
            .map_err(|source| MapAdapterError::Indicate { source })?;
        let activated_target = match outcome {
            InteractionOutcome::TargetActivated { target, .. } if !target.is_none() => Some(target),
            _ => None,
        };
        Ok(Some(InboundInteractionResult {
            device_sequence: frame.stamp.sequence,
            updated_pose: self.pose,
            outcome,
            activated_target,
        }))
    }
}

/// Encodes a client-side [`InteractionFrame`] into canonical wire bytes (`<= MAX_INTERACTION_BYTES`).
pub fn encode_inbound_interaction_bytes(
    frame: &InteractionFrame,
) -> Result<Vec<u8>, MapAdapterError> {
    let mut scratch = [0_u8; MAX_INTERACTION_BYTES];
    let written = encode_interaction(frame, &mut scratch)
        .map_err(|source| MapAdapterError::InteractionWire { source })?;
    Ok(scratch.get(..written).unwrap_or(&[]).to_vec())
}

/// Decodes an outbound companion packet on the client, returning `Ok(None)` when `sequence <= last_seen_seq`
/// (`drop-stale` WebTransport datagram semantics).
pub fn decode_outbound_packet(
    bytes: &[u8],
    last_seen_seq: Option<u64>,
) -> Result<Option<OutboundDisplayPacket>, MapAdapterError> {
    if bytes.len() < STREAM_HEADER_BYTES {
        return Err(MapAdapterError::StreamWire {
            detail: "packet shorter than fixed header",
        });
    }
    if bytes.get(0..4) != Some(PACKET_MAGIC.as_slice()) {
        return Err(MapAdapterError::StreamWire {
            detail: "packet magic mismatch",
        });
    }
    if bytes.get(4).copied() != Some(PACKET_VERSION) {
        return Err(MapAdapterError::StreamWire {
            detail: "unsupported companion stream version",
        });
    }
    let instrument = byte_to_instrument(bytes.get(5).copied().unwrap_or(255)).ok_or(
        MapAdapterError::StreamWire {
            detail: "unknown volumetric instrument discriminant",
        },
    )?;
    let seq = read_u64_le(bytes, 6)?;
    if let Some(prev) = last_seen_seq
        && seq <= prev
    {
        return Ok(None);
    }
    let pose_end = 14 + VIEW_POSE_BYTES;
    let pose_slice = bytes.get(14..pose_end).ok_or(MapAdapterError::StreamWire {
        detail: "truncated ViewPose3D slice",
    })?;
    let pose = decode_view_pose(pose_slice)
        .map_err(|source| MapAdapterError::InteractionWire { source })?;
    let scene_len = read_u32_le(bytes, pose_end)? as usize;
    let aux_len = read_u32_le(bytes, pose_end + 4)? as usize;
    if scene_len > VOLUME_SCENE_BYTES || aux_len > MAX_AUX_WIRE_BYTES {
        return Err(MapAdapterError::StreamWire {
            detail: "declared scene or aux payload exceeds bounds",
        });
    }
    let payload_start = STREAM_HEADER_BYTES;
    let scene_end = payload_start.saturating_add(scene_len);
    let total_end = scene_end.saturating_add(aux_len);
    if bytes.len() != total_end {
        return Err(MapAdapterError::StreamWire {
            detail: "packet byte length does not match header lengths",
        });
    }
    let scene_bytes = bytes.get(payload_start..scene_end).unwrap_or(&[]).to_vec();
    let terrain_aux_bytes = bytes.get(scene_end..total_end).unwrap_or(&[]).to_vec();
    if !terrain_aux_bytes.is_empty() {
        let _ = TerrainPatchAux::decode(&terrain_aux_bytes).map_err(|_| {
            MapAdapterError::StreamWire {
                detail: "received terrain aux failed Indicate validation",
            }
        })?;
    }
    Ok(Some(OutboundDisplayPacket {
        sequence: seq,
        instrument,
        pose,
        scene_bytes,
        terrain_aux_bytes,
    }))
}

const fn instrument_to_byte(instrument: VolumeInstrument) -> u8 {
    match instrument {
        VolumeInstrument::Terrain => 0,
        VolumeInstrument::Globe => 1,
        VolumeInstrument::Orrery => 2,
    }
}

const fn byte_to_instrument(raw: u8) -> Option<VolumeInstrument> {
    match raw {
        0 => Some(VolumeInstrument::Terrain),
        1 => Some(VolumeInstrument::Globe),
        2 => Some(VolumeInstrument::Orrery),
        _ => None,
    }
}

fn read_u32_le(bytes: &[u8], offset: usize) -> Result<u32, MapAdapterError> {
    let slice = bytes
        .get(offset..offset.saturating_add(4))
        .ok_or(MapAdapterError::StreamWire {
            detail: "unexpected EOF reading u32",
        })?;
    let arr: [u8; 4] = slice.try_into().map_err(|_| MapAdapterError::StreamWire {
        detail: "invalid u32 slice",
    })?;
    Ok(u32::from_le_bytes(arr))
}

fn read_u64_le(bytes: &[u8], offset: usize) -> Result<u64, MapAdapterError> {
    let slice = bytes
        .get(offset..offset.saturating_add(8))
        .ok_or(MapAdapterError::StreamWire {
            detail: "unexpected EOF reading u64",
        })?;
    let arr: [u8; 8] = slice.try_into().map_err(|_| MapAdapterError::StreamWire {
        detail: "invalid u64 slice",
    })?;
    Ok(u64::from_le_bytes(arr))
}
