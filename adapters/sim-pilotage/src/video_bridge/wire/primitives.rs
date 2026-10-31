use crate::PilotageBridgeError;

pub(super) const WIRE_VARINT: u8 = 0;
pub(super) const WIRE_FIXED64: u8 = 1;
pub(super) const WIRE_BYTES: u8 = 2;
pub(super) const WIRE_FIXED32: u8 = 5;

pub(super) fn put_tag(out: &mut Vec<u8>, field_num: u32, wire_type: u8) {
    put_varint(out, (u64::from(field_num) << 3) | u64::from(wire_type));
}

pub(super) fn put_varint(out: &mut Vec<u8>, mut value: u64) {
    while value >= 0x80 {
        out.push(((value as u8) & 0x7F) | 0x80);
        value >>= 7;
    }
    out.push(value as u8);
}

pub(super) fn put_u64_nz(out: &mut Vec<u8>, field_num: u32, val: u64) {
    if val != 0 {
        put_tag(out, field_num, WIRE_VARINT);
        put_varint(out, val);
    }
}

pub(super) fn put_u32_nz(out: &mut Vec<u8>, field_num: u32, val: u32) {
    put_u64_nz(out, field_num, u64::from(val));
}

pub(super) fn put_f64_nz(out: &mut Vec<u8>, field_num: u32, val: f64) {
    if val.to_bits() != 0 {
        put_tag(out, field_num, WIRE_FIXED64);
        out.extend_from_slice(&val.to_le_bytes());
    }
}

pub(super) fn put_f32_nz(out: &mut Vec<u8>, field_num: u32, val: f32) {
    if val.to_bits() != 0 {
        put_tag(out, field_num, WIRE_FIXED32);
        out.extend_from_slice(&val.to_le_bytes());
    }
}

pub(super) fn put_nested(out: &mut Vec<u8>, field_num: u32, body: &[u8]) {
    put_tag(out, field_num, WIRE_BYTES);
    put_varint(out, body.len() as u64);
    out.extend_from_slice(body);
}

pub(super) fn try_take_varint(bytes: &[u8]) -> Result<Option<(u64, usize)>, PilotageBridgeError> {
    let mut value: u64 = 0;
    let mut shift: u32 = 0;
    for (idx, &byte) in bytes.iter().enumerate() {
        value |= u64::from(byte & 0x7F) << shift;
        if (byte & 0x80) == 0 {
            return Ok(Some((value, idx.wrapping_add(1))));
        }
        shift = shift.wrapping_add(7);
        if shift >= 64 {
            return Err(wire_err("varint exceeds 64 bits"));
        }
    }
    Ok(None)
}

pub(super) fn take_varint(buf: &mut &[u8]) -> Result<u64, PilotageBridgeError> {
    let Some((val, used)) = try_take_varint(buf)? else {
        return Err(wire_err("truncated varint"));
    };
    *buf = buf.get(used..).unwrap_or(&[]);
    Ok(val)
}

pub(super) fn take_u32(buf: &mut &[u8]) -> Result<u32, PilotageBridgeError> {
    u32::try_from(take_varint(buf)?).map_err(|_| wire_err("uint32 overflow"))
}

pub(super) fn take_tag(buf: &mut &[u8]) -> Result<(u32, u8), PilotageBridgeError> {
    let raw = take_varint(buf)?;
    let field_num = u32::try_from(raw >> 3).map_err(|_| wire_err("field tag overflow"))?;
    if field_num == 0 {
        return Err(wire_err("zero protobuf field number"));
    }
    Ok((field_num, (raw & 0x07) as u8))
}

pub(super) fn take_bytes<'a>(buf: &mut &'a [u8]) -> Result<&'a [u8], PilotageBridgeError> {
    let len = usize::try_from(take_varint(buf)?).map_err(|_| wire_err("bytes len overflow"))?;
    let Some(slice) = buf.get(..len) else {
        return Err(wire_err("truncated length-delimited field"));
    };
    *buf = buf.get(len..).unwrap_or(&[]);
    Ok(slice)
}

pub(super) fn take_f64(buf: &mut &[u8]) -> Result<f64, PilotageBridgeError> {
    let Some(slice) = buf.get(..8) else {
        return Err(wire_err("truncated fixed64 field"));
    };
    let mut raw = [0_u8; 8];
    raw.copy_from_slice(slice);
    *buf = buf.get(8..).unwrap_or(&[]);
    Ok(f64::from_le_bytes(raw))
}

pub(super) fn take_f32(buf: &mut &[u8]) -> Result<f32, PilotageBridgeError> {
    let Some(slice) = buf.get(..4) else {
        return Err(wire_err("truncated fixed32 field"));
    };
    let mut raw = [0_u8; 4];
    raw.copy_from_slice(slice);
    *buf = buf.get(4..).unwrap_or(&[]);
    Ok(f32::from_le_bytes(raw))
}

pub(super) fn skip_field(buf: &mut &[u8], wire_type: u8) -> Result<(), PilotageBridgeError> {
    match wire_type {
        WIRE_VARINT => {
            let _ = take_varint(buf)?;
            Ok(())
        }
        WIRE_FIXED64 => {
            let _ = take_f64(buf)?;
            Ok(())
        }
        WIRE_BYTES => {
            let _ = take_bytes(buf)?;
            Ok(())
        }
        WIRE_FIXED32 => {
            let _ = take_f32(buf)?;
            Ok(())
        }
        _ => Err(wire_err("unsupported protobuf wire type")),
    }
}

pub(super) fn wire_err(detail: &str) -> PilotageBridgeError {
    PilotageBridgeError::VideoWire {
        detail: detail.to_owned(),
    }
}
