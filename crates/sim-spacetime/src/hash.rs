//! Canonical SHA-256 content hashes and domain-separated IEEE-754 hashing helpers.

use core::fmt;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// 32-byte SHA-256 content digest for kernels, gravity tables, configurations, and analysis artifacts.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ContentHash(pub [u8; 32]);

/// Error returned when parsing a 64-character hex SHA-256 string fails.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum HashParseError {
    /// Hex string was not 64 ASCII characters long.
    #[error("SHA-256 hex digest must be 64 characters, got {0}")]
    InvalidLength(usize),
    /// Hex string contained a non-hexadecimal byte at index `0`.
    #[error("invalid hex character at byte index {0}")]
    InvalidHexByte(usize),
}

impl ContentHash {
    /// Returns a reference to the 32-byte digest array.
    #[inline]
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    /// Computes the raw SHA-256 digest of `bytes`.
    #[must_use]
    pub fn of_bytes(bytes: &[u8]) -> Self {
        let mut hasher = Sha256::new();
        hasher.update(bytes);
        let out = hasher.finalize();
        let mut raw = [0_u8; 32];
        raw.copy_from_slice(&out);
        Self(raw)
    }

    /// Parses a 64-character ASCII hex digest into a [`ContentHash`].
    pub fn from_hex(hex: &str) -> Result<Self, HashParseError> {
        let bytes = hex.as_bytes();
        if bytes.len() != 64 {
            return Err(HashParseError::InvalidLength(bytes.len()));
        }
        let mut out = [0_u8; 32];
        for (i, slot) in out.iter_mut().enumerate() {
            let hi_idx = i.wrapping_mul(2);
            let lo_idx = hi_idx.wrapping_add(1);
            let hi = nibble(*bytes.get(hi_idx).ok_or(HashParseError::InvalidLength(0))?)
                .ok_or(HashParseError::InvalidHexByte(hi_idx))?;
            let lo = nibble(*bytes.get(lo_idx).ok_or(HashParseError::InvalidLength(0))?)
                .ok_or(HashParseError::InvalidHexByte(lo_idx))?;
            *slot = (hi << 4) | lo;
        }
        Ok(Self(out))
    }

    /// Formats this digest as a 64-character lowercase hex string.
    #[must_use]
    pub fn to_hex(self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut out = String::with_capacity(64);
        for &b in &self.0 {
            let hi = HEX.get((b >> 4) as usize).copied().unwrap_or(b'0');
            let lo = HEX.get((b & 0x0f) as usize).copied().unwrap_or(b'0');
            out.push(char::from(hi));
            out.push(char::from(lo));
        }
        out
    }
}

const fn nibble(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

impl fmt::Debug for ContentHash {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ContentHash({})", self.to_hex())
    }
}

impl fmt::Display for ContentHash {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

/// Domain-separated streaming SHA-256 builder with canonical IEEE-754 `f64` encoding
/// (`-0.0` canonicalized to `+0.0`, all `NaN` payloads canonicalized to `0x7ff8_0000_0000_0000`).
pub struct DomainHasher {
    hasher: Sha256,
}

impl DomainHasher {
    /// Initializes a domain-separated SHA-256 hasher with `domain_tag`.
    #[must_use]
    pub fn new(domain_tag: &[u8]) -> Self {
        let mut hasher = Sha256::new();
        hasher.update((domain_tag.len() as u64).to_le_bytes());
        hasher.update(domain_tag);
        Self { hasher }
    }

    /// Appends raw bytes with an explicit length prefix.
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        self.hasher.update((bytes.len() as u64).to_le_bytes());
        self.hasher.update(bytes);
    }

    /// Appends a `u8`.
    pub fn write_u8(&mut self, v: u8) {
        self.hasher.update([v]);
    }

    /// Appends a little-endian `i32`.
    pub fn write_i32(&mut self, v: i32) {
        self.hasher.update(v.to_le_bytes());
    }

    /// Appends a little-endian `u32`.
    pub fn write_u32(&mut self, v: u32) {
        self.hasher.update(v.to_le_bytes());
    }

    /// Appends a little-endian `i64`.
    pub fn write_i64(&mut self, v: i64) {
        self.hasher.update(v.to_le_bytes());
    }

    /// Appends a little-endian `u64`.
    pub fn write_u64(&mut self, v: u64) {
        self.hasher.update(v.to_le_bytes());
    }

    /// Appends a little-endian `i128`.
    pub fn write_i128(&mut self, v: i128) {
        self.hasher.update(v.to_le_bytes());
    }

    /// Appends a little-endian `u128`.
    pub fn write_u128(&mut self, v: u128) {
        self.hasher.update(v.to_le_bytes());
    }

    /// Appends an `f64` after canonicalizing `-0.0 -> +0.0` and any `NaN -> 0x7ff8_0000_0000_0000`.
    pub fn write_f64_canonical(&mut self, v: f64) {
        let bits = if v == 0.0 {
            0_u64
        } else if v.is_nan() {
            0x7ff8_0000_0000_0000_u64
        } else {
            v.to_bits()
        };
        self.hasher.update(bits.to_le_bytes());
    }

    /// Appends an optional `f64` with an explicit tag byte (`0` for `None`, `1` for `Some`).
    pub fn write_opt_f64_canonical(&mut self, v: Option<f64>) {
        match v {
            None => self.write_u8(0),
            Some(val) => {
                self.write_u8(1);
                self.write_f64_canonical(val);
            }
        }
    }

    /// Appends an optional `u32` with an explicit tag byte (`0` for `None`, `1` for `Some`).
    pub fn write_opt_u32(&mut self, v: Option<u32>) {
        match v {
            None => self.write_u8(0),
            Some(val) => {
                self.write_u8(1);
                self.write_u32(val);
            }
        }
    }

    /// Appends a 3-vector of canonicalized `f64` values.
    pub fn write_vec3(&mut self, v: glam::DVec3) {
        self.write_f64_canonical(v.x);
        self.write_f64_canonical(v.y);
        self.write_f64_canonical(v.z);
    }

    /// Appends a quaternion (`x, y, z, w`) canonicalized into the hemisphere where the first
    /// non-zero component is positive (so $q$ and $-q$ hash identically for physical equivalence)
    /// or raw sign-preserving when `preserve_double_cover` is `true`.
    pub fn write_quat(&mut self, q: glam::DQuat, preserve_double_cover: bool) {
        let sign = if preserve_double_cover {
            1.0
        } else if q.w != 0.0 {
            if q.w > 0.0 { 1.0 } else { -1.0 }
        } else if q.x != 0.0 {
            if q.x > 0.0 { 1.0 } else { -1.0 }
        } else if q.y != 0.0 {
            if q.y > 0.0 { 1.0 } else { -1.0 }
        } else if q.z >= 0.0 {
            1.0
        } else {
            -1.0
        };
        self.write_f64_canonical(q.x * sign);
        self.write_f64_canonical(q.y * sign);
        self.write_f64_canonical(q.z * sign);
        self.write_f64_canonical(q.w * sign);
    }

    /// Appends a child [`ContentHash`].
    pub fn write_hash(&mut self, hash: ContentHash) {
        self.hasher.update(hash.0);
    }

    /// Finalizes the SHA-256 digest.
    #[must_use]
    pub fn finish(self) -> ContentHash {
        let out = self.hasher.finalize();
        let mut raw = [0_u8; 32];
        raw.copy_from_slice(&out);
        ContentHash(raw)
    }
}
