//! Measured Mars terrain: a MOLA MEGDR topography window around Jezero crater.
//!
//! The committed bytes are unaltered 128 pixel/degree samples of NASA PDS product
//! `MEGT44N000HB.IMG` (see `scripts/extract_mola_jezero.py` for the source checksum and crop).

use sim_geo::BodyEllipsoid;
use thiserror::Error;

use crate::{ContentHash, DemElevationGrid, HashParseError, NaifId};

const MOLA_JEZERO_BYTES: &[u8] = include_bytes!("../data/mola_jezero_megt128.bin");

/// SHA-256 of the committed window file, checked on every load.
pub const MOLA_JEZERO_SHA256: &str =
    "2a612ffe129c39d2cab9f23ab7e856d9c35ea5becdfb900eeea3def58426bcef";

/// Identifier carried by the grid built by [`DemElevationGrid::mars_jezero_mola`].
pub const MOLA_JEZERO_DATASET_ID: &str = "mola-megt44n000hb-jezero-128ppd";

/// Mean of (areoid radius − [`BodyEllipsoid::MARS_IAU`] radius) over the window, metres.
///
/// MOLA topography is height above the GMM3 areoid, whereas terrain consumers measure height
/// above the IAU ellipsoid. Over the window the difference spans `234..394 m` with a standard
/// deviation of `23.5 m`, so applying the mean leaves at most about `+-100 m` and typically
/// `+-25 m` of datum error. The value comes from the MEGR radius product minus the MEGT
/// topography product, as printed by the extraction script.
pub const MARS_AREOID_ABOVE_IAU_ELLIPSOID_M: f64 = 335.0;

const MAGIC: &[u8; 8] = b"MOLAJZ01";
const HEADER_LEN: usize = 24;
const SOURCE_LINES_PER_DEGREE: f64 = 128.0;
const SOURCE_NORTH_EDGE_DEG: f64 = 44.0;

/// Failure to load the embedded MOLA window.
#[derive(Debug, Error)]
pub enum MolaLoadError {
    /// The embedded file digest differs from [`MOLA_JEZERO_SHA256`].
    #[error("embedded MOLA window digest {actual} does not match expected {expected}")]
    ChecksumMismatch {
        /// Expected digest (hex).
        expected: String,
        /// Digest of the embedded bytes (hex).
        actual: String,
    },
    /// The expected-digest constant is not valid hex.
    #[error("invalid expected MOLA digest constant")]
    InvalidDigest(#[from] HashParseError),
    /// The file header or payload length is not the documented layout.
    #[error("malformed MOLA window: {reason}")]
    Malformed {
        /// What was wrong.
        reason: &'static str,
    },
}

impl DemElevationGrid {
    /// Real MOLA topography around Jezero crater on [`NaifId::MARS`] (`256 x 256` cells,
    /// `1/128` degree spacing, about `17.44..19.44 N`, `76.45..78.45 E`).
    ///
    /// Cell heights are metres above [`BodyEllipsoid::MARS_IAU`]: each unaltered MOLA sample
    /// (height above the areoid) plus [`MARS_AREOID_ABOVE_IAU_ELLIPSOID_M`]. The areoid model
    /// error is documented there. MOLA latitudes are planetocentric while terrain queries use
    /// geodetic latitude, so the row bounds are the geodetic latitudes of the first and last
    /// row centres; bilinear spacing between them deviates from the exact mapping by well under
    /// `20 m` of ground distance. Longitudes are east-positive in both conventions. Bin centres
    /// lie half a bin inside the product's edges.
    ///
    /// The embedded bytes are verified against [`MOLA_JEZERO_SHA256`] before use.
    pub fn mars_jezero_mola() -> Result<Self, MolaLoadError> {
        let expected = ContentHash::from_hex(MOLA_JEZERO_SHA256)?;
        let actual = ContentHash::of_bytes(MOLA_JEZERO_BYTES);
        if actual != expected {
            return Err(MolaLoadError::ChecksumMismatch {
                expected: expected.to_hex(),
                actual: actual.to_hex(),
            });
        }
        parse_window(MOLA_JEZERO_BYTES)
    }
}

fn be_u32(bytes: &[u8], at: usize) -> Option<u32> {
    let raw: [u8; 4] = bytes.get(at..at.checked_add(4)?)?.try_into().ok()?;
    Some(u32::from_be_bytes(raw))
}

fn parse_window(bytes: &[u8]) -> Result<DemElevationGrid, MolaLoadError> {
    let bad = |reason| MolaLoadError::Malformed { reason };
    if bytes.get(..8) != Some(MAGIC.as_slice()) {
        return Err(bad("bad magic"));
    }
    let field = |i: usize| be_u32(bytes, 8 + 4 * i).ok_or_else(|| bad("short header"));
    let (rows, cols) = (field(0)? as usize, field(1)? as usize);
    let (first_line, first_sample) = (f64::from(field(2)?), f64::from(field(3)?));
    let payload = bytes.get(HEADER_LEN..).ok_or_else(|| bad("short header"))?;
    if rows < 2
        || cols < 2
        || rows.checked_mul(cols).and_then(|n| n.checked_mul(2)) != Some(payload.len())
    {
        return Err(bad("payload length does not match rows x cols"));
    }

    // File order is north line first; the grid wants its first row at the southern bound.
    let mut cells_m = Vec::with_capacity(rows.saturating_mul(cols));
    for line in payload.chunks_exact(cols.saturating_mul(2)).rev() {
        cells_m.extend(line.as_chunks::<2>().0.iter().map(|&pair| {
            Some(f64::from(i16::from_be_bytes(pair)) + MARS_AREOID_ABOVE_IAU_ELLIPSOID_M)
        }));
    }

    let north_centric_deg = SOURCE_NORTH_EDGE_DEG - (first_line + 0.5) / SOURCE_LINES_PER_DEGREE;
    let south_centric_deg =
        SOURCE_NORTH_EDGE_DEG - (first_line + rows as f64 - 0.5) / SOURCE_LINES_PER_DEGREE;
    let west_deg = (first_sample + 0.5) / SOURCE_LINES_PER_DEGREE;
    let east_deg = (first_sample + cols as f64 - 0.5) / SOURCE_LINES_PER_DEGREE;
    Ok(DemElevationGrid {
        dataset_id: MOLA_JEZERO_DATASET_ID.to_owned(),
        body: NaifId::MARS,
        lat_min_rad: geodetic_from_centric_rad(south_centric_deg.to_radians()),
        lat_max_rad: geodetic_from_centric_rad(north_centric_deg.to_radians()),
        lon_min_rad: west_deg.to_radians(),
        lon_max_rad: east_deg.to_radians(),
        rows,
        cols,
        cells_m,
    })
}

fn geodetic_from_centric_rad(centric_rad: f64) -> f64 {
    let e2 = BodyEllipsoid::MARS_IAU.eccentricity_squared();
    (centric_rad.tan() / (1.0 - e2)).atan()
}

#[cfg(test)]
mod tests;
