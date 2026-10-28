use fsg_geo::{Geodetic, UtcTime};

/// Broad cloud genus, which selects the raymarcher's density profile and noise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CloudKind {
    /// Flat layered cloud (stratus, altostratus).
    Stratiform,
    /// Heaped convective cloud (cumulus, towering cumulus).
    Cumuliform,
    /// Deep convection (cumulonimbus).
    Convective,
    /// High ice cloud (cirrus).
    Cirriform,
}

/// The weather-map value a volumetric cloud renderer reads for one column.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CloudSample {
    /// Fraction of the column covered, 0 (clear) to 1 (overcast).
    pub coverage: f32,
    /// Cloud base above the ellipsoid, metres.
    pub base_m: f32,
    /// Cloud top above the ellipsoid, metres.
    pub top_m: f32,
    /// Cloud genus.
    pub kind: CloudKind,
}

/// Cloud coverage over the globe. Decoded weather (METAR/TAF layers, GRIB
/// cloud fields) implements this so rendering and weather never diverge.
pub trait CloudField {
    /// The cloud layer over `position` at `utc`, or `None` for clear sky.
    fn sample(&self, position: &Geodetic, utc: UtcTime) -> Option<CloudSample>;
}

/// No cloud anywhere.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ClearSky;

impl CloudField for ClearSky {
    fn sample(&self, _position: &Geodetic, _utc: UtcTime) -> Option<CloudSample> {
        None
    }
}
