//! Reference frame axes (`REF_FRAME`, `COV_REF_FRAME`, `MAN_REF_FRAME`) and
//! origins (`CENTER_NAME`).
//!
//! The listed values are those ODM 502.0-B-3 names (3.2.3.3 and 3.2.4.11)
//! plus the Solar System bodies and barycentres its examples use; SANA keeps
//! the full registry, and everything else is carried in `Other`.

use crate::open_enum::open_enum;

open_enum! {
    /// Reference frame axes.
    RefFrame {
        /// Earth Mean Equator and Equinox of J2000.
        Eme2000 => "EME2000",
        /// Geocentric Celestial Reference Frame.
        Gcrf => "GCRF",
        /// Greenwich Rotating Coordinates.
        Grc => "GRC",
        /// International Celestial Reference Frame.
        Icrf => "ICRF",
        /// International Terrestrial Reference Frame 2000.
        Itrf2000 => "ITRF2000",
        /// International Terrestrial Reference Frame 1993.
        Itrf93 => "ITRF-93",
        /// International Terrestrial Reference Frame 1997.
        Itrf97 => "ITRF-97",
        /// Mars Centred Inertial.
        Mci => "MCI",
        /// True of Date, Rotating.
        Tdr => "TDR",
        /// True Equator, Mean Equinox (OMM only).
        Teme => "TEME",
        /// True of Date.
        Tod => "TOD",
        /// Radial, Transverse, Normal (local orbital frame).
        Rtn => "RTN",
        /// Another name for RTN.
        Rsw => "RSW",
        /// Tangential, Normal, cross-track (local orbital frame).
        Tnw => "TNW",
    }
}

open_enum! {
    /// Origin of a reference frame: a natural body or a barycentre.
    CenterName {
        /// The Sun.
        Sun => "SUN",
        /// Mercury.
        Mercury => "MERCURY",
        /// Venus.
        Venus => "VENUS",
        /// The Earth.
        Earth => "EARTH",
        /// The Moon.
        Moon => "MOON",
        /// Mars.
        Mars => "MARS",
        /// Jupiter.
        Jupiter => "JUPITER",
        /// Saturn.
        Saturn => "SATURN",
        /// Uranus.
        Uranus => "URANUS",
        /// Neptune.
        Neptune => "NEPTUNE",
        /// Pluto.
        Pluto => "PLUTO",
        /// The Solar System barycentre.
        SolarSystemBarycenter => "SOLAR SYSTEM BARYCENTER",
        /// The Earth–Moon barycentre.
        EarthBarycenter => "EARTH BARYCENTER",
        /// The Mars system barycentre.
        MarsBarycenter => "MARS BARYCENTER",
        /// The Jupiter system barycentre.
        JupiterBarycenter => "JUPITER BARYCENTER",
        /// The Saturn system barycentre.
        SaturnBarycenter => "SATURN BARYCENTER",
        /// The Uranus system barycentre.
        UranusBarycenter => "URANUS BARYCENTER",
        /// The Neptune system barycentre.
        NeptuneBarycenter => "NEPTUNE BARYCENTER",
        /// The Pluto system barycentre.
        PlutoBarycenter => "PLUTO BARYCENTER",
    }
}

open_enum! {
    /// Orbit theory of an OMM's mean elements (`MEAN_ELEMENT_THEORY`).
    MeanElementTheory {
        /// SGP4, spelled as ODM 502.0-B-3 does.
        Sgp4 => "SGP4",
        /// The `SGP/SGP4` spelling CelesTrak's KVN uses.
        SgpSgp4 => "SGP/SGP4",
        /// SGP4-XP, whose drag and solar terms are BTERM and AGOM.
        Sgp4Xp => "SGP4-XP",
        /// Draper Semi-analytical Satellite Theory.
        Dsst => "DSST",
        /// Brouwer mean elements.
        Brouwer => "BROUWER",
    }
}

open_enum! {
    /// Interpolation method recommended for an OEM or AEM segment.
    Interpolation {
        /// Hermite interpolation.
        Hermite => "HERMITE",
        /// Lagrange interpolation.
        Lagrange => "LAGRANGE",
        /// Linear interpolation.
        Linear => "LINEAR",
    }
}

/// Boolean `YES` / `NO` flag used in CCSDS CDM and TDM keywords.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum YesNo {
    /// `YES`.
    Yes,
    /// `NO`.
    No,
}

impl YesNo {
    /// The keyword value, as written in a message.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Yes => "YES",
            Self::No => "NO",
        }
    }

    /// Parses `YES` or `NO` (case-sensitive in strict mode, case-insensitive in lenient mode).
    pub fn from_name(name: &str, ignore_case: bool) -> Option<Self> {
        if (ignore_case && name.eq_ignore_ascii_case("YES")) || name == "YES" {
            Some(Self::Yes)
        } else if (ignore_case && name.eq_ignore_ascii_case("NO")) || name == "NO" {
            Some(Self::No)
        } else {
            None
        }
    }
}

impl std::fmt::Display for YesNo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

#[cfg(test)]
mod tests;
