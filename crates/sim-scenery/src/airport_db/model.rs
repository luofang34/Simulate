//! The airport database's records: identifiers, tiers, runways with their
//! surveyed ends, and communication frequencies.

use std::fmt;

use flightbag::{AuthoritySource, FacilityCode};

/// Handle to one record in an [`crate::AirportDatabase`]; valid only for the
/// database that issued it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AirportIdx(pub u32);

impl AirportIdx {
    /// The record's position in [`crate::AirportDatabase::records`].
    #[must_use]
    pub fn index(self) -> usize {
        usize::try_from(self.0).unwrap_or(usize::MAX)
    }
}

/// Up to eight ASCII characters held on the stack: an airport code, a
/// runway designator (`04L`, `04L/22R`) or a region code (`US-NJ`).
#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CompactCode([u8; 8]);

impl CompactCode {
    /// The trimmed, upper-cased code; `None` when empty, longer than eight
    /// bytes, or holding anything but ASCII letters, digits, `-` and `/`.
    #[must_use]
    pub fn parse(s: &str) -> Option<Self> {
        let bytes = s.trim().as_bytes();
        if bytes.is_empty() || bytes.len() > 8 {
            return None;
        }
        let mut buf = [0_u8; 8];
        for (dst, &src) in buf.iter_mut().zip(bytes) {
            if !(src.is_ascii_alphanumeric() || src == b'-' || src == b'/') {
                return None;
            }
            *dst = src.to_ascii_uppercase();
        }
        Some(Self(buf))
    }

    /// `a/b`, the runway identifier of two end designators, when it fits.
    #[must_use]
    pub fn joined(a: Self, b: Self) -> Option<Self> {
        let (a, b) = (a.as_str().as_bytes(), b.as_str().as_bytes());
        if a.len() + b.len() + 1 > 8 {
            return None;
        }
        let mut buf = [0_u8; 8];
        let bytes = a.iter().chain(b"/").chain(b);
        for (dst, &src) in buf.iter_mut().zip(bytes) {
            *dst = src;
        }
        Some(Self(buf))
    }

    /// The code as text.
    #[must_use]
    pub fn as_str(&self) -> &str {
        let len = self.0.iter().position(|&b| b == 0).unwrap_or(self.0.len());
        self.0
            .get(..len)
            .and_then(|bytes| std::str::from_utf8(bytes).ok())
            .unwrap_or("")
    }

    /// The code as a `Flightbag` facility code, which admits letters and
    /// digits only.
    #[must_use]
    pub fn to_facility_code(&self) -> Option<FacilityCode> {
        FacilityCode::parse(self.as_str()).ok()
    }

    /// Whether the code reads as an ICAO location indicator: four ASCII
    /// letters.
    #[must_use]
    pub fn is_icao_like(&self) -> bool {
        let text = self.as_str();
        text.len() == 4 && text.bytes().all(|b| b.is_ascii_uppercase())
    }
}

impl fmt::Debug for CompactCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "CompactCode({:?})", self.as_str())
    }
}

impl fmt::Display for CompactCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Every code an airport answers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AirportIds {
    /// The code shown first: the ICAO indicator (`KEWR`, `EGLL`) when the
    /// airport has one, else its FAA or local identifier (`47N`, `0J0`).
    pub primary: CompactCode,
    /// ICAO location indicator.
    pub icao: Option<CompactCode>,
    /// FAA location identifier for a US airport; elsewhere the source's
    /// local identifier, when the airport has no ICAO indicator.
    pub faa_lid: Option<CompactCode>,
    /// IATA airline code.
    pub iata: Option<CompactCode>,
}

impl AirportIds {
    /// Ids with [`Self::primary`] chosen from the others: a four-letter
    /// ICAO code, else the FAA identifier, else any other code given.
    /// `None` when no code is given.
    #[must_use]
    pub fn new(
        icao: Option<CompactCode>,
        faa_lid: Option<CompactCode>,
        iata: Option<CompactCode>,
    ) -> Option<Self> {
        let icao_like = icao.filter(CompactCode::is_icao_like);
        let primary = icao_like.or(faa_lid).or(icao).or(iata)?;
        Some(Self {
            primary,
            icao,
            faa_lid,
            iata,
        })
    }

    /// The distinct codes, primary first.
    pub fn codes(&self) -> impl Iterator<Item = CompactCode> + '_ {
        let all = [Some(self.primary), self.icao, self.faa_lid, self.iata];
        all.into_iter().enumerate().filter_map(move |(i, code)| {
            let code = code?;
            let earlier = all.iter().take(i).flatten().any(|seen| *seen == code);
            (!earlier).then_some(code)
        })
    }
}

/// How prominent an aerodrome is, which decides the zoom it appears at.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum AerodromeTier {
    /// Towered with a runway of at least 2,400 m: hubs and major fields.
    Tier1Major = 1,
    /// A hard-surfaced runway of at least 1,350 m.
    Tier2Regional = 2,
    /// Hard-surfaced, towered, or a runway of at least 750 m.
    Tier3Local = 3,
    /// Short turf and minor strips.
    Tier4Minor = 4,
}

impl AerodromeTier {
    /// Every tier, most prominent first.
    pub const ALL: [Self; 4] = [
        Self::Tier1Major,
        Self::Tier2Regional,
        Self::Tier3Local,
        Self::Tier4Minor,
    ];

    /// The tier the classification rules give an airport.
    #[must_use]
    pub fn classify(towered: bool, hard_surface: bool, longest_runway_m: f32) -> Self {
        if towered && longest_runway_m >= 2_400.0 {
            Self::Tier1Major
        } else if hard_surface && longest_runway_m >= 1_350.0 {
            Self::Tier2Regional
        } else if hard_surface || towered || longest_runway_m >= 750.0 {
            Self::Tier3Local
        } else {
            Self::Tier4Minor
        }
    }

    /// The least prominent tier a globe view shows at `zoom`.
    #[must_use]
    pub fn max_visible_at_zoom(zoom: f64) -> Self {
        if zoom < 2.6 {
            Self::Tier1Major
        } else if zoom < 4.5 {
            Self::Tier2Regional
        } else if zoom < 6.8 {
            Self::Tier3Local
        } else {
            Self::Tier4Minor
        }
    }

    /// The tier with discriminant `value`.
    #[must_use]
    pub fn from_u8(value: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|tier| *tier as u8 == value)
    }
}

/// What a runway is surfaced with.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum RunwaySurfaceKind {
    /// Asphalt or bitumen.
    Asphalt = 0,
    /// Concrete.
    Concrete = 1,
    /// Another hard pavement (macadam, treated, mixed).
    Paved = 2,
    /// Grass.
    Turf = 3,
    /// Gravel.
    Gravel = 4,
    /// Dirt, sand, clay or earth.
    Dirt = 5,
    /// A water runway.
    Water = 6,
    /// Not recorded or not recognised.
    Unknown = 7,
    /// Composite hard pavement and turf (`ASPH-TURF`, `CONC-TURF`), e.g. central
    /// asphalt with turf displaced extensions.
    PavedTurf = 8,
}

impl RunwaySurfaceKind {
    const ALL: [Self; 9] = [
        Self::Asphalt,
        Self::Concrete,
        Self::Paved,
        Self::Turf,
        Self::Gravel,
        Self::Dirt,
        Self::Water,
        Self::Unknown,
        Self::PavedTurf,
    ];

    /// Reads a surface code by its material tokens, case-insensitively:
    /// FAA NASR codes (`ASPH-CONC`, `ASPH-G`, `ASPH-TURF`, `TURF`, `GRVL`) and
    /// free-text OurAirports ones (`ASP`, `asphalt`, `GRS`, `concrete`).
    #[must_use]
    pub fn from_code(code: &str) -> Self {
        let lower = code.trim().to_ascii_lowercase();
        let has_hard = [
            "asp", "bit", "tar", "con", "cop", "cem", "pem", "pav", "hard",
        ]
        .iter()
        .any(|p| lower.contains(p));
        let has_turf = ["turf", "grs", "grass", "gras", "sod"]
            .iter()
            .any(|p| lower.contains(p));
        if has_hard && has_turf {
            return Self::PavedTurf;
        }
        let starts = |prefixes: &[&str]| prefixes.iter().any(|p| lower.starts_with(p));
        if starts(&["asp", "bit", "tar", "bitumen"]) {
            Self::Asphalt
        } else if starts(&["con", "cop", "cem"]) {
            Self::Concrete
        } else if starts(&["pem", "pav", "hard", "trtd", "tre", "mac", "seal", "bri"]) {
            Self::Paved
        } else if starts(&["turf", "grs", "grass", "gras", "tur", "sod"]) {
            Self::Turf
        } else if starts(&["grv", "gravel", "gvl", "gra"]) {
            Self::Gravel
        } else if starts(&[
            "dirt", "dir", "clay", "cla", "sand", "san", "earth", "ear", "soil", "lat",
        ]) {
            Self::Dirt
        } else if starts(&["wat"]) {
            Self::Water
        } else {
            Self::Unknown
        }
    }

    /// Whether the runway has hard-surfaced landing pavement.
    #[must_use]
    pub const fn is_hard(self) -> bool {
        matches!(
            self,
            Self::Asphalt | Self::Concrete | Self::Paved | Self::PavedTurf
        )
    }

    /// Whether the landing surface is unpaved (`Turf`, `Gravel`, or `Dirt`),
    /// portrayed with dashed SVS/HUD outlines and no painted centreline stripes.
    #[must_use]
    pub const fn is_unpaved(self) -> bool {
        matches!(self, Self::Turf | Self::Gravel | Self::Dirt)
    }

    /// The surface with discriminant `value`.
    #[must_use]
    pub fn from_u8(value: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|kind| *kind as u8 == value)
    }
}

/// What a frequency is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum FrequencyRole {
    /// Common traffic advisory frequency.
    Ctaf = 0,
    /// Aeronautical advisory station.
    Unicom = 1,
    /// Automatic terminal information service.
    Atis = 2,
    /// Automated weather observing (AWOS / ASOS).
    Awos = 3,
    /// Ground control.
    Ground = 4,
    /// Tower (local control).
    Tower = 5,
    /// Approach control.
    Approach = 6,
    /// Departure control.
    Departure = 7,
}

impl FrequencyRole {
    const ALL: [Self; 8] = [
        Self::Ctaf,
        Self::Unicom,
        Self::Atis,
        Self::Awos,
        Self::Ground,
        Self::Tower,
        Self::Approach,
        Self::Departure,
    ];

    /// The role with discriminant `value`.
    #[must_use]
    pub fn from_u8(value: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|role| *role as u8 == value)
    }
}

/// One published frequency.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AirportFrequency {
    /// What it is for.
    pub role: FrequencyRole,
    /// The frequency, kilohertz (`118_300` for 118.300 MHz).
    pub khz: u32,
}

impl AirportFrequency {
    /// The frequency, megahertz.
    #[must_use]
    pub fn mhz(&self) -> f64 {
        f64::from(self.khz) / 1_000.0
    }
}

/// One end of a runway, as surveyed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunwayEndRecord {
    /// Runway-end designator (`04L`, `22R`, `07`).
    pub designator: CompactCode,
    /// Latitude of the runway end, degrees.
    pub lat_deg: f64,
    /// Longitude of the runway end, degrees.
    pub lon_deg: f64,
    /// Surveyed threshold elevation, metres above mean sea level (EGM96).
    pub elevation_msl_m: f32,
    /// How far the landing threshold is displaced past the runway end,
    /// metres.
    pub displaced_threshold_m: f32,
    /// True heading from this end toward the other, degrees.
    pub true_heading_deg: f32,
    /// Magnetic heading from this end toward the other, degrees.
    pub magnetic_heading_deg: f32,
    /// Visual glide path angle, degrees; 3 where none is published.
    pub glidepath_deg: f32,
    /// An ILS (or LOC/GS family) approach serves this end.
    pub has_ils: bool,
    /// A visual glide slope indicator (PAPI / VASI) serves this end.
    pub has_vgsi: bool,
    /// An approach lighting system serves this end.
    pub has_approach_lights: bool,
}

/// One runway: dimensions, surface and its two ends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunwayRecord {
    /// Runway identifier (`04L/22R`).
    pub id: CompactCode,
    /// Published length, metres.
    pub length_m: f32,
    /// Published width, metres.
    pub width_m: f32,
    /// Surface material.
    pub surface: RunwaySurfaceKind,
    /// Edge lighting is installed.
    pub lighted: bool,
    /// The two ends, in the order the identifier names them.
    pub ends: [RunwayEndRecord; 2],
}

/// One airport in the database.
#[derive(Debug, Clone, PartialEq)]
pub struct AirportRecord {
    /// Every code the airport answers to.
    pub ids: AirportIds,
    /// [`AirportIds::primary`] as a `Flightbag` facility code, with any
    /// character `Flightbag` does not admit dropped.
    pub facility: FacilityCode,
    /// Airport name.
    pub name: Box<str>,
    /// City or town served.
    pub municipality: Box<str>,
    /// ISO 3166-1 alpha-2 country code bytes (`b"US"`).
    pub country_iso2: [u8; 2],
    /// ISO 3166-2 region code (`US-NJ`, `GB-ENG`, `JP-13`).
    pub region_code: CompactCode,
    /// Aerodrome reference point latitude, degrees.
    pub lat_deg: f64,
    /// Aerodrome reference point longitude, degrees.
    pub lon_deg: f64,
    /// Field elevation, metres above mean sea level (EGM96).
    pub elevation_msl_m: f32,
    /// Magnetic variation, degrees, east positive.
    pub mag_var_deg: f32,
    /// An air traffic control tower operates.
    pub towered: bool,
    /// Prominence class.
    pub tier: AerodromeTier,
    /// Longest runway, metres.
    pub longest_runway_m: f32,
    /// Some runway is hard-surfaced.
    pub hard_surface: bool,
    /// The authority the record was compiled from.
    pub authority: AuthoritySource,
    /// Effective date of that authority's edition, `[year, month, day]`.
    pub effective_ymd: [u16; 3],
    /// Runways with two surveyed ends.
    pub runways: Box<[RunwayRecord]>,
    /// Published frequencies, each role's primary first.
    pub frequencies: Box<[AirportFrequency]>,
}

/// The `Flightbag` facility code of `primary`: its letters and digits.
#[must_use]
pub fn facility_code_of(primary: CompactCode) -> Option<FacilityCode> {
    let mut buf = [0_u8; 8];
    let len = buf
        .iter_mut()
        .zip(primary.0.iter().filter(|b| b.is_ascii_alphanumeric()))
        .map(|(dst, &src)| *dst = src)
        .count();
    let text = std::str::from_utf8(buf.get(..len)?).ok()?;
    FacilityCode::parse(text).ok()
}

impl AirportRecord {
    /// The country code as text.
    #[must_use]
    pub fn country(&self) -> &str {
        std::str::from_utf8(&self.country_iso2).unwrap_or("")
    }

    /// Recomputes [`Self::longest_runway_m`], [`Self::hard_surface`] and
    /// [`Self::tier`] from the runways and [`Self::towered`].
    pub fn classify(&mut self) {
        self.longest_runway_m = self
            .runways
            .iter()
            .map(|runway| runway.length_m)
            .fold(0.0, f32::max);
        self.hard_surface = self.runways.iter().any(|runway| runway.surface.is_hard());
        self.tier = AerodromeTier::classify(self.towered, self.hard_surface, self.longest_runway_m);
    }
}

/// Magnetic heading of a true heading under east-positive `mag_var_deg`,
/// in `[0, 360)`.
#[must_use]
pub fn magnetic_heading_deg(true_heading_deg: f32, mag_var_deg: f32) -> f32 {
    (true_heading_deg - mag_var_deg).rem_euclid(360.0)
}
