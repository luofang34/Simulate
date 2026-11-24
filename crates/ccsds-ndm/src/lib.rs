//! Reader and writer for CCSDS Navigation Data Messages (ODM 502.0-B-3):
//! the Orbit Parameter, Mean-Elements and Ephemeris Messages in KVN and XML,
//! and the OMM in the flat JSON layout CelesTrak and Space-Track serve.
//!
//! Messages are typed: every keyword is a field, every unit a type
//! ([`Quantity<Km>`](Quantity)), and every time system a variant. Reference
//! frames and centres are open sets that keep unlisted names verbatim.
//! Epochs stay as written ([`Epoch`]); converting time scales is left to the
//! caller.
//!
//! Reading and writing are lossless: numbers are parsed with
//! `str::parse::<f64>` and written in shortest round-trip form, so every
//! value survives bit for bit, and comments and user-defined parameters are
//! kept. Writing a parsed message and reading it back gives an equal message;
//! writing that again gives identical text. What is normalised is layout:
//! spacing, number spelling (`6503.514000` becomes `6503.514`), and where
//! within a block a comment sits.
//!
//! [`ParseMode::Strict`] holds input to the standard; [`ParseMode::Lenient`]
//! accepts what real producers send. Errors carry the line, column and
//! keyword they concern.
//!
//! ```
//! use ccsds_ndm::{Opm, ParseMode, WriteOptions};
//!
//! let text = "CCSDS_OPM_VERS = 3.0\nCREATION_DATE = 2022-11-06T09:23:57\n\
//!     ORIGINATOR = JAXA\nOBJECT_NAME = OSPREY 5\nOBJECT_ID = 1998-999A\n\
//!     CENTER_NAME = EARTH\nREF_FRAME = ITRF2000\nTIME_SYSTEM = UTC\n\
//!     EPOCH = 2022-12-18T14:28:15.1172\nX = 6503.514 [km]\nY = 1239.647\n\
//!     Z = -717.49\nX_DOT = -0.87316\nY_DOT = 8.74042\nZ_DOT = -4.191076\n";
//! let opm = Opm::from_kvn(text, ParseMode::Strict)?;
//! assert_eq!(opm.data.state.position[0].value(), 6503.514);
//! let again = Opm::from_xml(&opm.to_xml(WriteOptions::default())?, ParseMode::Strict)?;
//! assert_eq!(again, opm);
//! # Ok::<(), ccsds_ndm::Error>(())
//! ```

mod block;
mod covariance;
mod epoch;
mod error;
mod extension;
mod flat;
mod frame;
mod header;
#[cfg(feature = "json")]
mod json;
mod kvn;
mod ndm;
mod number;
mod oem;
mod omm;
mod open_enum;
mod opm;
mod options;
mod state;
mod time_system;
mod unit;
mod xml;

pub use covariance::{Covariance6, CovarianceBlock};
pub use epoch::{DateForm, Epoch, EpochError};
pub use error::{Error, ErrorKind, Location, Result};
pub use extension::{Extension, ExtensionValue, UserDefined, UserParameter};
pub use frame::{CenterName, Interpolation, MeanElementTheory, RefFrame};
pub use header::{Header, Version};
pub use ndm::{Ndm, NdmMessage};
pub use oem::{EphemerisCovariance, EphemerisState, Oem, OemData, OemMetadata, OemSegment};
pub use omm::{
    MeanElements, Omm, OmmData, OmmMetadata, OrbitSize, TleDrag, TleParameters, TleSecondTerm,
};
pub use opm::{Anomaly, KeplerianElements, Maneuver, Opm, OpmData, OpmMetadata};
pub use options::{ParseMode, WriteOptions};
pub use state::{SpacecraftParameters, StateVector};
pub use time_system::TimeSystem;
pub use unit::{
    Deg, Kg, Km, Km3PerS2, KmPerS, KmPerS2, M2, M2PerKg, PerEarthRadius, Quantity, RevPerDay,
    RevPerDay2, RevPerDay3, Seconds, Unit, Vector3, vector3,
};
