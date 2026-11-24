//! CCSDS Navigation Data Messages in KVN, XML, and OMM flat JSON: ODM (OPM, OMM, OEM),
//! ADM (APM, AEM), CDM, and TDM.
//!
//! Typed keywords with explicit [`Quantity`] units, shortest round-trip floats,
//! lossless [`Epoch`] strings, and [`ParseMode::Strict`] / [`ParseMode::Lenient`] support.
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

mod adm;
mod aem;
mod apm;
mod block;
mod cdm;
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
mod tdm;
mod time_system;
mod unit;
mod xml;

pub use {adm::AttitudeDirection, adm::AttitudeType, adm::EulerRotSeq};
pub use {adm::Quaternion, adm::QuaternionDot, adm::QuaternionType};
pub use {aem::Aem, aem::AemAttitudeState, aem::AemCovariance, aem::AemData};
pub use {aem::AemMetadata, aem::AemSegment, aem::AemState};
pub use {apm::Apm, apm::ApmAngularVelocity, apm::ApmData, apm::ApmEuler};
pub use {apm::ApmInertia, apm::ApmManeuver, apm::ApmMetadata, apm::ApmQuaternion, apm::ApmSpin};
pub use {cdm::Cdm, cdm::CdmAdditionalParameters, cdm::CdmCovariance, cdm::CdmData};
pub use {cdm::CdmHeader, cdm::CdmMetadata, cdm::CdmObject, cdm::CdmOdParameters};
pub use {cdm::CdmRelativeMetadata, cdm::CdmRelativeStateVector};
pub use {cdm::CdmSegment, cdm::CdmStateVector, covariance::Covariance6};
pub use {covariance::CovarianceBlock, epoch::DateForm, epoch::Epoch, epoch::EpochError};
pub use {error::Error, error::ErrorKind, error::Location, error::Result};
pub use {extension::Extension, extension::ExtensionValue};
pub use {extension::UserDefined, extension::UserParameter, frame::CenterName};
pub use {frame::Interpolation, frame::MeanElementTheory, frame::RefFrame, frame::YesNo};
pub use {header::Header, header::Version, ndm::Ndm, ndm::NdmMessage, time_system::TimeSystem};
pub use {oem::EphemerisCovariance, oem::EphemerisState, oem::Oem, oem::OemData};
pub use {oem::OemMetadata, oem::OemSegment, omm::MeanElements, omm::Omm, omm::OmmData};
pub use {omm::OmmMetadata, omm::OrbitSize, omm::TleDrag, omm::TleParameters, omm::TleSecondTerm};
pub use {opm::Anomaly, opm::KeplerianElements, opm::Maneuver};
pub use {opm::Opm, opm::OpmData, opm::OpmMetadata, options::ParseMode, options::WriteOptions};
pub use {state::SpacecraftParameters, state::StateVector};
pub use {tdm::Tdm, tdm::TdmAngleType, tdm::TdmData, tdm::TdmDataQuality, tdm::TdmIntegrationRef};
pub use {tdm::TdmMetadata, tdm::TdmMode, tdm::TdmObservable, tdm::TdmObservation};
pub use {tdm::TdmRangeMode, tdm::TdmRangeUnits, tdm::TdmSegment, tdm::TdmTimetagRef};
pub use {unit::Days, unit::DbHz, unit::DbW, unit::Deg, unit::DegPerS, unit::Hpa, unit::Hz};
pub use {unit::HzPerS, unit::Kelvin, unit::Kg, unit::KgM2, unit::Km, unit::Km3PerS2};
pub use {unit::KmPerS, unit::KmPerS2, unit::M2, unit::M2PerKg, unit::MPerS, unit::MPerS2};
pub use {unit::Meters, unit::Nm, unit::PerEarthRadius, unit::PerS, unit::Percent, unit::Quantity};
pub use {unit::RevPerDay, unit::RevPerDay2, unit::RevPerDay3, unit::SPerS, unit::Seconds};
pub use {unit::Tecu, unit::Unit, unit::Vector3, unit::WPerKg, unit::vector3};
