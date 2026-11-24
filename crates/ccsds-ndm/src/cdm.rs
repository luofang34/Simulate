//! Conjunction Data Message (CDM 508.0-B-1): close-approach screening,
//! relative state vector, Object 1 / Object 2 states, and RTN covariances
//! in KVN and XML.

mod cov;
mod kvn;
mod layout;
mod segment;
mod xml;

use std::fmt;

use crate::error::Result;
use crate::extension::{Extension, UserDefined};
use crate::frame::{CenterName, RefFrame, YesNo};
use crate::header::Version;
use crate::unit::{
    Days, Kg, Km, KmPerS, M2, M2PerKg, MPerS, MPerS2, Meters, Percent, Quantity, Vector3, WPerKg,
};
use crate::{Epoch, ParseMode, WriteOptions, xml as xml_codec};

pub use cov::CdmCovariance;

/// A Conjunction Data Message.
#[derive(Debug, Clone, PartialEq)]
pub struct Cdm {
    /// `CCSDS_CDM_VERS` (`1.0`).
    pub version: Version,
    /// CDM header (supports optional `MESSAGE_FOR`).
    pub header: CdmHeader,
    /// Encounter geometry, screening window, and collision probability.
    pub relative_metadata: CdmRelativeMetadata,
    /// Primary object (`OBJECT = OBJECT1`).
    pub object1: CdmSegment,
    /// Secondary object (`OBJECT = OBJECT2`).
    pub object2: CdmSegment,
    /// Optional user-defined parameters.
    pub user_defined: Option<UserDefined>,
    /// Undefined keywords kept in lenient mode.
    pub extensions: Vec<Extension>,
}

/// Header of a [`Cdm`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CdmHeader {
    /// Comments at the start of the header.
    pub comments: Vec<String>,
    /// `CREATION_DATE`.
    pub creation_date: Epoch,
    /// `ORIGINATOR`.
    pub originator: String,
    /// Optional `MESSAGE_FOR` (recipient / asset identifier).
    pub message_for: Option<String>,
    /// `MESSAGE_ID` (mandatory in CDM 508.0-B-1).
    pub message_id: String,
}

/// Object role identifier in a CDM (`OBJECT1` or `OBJECT2`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CdmObject {
    /// `OBJECT1` (typically the primary asset).
    Object1,
    /// `OBJECT2` (typically the secondary / chaser object).
    Object2,
}

impl CdmObject {
    /// Keyword spelling (`OBJECT1` or `OBJECT2`).
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Object1 => "OBJECT1",
            Self::Object2 => "OBJECT2",
        }
    }

    /// Parses `OBJECT1` or `OBJECT2`.
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "OBJECT1" => Some(Self::Object1),
            "OBJECT2" => Some(Self::Object2),
            _ => None,
        }
    }
}

impl fmt::Display for CdmObject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Relative conjunction metadata and encounter state (`<relativeMetadataData>`).
#[derive(Debug, Clone, PartialEq)]
pub struct CdmRelativeMetadata {
    /// Comments attached to relative metadata.
    pub comments: Vec<String>,
    /// Time of closest approach (`TCA`).
    pub tca: Epoch,
    /// Miss distance at TCA (`MISS_DISTANCE`, `m`).
    pub miss_distance: Quantity<Meters>,
    /// Relative speed magnitude (`RELATIVE_SPEED`, `m/s`).
    pub relative_speed: Option<Quantity<MPerS>>,
    /// Relative RTN position and velocity at TCA.
    pub relative_state: Option<CdmRelativeStateVector>,
    /// Start of screening period (`START_SCREEN_PERIOD`).
    pub start_screen_period: Option<Epoch>,
    /// Stop of screening period (`STOP_SCREEN_PERIOD`).
    pub stop_screen_period: Option<Epoch>,
    /// Reference frame of screening volume (`SCREEN_VOLUME_FRAME`).
    pub screen_volume_frame: Option<RefFrame>,
    /// Screening volume shape (`SCREEN_VOLUME_SHAPE`, e.g. `ELLIPSOID` or `BOX`).
    pub screen_volume_shape: Option<String>,
    /// Screening volume X/R semi-axis or half-width (`SCREEN_VOLUME_X`, `m`).
    pub screen_volume_x: Option<Quantity<Meters>>,
    /// Screening volume Y/T semi-axis or half-width (`SCREEN_VOLUME_Y`, `m`).
    pub screen_volume_y: Option<Quantity<Meters>>,
    /// Screening volume Z/N semi-axis or half-width (`SCREEN_VOLUME_Z`, `m`).
    pub screen_volume_z: Option<Quantity<Meters>>,
    /// Screening volume entry epoch (`SCREEN_ENTRY_TIME`).
    pub screen_entry_time: Option<Epoch>,
    /// Screening volume exit epoch (`SCREEN_EXIT_TIME`).
    pub screen_exit_time: Option<Epoch>,
    /// Estimated collision probability (`COLLISION_PROBABILITY`, 0..1).
    pub collision_probability: Option<f64>,
    /// Method used to compute collision probability (`COLLISION_PROBABILITY_METHOD`).
    pub collision_probability_method: Option<String>,
}

/// Relative state vector at TCA in Object 1's RTN frame (`<relativeStateVector>`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CdmRelativeStateVector {
    /// `[RELATIVE_POSITION_R, RELATIVE_POSITION_T, RELATIVE_POSITION_N]` (`m`).
    pub position: Vector3<Meters>,
    /// `[RELATIVE_VELOCITY_R, RELATIVE_VELOCITY_T, RELATIVE_VELOCITY_N]` (`m/s`).
    pub velocity: Vector3<MPerS>,
}

/// One object's metadata and data segment (`<segment>`).
#[derive(Debug, Clone, PartialEq)]
pub struct CdmSegment {
    /// Object metadata.
    pub metadata: CdmMetadata,
    /// Object OD parameters, additional parameters, state vector, and RTN covariance.
    pub data: CdmData,
}

/// Metadata for `OBJECT1` or `OBJECT2` in a [`Cdm`].
#[derive(Debug, Clone, PartialEq)]
pub struct CdmMetadata {
    /// Comments in object metadata.
    pub comments: Vec<String>,
    /// `OBJECT` (`OBJECT1` or `OBJECT2`).
    pub object: CdmObject,
    /// `OBJECT_DESIGNATOR` (e.g., catalog number).
    pub object_designator: String,
    /// `CATALOG_NAME` (e.g., `SATCAT`).
    pub catalog_name: String,
    /// `OBJECT_NAME`.
    pub object_name: String,
    /// `INTERNATIONAL_DESIGNATOR`.
    pub international_designator: String,
    /// Optional `OBJECT_TYPE` (`PAYLOAD`, `DEBRIS`, `ROCKET BODY`, etc.).
    pub object_type: Option<String>,
    /// `OPERATOR_CONTACT_POSITION`.
    pub operator_contact_position: Option<String>,
    /// `OPERATOR_ORGANIZATION`.
    pub operator_organization: Option<String>,
    /// `OPERATOR_PHONE`.
    pub operator_phone: Option<String>,
    /// `OPERATOR_EMAIL`.
    pub operator_email: Option<String>,
    /// `EPHEMERIS_NAME` (e.g., `NONE` or ephemeris filename).
    pub ephemeris_name: String,
    /// `COVARIANCE_METHOD` (`CALCULATED` or `DEFAULT`).
    pub covariance_method: String,
    /// `MANEUVERABLE` (`YES`, `NO`, or `N/A`).
    pub maneuverable: String,
    /// Optional `ORBIT_CENTER`.
    pub orbit_center: Option<CenterName>,
    /// `REF_FRAME` (typically `EME2000`, `GCRF`, or `ITRF`).
    pub ref_frame: RefFrame,
    /// `GRAVITY_MODEL`.
    pub gravity_model: Option<String>,
    /// `ATMOSPHERIC_MODEL`.
    pub atmospheric_model: Option<String>,
    /// `N_BODY_PERTURBATIONS`.
    pub n_body_perturbations: Option<String>,
    /// `SOLAR_RAD_PRESSURE` (`YES`/`NO`).
    pub solar_rad_pressure: Option<YesNo>,
    /// `EARTH_TIDES` (`YES`/`NO`).
    pub earth_tides: Option<YesNo>,
    /// `INTRACK_THRUST` (`YES`/`NO`).
    pub intrack_thrust: Option<YesNo>,
}

/// Data section for one object in a [`Cdm`].
#[derive(Debug, Clone, PartialEq)]
pub struct CdmData {
    /// Comments at the start of the object data section.
    pub comments: Vec<String>,
    /// Optional orbit determination summary parameters (`<odParameters>`).
    pub od_parameters: Option<CdmOdParameters>,
    /// Optional physical/force-model additional parameters (`<additionalParameters>`).
    pub additional_parameters: Option<CdmAdditionalParameters>,
    /// Inertial or Earth-fixed Cartesian state vector at TCA (`<stateVector>`).
    pub state_vector: CdmStateVector,
    /// RTN 6x6 (plus optional 7x7..9x9 solve-for extensions) covariance matrix (`<covarianceMatrix>`).
    pub covariance: CdmCovariance,
}

/// Orbit determination parameters (`<odParameters>`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CdmOdParameters {
    /// Comments in the OD block.
    pub comments: Vec<String>,
    /// `TIME_LASTOB_START`.
    pub time_lastob_start: Option<Epoch>,
    /// `TIME_LASTOB_END`.
    pub time_lastob_end: Option<Epoch>,
    /// `RECOMMENDED_OD_SPAN` (`d`).
    pub recommended_od_span: Option<Quantity<Days>>,
    /// `ACTUAL_OD_SPAN` (`d`).
    pub actual_od_span: Option<Quantity<Days>>,
    /// `OBS_AVAILABLE`.
    pub obs_available: Option<u64>,
    /// `OBS_USED`.
    pub obs_used: Option<u64>,
    /// `TRACKS_AVAILABLE`.
    pub tracks_available: Option<u64>,
    /// `TRACKS_USED`.
    pub tracks_used: Option<u64>,
    /// `RESIDUALS_ACCEPTED` (`%`).
    pub residuals_accepted: Option<Quantity<Percent>>,
    /// `WEIGHTED_RMS`.
    pub weighted_rms: Option<f64>,
}

/// Object physical and perturbation parameters (`<additionalParameters>`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CdmAdditionalParameters {
    /// Comments in the additional-parameters block.
    pub comments: Vec<String>,
    /// Collision probability hard-body area (`AREA_PC`, `m**2`).
    pub area_pc: Option<Quantity<M2>>,
    /// Drag area (`AREA_DRG`, `m**2`).
    pub area_drg: Option<Quantity<M2>>,
    /// Solar radiation pressure area (`AREA_SRP`, `m**2`).
    pub area_srp: Option<Quantity<M2>>,
    /// Object mass (`MASS`, `kg`).
    pub mass: Option<Quantity<Kg>>,
    /// Ballistic drag parameter `Cd*A/m` (`CD_AREA_OVER_MASS`, `m**2/kg`).
    pub cd_area_over_mass: Option<Quantity<M2PerKg>>,
    /// Solar radiation pressure parameter `Cr*A/m` (`CR_AREA_OVER_MASS`, `m**2/kg`).
    pub cr_area_over_mass: Option<Quantity<M2PerKg>>,
    /// Thrust acceleration magnitude (`THRUST_ACCELERATION`, `m/s**2`).
    pub thrust_acceleration: Option<Quantity<MPerS2>>,
    /// Specific energy dissipation rate (`SEDR`, `W/kg`).
    pub sedr: Option<Quantity<WPerKg>>,
}

/// Object state vector at TCA (`<stateVector>`).
#[derive(Debug, Clone, PartialEq)]
pub struct CdmStateVector {
    /// Comments in the state vector block.
    pub comments: Vec<String>,
    /// `[X, Y, Z]` (`km`).
    pub position: Vector3<Km>,
    /// `[X_DOT, Y_DOT, Z_DOT]` (`km/s`).
    pub velocity: Vector3<KmPerS>,
}

impl Cdm {
    /// Parses a CDM from KVN text.
    pub fn from_kvn(text: &str, mode: ParseMode) -> Result<Self> {
        kvn::read(text, mode)
    }

    /// Parses a CDM from XML text.
    pub fn from_xml(text: &str, mode: ParseMode) -> Result<Self> {
        let root = xml_codec::tree::parse(text)?;
        Self::from_element(&root, mode)
    }

    pub(crate) fn from_element(root: &xml_codec::tree::Element, mode: ParseMode) -> Result<Self> {
        xml::read(root, mode)
    }

    /// Formats the CDM as KVN text.
    pub fn to_kvn(&self, options: WriteOptions) -> Result<String> {
        kvn::write(self, options)
    }

    /// Formats the CDM as a standalone XML document.
    pub fn to_xml(&self, options: WriteOptions) -> Result<String> {
        let mut out = xml_codec::writer::XmlWriter::document();
        self.write_element(&mut out, options, true)?;
        Ok(out.finish())
    }

    pub(crate) fn write_element(
        &self,
        out: &mut xml_codec::writer::XmlWriter,
        options: WriteOptions,
        standalone: bool,
    ) -> Result<()> {
        xml::write(self, out, options, standalone)
    }
}
