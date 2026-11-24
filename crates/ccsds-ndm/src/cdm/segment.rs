//! CDM segment metadata, OD parameters, and additional-parameters readers and writers.

use super::layout::parse_object;
use super::{CdmAdditionalParameters, CdmMetadata, CdmOdParameters};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::Result;
use crate::extension::Extension;
use crate::{ParseMode, WriteOptions};

pub(super) const METADATA_KEYWORDS: &[&str] = &[
    "OBJECT",
    "OBJECT_DESIGNATOR",
    "CATALOG_NAME",
    "OBJECT_NAME",
    "INTERNATIONAL_DESIGNATOR",
    "OBJECT_TYPE",
    "OPERATOR_CONTACT_POSITION",
    "OPERATOR_ORGANIZATION",
    "OPERATOR_PHONE",
    "OPERATOR_EMAIL",
    "EPHEMERIS_NAME",
    "COVARIANCE_METHOD",
    "MANEUVERABLE",
    "ORBIT_CENTER",
    "REF_FRAME",
    "GRAVITY_MODEL",
    "ATMOSPHERIC_MODEL",
    "N_BODY_PERTURBATIONS",
    "SOLAR_RAD_PRESSURE",
    "EARTH_TIDES",
    "INTRACK_THRUST",
];

pub(super) const OD_KEYWORDS: &[&str] = &[
    "TIME_LASTOB_START",
    "TIME_LASTOB_END",
    "RECOMMENDED_OD_SPAN",
    "ACTUAL_OD_SPAN",
    "OBS_AVAILABLE",
    "OBS_USED",
    "TRACKS_AVAILABLE",
    "TRACKS_USED",
    "RESIDUALS_ACCEPTED",
    "WEIGHTED_RMS",
];

pub(super) const ADDITIONAL_KEYWORDS: &[&str] = &[
    "AREA_PC",
    "AREA_DRG",
    "AREA_SRP",
    "MASS",
    "CD_AREA_OVER_MASS",
    "CR_AREA_OVER_MASS",
    "THRUST_ACCELERATION",
    "SEDR",
];

pub(super) fn read_metadata(
    block: Block,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<CdmMetadata> {
    let (comments, mut fields) = Fields::new(block, mode)?;
    let meta = CdmMetadata {
        comments,
        object: fields.required("OBJECT", parse_object)?,
        object_designator: fields.required("OBJECT_DESIGNATOR", Entry::string)?,
        catalog_name: fields.required("CATALOG_NAME", Entry::string)?,
        object_name: fields.required("OBJECT_NAME", Entry::string)?,
        international_designator: fields.required("INTERNATIONAL_DESIGNATOR", Entry::string)?,
        object_type: fields.optional("OBJECT_TYPE", Entry::string)?,
        operator_contact_position: fields.optional("OPERATOR_CONTACT_POSITION", Entry::string)?,
        operator_organization: fields.optional("OPERATOR_ORGANIZATION", Entry::string)?,
        operator_phone: fields.optional("OPERATOR_PHONE", Entry::string)?,
        operator_email: fields.optional("OPERATOR_EMAIL", Entry::string)?,
        ephemeris_name: fields.required("EPHEMERIS_NAME", Entry::string)?,
        covariance_method: fields.required("COVARIANCE_METHOD", Entry::string)?,
        maneuverable: fields.required("MANEUVERABLE", Entry::string)?,
        orbit_center: fields.optional("ORBIT_CENTER", Entry::center)?,
        ref_frame: fields.required("REF_FRAME", Entry::frame)?,
        gravity_model: fields.optional("GRAVITY_MODEL", Entry::string)?,
        atmospheric_model: fields.optional("ATMOSPHERIC_MODEL", Entry::string)?,
        n_body_perturbations: fields.optional("N_BODY_PERTURBATIONS", Entry::string)?,
        solar_rad_pressure: fields.optional("SOLAR_RAD_PRESSURE", Entry::yes_no)?,
        earth_tides: fields.optional("EARTH_TIDES", Entry::yes_no)?,
        intrack_thrust: fields.optional("INTRACK_THRUST", Entry::yes_no)?,
    };
    fields.finish(extensions)?;
    Ok(meta)
}

pub(super) fn write_metadata(meta: &CdmMetadata, options: WriteOptions) -> Block {
    let mut out = BlockWriter::new(&meta.comments, options);
    out.text("OBJECT", meta.object);
    out.text("OBJECT_DESIGNATOR", &meta.object_designator);
    out.text("CATALOG_NAME", &meta.catalog_name);
    out.text("OBJECT_NAME", &meta.object_name);
    out.text("INTERNATIONAL_DESIGNATOR", &meta.international_designator);
    out.text_opt("OBJECT_TYPE", meta.object_type.as_ref());
    out.text_opt(
        "OPERATOR_CONTACT_POSITION",
        meta.operator_contact_position.as_ref(),
    );
    out.text_opt("OPERATOR_ORGANIZATION", meta.operator_organization.as_ref());
    out.text_opt("OPERATOR_PHONE", meta.operator_phone.as_ref());
    out.text_opt("OPERATOR_EMAIL", meta.operator_email.as_ref());
    out.text("EPHEMERIS_NAME", &meta.ephemeris_name);
    out.text("COVARIANCE_METHOD", &meta.covariance_method);
    out.text("MANEUVERABLE", &meta.maneuverable);
    out.text_opt("ORBIT_CENTER", meta.orbit_center.as_ref());
    out.text("REF_FRAME", &meta.ref_frame);
    out.text_opt("GRAVITY_MODEL", meta.gravity_model.as_ref());
    out.text_opt("ATMOSPHERIC_MODEL", meta.atmospheric_model.as_ref());
    out.text_opt("N_BODY_PERTURBATIONS", meta.n_body_perturbations.as_ref());
    out.text_opt("SOLAR_RAD_PRESSURE", meta.solar_rad_pressure);
    out.text_opt("EARTH_TIDES", meta.earth_tides);
    out.text_opt("INTRACK_THRUST", meta.intrack_thrust);
    out.finish()
}

pub(super) fn read_od(comments: Vec<String>, fields: &mut Fields) -> Result<CdmOdParameters> {
    Ok(CdmOdParameters {
        comments,
        time_lastob_start: fields.optional("TIME_LASTOB_START", Entry::epoch)?,
        time_lastob_end: fields.optional("TIME_LASTOB_END", Entry::epoch)?,
        recommended_od_span: fields.optional("RECOMMENDED_OD_SPAN", Entry::quantity)?,
        actual_od_span: fields.optional("ACTUAL_OD_SPAN", Entry::quantity)?,
        obs_available: fields.optional("OBS_AVAILABLE", parse_u64)?,
        obs_used: fields.optional("OBS_USED", parse_u64)?,
        tracks_available: fields.optional("TRACKS_AVAILABLE", parse_u64)?,
        tracks_used: fields.optional("TRACKS_USED", parse_u64)?,
        residuals_accepted: fields.optional("RESIDUALS_ACCEPTED", Entry::quantity)?,
        weighted_rms: fields.optional("WEIGHTED_RMS", Entry::real)?,
    })
}

pub(super) fn write_od(od: &CdmOdParameters, options: WriteOptions) -> Result<Block> {
    let mut out = BlockWriter::new(&od.comments, options);
    out.text_opt("TIME_LASTOB_START", od.time_lastob_start.as_ref());
    out.text_opt("TIME_LASTOB_END", od.time_lastob_end.as_ref());
    out.quantity_opt("RECOMMENDED_OD_SPAN", od.recommended_od_span)?;
    out.quantity_opt("ACTUAL_OD_SPAN", od.actual_od_span)?;
    out.text_opt("OBS_AVAILABLE", od.obs_available);
    out.text_opt("OBS_USED", od.obs_used);
    out.text_opt("TRACKS_AVAILABLE", od.tracks_available);
    out.text_opt("TRACKS_USED", od.tracks_used);
    out.quantity_opt("RESIDUALS_ACCEPTED", od.residuals_accepted)?;
    out.real_opt("WEIGHTED_RMS", od.weighted_rms)?;
    Ok(out.finish())
}

pub(super) fn read_additional(
    comments: Vec<String>,
    fields: &mut Fields,
) -> Result<CdmAdditionalParameters> {
    Ok(CdmAdditionalParameters {
        comments,
        area_pc: fields.optional("AREA_PC", Entry::quantity)?,
        area_drg: fields.optional("AREA_DRG", Entry::quantity)?,
        area_srp: fields.optional("AREA_SRP", Entry::quantity)?,
        mass: fields.optional("MASS", Entry::quantity)?,
        cd_area_over_mass: fields.optional("CD_AREA_OVER_MASS", Entry::quantity)?,
        cr_area_over_mass: fields.optional("CR_AREA_OVER_MASS", Entry::quantity)?,
        thrust_acceleration: fields.optional("THRUST_ACCELERATION", Entry::quantity)?,
        sedr: fields.optional("SEDR", Entry::quantity)?,
    })
}

pub(super) fn write_additional(
    add: &CdmAdditionalParameters,
    options: WriteOptions,
) -> Result<Block> {
    let mut out = BlockWriter::new(&add.comments, options);
    out.quantity_opt("AREA_PC", add.area_pc)?;
    out.quantity_opt("AREA_DRG", add.area_drg)?;
    out.quantity_opt("AREA_SRP", add.area_srp)?;
    out.quantity_opt("MASS", add.mass)?;
    out.quantity_opt("CD_AREA_OVER_MASS", add.cd_area_over_mass)?;
    out.quantity_opt("CR_AREA_OVER_MASS", add.cr_area_over_mass)?;
    out.quantity_opt("THRUST_ACCELERATION", add.thrust_acceleration)?;
    out.quantity_opt("SEDR", add.sedr)?;
    Ok(out.finish())
}

fn parse_u64(entry: &Entry, _: ParseMode) -> Result<u64> {
    entry.value.trim().parse().map_err(|_| {
        entry.error(crate::error::ErrorKind::InvalidValue {
            text: entry.value.clone(),
            expected: "a non-negative integer",
        })
    })
}
