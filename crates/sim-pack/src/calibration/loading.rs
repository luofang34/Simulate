//! Loading a calibration point to its card mass and centre of gravity: the
//! payload goes on the model's point-mass stations, split between the two that
//! bracket the arm the card's CG needs.

use jsbsim::Fdm;

use crate::PackError;
use crate::calibration::cruise::{fdm_err, get_opt, set_if_exists};
use crate::calibration::solver::{POUNDS_PER_KG, ParameterOverrides};

#[cfg(test)]
mod tests;

/// The most point-mass stations probed.
const MAX_STATIONS: usize = 16;

/// The card's CG for one point, in the handbook's fuselage stations.
#[derive(Debug, Clone, Copy)]
pub(super) struct CgTarget<'a> {
    /// The point's label, for the error when the CG cannot be reached.
    pub label: &'a str,
    /// Target CG, fuselage station (`in` aft of the handbook datum).
    pub fs_in: f64,
    /// Structural x of fuselage station 0 (`in`).
    pub origin_in: f64,
}

/// The CG target for a point that gives `cg_fs_in`, refused when the suite
/// gives no fuselage-station origin to place it with.
pub(super) fn cg_target<'a>(
    label: &'a str,
    cg_fs_in: Option<f64>,
    origin_in: Option<f64>,
) -> Result<Option<CgTarget<'a>>, PackError> {
    match (cg_fs_in, origin_in) {
        (None, _) => Ok(None),
        (Some(fs_in), Some(origin_in)) => Ok(Some(CgTarget {
            label,
            fs_in,
            origin_in,
        })),
        (Some(_), None) => Err(PackError::InvalidCalibration {
            path: crate::CALIBRATION_SUITE_ASSET.to_owned(),
            detail: format!("{label}: a CG needs fuselage_station_origin_in"),
        }),
    }
}

/// Sets the point-mass payload for `gross_mass_kg`. Without a `cg`, the first
/// station carries what the others do not; with one, every station is emptied
/// and the payload split between the two that bracket the arm the target
/// needs. Returns the CG the model
/// then flies at, as a fuselage station when `cg` is given.
pub(super) fn load_point(
    fdm: &mut Fdm,
    gross_mass_kg: f64,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
    cg: Option<CgTarget<'_>>,
) -> Result<Option<f64>, PackError> {
    let target_lbs = gross_mass_kg * POUNDS_PER_KG + overrides.extra_mass_lbs;
    let Some(cg) = cg else {
        let payload_lbs = (target_lbs - base_non_payload_lbs).max(0.0);
        set_if_exists(fdm, "inertia/pointmass-weight-lbs[0]", payload_lbs)?;
        return Ok(None);
    };
    let stations = stations(fdm)?;
    for (i, _) in stations.iter().enumerate() {
        set_if_exists(fdm, &format!("inertia/pointmass-weight-lbs[{i}]"), 0.0)?;
    }
    // With every station emptied, the payload is all that the target mass
    // adds, whatever the stations carried by default.
    let (base_lbs, base_x_in) = mass_and_cg(fdm)?;
    let payload_lbs = target_lbs - base_lbs;
    let target_x_in = cg.fs_in + cg.origin_in;
    let lowest = stations.iter().copied().fold(f64::INFINITY, f64::min);
    let highest = stations.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let unreachable = || PackError::UnreachableCg {
        label: cg.label.to_owned(),
        target_fs_in: cg.fs_in,
        lowest_fs_in: lowest - cg.origin_in,
        highest_fs_in: highest - cg.origin_in,
    };
    if payload_lbs <= 0.0 {
        return Err(unreachable());
    }
    let arm_in = (target_lbs * target_x_in - base_lbs * base_x_in) / payload_lbs;
    let below = stations
        .iter()
        .copied()
        .enumerate()
        .filter(|s| s.1 <= arm_in);
    let above = stations
        .iter()
        .copied()
        .enumerate()
        .filter(|s| s.1 >= arm_in);
    let lo = below
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .ok_or_else(unreachable)?;
    let hi = above
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .ok_or_else(unreachable)?;
    let aft_share = if hi.1 > lo.1 {
        (arm_in - lo.1) / (hi.1 - lo.1)
    } else {
        1.0
    };
    set_if_exists(
        fdm,
        &format!("inertia/pointmass-weight-lbs[{}]", lo.0),
        payload_lbs * (1.0 - aft_share),
    )?;
    let aft_lbs = if hi.0 == lo.0 {
        payload_lbs
    } else {
        payload_lbs * aft_share
    };
    set_if_exists(
        fdm,
        &format!("inertia/pointmass-weight-lbs[{}]", hi.0),
        aft_lbs,
    )?;
    let (_, flown_x_in) = mass_and_cg(fdm)?;
    Ok(Some(flown_x_in - cg.origin_in))
}

/// Structural x of every point-mass station (`in`).
fn stations(fdm: &mut Fdm) -> Result<Vec<f64>, PackError> {
    let mut xs = Vec::new();
    for i in 0..MAX_STATIONS {
        match get_opt(fdm, &format!("inertia/pointmass-location-X-inches[{i}]"))? {
            Some(x) => xs.push(x),
            None => break,
        }
    }
    Ok(xs)
}

/// Total weight (`lbs`) and CG x (`in`) after the mass model updates.
fn mass_and_cg(fdm: &mut Fdm) -> Result<(f64, f64), PackError> {
    fdm.run_ic().map_err(fdm_err)?;
    Ok((
        fdm.get_property("inertia/weight-lbs").map_err(fdm_err)?,
        fdm.get_property("inertia/cg-x-in").map_err(fdm_err)?,
    ))
}
