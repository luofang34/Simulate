//! Power-lever plumbing shared by every flight-test solver, written so the same
//! test card flies a single fixed-pitch trainer and a turbocharged twin with
//! constant-speed propellers.

use jsbsim::Fdm;

use crate::PackError;
use crate::calibration::cruise::{fdm_err, get_opt};
use crate::calibration::solver::DT_S;

/// JSBSim allows at most this many engines per model in practice; the probe stops
/// at the first missing index well before it.
const MAX_ENGINES: usize = 8;

/// Number of engines the model declares, found by probing `propulsion/engine[n]`.
pub(super) fn engine_count(fdm: &Fdm) -> Result<usize, PackError> {
    let mut count = 0;
    while count < MAX_ENGINES {
        let probe = format!("propulsion/engine[{count}]/thrust-lbs");
        if fdm.node(&probe).map_err(fdm_err)?.is_none() {
            break;
        }
        count = count.wrapping_add(1);
    }
    Ok(count.max(1))
}

/// Writes `value` to `base[n]` for every engine. JSBSim's unindexed `fcs/*-norm`
/// names reach engine 0 only, so a twin flown through them climbs on one engine.
/// Properties the model does not bind (no propeller governor, say) are skipped.
pub(super) fn set_each_engine(fdm: &mut Fdm, base: &str, value: f64) -> Result<(), PackError> {
    for n in 0..engine_count(fdm)? {
        let prop = format!("{base}[{n}]");
        if fdm.node(&prop).map_err(fdm_err)?.is_some() {
            fdm.set_property(&prop, value).map_err(fdm_err)?;
        }
    }
    Ok(())
}

/// Whether engine 0 drives a constant-speed propeller, which decides whether a
/// target RPM is flown with the propeller lever or with the throttle. JSBSim sets
/// `constant-speed-mode` for its own governor; a model that governs in its flight
/// control system instead publishes `propulsion/engine[0]/governor-rpm-req`.
pub(super) fn has_governor(fdm: &mut Fdm) -> Result<bool, PackError> {
    let jsbsim_governor =
        get_opt(fdm, "propulsion/engine[0]/constant-speed-mode")?.is_some_and(|mode| mode != 0.0);
    let fcs_governor = fdm
        .node("propulsion/engine[0]/governor-rpm-req")
        .map_err(fdm_err)?
        .is_some();
    Ok(jsbsim_governor || fcs_governor)
}

/// Throttle loop holding engine 0's manifold pressure, the way a WWII test pilot
/// flew a power setting: boosted engines are reported by manifold pressure and RPM,
/// and full throttle would ride the regulator's takeoff clamp instead.
#[derive(Debug, Clone, Copy)]
pub(super) struct ManifoldPressureHold {
    integral: f64,
}

impl ManifoldPressureHold {
    /// Starts the loop at `throttle`.
    pub(super) const fn new(throttle: f64) -> Self {
        Self { integral: throttle }
    }

    /// Next throttle for a `target_inhg` hold. A model without a manifold-pressure
    /// output keeps the integrator's throttle.
    pub(super) fn step(&mut self, fdm: &mut Fdm, target_inhg: f64) -> Result<f64, PackError> {
        let Some(map) = get_opt(fdm, "propulsion/engine[0]/map-inhg")? else {
            return Ok(self.integral);
        };
        let err = target_inhg - map;
        self.integral = (self.integral + 0.02 * err * DT_S).clamp(0.05, 1.0);
        Ok((self.integral + 0.01 * err).clamp(0.05, 1.0))
    }
}

/// Propeller-lever loop holding engine 0's crankshaft RPM through the governor.
#[derive(Debug, Clone, Copy)]
pub(super) struct GovernorHold {
    integral: f64,
}

impl GovernorHold {
    /// Starts with the lever at full increase-RPM.
    pub(super) const fn new() -> Self {
        Self { integral: 1.0 }
    }

    /// Next propeller-lever position for a `target_rpm` (engine crankshaft) hold.
    pub(super) fn step(&mut self, fdm: &mut Fdm, target_rpm: f64) -> Result<f64, PackError> {
        let rpm = get_opt(fdm, "propulsion/engine[0]/engine-rpm")?.unwrap_or(target_rpm);
        let err = target_rpm - rpm;
        self.integral = (self.integral + 0.0004 * err * DT_S).clamp(0.0, 1.0);
        Ok((self.integral + 0.0005 * err).clamp(0.0, 1.0))
    }
}
