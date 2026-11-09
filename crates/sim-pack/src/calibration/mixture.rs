//! Mixture setting by the pilot's procedures rather than by fixed equivalence
//! ratios: a fixed ratio is only "best power" or "50°F rich of peak" for the one
//! mixture-efficiency curve it was tuned against, so an engine with its own
//! curve would be flown at the wrong point.

use jsbsim::Fdm;

use crate::PackError;
use crate::calibration::cruise::{get_opt, step_wings_level};

/// Leanest lever position the sweeps try.
const LEANEST: f64 = 0.20;
/// Lever travel between sweep samples.
const SWEEP_STEP: f64 = 0.01;
/// Frames flown at each sample so the engine model recomputes power and EGT.
const FRAMES_PER_SAMPLE: u32 = 2;
/// The sweep stops once power falls this far below its best, so it never leans
/// an engine to a stop it cannot restart from.
const STOP_POWER_FRACTION: f64 = 0.85;

/// How the mixture lever is set for a test point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum MixtureProcedure {
    /// Lever full forward.
    FullRich,
    /// Leaned for maximum power, which a fixed-pitch pilot reads as maximum RPM.
    BestPower,
    /// Leaned to peak exhaust-gas temperature, then enriched by this many °F.
    RichOfPeakEgt(f64),
}

impl MixtureProcedure {
    /// The POH climb rule: full rich at or below 3,000 ft unless the card says
    /// leaned, leaned for maximum RPM above it.
    pub(super) fn climb(pressure_alt_ft: f64, leaned: bool) -> Self {
        if !leaned && pressure_alt_ft <= 3_000.0 {
            Self::FullRich
        } else {
            Self::BestPower
        }
    }
}

/// One sweep sample: lever position, engine power and exhaust-gas temperature.
#[derive(Debug, Clone, Copy)]
struct Sample {
    lever: f64,
    power_hp: f64,
    egt_degf: f64,
}

/// Flies the lever sweep `procedure` calls for at the current condition and
/// returns the lever position it settles on. Models whose flight control system
/// owns the mixture (an automatic mixture control) see no change across the sweep
/// and keep full rich.
pub(super) fn set_mixture(
    fdm: &mut Fdm,
    procedure: MixtureProcedure,
    throttle: f64,
) -> Result<f64, PackError> {
    if procedure == MixtureProcedure::FullRich {
        return Ok(1.0);
    }
    let samples = sweep(fdm, throttle)?;
    let lever = match procedure {
        MixtureProcedure::FullRich => 1.0,
        MixtureProcedure::BestPower => best_power(&samples),
        MixtureProcedure::RichOfPeakEgt(offset) => rich_of_peak(&samples, offset),
    };
    step_wings_level(fdm, 0.0, throttle, lever)?;
    Ok(lever)
}

fn sweep(fdm: &mut Fdm, throttle: f64) -> Result<Vec<Sample>, PackError> {
    let mut samples = Vec::new();
    let mut best_hp = 0.0_f64;
    let mut lever = 1.0_f64;
    while lever >= LEANEST {
        for _ in 0..FRAMES_PER_SAMPLE {
            step_wings_level(fdm, 0.0, throttle, lever)?;
        }
        let power_hp = get_opt(fdm, "propulsion/engine[0]/power-hp")?.unwrap_or(0.0);
        let egt_degf = get_opt(fdm, "propulsion/engine[0]/egt-degF")?.unwrap_or(0.0);
        samples.push(Sample {
            lever,
            power_hp,
            egt_degf,
        });
        best_hp = best_hp.max(power_hp);
        if power_hp < STOP_POWER_FRACTION * best_hp {
            break;
        }
        lever -= SWEEP_STEP;
    }
    Ok(samples)
}

/// The richest lever position reaching the highest power, so a flat-topped
/// curve is not leaned further than it gains from.
fn best_power(samples: &[Sample]) -> f64 {
    let mut best = Sample {
        lever: 1.0,
        power_hp: f64::MIN,
        egt_degf: 0.0,
    };
    for s in samples {
        if s.power_hp > best.power_hp + 1e-6 {
            best = *s;
        }
    }
    best.lever
}

/// The leanest lever position on the rich side of peak EGT whose EGT is still at
/// least `offset_degf` below the peak; the richest of equal peaks counts, so an
/// engine whose EGT never moves stays full rich.
fn rich_of_peak(samples: &[Sample], offset_degf: f64) -> f64 {
    let Some(peak) = samples.iter().copied().max_by(|a, b| {
        a.egt_degf
            .total_cmp(&b.egt_degf)
            .then(a.lever.total_cmp(&b.lever))
    }) else {
        return 1.0;
    };
    samples
        .iter()
        .filter(|s| s.lever > peak.lever && s.egt_degf <= peak.egt_degf - offset_degf)
        .map(|s| s.lever)
        .fold(None, |acc: Option<f64>, l| {
            Some(acc.map_or(l, |a| a.min(l)))
        })
        .unwrap_or(peak.lever)
}

#[cfg(test)]
mod tests;
