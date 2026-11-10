//! When a vessel changes fidelity tier by itself.
//!
//! Flight dynamics are needed where air, ground or another vessel acts on
//! a vehicle, and orbital propagation is needed everywhere else. A
//! [`TierPolicy`] draws that line by height above each body, with a gap
//! between the two thresholds so a vessel skimming one does not flip tiers.

use std::collections::BTreeMap;

use sim_atmosphere::{AtmosphereField, MarsAtmosphere, ReentryAtmosphere};
use sim_core::{SimInstant, TickSpan};
use sim_geo::{Geodetic, UtcTime};
use sim_orbital::{NaifId, VesselId};

use crate::universe::Tier;

/// Default maximum dynamic pressure (`Pa`) at which a local vessel may be promoted to orbital propagation.
pub const DEFAULT_MAX_PROMOTE_DYNAMIC_PRESSURE_PA: f64 = 0.1;

/// Default dynamic pressure (`Pa`) above which an orbital vessel is demoted back to local flight dynamics.
pub const DEFAULT_DEMOTE_ABOVE_DYNAMIC_PRESSURE_PA: f64 = 0.5;

/// The heights about one body between which a vessel keeps the tier it has.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltitudeBand {
    /// A local vessel above this height on the reference ellipsoid, m, is
    /// handed to orbital propagation.
    pub promote_above_m: f64,
    /// An orbital vessel below this height, m, is handed to flight dynamics.
    /// Lower than [`Self::promote_above_m`].
    pub demote_below_m: f64,
}

/// Which tier a vessel belongs in, checked on a fixed grid of the master
/// clock so a change of tier never depends on how a frame loop slices time.
#[derive(Debug, Clone, PartialEq)]
pub struct TierPolicy {
    /// Tier changes happen only at whole multiples of this span.
    pub check_period: TickSpan,
    /// The band about each body. A vessel about a body with no band keeps
    /// its tier.
    pub bands: BTreeMap<NaifId, AltitudeBand>,
    /// Maximum dynamic pressure, Pa, at which a local vessel may be promoted
    /// to orbital propagation.
    pub max_promote_dynamic_pressure_pa: f64,
    /// Dynamic pressure, Pa, above which an orbital vessel is demoted to
    /// local flight dynamics even if above `demote_below_m`.
    pub demote_above_dynamic_pressure_pa: f64,
}

impl TierPolicy {
    /// Checks once a second, with bands that clear what flight dynamics
    /// must model: Earth's and Mars's sensible atmosphere, and the Moon's
    /// highest terrain, plus dynamic-pressure and ground-contact gates.
    #[must_use]
    pub fn standard() -> Self {
        let band = |promote_above_m, demote_below_m| AltitudeBand {
            promote_above_m,
            demote_below_m,
        };
        Self {
            check_period: TickSpan::SECOND,
            bands: BTreeMap::from([
                (NaifId::EARTH, band(160_000.0, 140_000.0)),
                (NaifId::MOON, band(20_000.0, 15_000.0)),
                (NaifId::MARS, band(130_000.0, 110_000.0)),
            ]),
            max_promote_dynamic_pressure_pa: DEFAULT_MAX_PROMOTE_DYNAMIC_PRESSURE_PA,
            demote_above_dynamic_pressure_pa: DEFAULT_DEMOTE_ABOVE_DYNAMIC_PRESSURE_PA,
        }
    }

    /// The first check instant after `now`, if it is no later than `until`.
    pub(crate) fn next_check(&self, now: SimInstant, until: SimInstant) -> Option<SimInstant> {
        let period = self.check_period.ticks_u128();
        if period == 0 {
            return None;
        }
        let next_ticks = (now.ticks_u128() / period)
            .wrapping_add(1)
            .saturating_mul(period);
        let next = SimInstant::START.advanced(TickSpan::from_ticks_u128(next_ticks));
        (next <= until).then_some(next)
    }

    /// The tier a vessel now in `tier` should move to, at `height_m` above
    /// `body` with `dynamic_pressure_pa` and `ground_contact`, or `None` to stay.
    #[must_use]
    pub fn wanted(
        &self,
        tier: Tier,
        body: NaifId,
        height_m: f64,
        dynamic_pressure_pa: f64,
        ground_contact: bool,
    ) -> Option<Tier> {
        let band = self.bands.get(&body)?;
        match tier {
            Tier::Local
                if !ground_contact
                    && dynamic_pressure_pa <= self.max_promote_dynamic_pressure_pa
                    && height_m > band.promote_above_m =>
            {
                Some(Tier::Orbital)
            }
            Tier::Orbital
                if ground_contact
                    || dynamic_pressure_pa > self.demote_above_dynamic_pressure_pa
                    || height_m < band.demote_below_m =>
            {
                Some(Tier::Local)
            }
            Tier::Local | Tier::Orbital => None,
        }
    }
}

/// Evaluates standard planetary atmospheric dynamic pressure $q = \frac{1}{2} \rho(h) v^2$ (`Pa`)
/// for `body` at ellipsoidal `height_m` and body-relative speed `speed_ecef_m_s`.
#[must_use]
pub fn planetary_dynamic_pressure_pa(body: NaifId, height_m: f64, speed_ecef_m_s: f64) -> f64 {
    let pos = Geodetic::from_degrees(0.0, 0.0, height_m);
    let utc = UtcTime::from_unix_seconds(0.0);
    let rho = if body == NaifId::EARTH {
        ReentryAtmosphere
            .sample(&pos, utc)
            .map_or(0.0, |s| s.density_kg_m3)
    } else if body == NaifId::MARS {
        MarsAtmosphere::new()
            .sample(&pos, utc)
            .map_or(0.0, |s| s.density_kg_m3)
    } else {
        0.0
    };
    0.5 * rho * speed_ecef_m_s * speed_ecef_m_s
}

/// One vessel's change of tier, made by a [`TierPolicy`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TierChange {
    /// The vessel.
    pub vessel: VesselId,
    /// The instant it changed.
    pub at: SimInstant,
    /// The tier it is in from that instant.
    pub to: Tier,
}
