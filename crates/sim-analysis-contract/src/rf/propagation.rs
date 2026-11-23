//! Serializable extra propagation-loss models.

use serde::{Deserialize, Serialize};

/// An extra propagation loss on top of free-space spreading.
///
/// Models that need environment data (an ionosphere from a TEC field, rain from a climate
/// grid) implement `sim-analysis`'s `PropagationLoss` instead of becoming variants here.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum PropagationModelSpec {
    /// Cosecant-law gaseous attenuation $A(\theta) = A_z/\sin\theta$ at every surface end of
    /// the link (ITU-R P.676 Annex 2 slant-path scaling; valid for $\theta \ge 5^\circ$).
    TroposphericCosecant {
        /// Zenith attenuation $A_z$ at the carrier (`dB`, `>= 0`).
        zenith_attenuation_db: f64,
        /// Lowest elevation the model is applied at (`rad`, `> 0`); below it the budget is withheld.
        min_elevation_rad: f64,
    },
}
