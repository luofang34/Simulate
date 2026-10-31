use mbd_core::{KinematicsFrame, LocalSiteKinematics};
use sim_core::BlockError;

mod landing;
mod rendezvous;

pub use landing::{
    LandingRadarAntenna, LandingRadarBlock, LandingRadarConfig, LandingRadarInput,
    LandingRadarOutput,
};
pub use rendezvous::{
    RelativeTrackInput, RendezvousRadarBlock, RendezvousRadarConfig, RendezvousRadarOutput,
    VhfRangingBlock, VhfRangingConfig, VhfRangingOutput,
};

#[cfg(test)]
mod tests;

/// Whether a radar's reading can be used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadarValidity {
    /// Locked on and inside its limits.
    Valid,
    /// Beyond its range or attitude limits; the data fields hold their last
    /// geometry and must not be used.
    OutOfLimits,
}

/// Rejects a site not in the central body's frame, or not finite.
fn require_fixed_site(block: &'static str, site: &LocalSiteKinematics) -> Result<(), BlockError> {
    if site.frame == KinematicsFrame::Ecef
        && site.position_m.is_finite()
        && site.velocity_m_s.is_finite()
        && site.attitude.is_finite()
    {
        return Ok(());
    }
    Err(BlockError::InputOutOfRange {
        block,
        input: "site",
        detail: format!("{site:?} is not a finite site in the body-fixed frame"),
    })
}
