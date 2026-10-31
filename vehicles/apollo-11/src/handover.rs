use glam::DQuat;

use crate::{AsaOutput, ImuOutput};

#[cfg(test)]
mod tests;

/// Which guidance system's attitude the vehicle is flying on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidanceSource {
    /// Primary Guidance, Navigation and Control System.
    Pgns,
    /// Abort Guidance System.
    Ags,
}

/// The PGNS-to-AGS selection: the crew's guidance-control switch, with an
/// automatic switch to AGS when the platform is captured.
///
/// The automatic transfer latches, so a realigned platform only comes back
/// into the loop when the crew selects it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidanceHandover {
    active: GuidanceSource,
}

impl Default for GuidanceHandover {
    fn default() -> Self {
        Self {
            active: GuidanceSource::Pgns,
        }
    }
}

impl GuidanceHandover {
    /// The system in control.
    pub fn active(&self) -> GuidanceSource {
        self.active
    }

    /// The crew moves the switch to `source`.
    pub fn crew_select(&mut self, source: GuidanceSource) {
        self.active = source;
    }

    /// Fails over if the platform has no reference, then reads the active
    /// system's attitude, navigation-base-to-J2000.
    pub fn attitude(&mut self, imu: &ImuOutput, asa: &AsaOutput) -> DQuat {
        if !imu.platform_valid {
            self.active = GuidanceSource::Ags;
        }
        match self.active {
            GuidanceSource::Pgns => imu.indicated_navbase_to_inertial,
            GuidanceSource::Ags => asa.site_to_inertial,
        }
    }
}
