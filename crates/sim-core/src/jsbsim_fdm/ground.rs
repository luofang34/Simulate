//! Where the ground is under a JSBSim aircraft: the uniform terrain height its default
//! gear-contact callback uses, kept in step with the world surface.

use sim_geo::{OrthometricHeightM, msl_egm96_to_ellipsoidal};

use super::{BLOCK, JsbsimFdmBlock, METRES_PER_FOOT, fdm_error};
use crate::BlockError;

impl JsbsimFdmBlock {
    /// Sets the uniform ground height used by JSBSim's default gear-contact
    /// callback and re-captures the output at this instant.
    ///
    /// Despite its `asl` name, `position/terrain-elevation-asl-ft` is height
    /// above JSBSim's WGS84 ellipsoid, the same surface as
    /// [`sim_geo::Geodetic::height_m`], so `height_m` is ellipsoidal.
    pub fn set_terrain_elevation_m(&mut self, height_m: f64) -> Result<(), BlockError> {
        self.sync_ground_elevation_m(height_m)?;
        self.capture(0.0)
    }

    /// Moves the ground under the aircraft to `ellipsoidal_height_m` (WGS84)
    /// for the next step, for a caller tracking real terrain every step.
    ///
    /// Only the terrain property is written, through a node resolved at load,
    /// so the call does no path lookup and leaves the latest output and
    /// snapshot as the last step produced them.
    pub fn sync_ground_elevation_m(&mut self, ellipsoidal_height_m: f64) -> Result<(), BlockError> {
        if !ellipsoidal_height_m.is_finite() {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "terrain_elevation_m",
                detail: format!("{ellipsoidal_height_m} is not finite"),
            });
        }
        self.fdm
            .write_node(
                self.nodes.terrain_elevation_asl_ft,
                ellipsoidal_height_m / METRES_PER_FOOT,
            )
            .map_err(fdm_error)?;
        self.last_ground_sync_m = Some(ellipsoidal_height_m);
        Ok(())
    }

    /// Updates terrain elevation to `ellipsoidal_height_m` while preserving the landing gear's
    /// exact height above ground when `surface_version` changes beneath a parked or
    /// weight-on-wheels aircraft.
    pub fn sync_ground_versioned_m(
        &mut self,
        ellipsoidal_height_m: f64,
        surface_version: u64,
    ) -> Result<(), BlockError> {
        let version_changed = match self.latest.ground_version {
            Some(prev) => prev != surface_version,
            None => self
                .last_ground_sync_m
                .is_some_and(|prev| (ellipsoidal_height_m - prev).abs() > 0.05),
        };
        let wow = self.latest.articulation.gear_wow.into_iter().any(|w| w);
        let agl_ft = self.fdm.read_node(self.nodes.h_agl_ft).map_err(fdm_error)?;
        let parked_low =
            self.latest.output.velocity_ecef.length_squared() < 4.0 && (0.0..8.0).contains(&agl_ft);
        if version_changed && (wow || parked_low) {
            self.sync_ground_elevation_m(ellipsoidal_height_m)?;
            self.fdm
                .write_node(self.nodes.h_agl_ft, agl_ft.max(0.1))
                .map_err(fdm_error)?;
            self.latest.ground_version = Some(surface_version);
            return self.capture(0.0);
        }
        self.sync_ground_elevation_m(ellipsoidal_height_m)?;
        self.latest.ground_version = Some(surface_version);
        Ok(())
    }

    /// The ground height above the WGS84 ellipsoid this block was last given,
    /// at construction or since; `None` while the ground is still JSBSim's
    /// default at the ellipsoid.
    #[must_use]
    pub fn last_ground_sync_m(&self) -> Option<f64> {
        self.last_ground_sync_m
    }

    /// [`Self::sync_ground_elevation_m`] for ground charted `msl_m` above
    /// EGM96 mean sea level at a latitude and longitude, raised onto the
    /// ellipsoid by the local geoid undulation.
    pub fn sync_ground_msl_egm96_m(
        &mut self,
        lat_rad: f64,
        lon_rad: f64,
        msl_m: f64,
    ) -> Result<(), BlockError> {
        let ellipsoidal = msl_egm96_to_ellipsoidal(lat_rad, lon_rad, OrthometricHeightM(msl_m));
        self.sync_ground_elevation_m(ellipsoidal.0)
    }
}
