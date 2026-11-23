//! The flying vehicle's own camera views and chase distance.

use sim_core::{CameraPoseEcef, VirtualCameraSpec};
use sim_geo::BodyEllipsoid;
use sim_render::Camera;
use sim_scenario::OperatorViewportMode;

use super::Flight;
use super::display;
use super::mode::Mode;
use crate::camera_view::{
    DEFAULT_CHASE_BACK_UP_M, camera_from_virtual_spec_on, pilot_eye_mount_with_override,
    vehicle_camera_suite, viewport_camera_for,
};
use crate::error::AppSimError;

/// The chase camera's distance behind and above `Raketa`'s centre of gravity,
/// metres: far enough to hold the whole hull in view.
const SPACE_CHASE_BACK_UP_M: (f64, f64) = (45.0, 4.0);

impl Flight {
    /// The free-flight vehicle's own camera views from its package, none in a scenario.
    #[must_use]
    pub fn vehicle_cameras(&self) -> &[VirtualCameraSpec] {
        match &self.mode {
            Mode::Free { vehicle, .. } => vehicle
                .external_slot()
                .and_then(|slot| self.liveries.external_meta(slot))
                .map_or_else(
                    || vehicle_camera_suite(*vehicle).cameras.as_slice(),
                    |meta| meta.camera_suite.cameras.as_slice(),
                ),
            Mode::Scenario(_) | Mode::Space(_) => &[],
        }
    }

    /// How far behind and above the vehicle the chase camera rides, metres.
    pub(super) fn chase_back_up_m(&self) -> (f64, f64) {
        match &self.mode {
            Mode::Free { vehicle, .. } => vehicle
                .external_slot()
                .and_then(|slot| self.liveries.external_meta(slot))
                .map_or_else(
                    || vehicle_camera_suite(*vehicle).chase_back_up_m,
                    |meta| meta.camera_suite.chase_back_up_m,
                ),
            Mode::Scenario(_) => DEFAULT_CHASE_BACK_UP_M,
            Mode::Space(_) => SPACE_CHASE_BACK_UP_M,
        }
    }

    /// `spec` placed on the displayed vehicle over `body`, its body-mounted offsets
    /// measured from the drawn airframe's origin.
    #[must_use]
    pub fn virtual_camera(&self, body: BodyEllipsoid, spec: &VirtualCameraSpec) -> Option<Camera> {
        camera_from_virtual_spec_on(
            body,
            spec,
            &self.display_vehicle_state(),
            self.model_origin_from_cg_frd_m(),
        )
    }

    /// Aboard pilot eye pose in body-fixed ECEF coordinates, the eye mounted on the drawn
    /// airframe.
    pub fn aboard_eye_pose(&self) -> Result<CameraPoseEcef, AppSimError> {
        match &self.mode {
            Mode::Free { vehicle, world, .. } => {
                let ext_eye = vehicle
                    .external_slot()
                    .and_then(|slot| self.liveries.external_meta(slot))
                    .map(|m| m.pilot_eye_frd_m);
                Ok(pilot_eye_mount_with_override(*vehicle, ext_eye)
                    .pose(&world.truth_state(), -self.model_origin_from_cg_frd_m()))
            }
            Mode::Scenario(run) => Ok(run.eye_pose()?),
            Mode::Space(world) => Ok(world.eye_pose()),
        }
    }

    /// Camera positioned for `viewport_mode` on [`Self::display_vehicle_state`].
    pub fn display_camera(
        &self,
        viewport_mode: OperatorViewportMode,
    ) -> Result<Camera, AppSimError> {
        let exact = self.vehicle_state();
        let shown = self.display_vehicle_state();
        let aboard = display::reanchor(self.aboard_eye_pose()?, &exact, &shown);
        Ok(viewport_camera_for(
            viewport_mode,
            self.profile(),
            self.chase_back_up_m(),
            &shown,
            aboard,
        ))
    }
}
