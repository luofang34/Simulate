//! The flying vehicle's own camera views and chase distance.

use sim_core::{CameraPoseEcef, VirtualCameraSpec};
use sim_geo::BodyEllipsoid;
use sim_render::Camera;
use sim_scenario::OperatorViewportMode;

use super::Flight;
use super::display;
use super::mode::Mode;
use crate::camera_view::{
    DEFAULT_CHASE_BACK_UP_M, camera_from_virtual_spec_on, pilot_eye_mount_for,
    vehicle_camera_suite, viewport_camera_for,
};
use crate::error::AppSimError;

/// The chase camera's distance behind and above `Raketa`'s centre of gravity,
/// metres: far enough to hold the whole hull in view.
const SPACE_CHASE_BACK_UP_M: (f64, f64) = (45.0, 4.0);

impl Flight {
    /// The free-flight vehicle's own camera views from its package, none in a scenario.
    #[must_use]
    pub fn vehicle_cameras(&self) -> &'static [VirtualCameraSpec] {
        match &self.mode {
            Mode::Free { vehicle, .. } => &vehicle_camera_suite(*vehicle).cameras,
            Mode::Scenario(_) | Mode::Space(_) => &[],
        }
    }

    /// How far behind and above the vehicle the chase camera rides, metres.
    pub(super) fn chase_back_up_m(&self) -> (f64, f64) {
        match &self.mode {
            Mode::Free { vehicle, .. } => vehicle_camera_suite(*vehicle).chase_back_up_m,
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
            Mode::Free { vehicle, world, .. } => Ok(pilot_eye_mount_for(*vehicle)
                .pose(&world.truth_state(), -self.model_origin_from_cg_frd_m())),
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
