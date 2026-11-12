//! What one frame of every map is drawn from.

use sim_core::{AirframeArticulation, SensorExposureEvent, SimInstant, VehicleState};
use sim_render::Camera;
use sim_scenario::ScenarioProfile;

use crate::sky_rig::SkyFrame;
use crate::supervisor_rig::SupervisorRig;
use crate::vehicles::{FreeFlightVehicle, SteeredFix};

/// Inputs driving one frame of every map.
pub struct MapFrameInput<'a> {
    /// Window swapchain `[width, height]` in physical pixels.
    pub surface_size: [u32; 2],
    /// Current simulation clock instant.
    pub timestamp: SimInstant,
    /// Active scripted scenario profile (`None` in free flight).
    pub profile: Option<ScenarioProfile>,
    /// Free-flight plant flying now (`None` in a scripted scenario).
    pub active_vehicle: Option<FreeFlightVehicle>,
    /// Display-interpolated truth: the physical scene's ownship.
    pub vehicle: VehicleState,
    /// Body FRD metres from `vehicle`'s position (its CG) to its drawn model's origin.
    pub model_origin_from_cg_frd_m: glam::DVec3,
    /// Ellipsoidal ground height under ownship, which receives its sun shadow.
    pub ground_m: Option<f64>,
    /// Display-interpolated navigation solution of the vehicle's own instruments.
    pub estimated_vehicle: VehicleState,
    /// The steered navigator's fix on the real chain: the moving maps' ownship position.
    pub steered: Option<SteeredFix>,
    /// `30 Hz` optical sensor exposures since the previous frame, oldest first; the sensor
    /// slot renders the newest at its exact instant and truth state, whatever the window
    /// shows.
    pub sensor_exposures: &'a [SensorExposureEvent],
    /// Ownship control surfaces, propeller, gear, and lights.
    pub articulation: AirframeArticulation,
    /// Active window camera in `sim-render` convention.
    pub otw_camera: Camera,
    /// Sun, Moon, stars, and twilight for the scene and relief lighting.
    pub sky: &'a SkyFrame<'a>,
    /// Supervisor holding OSM airport diagrams, virtual camera presets, and surveillance belief.
    pub supervisor: &'a SupervisorRig,
}
