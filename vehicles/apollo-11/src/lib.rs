//! Apollo 11 as a self-contained vehicle package: the Eagle and Columbia
//! manifests under `packages/`, loaded through `sim-pack` like any downloaded
//! vehicle, and the historical sensors unique to them, defects included, so
//! they stay out of the shared device library.
//!
//! [`CockpitInstrumentPanel`] gathers what the crew reads, Earth vehicles
//! included, with each reading's source health.
//!
//! Every sensor is a [`sim_core::Block`] fed from a
//! [`sim_core::RigidBody6DofBlock`] through [`InertialSiteSample`] or
//! [`site_in_fixed_frame`], and the optics are worked through a
//! [`sim_core::CrewStationPort`], so a person, an autopilot and a coding agent
//! fly the same vehicle.

mod aids;
mod asa_ags;
mod cockpit;
mod handover;
mod imu_pgncs;
mod optics;
mod radars;
mod rate_gyros;
mod site;
mod vehicle;

pub use aids::{
    CONTACT_LIGHT_LAMP, CorridorIndication, EntryMonitorBlock, EntryMonitorConfig,
    EntryMonitorOutput, EntryMonitorSnapshot, LpdProjector, LpdReading, LunarContactInput,
    LunarContactOutput, LunarContactProbeBlock,
};
pub use asa_ags::{AsaOutput, StrapdownAsaBlock, StrapdownAsaConfig};
pub use cockpit::{
    CockpitInstrumentPanel, CockpitInstrumentSnapshot, CommandModuleSources, EarthVehicleSources,
    InstrumentChange, LunarModuleSources, RadarSample,
};
pub use handover::{GuidanceHandover, GuidanceSource};
pub use imu_pgncs::{
    GIMBAL_LOCK_CAPTURE_DEG, GIMBAL_LOCK_LAMP, GIMBAL_LOCK_WARNING_DEG, GimbalAngles, ImuOutput,
    ImuSnapshot, PIPA_PULSE_CM_M_S, PIPA_PULSE_LM_M_S, ThreeGimbalImuBlock, ThreeGimbalImuConfig,
};
pub use optics::{
    AOT_DETENT_PANEL, AOT_DETENTS, AlignmentError, ImuRealignment, OpticalSightBlock,
    OpticalSightConfig, OpticalSightInput, OpticalSightOutput, OpticalSightSnapshot, SightKind,
    SightMark, StarSighting, realign_imu_from_two_star_marks,
};
pub use radars::{
    LandingRadarAntenna, LandingRadarBlock, LandingRadarConfig, LandingRadarInput,
    LandingRadarOutput, RadarValidity, RelativeTrackInput, RendezvousRadarBlock,
    RendezvousRadarConfig, RendezvousRadarOutput, VhfRangingBlock, VhfRangingConfig,
    VhfRangingOutput,
};
pub use rate_gyros::{RateGyroAssemblyBlock, RateGyroConfig, RateGyroOutput};
pub use site::{InertialSiteSample, SurfacePlane, site_in_fixed_frame};
pub use vehicle::*;
