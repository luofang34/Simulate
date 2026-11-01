//! Deterministic `mbd-core` component specification and `sim_core::Block` for an
//! avionics Synthetic / Combined Vision (`SVS` / `XVS`) processor LRU.
//!
//! Fed strictly by operational belief kinematics (`ROLE_OPERATIONAL_ESTIMATE`
//! in `sim-aviate` / `Aviate`, never raw simulation truth when fused) so that
//! navigation estimation errors manifest faithfully on the conformal horizon,
//! runway outline, and TAWS procedural relief reference altitude.

use glam::{DMat3, DVec3};
use indicate_instrument_descriptor::{UnderlayContract, UnderlayStatus};
use maplibre::render::camera::EyeFrustum;
use mbd_core::{ComponentSpec, Placement};
use sim_core::{
    Block, BlockError, CameraMount, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState,
};

use crate::{ConformalFrameStamp, conformal_stamp_from_vehicle};

/// Default `60 Hz` execution period on the `46.08 GHz` master clock (`768,000,000` ticks).
pub const XVS_DEFAULT_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 60);

/// Canonical `mbd-core` [`ComponentSpec`] for an avionics XVS/SVS graphics LRU (`1.8 kg`, `28 W`).
#[must_use]
pub fn xvs_svs_component_spec(name: impl Into<String>, placement: Placement) -> ComponentSpec {
    // Uniform box approximation (0.20 m x 0.16 m x 0.06 m, 1.8 kg).
    let ix = (1.8 / 12.0) * (0.16_f64.powi(2) + 0.06_f64.powi(2));
    let iy = (1.8 / 12.0) * (0.20_f64.powi(2) + 0.06_f64.powi(2));
    let iz = (1.8 / 12.0) * (0.20_f64.powi(2) + 0.16_f64.powi(2));
    ComponentSpec {
        name: name.into(),
        placement,
        dry_mass_kg: 1.8,
        dry_inertia_body: DMat3::from_diagonal(DVec3::new(ix, iy, iz)),
        thermal_capacity_j_k: 1_620.0,
        nominal_power_w: 28.0,
    }
}

/// Static configuration of [`XvsSvsMbdBlock`].
#[derive(Clone, Copy, Debug)]
pub struct XvsSvsConfig {
    /// Execution period on the simulation master clock.
    pub period: TickSpan,
    /// Design eye / bore-sighted camera mount in vehicle body coordinates.
    pub mount: CameraMount,
    /// Conformal underlay camera frustum.
    pub frustum: EyeFrustum,
}

/// Step-wise belief inputs consumed by [`XvsSvsMbdBlock`].
#[derive(Clone, Copy, Debug)]
pub struct XvsSvsInput {
    /// Master-clock instant of this step.
    pub instant: SimInstant,
    /// Operational belief vehicle state (`ROLE_OPERATIONAL_ESTIMATE`, never raw truth).
    pub belief_vehicle: VehicleState,
    /// Current vehicle center-of-gravity location in body metres.
    pub cg_body_m: DVec3,
    /// Local geoid undulation (`h_wgs84 - h_msl`, metres).
    pub geoid_undulation_m: f64,
    /// Whether the onboard DEM database is valid and resident around ownship.
    pub belief_terrain_ready: bool,
    /// Whether calibrated EVS / forward video is available for [`UnderlayContract::CombinedXvs`].
    pub evs_video_available: bool,
    /// Whether the EVS camera bore-sight registration is calibrated.
    pub evs_calibrated: bool,
    /// Active belief surveillance track count.
    pub surveillance_track_count: u32,
    /// Active aeronautical notice / TFR count.
    pub active_notice_count: u32,
}

/// Deterministic conformal stamp and contract state emitted by [`XvsSvsMbdBlock`].
#[derive(Clone, Copy, Debug)]
pub struct XvsSvsOutput {
    /// Exact conformal camera stamp for `BeliefAvionicsMap::render_svs_frame` (`None` before step 1).
    pub stamp: Option<ConformalFrameStamp>,
    /// Ownship orthometric MSL altitude (metres) driving TAWS procedural DEM shading.
    pub taws_reference_altitude_m: f32,
    /// Resolved `Indicate` [`UnderlayContract`] (`CombinedXvs`, `ConformalSvs3D`, or `None`).
    pub active_contract: UnderlayContract,
    /// Resolved `Indicate` [`UnderlayStatus`] reported to PFD / HUD descriptors.
    pub status: UnderlayStatus,
    /// Active belief surveillance track count latched at this step.
    pub surveillance_track_count: u32,
    /// Active aeronautical notice count latched at this step.
    pub active_notice_count: u32,
    /// Monotonic number of completed steps.
    pub step_count: u64,
}

/// Exact serializable state of [`XvsSvsMbdBlock`] for deterministic save/restore and replay.
#[derive(Clone, Copy, Debug)]
pub struct XvsSvsSnapshot {
    output: XvsSvsOutput,
}

/// Deterministic `sim_core::Block` driving `Indicate` SVS / XVS conformal avionics underlays.
#[derive(Clone, Debug)]
pub struct XvsSvsMbdBlock {
    config: XvsSvsConfig,
    output: XvsSvsOutput,
}

impl XvsSvsMbdBlock {
    /// Creates an initialized XVS/SVS block prior to its first step.
    #[must_use]
    pub const fn new(config: XvsSvsConfig) -> Self {
        Self {
            config,
            output: XvsSvsOutput {
                stamp: None,
                taws_reference_altitude_m: 0.0,
                active_contract: UnderlayContract::None,
                status: UnderlayStatus::UNAVAILABLE,
                surveillance_track_count: 0,
                active_notice_count: 0,
                step_count: 0,
            },
        }
    }

    /// Block configuration.
    #[must_use]
    pub const fn config(&self) -> &XvsSvsConfig {
        &self.config
    }
}

impl Block for XvsSvsMbdBlock {
    type Input = XvsSvsInput;
    type Output = XvsSvsOutput;
    type Snapshot = XvsSvsSnapshot;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &Self::Input) -> Result<(), BlockError> {
        let stamp = conformal_stamp_from_vehicle(
            input.instant,
            &input.belief_vehicle,
            &self.config.mount,
            input.cg_body_m,
            self.config.frustum,
            input.geoid_undulation_m,
        )
        .map_err(|err| BlockError::InputOutOfRange {
            block: "xvs_svs_mbd",
            input: "belief_vehicle",
            detail: err.to_string(),
        })?;

        let (active_contract, status) = match (
            input.belief_terrain_ready,
            input.evs_video_available,
            input.evs_calibrated,
        ) {
            (true, true, true) => (
                UnderlayContract::CombinedXvs,
                UnderlayStatus {
                    available: true,
                    calibrated: true,
                    quality_pct: 100,
                },
            ),
            (true, _, _) => (
                UnderlayContract::ConformalSvs3D,
                UnderlayStatus {
                    available: true,
                    calibrated: true,
                    quality_pct: 95,
                },
            ),
            (false, true, calibrated) => (
                UnderlayContract::CalibratedVideo,
                UnderlayStatus {
                    available: true,
                    calibrated,
                    quality_pct: if calibrated { 85 } else { 50 },
                },
            ),
            (false, false, _) => (UnderlayContract::None, UnderlayStatus::UNAVAILABLE),
        };

        self.output = XvsSvsOutput {
            stamp: Some(stamp),
            taws_reference_altitude_m: stamp.anchor.altitude_msl_m as f32,
            active_contract,
            status,
            surveillance_track_count: input.surveillance_track_count,
            active_notice_count: input.active_notice_count,
            step_count: self.output.step_count.wrapping_add(1),
        };
        Ok(())
    }

    fn output(&self) -> Self::Output {
        self.output
    }

    fn snapshot(&self) -> Self::Snapshot {
        XvsSvsSnapshot {
            output: self.output,
        }
    }

    fn restore(&mut self, snapshot: &Self::Snapshot) {
        self.output = snapshot.output;
    }
}
