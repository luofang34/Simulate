//! `Raketa` single-stage interplanetary test rocket (`vehicles/raketa`):
//! - SHA-256 & OpenUSD schema-verified `retro_rocket_v4.usd` (`upAxis = Y`, `metersPerUnit = 1.0`),
//! - Exact quadratic feasible-interval `4.0G` proper-acceleration limiter (`a_eff = clamp(-b, 0, a_cmd)` when infeasible),
//! - Full `[0°, 180°]` aerodynamic envelope with `< 50 m` / pad fin ground-lock (`±18°` -> `0°`),
//! - Reserved 3-axis CG translational RCS (`±0.25G`), momentum-conserving reaction wheels ($5\times 10^6\text{ N}\cdot\text{m}\cdot\text{s}$),
//! - Continuous pad-to-surface execution (`run_uninterrupted_mission`, one propagated trajectory to rest
//!   on the Moon or Mars) and explicitly tagged staged phase demonstrations (`run_staged_phase_demonstration`).
//! - A `sim_universe::LocalPlant` (`RaketaPlant`) that a `Universe` flies off the pad, hands to
//!   orbital propagation above the atmosphere, and takes back on the way down.
//!
//! ## GNC Stack Alignment
//! - **Plant physics (`vehicles/raketa`)**: Aerodynamics (`[0°, 180°]`), planetary atmosphere, mass/inertia, and USD articulation geometry stay in this crate.
//! - **Outer-loop trajectory guidance (`Navigate`)**: `f64` orbital burn, CW rendezvous, entry corridor bank, and powered descent E-guidance live in `navigate-trajectory` and `navigate-guidance`, emitting `GuidanceSetpoint` (`ThrustVector`, `EntryBank`, `CartesianVelocity`).
//! - **Mission intent & phase sequencing (`Pilotage`)**: Operational phase sequencing, authority leases, and receipts live in `pilotage-mission-core`.
//! - **Inner-loop flight control & actuation (`Aviate`, `--features spacecraft`)**: 1 kHz `f32` SAS (`KillRot`, `HoldAttitude`, `DirectionLock`, `EntryBank`), RW + rotational/translational RCS allocation, `4.0G` proper-acceleration limiting, and `< 50 m` fin ground-lock live in `aviate-core::control::spacecraft` and `aviate-core::mixer::spacecraft`.

mod aero_actuators;
mod flight_plant;
mod gnc_boundary;
mod limiter;
mod mission;
mod usd_contract;

pub use aero_actuators::{
    RAKETA_CG_Y_M, RAKETA_FIN_LANDING_LOCK_ALT_M, RAKETA_FIN_MAX_DEFLECTION_DEG,
    RAKETA_INERTIA_DIAGONAL_KG_M2, RAKETA_MASS_KG, RAKETA_RCS_MAX_ACCEL_M_S2, RAKETA_REF_AREA_M2,
    RAKETA_REF_LENGTH_M, RAKETA_RW_MAX_MOMENTUM_NMS, RAKETA_RW_MAX_TORQUE_NM, RaketaAeroOutput,
    RaketaStageLaw, evaluate_cg_rcs_force_body_n, evaluate_raketa_aerodynamics,
    raketa_default_rw_mode, raketa_mass_model, resolve_fin_deflections_deg,
    sample_planetary_atmosphere,
};
pub use flight_plant::{
    RAKETA_BOUNDING_RADIUS_M, RAKETA_PLANT_PERIOD, RAKETA_SENSIBLE_AIR_Q_PA, RaketaCommand,
    RaketaEnvironment, RaketaPlant, RaketaPlantError,
};
pub use gnc_boundary::{
    RAKETA_RCS_VEL_DEADBAND_M_S, RAKETA_RCS_VEL_KP_PER_S, RAKETA_RW_DUMP_THRESHOLD_FRAC,
    RaketaInnerAllocation, RaketaOuterGuidanceSetpoint, allocate_rw_and_rotational_rcs_dump,
    allocate_translational_rcs_unit,
};
pub use limiter::{
    ALERT_PROPER_ACCEL_EXCEEDANCE, ProperAccelLimiterInput, ProperAccelLimiterOutput,
    limit_main_thrust_proper_accel,
};
pub use mission::{
    RaketaContinuousTransferSummary, RaketaMissionError, RaketaMissionPhase, RaketaMissionSummary,
    RaketaMissionTarget, RaketaOriginChange, RaketaStagedDemoSummary, RaketaSurfaceMissionSummary,
    run_continuous_pad_to_transfer, run_staged_phase_demonstration, run_uninterrupted_mission,
};
pub use usd_contract::{
    EXPECTED_RAKETA_USD_SHA256, RAKETA_APAS95_PORT_POS_M, RAKETA_FIN_RUDDER_PIVOT_M,
    RAKETA_NOSE_CARRIER_PIVOT_M, RAKETA_NOSE_WRIST_PIVOT_M, RAKETA_PROPULSION_POS_M,
    RaketaArticulatedTransforms, RaketaArticulationState, RaketaUsdContractReport, RaketaUsdError,
    raketa_package, raketa_package_source, raketa_usd_bytes, verify_raketa_usd_contract,
};
