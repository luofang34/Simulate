//! `Raketa` single-stage interplanetary test rocket (`vehicles/raketa`):
//! - SHA-256 & OpenUSD schema-verified `retro_rocket_v4.usd` (`upAxis = Y`, `metersPerUnit = 1.0`),
//! - Exact quadratic feasible-interval `4.0G` proper-acceleration limiter (`a_eff = clamp(-b, 0, a_cmd)` when infeasible),
//! - Full `[0°, 180°]` aerodynamic envelope with `< 50 m` / pad fin ground-lock (`±18°` -> `0°`),
//! - Reserved 3-axis CG translational RCS (`±0.25G`), momentum-conserving reaction wheels ($5\times 10^6\text{ N}\cdot\text{m}\cdot\text{s}$),
//! - Continuous pad-to-ascent execution (`run_uninterrupted_mission` failing closed at unimplemented continuous transitions)
//!   and explicitly tagged staged phase demonstrations (`run_staged_phase_demonstration`).
//! - A `sim_universe::LocalPlant` (`RaketaPlant`) that a `Universe` flies off the pad, hands to
//!   orbital propagation above the atmosphere, and takes back on the way down.

mod aero_actuators;
mod flight_plant;
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
pub use limiter::{
    ALERT_PROPER_ACCEL_EXCEEDANCE, ProperAccelLimiterInput, ProperAccelLimiterOutput,
    limit_main_thrust_proper_accel,
};
pub use mission::{
    RaketaMissionError, RaketaMissionPhase, RaketaMissionSummary, RaketaMissionTarget,
    RaketaStagedDemoSummary, run_staged_phase_demonstration, run_uninterrupted_mission,
};
pub use usd_contract::{
    EXPECTED_RAKETA_USD_SHA256, RAKETA_APAS95_PORT_POS_M, RAKETA_FIN_RUDDER_PIVOT_M,
    RAKETA_NOSE_CARRIER_PIVOT_M, RAKETA_NOSE_WRIST_PIVOT_M, RAKETA_PROPULSION_POS_M,
    RaketaArticulatedTransforms, RaketaArticulationState, RaketaUsdContractReport, RaketaUsdError,
    raketa_package, raketa_package_source, raketa_usd_bytes, verify_raketa_usd_contract,
};
