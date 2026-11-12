//! Command-line configuration and execution for the `ironbird` binary.

use std::net::SocketAddr;
use std::path::PathBuf;

use sim_aviate::DirectNoiseTier;
use sim_core::{HitlSyncMode, TICKS_PER_SECOND, TickSpan};
use sim_geo::Geodetic;
use sim_pack::{
    AceFeedbackReport, AceVehicleInputs, CalibrationReport, DesignPerformanceReport,
    DesignTradeStudyReport, DirectoryPackageSource, LoadedSyspack, SensitivityReport,
    build_ace_feedback_report, builtin_c172s_package, default_design_trades,
    estimate_design_performance, evaluate_package, identify_package_sensitivities,
    run_design_trade_study, write_syspack_with_manual_assets,
};
use sim_xil::betaflight::{BetaflightUdpConfig, BetaflightUdpEndpoint};
use sim_xil::mavlink_hil::{
    HilControlMapping, MavlinkHilConfig, MavlinkHilTcpEndpoint, MavlinkHilUdpEndpoint,
};
use tracing::info;

use crate::{
    FixedWingVehicle, IronbirdError, RigSummary, run_tier0_fixed_wing_cruise,
    run_tier0_package_cruise, run_tier0_x500_hover_climb, run_tier2_betaflight_x500,
    run_tier2_mavlink_fixed_wing,
};

/// Selected ironbird execution mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CliMode {
    /// Tier-0 in-process Aviate X500 multirotor hover/climb.
    Tier0X500,
    /// Tier-0 in-process Aviate C172 cruise.
    Tier0C172,
    /// Tier-0 in-process Aviate fixed-wing cruise for an on-disk `syspack` directory.
    Tier0Dir,
    /// Closed-loop POH + empirical flight-test calibration of built-in C172 (`N642SP`).
    CalibrateC172,
    /// Closed-loop POH + empirical calibration of an on-disk `syspack` directory.
    CalibrateDir,
    /// Closed-loop V-speed, ceiling, glide, and range estimation of built-in C172.
    EstimateC172,
    /// Closed-loop V-speed, ceiling, glide, and range estimation of an on-disk `syspack` directory.
    EstimateDir,
    /// Overall airframe & powerplant trade study on built-in C172.
    TradeC172,
    /// Overall airframe & powerplant trade study on an on-disk `syspack` directory.
    TradeDir,
    /// Flies a package `ace-rs` exported (`--package-dir`) and compares it with
    /// the design it came from (`--ace-vehicle`).
    AceBridge,
    /// Tier-2 MAVLink v2 HIL TCP listener (e.g. PX4 SITL `127.0.0.1:4560`).
    MavlinkTcpListen,
    /// Tier-2 MAVLink v2 HIL TCP client (e.g. router or UART bridge).
    MavlinkTcpConnect,
    /// Tier-2 MAVLink v2 HIL UDP endpoint.
    MavlinkUdp,
    /// Tier-2 Betaflight SITL UDP endpoint (`9002`/`9003`).
    BetaflightUdp,
}

/// Outcome returned by [`execute_cli`].
#[derive(Debug, Clone)]
pub enum CliOutcome {
    /// Completed XIL simulation rig summary.
    Rig(RigSummary),
    /// Completed closed-loop POH & empirical calibration report.
    Calibration(CalibrationReport),
    /// Completed closed-loop calibration plus Jacobian Gauss-Newton sensitivity report.
    Sensitivity(SensitivityReport),
    /// Completed closed-loop new-design V-speed, climb, glide, and range estimate.
    DesignEstimate(DesignPerformanceReport),
    /// Completed overall airframe & powerplant trade study comparison.
    TradeStudy(DesignTradeStudyReport),
    /// Completed `~/ace-rs` bi-directional co-design estimate and feedback report.
    AceCoDesign(DesignPerformanceReport, AceFeedbackReport),
}

/// Parsed command-line options for `ironbird`.
#[derive(Debug, Clone, PartialEq)]
pub struct CliArgs {
    /// Execution mode.
    pub mode: CliMode,
    /// Simulated duration (`s`).
    pub duration_s: f64,
    /// Local bind address for Tier-2 sockets.
    pub bind_addr: SocketAddr,
    /// Remote peer address for Tier-2 client/UDP sockets.
    pub peer_addr: SocketAddr,
    /// Optional lockstep barrier delay (`µs`); `None` selects `RealTimeStamped`.
    pub lockstep_delay_us: Option<u64>,
    /// Package directory when `mode == CliMode::CalibrateDir`.
    pub package_dir: Option<PathBuf>,
    /// The `ace-rs` `vehicle.json` a `--mode ace-bridge` package was exported from (`--ace-vehicle`).
    pub ace_vehicle: Option<PathBuf>,
    /// Optional output directory for exporting a calibrated `syspack` (`--export-syspack`).
    pub export_syspack: Option<PathBuf>,
    /// Optional output path for Markdown calibration/sensitivity report.
    pub report_out: Option<PathBuf>,
    /// Whether to run finite-difference Jacobian sensitivity identification.
    pub with_sensitivity: bool,
}

impl Default for CliArgs {
    fn default() -> Self {
        Self {
            mode: CliMode::Tier0X500,
            duration_s: 10.0,
            bind_addr: SocketAddr::from(([127, 0, 0, 1], 4560)),
            peer_addr: SocketAddr::from(([127, 0, 0, 1], 14550)),
            lockstep_delay_us: Some(2_500),
            package_dir: None,
            ace_vehicle: None,
            export_syspack: None,
            report_out: None,
            with_sensitivity: false,
        }
    }
}

/// Parses process arguments (excluding `argv[0]`).
pub fn parse_args<I, S>(iter: I) -> Result<CliArgs, IronbirdError>
where
    I: IntoIterator<Item = S>,
    S: AsRef<str>,
{
    let mut cfg = CliArgs::default();
    let mut args = iter.into_iter();
    while let Some(raw) = args.next() {
        let flag = raw.as_ref();
        match flag {
            "--mode" => {
                let val = next_val(&mut args, "--mode")?;
                cfg.mode = parse_mode(val.as_ref())?;
            }
            "--duration-s" => {
                let val = next_val(&mut args, "--duration-s")?;
                cfg.duration_s = val
                    .as_ref()
                    .parse::<f64>()
                    .map_err(|_| usage("bad --duration-s"))?;
            }
            "--bind" => {
                let val = next_val(&mut args, "--bind")?;
                cfg.bind_addr = val
                    .as_ref()
                    .parse()
                    .map_err(|_| usage("bad --bind SocketAddr"))?;
            }
            "--peer" => {
                let val = next_val(&mut args, "--peer")?;
                cfg.peer_addr = val
                    .as_ref()
                    .parse()
                    .map_err(|_| usage("bad --peer SocketAddr"))?;
            }
            "--realtime" => {
                cfg.lockstep_delay_us = None;
            }
            "--lockstep-us" => {
                let val = next_val(&mut args, "--lockstep-us")?;
                let us = val
                    .as_ref()
                    .parse::<u64>()
                    .map_err(|_| usage("bad --lockstep-us"))?;
                cfg.lockstep_delay_us = Some(us);
            }
            "--package-dir" => {
                let val = next_val(&mut args, "--package-dir")?;
                cfg.package_dir = Some(PathBuf::from(val.as_ref()));
            }
            "--ace-vehicle" => {
                let val = next_val(&mut args, "--ace-vehicle")?;
                cfg.ace_vehicle = Some(PathBuf::from(val.as_ref()));
            }
            "--export-syspack" => {
                let val = next_val(&mut args, "--export-syspack")?;
                cfg.export_syspack = Some(PathBuf::from(val.as_ref()));
            }
            "--report-out" => {
                let val = next_val(&mut args, "--report-out")?;
                cfg.report_out = Some(PathBuf::from(val.as_ref()));
            }
            "--sensitivity" => {
                cfg.with_sensitivity = true;
            }
            other => return Err(usage(&format!("unknown flag `{other}`"))),
        }
    }
    Ok(cfg)
}

/// Executes the configured ironbird scenario or calibration suite.
pub fn execute_cli(args: &CliArgs) -> Result<CliOutcome, IronbirdError> {
    if matches!(
        args.mode,
        CliMode::TradeC172 | CliMode::TradeDir | CliMode::AceBridge
    ) {
        return execute_trades_or_ace_cli(args);
    }
    if matches!(
        args.mode,
        CliMode::CalibrateC172
            | CliMode::CalibrateDir
            | CliMode::EstimateC172
            | CliMode::EstimateDir
    ) {
        return execute_calibration_cli(args);
    }
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 120.0);
    let sync = args
        .lockstep_delay_us
        .map_or(HitlSyncMode::RealTimeStamped, |us| {
            HitlSyncMode::LockstepBarrier {
                processing_delay: TickSpan::from_ticks(
                    (TICKS_PER_SECOND / 1_000_000).saturating_mul(us),
                ),
            }
        });
    let noise = DirectNoiseTier::Industrial;
    let summary = match args.mode {
        CliMode::Tier0X500 => run_tier0_x500_hover_climb(origin, 5.0, args.duration_s, noise, 1)?,
        CliMode::Tier0C172 => {
            run_tier0_fixed_wing_cruise(FixedWingVehicle::C172, origin, args.duration_s, noise, 1)?
        }
        CliMode::Tier0Dir => {
            let dir = args
                .package_dir
                .as_ref()
                .ok_or_else(|| usage("--mode tier0-dir requires --package-dir <path>"))?;
            let package = LoadedSyspack::load(&DirectoryPackageSource::new(dir))?;
            run_tier0_package_cruise(&package, origin, args.duration_s, noise, 1)?
        }
        _ => execute_tier2_cli(args, origin, sync)?,
    };
    info!(
        mode = ?args.mode,
        steps = summary.steps_executed,
        sim_s = summary.simulated_seconds,
        wall_s = summary.wall_seconds,
        speedup_x = summary.realtime_speedup,
        final_alt_m = summary.final_geodetic.height_m,
        "ironbird run completed"
    );
    Ok(CliOutcome::Rig(summary))
}

fn execute_calibration_cli(args: &CliArgs) -> Result<CliOutcome, IronbirdError> {
    let package = match args.mode {
        CliMode::CalibrateC172 | CliMode::EstimateC172 => builtin_c172s_package()?,
        CliMode::CalibrateDir | CliMode::EstimateDir => {
            let dir = args.package_dir.as_ref().ok_or_else(|| {
                usage("--mode calibrate-dir|estimate-dir requires --package-dir <path>")
            })?;
            LoadedSyspack::load(&DirectoryPackageSource::new(dir))?
        }
        _ => return Err(usage("internal calibration dispatch mismatch")),
    };
    if matches!(args.mode, CliMode::EstimateC172 | CliMode::EstimateDir) {
        let report = estimate_design_performance(&package)?;
        let md = report.to_markdown();
        if let Some(path) = &args.report_out {
            std::fs::write(path, &md)?;
        }
        info!(
            aircraft = %report.package_id,
            vs0_kcas = report.vs0_kcas,
            vs1_kcas = report.vs1_kcas,
            vx_kcas = report.vx_kcas,
            vy_kcas = report.vy_kcas,
            vg_kcas = report.vg_kcas,
            service_ceiling_ft = report.service_ceiling_ft,
            "\n{md}"
        );
        return Ok(CliOutcome::DesignEstimate(report));
    }
    if args.with_sensitivity {
        let report = identify_package_sensitivities(&package)?;
        let md = report.to_markdown();
        if let Some(path) = &args.report_out {
            std::fs::write(path, &md)?;
        }
        info!(
            aircraft = %report.baseline.type_designator,
            all_passed = report.baseline.all_passed,
            baseline_cost = report.baseline_cost,
            predicted_cost = report.linearized_predicted_cost,
            "\n{md}"
        );
        Ok(CliOutcome::Sensitivity(report))
    } else {
        let report = evaluate_package(&package)?;
        let md = report.to_markdown();
        if let Some(path) = &args.report_out {
            std::fs::write(path, &md)?;
        }
        info!(
            aircraft = %report.type_designator,
            all_passed = report.all_passed,
            stall_rmse_kt = report.stall_kcas_rmse_kt,
            climb_rmse_fpm = report.climb_rmse_fpm,
            cruise_rmse_ktas = report.cruise_rmse_ktas,
            empirical_gs_rmse_kt = report.empirical_gs_rmse_kt,
            "\n{md}"
        );
        Ok(CliOutcome::Calibration(report))
    }
}

fn execute_trades_or_ace_cli(args: &CliArgs) -> Result<CliOutcome, IronbirdError> {
    if args.mode == CliMode::AceBridge {
        return execute_ace_bridge(args);
    }
    let package = match args.mode {
        CliMode::TradeC172 => builtin_c172s_package()?,
        CliMode::TradeDir => {
            let dir = args
                .package_dir
                .as_ref()
                .ok_or_else(|| usage("--mode trade-dir requires --package-dir <path>"))?;
            LoadedSyspack::load(&DirectoryPackageSource::new(dir))?
        }
        _ => return Err(usage("internal trade study dispatch mismatch")),
    };
    let study = run_design_trade_study(&package, &default_design_trades())?;
    let md = study.to_markdown();
    if let Some(path) = &args.report_out {
        std::fs::write(path, &md)?;
    }
    info!(aircraft = %study.base_package_id, "\n{md}");
    Ok(CliOutcome::TradeStudy(study))
}

fn execute_tier2_cli(
    args: &CliArgs,
    origin: Geodetic,
    sync: HitlSyncMode,
) -> Result<RigSummary, IronbirdError> {
    match args.mode {
        CliMode::MavlinkTcpListen => {
            let ep = MavlinkHilTcpEndpoint::bind_and_accept(
                args.bind_addr,
                MavlinkHilConfig::default(),
            )?;
            run_tier2_mavlink_fixed_wing(
                FixedWingVehicle::C172,
                origin,
                args.duration_s,
                sync,
                HilControlMapping::default(),
                ep,
            )
        }
        CliMode::MavlinkTcpConnect => {
            let ep = MavlinkHilTcpEndpoint::connect(args.peer_addr, MavlinkHilConfig::default())?;
            run_tier2_mavlink_fixed_wing(
                FixedWingVehicle::C172,
                origin,
                args.duration_s,
                sync,
                HilControlMapping::default(),
                ep,
            )
        }
        CliMode::MavlinkUdp => {
            let ep = MavlinkHilUdpEndpoint::bind(
                args.bind_addr,
                args.peer_addr,
                MavlinkHilConfig::default(),
            )?;
            run_tier2_mavlink_fixed_wing(
                FixedWingVehicle::C172,
                origin,
                args.duration_s,
                sync,
                HilControlMapping::default(),
                ep,
            )
        }
        CliMode::BetaflightUdp => {
            let ep = BetaflightUdpEndpoint::bind(
                args.bind_addr,
                args.peer_addr,
                BetaflightUdpConfig::default(),
            )?;
            run_tier2_betaflight_x500(origin, args.duration_s, sync, ep)
        }
        _ => Err(usage("internal mode dispatch mismatch")),
    }
}

fn next_val<I, S>(iter: &mut I, flag: &str) -> Result<S, IronbirdError>
where
    I: Iterator<Item = S>,
{
    iter.next()
        .ok_or_else(|| usage(&format!("missing value for {flag}")))
}

fn parse_mode(s: &str) -> Result<CliMode, IronbirdError> {
    match s {
        "tier0-x500" => Ok(CliMode::Tier0X500),
        "tier0-c172" => Ok(CliMode::Tier0C172),
        "tier0-dir" => Ok(CliMode::Tier0Dir),
        "calibrate-c172" => Ok(CliMode::CalibrateC172),
        "calibrate-dir" => Ok(CliMode::CalibrateDir),
        "estimate-c172" => Ok(CliMode::EstimateC172),
        "estimate-dir" => Ok(CliMode::EstimateDir),
        "trade-c172" => Ok(CliMode::TradeC172),
        "trade-dir" => Ok(CliMode::TradeDir),
        "ace-bridge" => Ok(CliMode::AceBridge),
        "mavlink-tcp-listen" => Ok(CliMode::MavlinkTcpListen),
        "mavlink-tcp-connect" => Ok(CliMode::MavlinkTcpConnect),
        "mavlink-udp" => Ok(CliMode::MavlinkUdp),
        "betaflight-udp" => Ok(CliMode::BetaflightUdp),
        other => Err(usage(&format!("unknown --mode `{other}`"))),
    }
}

fn usage(msg: &str) -> IronbirdError {
    IronbirdError::CliUsage {
        detail: format!(
            "{msg} (expected --mode tier0-x500|tier0-c172|tier0-dir|calibrate-c172|calibrate-dir|estimate-c172|estimate-dir|trade-c172|trade-dir|ace-bridge|mavlink-tcp-listen|mavlink-tcp-connect|mavlink-udp|betaflight-udp)"
        ),
    }
}

/// Flies the exported package and compares it with its design; refuses a
/// package that was not exported from the design.
fn execute_ace_bridge(args: &CliArgs) -> Result<CliOutcome, IronbirdError> {
    let (Some(vehicle), Some(dir)) = (&args.ace_vehicle, &args.package_dir) else {
        return Err(usage(
            "--mode ace-bridge requires --ace-vehicle <vehicle.json> and --package-dir <dir> \
             (the package ace-export wrote from that vehicle)",
        ));
    };
    let bytes = std::fs::read(vehicle)?;
    let inputs = AceVehicleInputs::from_vehicle_json(&vehicle.display().to_string(), &bytes)
        .map_err(sim_pack::PackError::from)?;
    let package = LoadedSyspack::load(&DirectoryPackageSource::new(dir))?;
    inputs.check_package(&package)?;
    let report = estimate_design_performance(&package)?;
    if let Some(out_dir) = &args.export_syspack {
        write_syspack_with_manual_assets(&package, &report, out_dir)?;
    }
    let feedback =
        build_ace_feedback_report(&inputs, vehicle, &report).map_err(sim_pack::PackError::from)?;
    let md = format!("{}\n{}", report.to_markdown(), feedback.to_markdown());
    if let Some(path) = &args.report_out {
        std::fs::write(path, &md)?;
    }
    info!(aircraft = %report.package_id, "\n{md}");
    Ok(CliOutcome::AceCoDesign(report, feedback))
}
