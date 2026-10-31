//! Command-line configuration and execution for the `ironbird` binary.

use std::net::SocketAddr;

use sim_aviate::DirectNoiseTier;
use sim_core::{HitlSyncMode, TICKS_PER_SECOND, TickSpan};
use sim_geo::Geodetic;
use sim_xil::betaflight::{BetaflightUdpConfig, BetaflightUdpEndpoint};
use sim_xil::mavlink_hil::{
    HilControlMapping, MavlinkHilConfig, MavlinkHilTcpEndpoint, MavlinkHilUdpEndpoint,
};
use tracing::info;

use crate::{
    FixedWingVehicle, IronbirdError, RigSummary, run_tier0_fixed_wing_cruise,
    run_tier0_x500_hover_climb, run_tier2_betaflight_x500, run_tier2_mavlink_fixed_wing,
};

/// Selected ironbird execution mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CliMode {
    /// Tier-0 in-process Aviate X500 multirotor hover/climb.
    Tier0X500,
    /// Tier-0 in-process Aviate C172 cruise.
    Tier0C172,
    /// Tier-0 in-process Aviate Light Trainer cruise.
    Tier0Trainer,
    /// Tier-2 MAVLink v2 HIL TCP listener (e.g. PX4 SITL `127.0.0.1:4560`).
    MavlinkTcpListen,
    /// Tier-2 MAVLink v2 HIL TCP client (e.g. router or UART bridge).
    MavlinkTcpConnect,
    /// Tier-2 MAVLink v2 HIL UDP endpoint.
    MavlinkUdp,
    /// Tier-2 Betaflight SITL UDP endpoint (`9002`/`9003`).
    BetaflightUdp,
}

/// Parsed command-line options for `ironbird`.
#[derive(Debug, Clone, Copy, PartialEq)]
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
}

impl Default for CliArgs {
    fn default() -> Self {
        Self {
            mode: CliMode::Tier0X500,
            duration_s: 10.0,
            bind_addr: SocketAddr::from(([127, 0, 0, 1], 4560)),
            peer_addr: SocketAddr::from(([127, 0, 0, 1], 14550)),
            lockstep_delay_us: Some(2_500),
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
            other => return Err(usage(&format!("unknown flag `{other}`"))),
        }
    }
    Ok(cfg)
}

/// Executes the configured ironbird scenario and logs the resulting [`RigSummary`].
pub fn execute_cli(args: &CliArgs) -> Result<RigSummary, IronbirdError> {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 120.0);
    let sync = match args.lockstep_delay_us {
        Some(us) => HitlSyncMode::LockstepBarrier {
            processing_delay: TickSpan::from_ticks(
                (TICKS_PER_SECOND / 1_000_000).saturating_mul(us),
            ),
        },
        None => HitlSyncMode::RealTimeStamped,
    };

    let summary = match args.mode {
        CliMode::Tier0X500 => run_tier0_x500_hover_climb(
            origin,
            5.0,
            args.duration_s,
            DirectNoiseTier::Industrial,
            1,
        )?,
        CliMode::Tier0C172 => run_tier0_fixed_wing_cruise(
            FixedWingVehicle::C172,
            origin,
            args.duration_s,
            DirectNoiseTier::Industrial,
            1,
        )?,
        CliMode::Tier0Trainer => run_tier0_fixed_wing_cruise(
            FixedWingVehicle::LightTrainer,
            origin,
            args.duration_s,
            DirectNoiseTier::Industrial,
            1,
        )?,
        _ => execute_tier2_cli(args, origin, sync)?,
    };

    info!(
        mode = ?args.mode,
        steps = summary.steps_executed,
        sim_s = summary.simulated_seconds,
        wall_s = summary.wall_seconds,
        speedup_x = summary.realtime_speedup,
        final_alt_m = summary.final_geodetic.height_m,
        delivered = summary.bridge_stats.delivered,
        dropped = summary.bridge_stats.dropped,
        "ironbird run completed"
    );
    Ok(summary)
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
        "tier0-trainer" => Ok(CliMode::Tier0Trainer),
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
            "{msg} (expected --mode tier0-x500|tier0-c172|tier0-trainer|mavlink-tcp-listen|mavlink-tcp-connect|mavlink-udp|betaflight-udp)"
        ),
    }
}
