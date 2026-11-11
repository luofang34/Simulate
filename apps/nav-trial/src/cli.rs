//! The command line.
//!
//! - `nav-trial prepare --request <file> --onboard <dir> --world <dir> [--cache <dir>]`
//!   fetches the route corridor's tiles before departure and writes the
//!   onboard package and the simulated world's imagery.
//! - `nav-trial fly --request <file> --out <file>` runs the trial offline; a
//!   request without a subcommand flies too.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::Serialize;
use sim_navigate::trial::{
    FrameTiming, TrialReport, TrialRequest, default_sensor_sets, run_trial_with_timings,
};
use tracing::info;

use crate::NavTrialError;

/// What the command line asks for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CliCommand {
    /// Write the packages of a request's route.
    Prepare {
        /// The request file.
        request: PathBuf,
        /// Where the onboard package goes.
        onboard: PathBuf,
        /// Where the world imagery goes.
        world: PathBuf,
        /// Where fetched tiles are cached.
        cache: Option<PathBuf>,
    },
    /// Fly a request.
    Fly {
        /// The request file.
        request: PathBuf,
        /// Where the result goes.
        out: PathBuf,
    },
}

/// Parses the arguments after the program name.
pub fn parse_args(args: impl IntoIterator<Item = String>) -> Result<CliCommand, NavTrialError> {
    let usage = |detail: String| NavTrialError::Usage { detail };
    let mut args = args.into_iter().peekable();
    let prepare = match args.peek().map(String::as_str) {
        Some("prepare") => true,
        Some("fly") => false,
        _ => {
            return fly_args(BTreeMap::new(), args);
        }
    };
    args.next();
    let flags = flags(args)?;
    if !prepare {
        return fly_args(flags, std::iter::empty());
    }
    let take = |name: &str| {
        flags
            .get(name)
            .cloned()
            .ok_or_else(|| usage(format!("no `--{name}`")))
    };
    let allowed = ["request", "onboard", "world", "cache"];
    if let Some(unknown) = flags.keys().find(|k| !allowed.contains(&k.as_str())) {
        return Err(usage(format!("unknown argument `--{unknown}`")));
    }
    Ok(CliCommand::Prepare {
        request: take("request")?,
        onboard: take("onboard")?,
        world: take("world")?,
        cache: flags.get("cache").cloned(),
    })
}

fn fly_args(
    mut flags: BTreeMap<String, PathBuf>,
    rest: impl Iterator<Item = String>,
) -> Result<CliCommand, NavTrialError> {
    let usage = |detail: String| NavTrialError::Usage { detail };
    flags.extend(self::flags(rest)?);
    if let Some(unknown) = flags
        .keys()
        .find(|k| !["request", "out"].contains(&k.as_str()))
    {
        return Err(usage(format!("unknown argument `--{unknown}`")));
    }
    Ok(CliCommand::Fly {
        request: flags
            .remove("request")
            .ok_or_else(|| usage("no `--request`".into()))?,
        out: flags
            .remove("out")
            .ok_or_else(|| usage("no `--out`".into()))?,
    })
}

fn flags(args: impl Iterator<Item = String>) -> Result<BTreeMap<String, PathBuf>, NavTrialError> {
    let usage = |detail: String| NavTrialError::Usage { detail };
    let mut flags = BTreeMap::new();
    let mut args = args;
    while let Some(flag) = args.next() {
        let Some(name) = flag.strip_prefix("--") else {
            return Err(usage(format!("unknown argument `{flag}`")));
        };
        let value = args
            .next()
            .ok_or_else(|| usage(format!("`{flag}` needs a value")))?;
        if flags
            .insert(name.to_owned(), PathBuf::from(value))
            .is_some()
        {
            return Err(usage(format!("`{flag}` given twice")));
        }
    }
    Ok(flags)
}

/// Reads and parses a request file. A request that lists no sensor sets
/// flies every aiding means alone and fused.
pub fn read_request_blocking(path: &Path) -> Result<TrialRequest, NavTrialError> {
    let text = std::fs::read_to_string(path).map_err(|source| NavTrialError::Io {
        action: "reading",
        path: path.to_path_buf(),
        source,
    })?;
    let mut request = TrialRequest::from_json(&text)?;
    if request.sensor_sets.is_empty() {
        request.sensor_sets = default_sensor_sets(request.visual.is_some());
    }
    Ok(request)
}

/// Per-frame wall times of the visual chain, written beside the result.
#[derive(Debug, Serialize)]
struct TimingSidecar<'a> {
    note: &'static str,
    sets: Vec<SetTiming<'a>>,
}

#[derive(Debug, Serialize)]
struct SetTiming<'a> {
    name: &'a str,
    frames: usize,
    mean_live_render_s: f64,
    mean_reference_s: f64,
    mean_matching_s: f64,
    per_frame: &'a [FrameTiming],
}

/// The sidecar file of a result path.
pub fn timing_path(out: &Path) -> PathBuf {
    let mut name = out.as_os_str().to_owned();
    name.push(".timing.json");
    PathBuf::from(name)
}

/// Reads the request at `request`, runs it with no network, and writes the
/// result to `out` and the visual chain's wall times beside it.
pub fn run_files_blocking(request: &Path, out: &Path) -> Result<TrialReport, NavTrialError> {
    let request_value = read_request_blocking(request)?;
    info!(
        sets = request_value.sensor_sets.len(),
        waypoints = request_value.waypoints.len(),
        "running trial"
    );
    let (report, timings) = run_trial_with_timings(&request_value)?;
    let write = |path: &Path, text: String| {
        std::fs::write(path, text).map_err(|source| NavTrialError::Io {
            action: "writing",
            path: path.to_path_buf(),
            source,
        })
    };
    write(out, report.to_json()?)?;
    if !timings.is_empty() {
        let mean = |f: fn(&FrameTiming) -> f64, frames: &[FrameTiming]| {
            frames.iter().map(f).sum::<f64>() / frames.len().max(1) as f64
        };
        let sidecar = TimingSidecar {
            note: "wall time on this host; not part of the deterministic result",
            sets: timings
                .iter()
                .map(|(name, frames)| SetTiming {
                    name,
                    frames: frames.len(),
                    mean_live_render_s: mean(|t| t.live_render_s, frames),
                    mean_reference_s: mean(|t| t.reference_s, frames),
                    mean_matching_s: mean(|t| t.matching_s, frames),
                    per_frame: frames,
                })
                .collect(),
        };
        let text = serde_json::to_string_pretty(&sidecar).map_err(|source| {
            NavTrialError::Trial(sim_navigate::trial::TrialError::Json {
                what: "timing",
                source,
            })
        })?;
        write(&timing_path(out), text)?;
    }
    info!(out = %out.display(), "result written");
    Ok(report)
}
