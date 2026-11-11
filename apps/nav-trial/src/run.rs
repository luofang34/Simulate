//! Running a parsed command line: loading or completing the request,
//! streaming progress as NDJSON, and writing the result.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use sim_navigate::trial::{
    FinishedRecord, ProgressRecord, RoutePoint, TrialReport, TrialRequest, request_from_route,
    run_trial_timed_blocking,
};
use tracing::info;

use crate::NavTrialError;
use crate::cli::{CliArgs, ProgressTarget, RequestSource, RouteOverrides};

/// Runs `args`: the result goes to its `--out`, progress to its
/// `--progress`, and a route's completed request beside the result.
pub fn run_blocking(args: &CliArgs) -> Result<TrialReport, NavTrialError> {
    let started = Instant::now();
    let request = load_request_blocking(&args.source, &args.out)?;
    info!(
        sets = request.sensor_sets.len(),
        waypoints = request.waypoints.len(),
        "running trial"
    );
    let mut progress = open_progress_blocking(args.progress.as_ref())?;
    let outcome = {
        let mut sink = |record: &ProgressRecord| write_record(progress.as_mut(), record);
        run_trial_timed_blocking(&request, &mut sink)
    };
    let written = match &outcome {
        Ok((report, timing)) => {
            write_blocking(&args.out, &report.to_json()?)?;
            write_blocking(&timing_path(&args.out), &timing.to_json()?)?;
            info!(out = %args.out.display(), "result written");
            Some(args.out.display().to_string())
        }
        Err(_) => None,
    };
    let finished = ProgressRecord::Finished(FinishedRecord {
        result_path: written,
        wall_time_s: started.elapsed().as_secs_f64(),
    });
    write_record(progress.as_mut(), &finished).map_err(|source| NavTrialError::Io {
        action: "writing progress to",
        path: progress_path(args.progress.as_ref()),
        source,
    })?;
    Ok(outcome?.0)
}

/// Where a run's wall-clock timing is written: beside `out`, as
/// `<out stem>.timing.json`, apart from the result because it depends on
/// the machine.
pub fn timing_path(out: &Path) -> PathBuf {
    out.with_extension("timing.json")
}

/// Where a route's completed request is written: beside `out`, as
/// `<out stem>.request.json`.
pub fn completed_request_path(out: &Path) -> PathBuf {
    out.with_extension("request.json")
}

/// The request `source` names; a route is completed, written beside `out`,
/// and read back from the text written.
pub fn load_request_blocking(
    source: &RequestSource,
    out: &Path,
) -> Result<TrialRequest, NavTrialError> {
    let (route, defaults, overrides) = match source {
        RequestSource::Request(path) => {
            return Ok(TrialRequest::from_json(&read_blocking(path)?)?);
        }
        RequestSource::Route {
            route,
            defaults,
            overrides,
        } => (route, defaults, overrides),
    };
    let points = RoutePoint::parse_route(&read_blocking(route)?)?;
    let defaults = defaults.as_deref().map(read_blocking).transpose()?;
    let mut request = request_from_route(&points, defaults.as_deref())?;
    apply(overrides, &mut request);
    let text = request.to_json()?;
    write_blocking(&completed_request_path(out), &text)?;
    Ok(TrialRequest::from_json(&text)?)
}

fn apply(overrides: &RouteOverrides, request: &mut TrialRequest) {
    if let Some(epoch) = &overrides.epoch_utc {
        request.epoch_utc.clone_from(epoch);
    }
    if let Some(grade) = overrides.imu_grade {
        request.imu_grade = grade;
    }
    if let Some(guidance) = overrides.guidance {
        request.vehicle.guidance = guidance;
    }
}

fn open_progress_blocking(
    target: Option<&ProgressTarget>,
) -> Result<Option<Box<dyn Write>>, NavTrialError> {
    Ok(match target {
        None => None,
        Some(ProgressTarget::Stdout) => Some(Box::new(io::stdout())),
        Some(ProgressTarget::File(path)) => {
            let file = File::create(path).map_err(|source| NavTrialError::Io {
                action: "creating",
                path: path.clone(),
                source,
            })?;
            Some(Box::new(BufWriter::new(file)))
        }
    })
}

fn progress_path(target: Option<&ProgressTarget>) -> PathBuf {
    match target {
        Some(ProgressTarget::File(path)) => path.clone(),
        Some(ProgressTarget::Stdout) | None => PathBuf::from("-"),
    }
}

/// One NDJSON line, flushed so a monitor sees it at once.
fn write_record(writer: Option<&mut Box<dyn Write>>, record: &ProgressRecord) -> io::Result<()> {
    let Some(writer) = writer else {
        return Ok(());
    };
    let line = record.to_json_line().map_err(io::Error::other)?;
    writer.write_all(line.as_bytes())?;
    writer.write_all(b"\n")?;
    writer.flush()
}

fn read_blocking(path: &Path) -> Result<String, NavTrialError> {
    std::fs::read_to_string(path).map_err(|source| NavTrialError::Io {
        action: "reading",
        path: path.to_path_buf(),
        source,
    })
}

fn write_blocking(path: &Path, text: &str) -> Result<(), NavTrialError> {
    std::fs::write(path, text).map_err(|source| NavTrialError::Io {
        action: "writing",
        path: path.to_path_buf(),
        source,
    })
}
