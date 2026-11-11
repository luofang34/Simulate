//! The command line: `nav-trial --request <file> --out <file>`.

use std::path::{Path, PathBuf};

use sim_navigate::trial::{TrialReport, TrialRequest, run_trial};
use tracing::info;

use crate::NavTrialError;

/// The parsed command line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CliArgs {
    /// The request file.
    pub request: PathBuf,
    /// Where the result goes.
    pub out: PathBuf,
}

/// Parses the arguments after the program name.
pub fn parse_args(args: impl IntoIterator<Item = String>) -> Result<CliArgs, NavTrialError> {
    let usage = |detail: String| NavTrialError::Usage { detail };
    let (mut request, mut out) = (None, None);
    let mut args = args.into_iter();
    while let Some(flag) = args.next() {
        let slot = match flag.as_str() {
            "--request" => &mut request,
            "--out" => &mut out,
            other => return Err(usage(format!("unknown argument `{other}`"))),
        };
        let value = args
            .next()
            .ok_or_else(|| usage(format!("`{flag}` needs a file")))?;
        if slot.replace(PathBuf::from(value)).is_some() {
            return Err(usage(format!("`{flag}` given twice")));
        }
    }
    Ok(CliArgs {
        request: request.ok_or_else(|| usage("no `--request`".into()))?,
        out: out.ok_or_else(|| usage("no `--out`".into()))?,
    })
}

/// Reads the request at `request`, runs it, and writes the result to `out`.
pub fn run_files_blocking(request: &Path, out: &Path) -> Result<TrialReport, NavTrialError> {
    let text = std::fs::read_to_string(request).map_err(|source| NavTrialError::Io {
        action: "reading",
        path: request.to_path_buf(),
        source,
    })?;
    let request_value = TrialRequest::from_json(&text)?;
    info!(
        sets = request_value.sensor_sets.len(),
        waypoints = request_value.waypoints.len(),
        "running trial"
    );
    let report = run_trial(&request_value)?;
    std::fs::write(out, report.to_json()?).map_err(|source| NavTrialError::Io {
        action: "writing",
        path: out.to_path_buf(),
        source,
    })?;
    info!(out = %out.display(), "result written");
    Ok(report)
}
