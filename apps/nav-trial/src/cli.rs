//! The command line:
//!
//! ```text
//! nav-trial --request <request.json> --out <result.json> [--progress <file|->]
//! nav-trial --route <route.json> [--defaults <partial-request.json>]
//!           [--epoch <YYYY-MM-DDTHH:MM:SSZ>] [--imu consumer_mems|tactical|navigation]
//!           [--guidance truth|estimate] --out <result.json> [--progress <file|->]
//! ```
//!
//! `--route` completes the request from the defaults of
//! [`sim_navigate::trial::request_from_route`], the defaults file and the
//! flags, writes it beside the result as `<out stem>.request.json`, and runs
//! exactly what it wrote, so re-running that file with `--request` gives the
//! same result.

use std::path::PathBuf;

use sim_navigate::trial::{GuidanceSource, ImuGradeName};

use crate::NavTrialError;

/// Where the request comes from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RequestSource {
    /// A full request file.
    Request(PathBuf),
    /// A route file completed with defaults.
    Route {
        /// The route file: a JSON array of points.
        route: PathBuf,
        /// A partial request overriding the defaults.
        defaults: Option<PathBuf>,
        /// Overrides from flags, applied last.
        overrides: RouteOverrides,
    },
}

/// The flags that override a route's defaults.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RouteOverrides {
    /// `--epoch`.
    pub epoch_utc: Option<String>,
    /// `--imu`.
    pub imu_grade: Option<ImuGradeName>,
    /// `--guidance`.
    pub guidance: Option<GuidanceSource>,
}

/// Where progress records go.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProgressTarget {
    /// Standard output, for `-`.
    Stdout,
    /// A file, replaced.
    File(PathBuf),
}

/// The parsed command line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CliArgs {
    /// The request.
    pub source: RequestSource,
    /// Where the result goes.
    pub out: PathBuf,
    /// Where progress goes, if anywhere.
    pub progress: Option<ProgressTarget>,
}

/// Every flag; each takes one value.
const FLAGS: [&str; 8] = [
    "--request",
    "--route",
    "--defaults",
    "--epoch",
    "--imu",
    "--guidance",
    "--out",
    "--progress",
];

/// Parses the arguments after the program name.
pub fn parse_args(args: impl IntoIterator<Item = String>) -> Result<CliArgs, NavTrialError> {
    let usage = |detail: String| NavTrialError::Usage { detail };
    let mut values: [Option<String>; 8] = Default::default();
    let mut args = args.into_iter();
    while let Some(flag) = args.next() {
        let Some(slot) = FLAGS
            .iter()
            .position(|known| *known == flag)
            .and_then(|index| values.get_mut(index))
        else {
            return Err(usage(format!("unknown argument `{flag}`")));
        };
        let value = args
            .next()
            .ok_or_else(|| usage(format!("`{flag}` needs a value")))?;
        if slot.replace(value).is_some() {
            return Err(usage(format!("`{flag}` given twice")));
        }
    }
    let [
        request,
        route,
        defaults,
        epoch,
        imu,
        guidance,
        out,
        progress,
    ] = values;
    let overrides = RouteOverrides {
        epoch_utc: epoch,
        imu_grade: imu.map(|name| imu_grade(&name)).transpose()?,
        guidance: guidance.map(|name| guidance_source(&name)).transpose()?,
    };
    let source = match (request, route) {
        (Some(request), None) => {
            if defaults.is_some() || overrides != RouteOverrides::default() {
                return Err(usage(
                    "`--defaults`, `--epoch`, `--imu` and `--guidance` go with `--route`".into(),
                ));
            }
            RequestSource::Request(request.into())
        }
        (None, Some(route)) => RequestSource::Route {
            route: route.into(),
            defaults: defaults.map(PathBuf::from),
            overrides,
        },
        _ => return Err(usage("exactly one of `--request` and `--route`".into())),
    };
    Ok(CliArgs {
        source,
        out: out.ok_or_else(|| usage("no `--out`".into()))?.into(),
        progress: progress.map(|target| match target.as_str() {
            "-" => ProgressTarget::Stdout,
            _ => ProgressTarget::File(target.into()),
        }),
    })
}

fn imu_grade(name: &str) -> Result<ImuGradeName, NavTrialError> {
    match name {
        "consumer_mems" => Ok(ImuGradeName::ConsumerMems),
        "tactical" => Ok(ImuGradeName::Tactical),
        "navigation" => Ok(ImuGradeName::Navigation),
        other => Err(NavTrialError::Usage {
            detail: format!("`--imu {other}`: consumer_mems, tactical or navigation"),
        }),
    }
}

fn guidance_source(name: &str) -> Result<GuidanceSource, NavTrialError> {
    match name {
        "truth" => Ok(GuidanceSource::Truth),
        "estimate" => Ok(GuidanceSource::Estimate),
        other => Err(NavTrialError::Usage {
            detail: format!("`--guidance {other}`: truth or estimate"),
        }),
    }
}
