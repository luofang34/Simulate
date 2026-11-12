//! Command-line parser for `link-replay`.

use std::path::PathBuf;

use crate::ReplayError;

#[cfg(test)]
mod tests;

/// Brief CLI usage string shown on [`ReplayError::Usage`].
pub const USAGE: &str = "usage: link-replay [<link> | --link <url|#v=...> | --scenario-file <path> | \
--preset <zaragoza|vladivostok|spoof-ramp|spoof-step|gnss-recovery|lost-pos-orbit>] \
[--fly <set>] [--duration-s <sec>] [--until-s <sec>] [--hold-s <sec>] \
[--sample-hz <hz>] [--seed <n> | --seeds <a..=b>] [--no-camera] \
[--camera-backend <wgpu|cpu>] [--offline] [--cache-dir <dir>] [--rnp-nm <nm>] \
[--out <trace.ndjson>] [--summary <summary.json>]";

/// Built-in deterministic test scenario presets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PresetName {
    /// Zaragoza visual + GNSS daytime corridor (`800x600` nadir terrain camera).
    Zaragoza,
    /// Vladivostok / Ussuriysk visual + GNSS corridor (`800x600` nadir camera).
    Vladivostok,
    /// Coherent GNSS position+velocity ramp spoofing window followed by recovery.
    SpoofRamp,
    /// Instantaneous GNSS position step-offset spoofing window followed by recovery.
    SpoofStep,
    /// Visual + GNSS flight with mid-flight spoofing and post-spoof GNSS recovery.
    GnssRecovery,
    /// Complete GNSS loss driving dead-reckoning uncertainty into protective orbit hold.
    LostPosOrbit,
}

impl PresetName {
    /// Parses a CLI preset identifier.
    pub fn parse(name: &str) -> Result<Self, ReplayError> {
        match name {
            "zaragoza" => Ok(Self::Zaragoza),
            "vladivostok" => Ok(Self::Vladivostok),
            "spoof-ramp" => Ok(Self::SpoofRamp),
            "spoof-step" => Ok(Self::SpoofStep),
            "gnss-recovery" => Ok(Self::GnssRecovery),
            "lost-pos-orbit" => Ok(Self::LostPosOrbit),
            _ => Err(ReplayError::Usage(format!(
                "unknown preset `{name}` (expected zaragoza, vladivostok, spoof-ramp, \
                 spoof-step, gnss-recovery, or lost-pos-orbit)"
            ))),
        }
    }

    /// Canonical kebab-case preset name.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Zaragoza => "zaragoza",
            Self::Vladivostok => "vladivostok",
            Self::SpoofRamp => "spoof-ramp",
            Self::SpoofStep => "spoof-step",
            Self::GnssRecovery => "gnss-recovery",
            Self::LostPosOrbit => "lost-pos-orbit",
        }
    }
}

/// Source of the flight scenario to execute.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InputSource {
    /// Shareable `#v=2` link or full page URL.
    Link(String),
    /// Structured JSON scenario file on disk.
    ScenarioFile(PathBuf),
    /// Named built-in preset.
    Preset(PresetName),
}

/// Offscreen terrain-camera rendering backend.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CameraBackend {
    /// Headless `wgpu` offscreen terrain & orthoimagery pass (with CPU fallback if no GPU adapter exists).
    #[default]
    Wgpu,
    /// Deterministic software raycaster (`CameraWorld`).
    Cpu,
}

impl CameraBackend {
    fn parse(text: &str) -> Result<Self, ReplayError> {
        match text {
            "wgpu" => Ok(Self::Wgpu),
            "cpu" => Ok(Self::Cpu),
            _ => Err(ReplayError::Usage(format!(
                "unknown --camera-backend `{text}` (expected `wgpu` or `cpu`)"
            ))),
        }
    }
}

/// Parsed command-line configuration for `link-replay`.
#[derive(Debug, Clone, PartialEq)]
pub struct ReplayArgs {
    /// Scenario input source (`--link`, `--scenario-file`, or `--preset`).
    pub source: InputSource,
    /// Optional steering navigator override (`--fly`).
    pub fly: Option<String>,
    /// Maximum simulated route time to fly, seconds (`--duration-s` / `--until-s`).
    pub duration_s: f64,
    /// Maximum time to stay in terminal holding before stopping, seconds (`--hold-s`).
    pub hold_s: f64,
    /// Telemetry recording cadence, Hz (`--sample-hz`, e.g. `1`, `10`, `30`).
    pub sample_hz: f64,
    /// Optional explicit seed list for single-seed or Monte Carlo batch runs.
    pub seeds: Vec<u64>,
    /// Disables terrain-camera package sealing and frame rendering (`--no-camera`).
    pub no_camera: bool,
    /// Offscreen terrain-camera renderer backend (`--camera-backend`).
    pub camera_backend: CameraBackend,
    /// Forbids network tile downloads and serves exclusively from local disk cache (`--offline`).
    pub offline: bool,
    /// Optional override for `SIMULATE_TILE_CACHE` directory (`--cache-dir`).
    pub cache_dir: Option<PathBuf>,
    /// Optional explicit RNP containment limit in nautical miles (`--rnp-nm`).
    pub rnp_limit_nm: Option<f64>,
    /// Optional NDJSON per-sample output path (`--out`).
    pub out: Option<PathBuf>,
    /// Optional run-level JSON statistical report path (`--summary`).
    pub summary: Option<PathBuf>,
}

/// Parses command-line tokens (excluding `argv[0]`) into [`ReplayArgs`].
///
/// # Errors
///
/// Returns [`ReplayError::Usage`] when required values are missing, conflicting
/// scenario inputs are supplied, or numeric parameters are invalid.
pub fn parse_args(args: impl IntoIterator<Item = String>) -> Result<ReplayArgs, ReplayError> {
    let mut source: Option<InputSource> = None;
    let mut fly = None;
    let mut duration_s = 120.0;
    let mut hold_s = 600.0;
    let mut sample_hz = 1.0;
    let mut seeds = Vec::new();
    let mut no_camera = false;
    let mut camera_backend = CameraBackend::Wgpu;
    let mut offline = false;
    let mut cache_dir = None;
    let mut rnp_limit_nm = None;
    let mut out = None;
    let mut summary = None;

    let mut iter = args.into_iter();
    while let Some(arg) = iter.next() {
        let mut next_val = || {
            iter.next()
                .ok_or_else(|| ReplayError::Usage(format!("{arg} requires a value")))
        };
        match arg.as_str() {
            "--link" => set_source(&mut source, InputSource::Link(next_val()?))?,
            "--scenario-file" => {
                set_source(
                    &mut source,
                    InputSource::ScenarioFile(PathBuf::from(next_val()?)),
                )?;
            }
            "--preset" => {
                set_source(
                    &mut source,
                    InputSource::Preset(PresetName::parse(&next_val()?)?),
                )?;
            }
            "--fly" => fly = Some(next_val()?),
            "--duration-s" | "--until-s" => duration_s = positive_f64(&arg, &next_val()?)?,
            "--hold-s" => hold_s = positive_f64(&arg, &next_val()?)?,
            "--sample-hz" => sample_hz = sample_rate_hz(&next_val()?)?,
            "--seed" => seeds.push(parse_u64("--seed", &next_val()?)?),
            "--seeds" => seeds.extend(parse_seed_range(&next_val()?)?),
            "--no-camera" => no_camera = true,
            "--camera-backend" => camera_backend = CameraBackend::parse(&next_val()?)?,
            "--offline" => offline = true,
            "--cache-dir" => cache_dir = Some(PathBuf::from(next_val()?)),
            "--rnp-nm" => rnp_limit_nm = Some(positive_f64("--rnp-nm", &next_val()?)?),
            "--out" => out = Some(PathBuf::from(next_val()?)),
            "--summary" => summary = Some(PathBuf::from(next_val()?)),
            _ if arg.starts_with("--") => {
                return Err(ReplayError::Usage(format!("unknown option `{arg}`")));
            }
            _ => set_source(&mut source, InputSource::Link(arg))?,
        }
    }

    let source = source.ok_or_else(|| {
        ReplayError::Usage(
            "missing scenario input: pass <link>, --link, --scenario-file, or --preset".to_owned(),
        )
    })?;

    Ok(ReplayArgs {
        source,
        fly,
        duration_s,
        hold_s,
        sample_hz,
        seeds,
        no_camera,
        camera_backend,
        offline,
        cache_dir,
        rnp_limit_nm,
        out,
        summary,
    })
}

/// Extracts the URL fragment following `#`, or returns `link` verbatim if it
/// already starts without a page prefix.
#[must_use]
pub fn fragment(link: &str) -> &str {
    link.split_once('#').map_or(link, |(_, frag)| frag)
}

fn set_source(slot: &mut Option<InputSource>, next: InputSource) -> Result<(), ReplayError> {
    if slot.is_some() {
        return Err(ReplayError::Usage(
            "multiple scenario inputs supplied; choose one of <link>, --link, --scenario-file, or --preset"
                .to_owned(),
        ));
    }
    *slot = Some(next);
    Ok(())
}

fn positive_f64(flag: &str, text: &str) -> Result<f64, ReplayError> {
    text.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite() && *v > 0.0)
        .ok_or_else(|| ReplayError::Usage(format!("{flag}: `{text}` is not a positive number")))
}

fn sample_rate_hz(text: &str) -> Result<f64, ReplayError> {
    let hz = positive_f64("--sample-hz", text)?;
    if hz > 240.0 {
        return Err(ReplayError::Usage(format!(
            "--sample-hz `{text}` exceeds 240 Hz flight-controller rate"
        )));
    }
    Ok(hz)
}

fn parse_u64(flag: &str, text: &str) -> Result<u64, ReplayError> {
    text.parse::<u64>()
        .map_err(|_| ReplayError::Usage(format!("{flag}: `{text}` is not a valid u64 seed")))
}

fn parse_seed_range(spec: &str) -> Result<Vec<u64>, ReplayError> {
    const MAX_BATCH_SEEDS: u64 = 10_000;
    if let Some((a, b)) = spec.split_once("..=") {
        let start = parse_u64("--seeds", a)?;
        let end = parse_u64("--seeds", b)?;
        if end < start || end.saturating_sub(start) >= MAX_BATCH_SEEDS {
            return Err(ReplayError::Usage(format!("invalid seed range `{spec}`")));
        }
        return Ok((start..=end).collect());
    }
    if let Some((a, b)) = spec.split_once("..") {
        let start = parse_u64("--seeds", a)?;
        let end = parse_u64("--seeds", b)?;
        if end <= start || end.saturating_sub(start) > MAX_BATCH_SEEDS {
            return Err(ReplayError::Usage(format!("invalid seed range `{spec}`")));
        }
        return Ok((start..end).collect());
    }
    spec.split(',')
        .map(|part| parse_u64("--seeds", part.trim()))
        .collect()
}
