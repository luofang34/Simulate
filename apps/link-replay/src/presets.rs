//! Built-in flight presets, structured JSON scenario loading, `--no-camera`
//! adaptation, and deterministic Monte Carlo seed injection.

use std::fs;
use std::path::Path;

use navigate_data::Sha256Digest;
use serde::{Deserialize, Serialize};
use sim_chain_trial::{
    ChainTrialSpec, DeviceOutage, HoldSpec, SpoofKind, SpoofWindowSpec, Steering,
};
use sim_link::{SimulationLink, decode_fragment};
use sim_link_host::{HostedRequest, chain_spec, hosted_request};
use sim_navigate::NavMeans;
use sim_navigate::trial::SeedSpec;

use crate::ReplayError;
use crate::cli::{InputSource, PresetName, fragment};

/// Zaragoza daytime visual + GNSS corridor (`800x600`, `67.4 deg` HFOV, `1 Hz`).
pub const ZARAGOZA_LINK: &str = "v=2&r=IF,LEZG:41.666242/-1.041553;TF,Z1:41.75/-0.8;\
TF,Z_2_A:41.6/-0.62&h=1981.2&hd=msl&start=41.666250,-1.041550,,75.0&\
t=20260615T120000Z&veh=c172&seed=1&nav=ins.baro.visual;ins.baro.gnss;ins.baro&\
fly=ins.baro.visual&cam=800x600,67.4,90,1&go=1&mb=200";

/// Vladivostok / Ussuriysk visual + GNSS corridor matching the reference session flight.
pub const VLADIVOSTOK_LINK: &str = "v=2&r=IF,WP1:43.7966/131.9564;TF,WP2:43.98/131.82;\
TF,WP3:44.06/131.76&h=2133.6&hd=msl&t=20260615T030000Z&veh=c172&seed=1&\
nav=ins.baro.visual;ins.baro.gnss;ins.baro&fly=ins.baro.visual&\
cam=800x600,67.4,90,1&go=1&mb=200";

const ZARAGOZA_RECOVERY_LINK: &str = "v=2&r=IF,LEZG:41.666242/-1.041553;TF,Z1:41.75/-0.8;\
TF,Z_2_A:41.6/-0.62&h=1981.2&hd=msl&start=41.666250,-1.041550,,75.0&\
t=20260615T120000Z&veh=c172&seed=1&\
nav=ins.baro.gnss.visual;ins.baro.visual;ins.baro.gnss;ins.baro&\
fly=ins.baro.gnss.visual&cam=800x600,67.4,90,1&go=1&mb=200";

const ZARAGOZA_GNSS_LINK: &str = "v=2&r=IF,ZA1:41.7/-1.05;TF,ZA2:41.7/-0.8;TF,ZA3:41.8/-0.8&\
h=2000.0&hd=msl&start=41.700000,-1.070000,,90.0&t=20260615T120400Z&veh=c172&seed=1&\
nav=ins.baro.gnss;ins.baro&fly=ins.baro.gnss&go=1&mb=200";

const LOST_POS_ORBIT_LINK: &str = "v=2&r=IF,ZA1:41.7/-1.05;TF,ZA2:41.7/-0.8;TF,ZA3:41.8/-0.8&\
h=2000.0&hd=msl&start=41.700000,-1.070000,,90.0&t=20260615T120400Z&veh=c172&seed=1&\
nav=ins.baro.gnss;ins.baro&fly=ins.baro.gnss&out=gnss:5-&go=1&mb=200";

/// Flexible JSON scenario specification accepted by `--scenario-file`.
///
/// A scenario file can either be a raw [`ChainTrialSpec`] JSON object or this
/// envelope combining a `#v=2` `link` / `preset` with structured multi-window
/// spoofing, sensor outages, RNP limits, or steering overrides.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioFile {
    /// Optional human-readable scenario identifier.
    #[serde(default)]
    pub name: Option<String>,
    /// Optional `#v=2` link or full URL providing the base route & vehicle.
    #[serde(default)]
    pub link: Option<String>,
    /// Optional built-in preset name providing the base route & vehicle.
    #[serde(default)]
    pub preset: Option<String>,
    /// Optional inline [`ChainTrialSpec`] when no `link` or `preset` is used.
    #[serde(default)]
    pub spec: Option<ChainTrialSpec>,
    /// Optional steering navigator override.
    #[serde(default)]
    pub fly: Option<String>,
    /// Additional or replacement adversarial GNSS spoofing windows.
    #[serde(default)]
    pub spoof_windows: Vec<SpoofWindowSpec>,
    /// Additional device outages.
    #[serde(default)]
    pub outages: Vec<DeviceOutage>,
    /// Optional terminal holding pattern override.
    #[serde(default)]
    pub hold: Option<HoldSpec>,
    /// Optional default RNP containment limit in nautical miles.
    #[serde(default)]
    pub rnp_limit_nm: Option<f64>,
}

/// Prepared scenario before sealing map packages.
#[derive(Debug, Clone)]
pub struct PreparedScenario {
    /// Label describing the scenario source.
    pub label: String,
    /// Decoded link and hosted request when backed by a `#v=2` link.
    pub link_bundle: Option<(SimulationLink, HostedRequest)>,
    /// Direct [`ChainTrialSpec`] when loaded without a `#v=2` link.
    pub direct_spec: Option<ChainTrialSpec>,
    /// Spoofing windows to attach onto the constructed [`ChainTrialSpec`].
    pub spoof_windows: Vec<SpoofWindowSpec>,
    /// Extra outages to attach onto the constructed [`ChainTrialSpec`].
    pub extra_outages: Vec<DeviceOutage>,
    /// Optional steering override from scenario file.
    pub fly_override: Option<String>,
    /// Optional terminal hold override from scenario file.
    pub hold_override: Option<HoldSpec>,
    /// Optional RNP limit override from scenario file.
    pub rnp_limit_nm: Option<f64>,
}

impl PreparedScenario {
    /// Loads and validates `source`.
    ///
    /// # Errors
    ///
    /// Returns [`ReplayError`] if the link, preset, or JSON scenario file is invalid.
    pub fn load(source: &InputSource) -> Result<Self, ReplayError> {
        match source {
            InputSource::Link(raw) => Self::from_link_str("link".to_owned(), raw),
            InputSource::Preset(preset) => Self::from_preset(*preset),
            InputSource::ScenarioFile(path) => Self::from_file(path),
        }
    }

    fn from_link_str(label: String, raw: &str) -> Result<Self, ReplayError> {
        let link = decode_fragment(fragment(raw))?;
        let hosted = hosted_request(&link)?;
        Ok(Self {
            label,
            link_bundle: Some((link, hosted)),
            direct_spec: None,
            spoof_windows: Vec::new(),
            extra_outages: Vec::new(),
            fly_override: None,
            hold_override: None,
            rnp_limit_nm: None,
        })
    }

    fn from_preset(preset: PresetName) -> Result<Self, ReplayError> {
        let (frag, spoof_windows, fly_override) = match preset {
            PresetName::Zaragoza => (ZARAGOZA_LINK, Vec::new(), None),
            PresetName::Vladivostok => (VLADIVOSTOK_LINK, Vec::new(), None),
            PresetName::SpoofRamp => (
                ZARAGOZA_GNSS_LINK,
                vec![SpoofWindowSpec {
                    start_s: 12.0,
                    end_s: Some(55.0),
                    kind: SpoofKind::Ramp,
                    offset_enu_m: [0.0, 0.0, 0.0],
                    rate_enu_mps: [14.0, 4.0, 0.0],
                    velocity_enu_mps: None,
                }],
                None,
            ),
            PresetName::SpoofStep => (
                ZARAGOZA_GNSS_LINK,
                vec![SpoofWindowSpec {
                    start_s: 12.0,
                    end_s: Some(50.0),
                    kind: SpoofKind::Offset,
                    offset_enu_m: [420.0, 180.0, 0.0],
                    rate_enu_mps: [0.0, 0.0, 0.0],
                    velocity_enu_mps: None,
                }],
                None,
            ),
            PresetName::GnssRecovery => (
                ZARAGOZA_RECOVERY_LINK,
                vec![SpoofWindowSpec {
                    start_s: 12.0,
                    end_s: Some(45.0),
                    kind: SpoofKind::Ramp,
                    offset_enu_m: [80.0, 0.0, 0.0],
                    rate_enu_mps: [16.0, 0.0, 0.0],
                    velocity_enu_mps: None,
                }],
                None,
            ),
            PresetName::LostPosOrbit => (LOST_POS_ORBIT_LINK, Vec::new(), None),
        };
        let mut prepared = Self::from_link_str(preset.as_str().to_owned(), frag)?;
        prepared.spoof_windows = spoof_windows;
        prepared.fly_override = fly_override;
        Ok(prepared)
    }

    fn from_file(path: &Path) -> Result<Self, ReplayError> {
        let display = path.display().to_string();
        let text = fs::read_to_string(path).map_err(|source| ReplayError::Io {
            path: display.clone(),
            source,
        })?;
        if let Ok(spec) = serde_json::from_str::<ChainTrialSpec>(&text) {
            return Ok(Self {
                label: display,
                link_bundle: None,
                direct_spec: Some(spec),
                spoof_windows: Vec::new(),
                extra_outages: Vec::new(),
                fly_override: None,
                hold_override: None,
                rnp_limit_nm: None,
            });
        }
        let file: ScenarioFile =
            serde_json::from_str(&text).map_err(|source| ReplayError::Json {
                what: display.clone(),
                source,
            })?;
        let label = file.name.unwrap_or(display);
        let mut base = if let Some(raw_link) = &file.link {
            Self::from_link_str(label, raw_link)?
        } else if let Some(preset_str) = &file.preset {
            let mut p = Self::from_preset(PresetName::parse(preset_str)?)?;
            p.label = label;
            p
        } else if let Some(spec) = file.spec {
            Self {
                label,
                link_bundle: None,
                direct_spec: Some(spec),
                spoof_windows: Vec::new(),
                extra_outages: Vec::new(),
                fly_override: None,
                hold_override: None,
                rnp_limit_nm: None,
            }
        } else {
            return Err(ReplayError::Usage(
                "scenario JSON must provide `link`, `preset`, or `spec`".to_owned(),
            ));
        };
        if !file.spoof_windows.is_empty() {
            base.spoof_windows = file.spoof_windows;
        }
        base.extra_outages.extend(file.outages);
        if file.fly.is_some() {
            base.fly_override = file.fly;
        }
        if file.hold.is_some() {
            base.hold_override = file.hold;
        }
        if file.rnp_limit_nm.is_some() {
            base.rnp_limit_nm = file.rnp_limit_nm;
        }
        Ok(base)
    }

    /// Builds the concrete [`ChainTrialSpec`] once optional map sealing completes.
    ///
    /// # Errors
    ///
    /// Returns [`ReplayError`] if link-to-chain conversion fails.
    pub fn build_spec(
        &self,
        package_identity: Option<Sha256Digest>,
        cli_fly: Option<&str>,
        no_camera: bool,
    ) -> Result<ChainTrialSpec, ReplayError> {
        let mut spec = if let Some((link, hosted)) = &self.link_bundle {
            if no_camera {
                let mut stripped = hosted.clone();
                strip_visual_from_hosted(&mut stripped);
                chain_spec(link, &stripped, None)?
            } else {
                chain_spec(link, hosted, package_identity)?
            }
        } else if let Some(direct) = &self.direct_spec {
            let mut cloned = direct.clone();
            if no_camera {
                strip_visual_from_spec(&mut cloned);
            }
            cloned
        } else {
            return Err(ReplayError::Usage("empty prepared scenario".to_owned()));
        };

        if !self.spoof_windows.is_empty() {
            spec.spoof_windows.clone_from(&self.spoof_windows);
        }
        spec.outages.extend(self.extra_outages.iter().cloned());
        if let Some(hold) = self.hold_override {
            spec.hold = Some(hold);
        }
        if let Some(fly) = cli_fly.or(self.fly_override.as_deref()) {
            spec.steering = Steering::Set(fly.to_owned());
        } else if no_camera {
            ensure_valid_steering(&mut spec);
        }
        Ok(spec)
    }
}

/// Applies a deterministic master `seed` across all sensor noise streams in `spec`.
pub fn apply_seed(spec: &mut ChainTrialSpec, seed: u64) {
    let has_cam = spec.request.terrain_camera.is_some();
    spec.request.seeds = SeedSpec::from_master(seed);
    if !has_cam {
        spec.request.seeds.terrain_camera = None;
    }
}

fn strip_visual_from_hosted(hosted: &mut HostedRequest) {
    hosted.camera = None;
    hosted.trial.terrain_camera = None;
    hosted.trial.seeds.terrain_camera = None;
    hosted.trial.maps = None;
    strip_visual_sets(&mut hosted.trial.sensor_sets);
    if !hosted
        .trial
        .sensor_sets
        .iter()
        .any(|s| s.name == hosted.steering_set)
    {
        let fallback = hosted
            .trial
            .sensor_sets
            .iter()
            .find(|s| s.means.contains(&NavMeans::Gnss))
            .or_else(|| hosted.trial.sensor_sets.first())
            .map(|s| s.name.clone())
            .unwrap_or_default();
        hosted.steering_set = fallback;
    }
}

fn strip_visual_from_spec(spec: &mut ChainTrialSpec) {
    spec.request.terrain_camera = None;
    spec.request.seeds.terrain_camera = None;
    spec.request.maps = None;
    strip_visual_sets(&mut spec.request.sensor_sets);
    ensure_valid_steering(spec);
}

fn strip_visual_sets(sets: &mut Vec<sim_navigate::trial::SensorSetSpec>) {
    sets.retain(|set| !set.means.contains(&NavMeans::Visual));
    if sets.is_empty() {
        sets.push(sim_navigate::trial::SensorSetSpec {
            name: "ins.baro".to_owned(),
            means: vec![NavMeans::DeadReckoning, NavMeans::Barometric],
            availability: Default::default(),
        });
    }
}

fn ensure_valid_steering(spec: &mut ChainTrialSpec) {
    let valid = match &spec.steering {
        Steering::FlightController => true,
        Steering::Set(name) => spec.request.sensor_sets.iter().any(|s| &s.name == name),
    };
    if !valid {
        let fallback = spec
            .request
            .sensor_sets
            .iter()
            .find(|s| s.means.contains(&NavMeans::Gnss))
            .or_else(|| spec.request.sensor_sets.first())
            .map(|s| Steering::Set(s.name.clone()))
            .unwrap_or(Steering::FlightController);
        spec.steering = fallback;
    }
}
