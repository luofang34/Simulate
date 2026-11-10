# `ironbird` CLI & JSON Schema Reference

## 1. `ironbird` Command-Line Interface

`cargo run --release -p ironbird -- --mode <MODE> [OPTIONS]`

Arguments are parsed by hand in `apps/ironbird/src/cli.rs`. There is no `--help`: an unknown flag or mode exits with an error that lists every valid mode. Logging follows `RUST_LOG` (default `info`).

### Execution Modes (`--mode <MODE>`)

| Mode | Category | Description |
|---|---|---|
| `tier0-x500` | Tier-0 XIL | In-process `AviateDirectRunner` driving `X500RigidPlant` (`400 Hz` RK4) hover/climb (`default`). |
| `tier0-c172` | Tier-0 XIL | In-process `AviateDirectRunner` driving built-in `C172` (`120 Hz` JSBSim) cruise. |
| `tier0-dir` | Tier-0 XIL | In-process `AviateDirectRunner` driving any on-disk `syspack` directory (`--package-dir <path>`). |
| `calibrate-c172` | Calibration | Closed-loop POH + empirical calibration of built-in `C172` (`N642SP`). |
| `calibrate-dir` | Calibration | Closed-loop POH + empirical calibration of an on-disk `syspack` (`--package-dir <path>`). |
| `estimate-c172` | Design Est. | Closed-loop V-speed, ceiling, glide, range, and multi-altitude mission estimation of built-in `C172`. |
| `estimate-dir` | Design Est. | Closed-loop V-speed, ceiling, glide, range, and mission estimation of an on-disk `syspack` (`--package-dir <path>`). |
| `trade-c172` | Trade Study | Airframe & powerplant trade study (`S_ref`, `AR`, `CD0`, `Power`, `BSFC`, `Fuel`) on built-in `C172`. |
| `trade-dir` | Trade Study | Airframe & powerplant trade study on an on-disk `syspack` (`--package-dir <path>`). |
| `ace-bridge` | Co-Design | Flies the package `ace-export` wrote (`--package-dir <dir>`), compares it with the design it came from (`--ace-vehicle <vehicle.json>`), and emits an `aex edit` command carrying the measured values. Refuses a package from another design or with another engine count. |
| `mavlink-tcp-listen` | Tier-2 XIL | Built-in `C172` behind a MAVLink v2 HIL TCP server on `--bind`; blocks until one client (e.g. PX4 SITL) connects. |
| `mavlink-tcp-connect` | Tier-2 XIL | Built-in `C172` behind a MAVLink v2 HIL TCP client connecting to `--peer`. |
| `mavlink-udp` | Tier-2 XIL | Built-in `C172` behind a MAVLink v2 HIL UDP endpoint bound to `--bind` and sending to `--peer`. |
| `betaflight-udp` | Tier-2 XIL | `X500RigidPlant` behind a Betaflight SITL UDP endpoint: `144 B` FDM packets to `--peer`, servo packets received on `--bind`. |

### CLI Options

| Flag | Default | Description |
|---|---|---|
| `--package-dir <PATH>` | `None` | Path to a `syspack` directory (required for `tier0-dir`, `calibrate-dir`, `estimate-dir`, `trade-dir`, and `ace-bridge`). |
| `--ace-vehicle <PATH>` | `None` | The `ace-rs` vehicle.json the `--package-dir` package was exported from (required for `--mode ace-bridge`). |
| `--export-syspack <DIR>` | `None` | Writes a complete SHA-256-verified `syspack` directory with synthesized `performance/airspeed_envelope.json` (`--mode ace-bridge`). |
| `--report-out <PATH>` | `None` | Writes the Markdown calibration, sensitivity, design-estimation, trade-study, or `ace-rs` co-design report to `<PATH>`. |
| `--sensitivity` | `false` | Appends finite-difference Jacobian & damped Gauss-Newton parameter identification (`calibrate-*` modes). |
| `--duration-s <F64>` | `10.0` | Simulated flight duration in seconds (`tier0-*` and Tier-2 modes). |
| `--bind <ADDR:PORT>` | `127.0.0.1:4560` | Local socket bind address for Tier-2 modes (the servo listen port for `betaflight-udp`, normally `127.0.0.1:9002`). |
| `--peer <ADDR:PORT>` | `127.0.0.1:14550` | Remote peer socket address for Tier-2 client/UDP modes (Betaflight's FDM port for `betaflight-udp`, normally `127.0.0.1:9003`). |
| `--lockstep-us <U64>` | `2500` | Lockstep barrier processing delay in microseconds (`HitlSyncMode::LockstepBarrier`); the run fails if a reply takes longer than 2 s of wall-clock time. |
| `--realtime` | `false` | Disables lockstep barrier and paces simulation to wall-clock (`HitlSyncMode::RealTimeStamped`). |

---

## 2. Performance Asset JSON Schemas

All three optional performance assets reside under `performance/` inside a fixed-wing `syspack` and must be listed in `syspack.json` `assets` with their exact `sha256` and `size_bytes`.

### `performance/airspeed_envelope.json` (`sim_pack::AirspeedEnvelope`)

| Field | Type | Required | Description |
|---|---|:---:|---|
| `type_designator` | `String` | Yes | Aircraft designator (e.g. `"C172"`). |
| `reference_document` | `String` | Yes | POH or engineering source description. |
| `reference_mass_kg` | `f64` | Yes | Reference gross mass ($\text{kg}$, $>0$) for $V_A$ and $V_G$ scaling. |
| `positive_limit_g` | `f64` | Yes | Positive limit load factor ($\ge 1.0$, e.g. `3.8`). |
| `negative_limit_g` | `f64` | Yes | Negative limit load factor ($\le 0.0$, e.g. `-1.52`). |
| `vs0_kt` | `f32` | Yes | Power-off stall speed in landing configuration ($\text{kt}$, bottom of white arc). |
| `vs1_kt` | `f32` | Yes | Power-off stall speed in clean configuration ($\text{kt}$, bottom of green arc). |
| `vfe_kt` | `f32` | Yes | Full-flap maximum extended speed ($\text{kt}$, top of white arc). |
| `vfe_takeoff_kt` | `Option<f32>` | No | Partial/takeoff-flap maximum speed ($\text{kt}$). |
| `vr_kt` | `Option<f32>` | No | Rotation speed bug ($\text{kt}$). |
| `vx_kt` | `Option<f32>` | No | Sea-level best angle-of-climb bug ($\text{kt}$, $\le V_Y$). |
| `vy_kt` | `Option<f32>` | No | Sea-level best rate-of-climb bug ($\text{kt}$). |
| `vg_kt` | `Option<f32>` | No | Best power-off glide speed bug ($\text{kt}$). |
| `va_ref_kt` | `Option<f32>` | No | Maneuvering speed at `reference_mass_kg` ($\text{kt}$, $> V_{S1}$). |
| `vref_kt` | `Option<f32>` | No | Final approach reference bug ($\text{kt}$). |
| `vno_kt` | `f32` | Yes | Maximum structural cruising speed ($\text{kt}$, top of green arc). |
| `vne_kt` | `f32` | Yes | Never-exceed speed ($\text{kt}$, red line). |

### `performance/pitot_static_calibration.json` (`sim_pack::PitotStaticCalibration`)

| Field | Type | Required | Description |
|---|---|:---:|---|
| `type_designator` | `String` | Yes | Aircraft designator. |
| `reference_document` | `String` | Yes | POH Position Error Correction source. |
| `curves` | `Vec<PitotStaticCurve>` | Yes | Sorted by ascending `flap_norm` $\in [0.0, 1.0]$, each with `flap_deg`, `flap_norm`, and strictly ascending `points: [{ "kias_kt", "kcas_kt" }]`. |

### `performance/poh_benchmarks.json` (`sim_pack::CalibrationSuite`)

| Field | Type | Required | Description |
|---|---|:---:|---|
| `type_designator` | `String` | Yes | Aircraft designator. |
| `reference_document` | `String` | Yes | Authoritative POH / TCDS / flight-test reference. |
| `rated_bhp` | `Option<f64>` | No | Rated sea-level engine horsepower ($\text{BHP}$) for `%BHP` calculation. |
| `tail_profile` | `Option<TailProfile>` | No | Registration, S/N, model year, home base, powerplant, avionics, empty mass/CG, and usable fuel ($\text{kg}$). |
| `tolerances` | `CalibrationTolerances` | Yes | Acceptance limits: `stall_kt`, `climb_fpm`, `cruise_ktas`, `cruise_gph`, `cruise_pct_bhp`. |
| `static_runup` | `Option<StaticRunupBenchmark>` | No | `{ "pressure_alt_ft", "min_rpm", "max_rpm" }`. |
| `stalls` | `Vec<StallBenchmark>` | Yes* | `{ "label", "flap_deg", "flap_norm", "bank_deg", "gross_mass_kg", "target_kias", "target_kcas" }`. |
| `climb_points` | `Vec<ClimbBenchmark>` | Yes* | `{ "pressure_alt_ft", "gross_mass_kg", "climb_kcas", "target_kias", "lean_best_power", "target_fpm" }`. |
| `cruise_points` | `Vec<CruiseBenchmark>` | Yes* | `{ "pressure_alt_ft", "gross_mass_kg", "target_rpm", "fixed_throttle", "target_pct_bhp", "target_ktas", "target_gph" }`. |
| `empirical_segments` | `Vec<EmpiricalFlightSegment>` | No | Recorded flight segments replayed in closed loop (`LevelCruiseRpm`, `LevelCruiseThrottle`, `FullThrottleClimbKcas`). |
