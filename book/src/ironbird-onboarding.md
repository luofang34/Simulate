# Onboarding an Aircraft into `ironbird`

`ironbird` (`apps/ironbird`) supports two complementary ways to onboard a new aircraft:

1. **Zero-Recompile Directory Onboarding (`--package-dir`)** — load any `syspack` directory directly from disk for closed-loop Tier-0 flight, performance/range estimation, or POH calibration.
2. **Built-In Workspace Registration** — embed the `syspack` bytes into `sim-pack` so the aircraft loads without filesystem access (including `wasm32` browser builds), then wire it into `ironbird` named modes and, if it should be flyable interactively, the `simulate-app` free-flight roster. `simulate-app` only flies built-in packages; on-disk directories are an `ironbird` feature.

---

## Stage 1: Zero-Recompile Directory Onboarding

As soon as your `vehicles/<my-aircraft>/syspack.json` and `fdm/<model>.xml` are in place with valid SHA-256 digests, you can exercise all three `ironbird` pipelines without editing any Rust source files:

### 1. Estimate Baseline V-Speeds, Ceiling, Glide & Range (`estimate-dir`)

```sh
cargo run --release -p ironbird -- \
  --mode estimate-dir \
  --package-dir vehicles/<my-aircraft> \
  --report-out <my-aircraft>_design.md
```

This runs `sim_pack::estimate_design_performance`, sweeping stalls, sea-level $V_X/V_Y$, multi-altitude climb, power-off glide, and cruise fuel/range polars in closed loop.

### 2. Fly in Closed-Loop Tier-0 `Aviate` XIL (`tier0-dir`)

```sh
cargo run --release -p ironbird -- \
  --mode tier0-dir \
  --package-dir vehicles/<my-aircraft> \
  --duration-s 30
```

Under the hood, `ironbird`:

1. Loads and verifies the directory via `LoadedSyspack::load(&DirectoryPackageSource::new(dir))`.
2. Determines the nominal cruise trim speed from `package.airspeed_envelope()` (`0.40 * vno_kt` in $\text{m/s}$, or `50.0 m/s` when no envelope is present).
3. Instantiates `JsbsimPlant::from_package(&package, origin, speed_m_s)` at `120 Hz` (`JSBSIM_DEFAULT_PERIOD`).
4. Couples the plant to `AviateDirectRunner` (`build_direct_fixed_wing_kernel()`) inside a `sim_core::SimSession` with a `30 Hz` optical exposure channel (`OPTICAL_CHANNEL`), logging executed physics steps, simulated and wall-clock seconds, real-time speed-up, and final geodetic altitude.

### 3. Run Closed-Loop POH & Empirical Calibration (`calibrate-dir`)

Once `performance/poh_benchmarks.json` (and optionally `performance/pitot_static_calibration.json`) are added to `syspack.json`:

```sh
cargo run --release -p ironbird -- \
  --mode calibrate-dir \
  --package-dir vehicles/<my-aircraft> \
  --sensitivity \
  --report-out <my-aircraft>_calibration.md
```

---

## Stage 2: Promoting a Package to a Built-In Vehicle

When a vehicle package is ready to ship inside the native and `wasm32` binaries (like `vehicles/c172s`, `vehicles/c172p`, and `vehicles/p38g`):

### 1. Embed the Package in `crates/sim-pack/src/builtin.rs`

Add a `SlicePackageSource` table using `include_bytes!` so the package loads identically on native and `wasm32-unknown-unknown` targets with zero filesystem access:

```rust
const MY_AIRCRAFT_FILES: &[(&str, &[u8])] = &[
    (
        "fdm/my_aircraft.xml",
        include_bytes!("../../../vehicles/my-aircraft/fdm/my_aircraft.xml"),
    ),
    (
        "performance/airspeed_envelope.json",
        include_bytes!("../../../vehicles/my-aircraft/performance/airspeed_envelope.json"),
    ),
    (
        "performance/poh_benchmarks.json",
        include_bytes!("../../../vehicles/my-aircraft/performance/poh_benchmarks.json"),
    ),
];
```

Expose `builtin_my_aircraft_source()` and `builtin_my_aircraft_package() -> Result<LoadedSyspack, PackError>` from `sim_pack`.

### 2. Add a Variant to `ironbird::FixedWingVehicle`

In `apps/ironbird/src/plant.rs`, add your variant to `FixedWingVehicle` with its `nominal_cruise_speed_m_s()`, and wire it into `JsbsimPlant::new_cruise` and `run_tier0_fixed_wing_cruise`. The library entry points (`run_tier0_fixed_wing_cruise`, `run_tier2_mavlink_fixed_wing`) then accept it. The CLI modes in `apps/ironbird/src/cli.rs` name their vehicle explicitly (the Tier-2 MAVLink modes fly the C172), so add a `--mode` there for the new aircraft and list it in the [CLI reference](cli-and-schema-reference.md).

### 3. Add Regression Tests in `crates/sim-pack/tests/`

Add an integration test verifying:

1. `builtin_my_aircraft_package()` loads without error and its embedded `SlicePackageSource` matches `DirectoryPackageSource::new("vehicles/my-aircraft")` byte-for-byte.
2. `evaluate_package(&pkg)` passes all stall, climb, cruise, and empirical benchmarks (`assert!(report.all_passed)`).
3. `estimate_design_performance(&pkg)` produces physically ordered V-speeds and positive service ceiling and range.
