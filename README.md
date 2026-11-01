# Simulate

A flight simulator in Rust that runs natively (Metal, Vulkan, DX12) and in the
browser (WebGPU, with WebGL2 fallback) from one code base.

## Layout

The workspace has four top-level directories:

- `crates/`: the engine and core libraries.
- `vehicles/`: vehicle packages.
- `adapters/`: bridges to the sibling repositories.
- `apps/`: executables.

| Package | Path | Role |
|---|---|---|
| `sim-geo` | `crates/` | WGS84 and lunar geodesy, J2000 frame and Apollo stars, local frames, floating origin, pinhole intrinsics, UTC |
| `sim-atmosphere` | `crates/` | Atmosphere and cloud fields (ICAO, `world-atmosphere` / `world-airmass` bridges, and vacuum) |
| `sim-core` | `crates/` | Integer master clock, the `Block` contract, multi-rate scheduling, vehicle state, 6-DoF rigid-body and JSBSim flight dynamics, crew and hardware-in-the-loop bridges, source-stamped instrument readings, mounted camera and visual-navigation harness |
| `sim-devices` | `crates/` | Shared sensors and buses: ICM-42688 IMU, BMP390 barometer, ARINC 429 channel |
| `sim-xil` | `crates/` | FC-blind external SITL/HITL wire endpoints: MAVLink v2 (`HIL_SENSOR`/`HIL_GPS`/`HIL_ACTUATOR_CONTROLS`) and Betaflight (`144` B FDM/servo UDP + MSP v1/v2) |
| `sim-pack` | `crates/` | Binds verified system packages to the simulator: device blocks, camera mounts, JSBSim resolvers, and the built-in vehicles |
| `sim-input` | `crates/` | Gamepad, joystick, and yoke mapping to pilot controls |
| `sim-scenery` | `crates/` | Elevation, imagery, radiometry, and photogrammetry sources: Web Mercator Terrarium and orthoimagery catalogs, a synthetic corridor, the procedural Apollo 11 site (photogrammetry is a deferred stub) |
| `sim-render` | `crates/` | wgpu renderer: camera-relative terrain on any body, ortho draping, offscreen visible, starfield, and thermal sensor apertures, zero-copy viewport textures, reversed-Z depth |
| `sim-scenario` | `crates/` | The six deterministic scenarios, their terrain, `SimSupervisor` truth-to-belief radio projection, NOTAM-gated `Flightbag` spawn, and the bridge from camera captures to rendered frames |
| `light-trainer` | `vehicles/` | Data-only vehicle package: a JSBSim light trainer with no Rust code of its own |
| `c172` | `vehicles/` | Data-only vehicle package: a piston C172-class aeroplane, exported from its `ace-rs` design |
| `sim-apollo` | `vehicles/apollo-11/` | Apollo 11: the Eagle and Columbia packages under `packages/`, gimballed IMU with gimbal lock, abort sensors, radars, optics, entry monitor, source-monitored cockpit panel |
| `sim-indicate` | `adapters/` | Indicate: aircraft state and tagged frames from flight state, dual-source attitude monitoring, deterministic PFD/HSI framebuffers, and 3D volumetric instrument scenes |
| `sim-maplibre` | `adapters/` | Shared-GPU `maplibre-rs` adapter: strict Dual-World `TruthSceneMap` (OTW + `30 Hz` optical sensor) vs. `BeliefAvionicsMap` (procedural-relief SVS, dirty-gated `15 Hz` chart, 3D volumetric patch export, and companion gesture streaming) |
| `sim-navigate` | `adapters/` | Navigate: master-clock to monotonic-time bridge, visual fix admission through a real visual session, lever-arm removal, acquisition-ordered navigation filter loop, solution scoring against truth |
| `sim-aviate` | `adapters/` | Aviate: Tier 0 in-process `DirectXilBoard` runner (`0` syscalls, `0` allocs/step) and Tier 1 shared-memory `SharedStateV2` seqlock endpoint |
| `sim-pilotage` | `adapters/` | Pilotage: `VehicleAdapter` over kinematic and JSBSim vehicles, verified camera calibration from a package camera, aircraft profile/loading bridges, and `pilotage.bridge.v1` synthetic camera sidecar |
| `ironbird` | `apps/` | Headless multi-tier XIL bench runner across Tier 0 (`aviate-xil-direct`), Tier 1 (`SharedStateV2`), and Tier 2 (`mavlink_hil` / `betaflight`) |
| `simulate-app` | `apps/` | Composite split-viewport cockpit for desktop and browser (`TruthSceneMap` OTW/sensor, `BeliefAvionicsMap` SVS/MFD chart, `Indicate` PFD/HSI/HUD, `SimSupervisor`, JSBSim, and Tier-0 `Aviate`) |

A vehicle package is a `syspack`: a folder holding a `syspack.json` manifest and
the asset files it lists with their SHA-256 and size. Changing an asset means
updating both in the manifest; a package that does not verify will not load.
Built-in and downloaded vehicles share one loader: `vehicles/` packages are
embedded in the binary, others arrive through a `PackageSource` as bytes, and
`sim-pack` binds what `syspack` verified to the simulator. The component tree,
ports, and aerodynamics are `mbd-core`'s.

`vehicles/c172` is generated, not edited. Rebuild it from `ace-rs` with:

```sh
cargo run -p ace-export --example export_syspack -- examples/c172/vehicle.json ../Simulate/vehicles/c172
```

A JSBSim vehicle takes its ignition, mixture, flaps, and brakes as
`SystemControls` latched on `JsbsimFdmBlock`, and starts `ColdAndDark` or
`Running`. A piston aeroplane needs leaning at altitude as a real one does: the
C172 cannot hold 8,000 ft full rich.

These repositories must be checked out next to this one, because the workspace
depends on them by path: `jsbsim-rs`, `mbd`, `world`, `Flightbag`, `Indicate`,
`Navigate`, `Aviate`, `Pilotage`, and `maplibre-rs-experimental`.

## Run

Desktop:

```sh
cargo run -p simulate-app --release
```

Browser (needs [trunk](https://trunkrs.dev) and the `wasm32-unknown-unknown` target):

```sh
cd apps/simulate-app
trunk serve --release
```

Controls: arrow keys fly (Down is stick back), `A`/`D` rudder, `W`/`S` throttle.
`V` cycles free-flight vehicles (`C172 Skyhawk`, `Light Trainer`, `Aviate X500 Direct`, `Aviate Fixed-Wing Direct`, and `Kinematic`); `Tab` cycles `OperatorViewportMode` (`Cockpit`, `Chase`, `SensorAperture`, `ChartMap`, and `VolumetricScene`); `H` toggles the glass-cockpit PFD/MFD/HSI dock; `U` toggles the conformal HUD; `Space` pauses/resumes; `]` cycles time warp (`1x`/`2x`/`4x`). On JSBSim aircraft, `F`/`G` move flaps, `B` holds wheel brakes, `M` cycles magnetos, `E` engages the starter, and `R`/`T` adjust mixture.
A standard-layout gamepad flies with the left stick, rudder on the right stick,
and throttle on the right trigger. Other joysticks and yokes bind axis 0 to
roll, axis 1 to pitch, and axis 2 to the throttle lever.

Scenarios fly themselves on an autopilot, and every run of one is identical.
A number key starts one from the beginning; the window title shows its
instruments.

| Key | Scenario |
|---|---|
| `1` | Delaware corridor cruise |
| `2` | Off-seed acquisition, 900 m off the corridor and 200 m high |
| `3` | Aggressive bank and pitch, losing the horizon |
| `4` | Same-cell loiter |
| `5` | Apollo 11 lunar landing at the Sea of Tranquility |
| `6` | Apollo 11 Command Module reentry |
| `0` | Free flight |

## Checks

`scripts/ci.sh` runs every gate in order: formatting, Clippy (native and
wasm32), tests, docs, release build, the web bundle, and the structure checks.
