# Simulate

> [!WARNING]
> Work in progress. APIs, file formats, and vehicle packages change without
> notice, and nothing here is fit for real-world flight or navigation use.

Simulate is a deterministic 6-DoF flight and spaceflight simulator in Rust. One
code base runs natively (Metal, Vulkan, DX12) and in the browser (WebGPU, with a
WebGL2 fallback). It has two front ends. `simulate-app` is an interactive
cockpit. `ironbird` is a headless bench that flies vehicle packages against
flight controllers: Aviate in-process, or PX4 and Betaflight SITL over sockets.
Everything steps on one integer master clock, so scripted and lockstep runs replay
bit for bit.

## Prerequisites

- The Rust toolchain named in `rust-toolchain.toml`.
- Network access to GitHub on the first build: the sibling repositories
  (Navigate, Indicate, Aviate, and the others in `Cargo.toml`) are git
  dependencies pinned to a commit `rev`, and Cargo fetches them itself.
  [Getting Started](book/src/getting-started.md) covers building against local
  checkouts of them instead.
- For the browser build: [trunk](https://trunkrs.dev) and the
  `wasm32-unknown-unknown` target.

## Quick start

```sh
cargo run -p simulate-app --release                      # desktop cockpit
SIMULATE_OFFLINE_SCENERY=1 cargo run -p simulate-app --release   # no network
cargo run --release -p ironbird -- --mode tier0-c172 --duration-s 30   # headless Aviate loop
(cd apps/simulate-app && trunk serve --release)          # browser
```

Press `Esc` in the cockpit to open the Flight Hub. There you can pick an
aircraft, an aerodrome, a scenario, and your controls, and rewind the flight.

## Workspace

| Package | Path | Role |
|---|---|---|
| `sim-spacetime` | `crates/sim-spacetime` | Clock, time scales, frames, and trajectory contracts shared by the spaceflight crates |
| `sim-orbital` | `crates/sim-orbital` | Multi-body orbital dynamics, ephemerides, and spaceflight GNC |
| `sim-analysis-contract` | `crates/sim-analysis-contract` | Versioned analysis request and result types |
| `sim-analysis` | `crates/sim-analysis` | Geometric relations, constraints, and access windows |
| `sim-mission` | `crates/sim-mission` | Spaceflight mission plans and segment propagation |
| `sim-universe` | `crates/sim-universe` | Vessels on one clock across flight and orbital tiers, and the handoff between them |
| `sim-geo` | `crates/sim-geo` | WGS84 and lunar geodesy, local frames, floating origin, pinhole intrinsics |
| `sim-atmosphere` | `crates/sim-atmosphere` | Atmosphere and cloud fields |
| `sim-core` | `crates/sim-core` | Master clock, the `Block` contract, vehicle state, rigid-body and JSBSim dynamics, XIL bridges |
| `sim-devices` | `crates/sim-devices` | Sensor and bus models (IMU, barometer, ARINC 429) |
| `sim-xil` | `crates/sim-xil` | External SITL/HITL wire protocols: MAVLink v2 HIL and Betaflight SITL |
| `sim-pack` | `crates/sim-pack` | Binds verified vehicle packages to the simulator; calibration and performance estimation |
| `sim-input` | `crates/sim-input` | Gamepad, joystick, and yoke mapping |
| `sim-scenery` | `crates/sim-scenery` | Elevation, imagery, and airport data |
| `sim-render` | `crates/sim-render` | wgpu renderer and offscreen sensor apertures |
| `sim-scenario` | `crates/sim-scenario` | Deterministic scripted scenarios and the simulation supervisor |
| `sim-apollo` | `vehicles/apollo-11` | Apollo 11 LM and CSM packages and their sensors |
| `sim-p38g` | `vehicles/p38g` | Lockheed P-38G Lightning package |
| `sim-raketa` | `vehicles/raketa` | Raketa test rocket |
| `sim-aviate` | `adapters/sim-aviate` | Aviate flight controller, in-process (Tier 0) and shared-memory (Tier 1) |
| `sim-indicate` | `adapters/sim-indicate` | Indicate instruments, PFD/HSI, and HUD |
| `sim-maplibre` | `adapters/sim-maplibre` | `maplibre-rs` out-the-window scene and instrument maps |
| `sim-navigate` | `adapters/sim-navigate` | Navigate visual navigation and fusion |
| `sim-pilotage` | `adapters/sim-pilotage` | Pilotage vehicle and camera bridge |
| `ironbird` | `apps/ironbird` | Headless XIL, calibration, and design-estimation runner |
| `simulate-app` | `apps/simulate-app` | Interactive cockpit for desktop and browser |

`vehicles/c172` is a data-only package with no crate of its own. A vehicle
package is a `syspack`: a `syspack.json` manifest plus the assets it lists with
their SHA-256 and size, and one that does not verify will not load.

## Documentation

The book in `book/` is the user guide and engineering manual. Build or serve it
with `mdbook build book` or `mdbook serve book`. `scripts/ci.sh` runs every gate:
formatting, Clippy, tests, docs, release build, web bundle, book, and the
`scripts/check-*.sh` structure checks. `scripts/check-docs.sh` among them keeps
this README and the book consistent with the code.
