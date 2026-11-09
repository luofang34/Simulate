# Introduction

**Simulate** is a deterministic, closed-loop 6-DoF flight simulator in Rust that runs natively (Metal, Vulkan, DX12) and in the browser (WebGPU/WebGL2) from a single workspace. Alongside the interactive split-viewport cockpit (`simulate-app`), the workspace ships **`ironbird`** (`apps/ironbird`) and **`sim-pack`** (`crates/sim-pack`): a headless software- and hardware-in-the-loop (`XIL`) verification, flight-test calibration, and aircraft performance estimation bench.

Every simulation run in `Simulate` and `ironbird` executes on an integer nanosecond master clock (`sim_core::SimInstant`, `TICKS_PER_SECOND = 1_000_000_000`) through the `sim_core::Block` contract. Whether an aircraft is flown interactively in the cockpit, stepped at `>1,000x` real-time in headless Tier-0 `Aviate` XIL, coupled to an external PX4/ArduPilot/Betaflight controller over Tier-2 sockets, or swept across altitude and power settings in a flight-test suite, it runs the exact same verified vehicle package (`syspack`) and 6-DoF physics blocks.

## Three Core Aircraft Engineering Workflows

This manual covers the end-to-end engineering lifecycle of an aircraft model in `Simulate` and `ironbird`:

1. **Onboarding & XIL Verification of a New Aircraft (`syspack`)**
   - Package a fixed-wing aircraft or rotorcraft as a SHA-256-verified `syspack` directory (`syspack.json` + JSBSim `fdm/*.xml` + `mbd-core` component tree + mounted COTS sensors/cameras).
   - Fly and verify the package immediately from disk in `ironbird` (`--mode tier0-dir --package-dir <path>`) with zero Rust code changes, or register it as a compile-time built-in aircraft across `sim-pack`, `ironbird`, and `simulate-app`.
   - Exercise closed-loop guidance, navigation, and control across **Tier 0** (`aviate-xil-direct`), **Tier 1** (`SharedStateV2` seqlock), and **Tier 2** (`MAVLink v2 HIL` TCP/UDP and `Betaflight` SITL UDP), synchronized with a deterministic `30 Hz` optical camera exposure channel.

2. **Closed-Loop POH & Empirical Flight-Test Calibration (`calibrate-*`)**
   - When modeling an existing certificated aircraft or a fielded UAV with flight logs, static aerodynamic table checks are not enough. `sim_pack::calibration` flies virtual flight-test maneuvers in closed loop (static WOT ground runup, power-off stalls across flap/bank combinations, best-rate `Vy` climbs across altitude, leaned level cruise, and recorded GPS groundspeed segments).
   - Pair physical `KCAS` aerodynamics with flap-indexed Pitot-Static Position Error (`PEC`, `performance/pitot_static_calibration.json`) so the flight dynamics plant flies calibrated airspeed while `Indicate` PFD/HUD and standalone instruments display true indicated airspeed (`KIAS`).
   - Run finite-difference Jacobian sensitivity probing and damped Gauss-Newton parameter identification (`--sensitivity`) to converge engine volumetric efficiency, BSFC, friction horsepower, and mass offsets against Pilot's Operating Handbook (`POH`) tables and real flight recordings (demonstrated on **1998 Cessna 172S `N642SP`**).

3. **Estimating Performance, V-Speeds & Range of a New Design (`estimate-*`)**
   - When evaluating a **newly designed aircraft** (for example exported from `ace-rs` or prototyped in JSBSim XML before any POH or flight-test data exists), `ironbird --mode estimate-dir --package-dir <path>` automatically sweeps the 6-DoF plant in closed loop to synthesize:
     - Stall speeds ($V_{S0}, V_{S1}$), best angle-of-climb speed ($V_X$, $\gamma_{\max}$), best rate-of-climb speed ($V_Y$, $RC_{\max}$), best power-off glide speed ($V_G$, $(L/D)_{\max}$), maneuvering speed ($V_A$), and a ready-to-use `performance/airspeed_envelope.json`;
     - Rate-of-climb decay across altitude, **service ceiling** ($RC = 100\text{ fpm}$), and **absolute ceiling** ($RC = 0\text{ fpm}$);
     - Multi-altitude cruise polars, **specific range** ($\text{NM/gal}$ and $\text{NM/kg}$), **endurance** ($\text{hr}$), **still-air range** ($\text{NM}$ with `45-min` IFR reserve and dry-tank ferry), and a **3-Point Payload–Range Envelope**.

## Quick-Start Commands

```sh
# 1. Run closed-loop POH + empirical calibration & Jacobian identification on built-in C172 (N642SP)
cargo run --release -p ironbird -- --mode calibrate-c172 --sensitivity --report-out c172_calibration.md

# 2. Estimate V-speeds, ceiling, glide polar, specific range, and payload-range for a design directory
cargo run --release -p ironbird -- --mode estimate-dir --package-dir vehicles/c172 --report-out c172_estimate.md

# 3. Fly any on-disk syspack directory in Tier-0 closed-loop Aviate XIL (30 Hz optical channel)
cargo run --release -p ironbird -- --mode tier0-dir --package-dir vehicles/c172 --duration-s 20
```
