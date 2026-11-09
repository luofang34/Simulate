# Multi-Tier XIL Verification (Tiers 0, 1, 2)

`ironbird` verifies flight control laws, navigation filters, and vehicle plants across three software- and hardware-in-the-loop (`XIL`) tiers. Every tier steps its plant inside a `sim_core::SimSession` alongside a deterministic **`30 Hz` optical exposure channel** (`OPTICAL_CHANNEL`, `OPTICAL_PERIOD = TICKS_PER_SECOND / 30`), ensuring that camera exposures and truth state interpolation are identical regardless of wall-clock pacing. [Flying an External Autopilot](external-autopilots.md) is the step-by-step guide to running each tier.

| Tier | Coupling Mechanism | Typical Use Case | Pacing |
|---|---|---|---|
| **Tier 0 (`direct-aviate`)** | In-process `AviateDirectRunner` (`0` syscalls, `0` heap allocs/step) | Fast regression testing, Monte Carlo sweeps, controller gain tuning | Unpaced, as fast as the CPU allows |
| **Tier 1 (`shm-aviate`)** | `SharedStateV2` seqlock (`AviateSharedMemoryEndpoint` through `ExternalNodeBridge`) | Verifying lockstep barrier & seqlock memory-ordering contracts | Lockstep (unpaced) or wall-clock |
| **Tier 2 (`sim-xil` wire)** | Controller-blind `MAVLink v2 HIL` (TCP/UDP) & `Betaflight` SITL UDP | PX4 SITL, Betaflight SITL, or a physical flight controller behind a TCP/UDP-to-UART bridge | Lockstep or wall-clock |

---

## Tier 0: In-Process `AviateDirectRunner`

Tier 0 links `aviate-xil-direct` directly into the `ironbird` process:

- **Multirotor (`--mode tier0-x500`)**: Steps `X500RigidPlant` at `400 Hz` RK4 (`RigidBody6DofBlock` + `RotorLayout::x500()`), warms up the estimator until `fc.is_ready()`, arms the controller, and commands a `PositionHold` climb of `5.0 m`.
- **Fixed-Wing (`--mode tier0-c172`, `tier0-dir`)**: Steps `JsbsimPlant` at `120 Hz` (`JsbsimFdmBlock`), computes exact body-axis FRD specific force
  $$\mathbf{f}_{\text{FRD}} = \mathbf{R}_{\text{ECEF}\to\text{body}}\left(\frac{\Delta \mathbf{v}_{\text{ECEF}}}{\Delta t} - \mathbf{g}_{\text{ECEF}}\right)$$
  and drives `elevator`, `aileron`, `rudder`, and `throttle` via `build_direct_fixed_wing_kernel()`.

```sh
# Built-in C172 Tier-0 cruise for 30 simulated seconds
cargo run --release -p ironbird -- --mode tier0-c172 --duration-s 30

# Arbitrary syspack directory Tier-0 cruise
cargo run --release -p ironbird -- --mode tier0-dir --package-dir vehicles/c172 --duration-s 30
```

---

## Tier 1: Shared-Memory Seqlock (`SharedStateV2`)

`ironbird::run_tier1_shm_x500` couples `X500RigidPlant` to `sim_aviate::AviateSharedMemoryEndpoint` through `sim_core::ExternalNodeBridge`. Each step writes `AviateStimulus` (ECEF vehicle state + root component `LocalSiteKinematics`) into a `SharedStateV2` seqlock buffer, runs the peer function on the calling thread, and polls actuator commands back under either:

- `HitlSyncMode::LockstepBarrier { processing_delay }`: deterministic tick-exact transport delay.
- `HitlSyncMode::RealTimeStamped`: wall-clock paced via `RealtimePacer` (`2 ms` overrun tolerance).

Tier 1 is a library entry point with no `--mode`; `apps/ironbird/tests/closed_loop_tiers.rs` drives it.

---

## Tier 2: External SITL / HITL Wire Endpoints (`sim-xil`)

Tier 2 speaks standard external autopilot wire protocols without assuming which flight stack is on the other side. `--lockstep-us <N>` (default `2500`) selects `HitlSyncMode::LockstepBarrier`: the bridge blocks on `ExternalEndpoint::exchange_blocking` at every stimulus (2 s timeout) and delivers the reply `N` µs of master-clock time later. `--realtime` selects `HitlSyncMode::RealTimeStamped`: the bridge posts without waiting and applies the newest reply that has arrived.

### 1. MAVLink v2 HIL (`--mode mavlink-tcp-listen | mavlink-tcp-connect | mavlink-udp`)

Flies the built-in C172 (`FixedWingVehicle::C172`). Each plant step streams `HIL_SENSOR` derived from `JsbsimPlant::xil_stimulus()` (using `StandardAtmosphere` pressure/temperature and WGS84 normal gravity), with `HIL_GPS` at most every `100 ms` and `HEARTBEAT` every second (`MavlinkHilConfig::default()`), and ingests `HIL_ACTUATOR_CONTROLS` via `HilControlMapping`. `HIL_STATE_QUATERNION` ground truth is emitted only when `MavlinkHilConfig::send_state_quaternion` is set, which the CLI does not do.

```sh
# Listen for PX4 SITL on TCP 127.0.0.1:4560 in lockstep (2,500 us delay)
cargo run --release -p ironbird -- \
  --mode mavlink-tcp-listen \
  --bind 127.0.0.1:4560 \
  --duration-s 60 \
  --lockstep-us 2500

# Real-time paced UDP bridge to an external autopilot or router
cargo run --release -p ironbird -- \
  --mode mavlink-udp \
  --bind 127.0.0.1:14560 \
  --peer 127.0.0.1:14550 \
  --realtime
```

### 2. Betaflight SITL UDP (`--mode betaflight-udp`)

Flies the X500 plant. Sends `144`-byte `BetaflightFdmPacket`s to `--peer` (Betaflight's FDM port `9003`) and receives `BetaflightServoPacket` 4-motor normalized outputs on `--bind` (Betaflight's servo port `9002`):

```sh
cargo run --release -p ironbird -- \
  --mode betaflight-udp \
  --bind 127.0.0.1:9002 \
  --peer 127.0.0.1:9003 \
  --realtime
```
