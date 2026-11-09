# Multi-Tier XIL Verification (Tiers 0, 1, 2)

`ironbird` verifies flight control laws, navigation filters, and vehicle plants across three software- and hardware-in-the-loop (`XIL`) tiers. Every tier steps its plant inside a `sim_core::SimSession` alongside a deterministic **`30 Hz` optical exposure channel** (`OPTICAL_CHANNEL`, `OPTICAL_PERIOD = TICKS_PER_SECOND / 30`), ensuring that camera exposures and truth state interpolation are identical regardless of wall-clock pacing.

| Tier | Coupling Mechanism | Typical Use Case | Execution Speed |
|---|---|---|---|
| **Tier 0 (`direct-aviate`)** | In-process `AviateDirectRunner` (`0` syscalls, `0` heap allocs/step) | Fast regression testing, Monte Carlo sweeps, controller gain tuning | `>1,000x` real-time |
| **Tier 1 (`shm-aviate`)** | In-process/shared-memory `SharedStateV2` seqlock (`ExternalNodeBridge`) | Verifying lockstep barrier & seqlock memory-ordering contracts | `>500x` real-time (lockstep) or `1x` (real-time) |
| **Tier 2 (`sim-xil` wire)** | Controller-blind `MAVLink v2 HIL` (TCP/UDP) & `Betaflight` SITL UDP | PX4 SITL, ArduPilot SITL, Betaflight SITL, or physical flight controller over UART/UDP | Lockstep or `1x` wall-clock |

---

## Tier 0: In-Process `AviateDirectRunner`

Tier 0 links `aviate-xil-direct` directly into the `ironbird` process:

- **Multirotor (`--mode tier0-x500`)**: Steps `X500RigidPlant` at `400 Hz` RK4 (`RigidBody6DofBlock` + `RotorLayout::x500()`), warms up the estimator for up to `500` steps until `fc.is_ready()`, arms the controller, and commands a `PositionHold` climb of `5.0 m`.
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

`ironbird::run_tier1_shm_x500` couples `X500RigidPlant` to `sim_aviate::AviateSharedMemoryEndpoint` through `sim_core::ExternalNodeBridge`. Each step writes `AviateStimulus` (ECEF vehicle state + root component `LocalSiteKinematics`) into a `SharedStateV2` seqlock buffer and polls actuator commands back under either:

- `HitlSyncMode::LockstepBarrier { processing_delay }`: deterministic tick-exact transport delay (`--lockstep-us 2500` default).
- `HitlSyncMode::RealTimeStamped`: wall-clock paced via `RealtimePacer` (`2 ms` overrun tolerance).

---

## Tier 2: External SITL / HITL Wire Endpoints (`sim-xil`)

Tier 2 speaks standard external autopilot wire protocols without assuming which flight stack is on the other side:

### 1. MAVLink v2 HIL (`--mode mavlink-tcp-listen | mavlink-tcp-connect | mavlink-udp`)

Streams `HIL_SENSOR`, `HIL_GPS`, and `HIL_STATE_QUATERNION` derived from `JsbsimPlant::xil_stimulus()` (using `StandardAtmosphere` pressure/temperature and WGS84 normal gravity) and ingests `HIL_ACTUATOR_CONTROLS` via `HilControlMapping`:

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

Exchanges `144`-byte FDM state packets and `BetaflightServoPacket` 4-motor PWM/normalized outputs over UDP (`9002`/`9003`):

```sh
cargo run --release -p ironbird -- \
  --mode betaflight-udp \
  --bind 127.0.0.1:9003 \
  --peer 127.0.0.1:9002 \
  --realtime
```
