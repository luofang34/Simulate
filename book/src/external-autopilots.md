# Flying an External Autopilot

Simulate closes a flight controller's loop around its 6-DoF plant in three
ways. The tiers are described in [Multi-Tier XIL
Verification](xil-tiers.md); this chapter is about running them.

| Autopilot | Coupling | Where it runs |
|---|---|---|
| Aviate | Tier 0, in-process | `simulate-app` (two of its vehicles) and `ironbird --mode tier0-*` |
| Aviate | Tier 1, `SharedStateV2` shared memory | Library API and tests; there is no CLI mode |
| PX4 SITL | Tier 2, MAVLink v2 HIL over TCP or UDP | `ironbird --mode mavlink-*` |
| Betaflight SITL | Tier 2, Betaflight SITL UDP | `ironbird --mode betaflight-udp` |
| ArduPilot SITL | Not supported | See [ArduPilot](#ardupilot) |

`simulate-app` has no socket connection to an external autopilot. External SITL
runs go through the headless `ironbird` binary. `ironbird` has no `--help`. The
[CLI reference](cli-and-schema-reference.md) lists its modes and flags, and an
unknown flag fails with the list of valid modes.

## Tier 0: Aviate in-process

Aviate's flight controller is linked into the process and stepped from the
plant's truth state and specific force. It makes no syscalls or allocations
per step, and nothing waits on a socket.

In `simulate-app`, press `V` until the vehicle is the X500 quadrotor or the
fixed-wing fly-by-wire C172. The stick then commands Aviate's setpoints rather
than the control surfaces (see [Vehicles](flying.md#vehicles)).

Headless, `ironbird` runs the same loop as fast as the CPU allows:

```sh
# X500 quadrotor: arm, then climb 5 m in position hold
cargo run --release -p ironbird -- --mode tier0-x500 --duration-s 10

# Built-in C172 under Aviate's fixed-wing controller
cargo run --release -p ironbird -- --mode tier0-c172 --duration-s 30

# Any syspack directory on disk
cargo run --release -p ironbird -- --mode tier0-dir --package-dir vehicles/p38g --duration-s 20
```

Each run ends with one log line: plant steps executed, simulated and
wall-clock seconds, speed-up over real time, and final altitude.

## Tier 1: Aviate over shared memory

`sim_aviate::AviateSharedMemoryEndpoint` exchanges `SharedStateV2` seqlock
frames with an Aviate peer through `sim_core::ExternalNodeBridge`, the same
contract Aviate uses with Gazebo. `ironbird::run_tier1_shm_x500` flies the X500
plant against a peer function that runs on the calling thread. The examples
are in `apps/ironbird/tests/closed_loop_tiers.rs`. No CLI mode starts Tier 1.

## Lockstep and real time

Every Tier 2 mode takes one of two sync modes:

| Flag | Behaviour |
|---|---|
| `--lockstep-us <N>` (default `2500`) | At every sensor frame the simulation waits for the autopilot's reply. The reply takes effect `N` µs of simulated time later, so a run replays exactly however long the autopilot actually took. If no reply arrives within 2 s of wall-clock time, the run stops with an error. |
| `--realtime` | The simulation is paced to the wall clock and never waits. The newest reply that has arrived is applied, and older ones are dropped. |

## PX4 SITL over MAVLink HIL

The MAVLink modes fly the built-in C172 (JSBSim, 120 Hz). It starts in the air
at cruise speed over Delaware (39.1582° N, 75.5244° W, 120 m). `ironbird`
speaks MAVLink v2 as system 1, component 195:

| Direction | Message | Rate |
|---|---|---|
| Simulate → autopilot | `HIL_SENSOR` (IMU, magnetometer, barometer, differential pressure) | Every plant step |
| Simulate → autopilot | `HIL_GPS` | 10 Hz |
| Simulate → autopilot | `HEARTBEAT` | 1 Hz |
| Autopilot → Simulate | `HIL_ACTUATOR_CONTROLS` | Every reply |

`HIL_STATE_QUATERNION` is supported by `sim_xil::mavlink_hil`, but the CLI
leaves it off. `HIL_ACTUATOR_CONTROLS` channels map as follows
(`HilControlMapping::default()`):

| Channel | Control | Range |
|---|---|---|
| 0 | Aileron | −1 to 1 |
| 1 | Elevator | −1 to 1 |
| 2 | Rudder | −1 to 1 |
| 3 | Throttle | 0 to 1, held at 0 while the autopilot reports disarmed |

Until the first actuator message arrives, throttle stays at the trim setting.

| Mode | Simulate side | Autopilot side |
|---|---|---|
| `mavlink-tcp-listen` | Listens on `--bind` (default `127.0.0.1:4560`) and blocks until one client connects | Connects as a TCP client |
| `mavlink-tcp-connect` | Dials `--peer` | Listens, e.g. a MAVLink router or serial bridge |
| `mavlink-udp` | Binds `--bind` and sends to `--peer` (default `127.0.0.1:14550`) | Sends replies to the bound address |

Start `ironbird` first, then PX4:

```sh
cargo run --release -p ironbird -- \
  --mode mavlink-tcp-listen \
  --bind 127.0.0.1:4560 \
  --duration-s 120
```

`--duration-s` counts simulated time from the moment PX4 connects.

**PX4 side.** This part comes from PX4's documentation and conventions, not
from this repository. Check it against your PX4 release.

- Build PX4 SITL without a bundled simulator (`make px4_sitl`). Start it with a
  fixed-wing airframe: set `PX4_SYS_AUTOSTART` to a plane airframe from
  `ROMFS/px4fmu_common/init.d-posix/airframes`. PX4's `simulator_mavlink`
  module then connects to TCP port `4560` plus the instance number on
  localhost, which is what `mavlink-tcp-listen` expects.
- PX4 runs lockstep with its simulator by default. It advances its clock from
  `HIL_SENSOR.time_usec` and answers each frame with `HIL_ACTUATOR_CONTROLS`,
  which suits the default `--lockstep-us`.
- On releases with control allocation, set PX4's HIL actuator output functions
  (`HIL_ACT_FUNC1` to `HIL_ACT_FUNC4`) so that outputs 1 to 4 are aileron,
  elevator, rudder, and throttle, matching the table above.
- Arm PX4 and choose a flight mode from a ground station or the PX4 shell. The
  aircraft is already airborne when the run starts.

## Betaflight SITL

`betaflight-udp` flies the X500 quadrotor rigid body (400 Hz) from the same
origin. Each step sends Betaflight's 144-byte `fdm_packet` (IMU, attitude,
velocity, position, pressure). Betaflight's `servo_packet` brings back four
motor commands from 0 to 1, and motor `i` drives rotor `i` of the X500 layout.

| Port | Owner | Traffic |
|---|---|---|
| `9003` | Betaflight listens | `fdm_packet` from Simulate (`--peer`) |
| `9002` | Simulate listens (`--bind`) | `servo_packet` from Betaflight |
| `9004` | Betaflight listens | RC input; Simulate sends none |

The CLI defaults are the MAVLink ports, so pass both addresses:

```sh
cargo run --release -p ironbird -- \
  --mode betaflight-udp \
  --bind 127.0.0.1:9002 \
  --peer 127.0.0.1:9003 \
  --realtime \
  --duration-s 30
```

Use `--realtime`. Betaflight SITL sends a motor packet only when its own loop
has run since the last FDM packet, not one per FDM packet. A lockstep run
therefore misses a reply and stops at the 2 s timeout.

`ironbird` sends no RC channels, so Betaflight stays disarmed and the
quadrotor falls with its motors idle. `sim_xil::betaflight` has MSP v1/v2
codecs, including `MSP_SET_RAW_RC`, for a harness that needs to arm the
controller and fly it.

**Betaflight side** (from Betaflight's SITL target, not this repository): build
it with `make TARGET=SITL` and run `obj/main/betaflight_SITL.elf`. At startup
it prints the ports it uses. It writes `eeprom.bin` to its working directory,
so run it from a scratch directory.

## ArduPilot

There is no ArduPilot path. ArduPilot's SITL does not read `HIL_SENSOR` or
write `HIL_ACTUATOR_CONTROLS`. Its external-simulator backends (for example
`--model JSON`) use ArduPilot's own UDP protocols, which `sim-xil` does not
implement. Supporting ArduPilot means adding such an endpoint to `sim-xil`
and a mode to `ironbird`.
