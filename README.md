# FSgame

A flight simulator in Rust that runs natively (Metal, Vulkan, DX12) and in the
browser (WebGPU, with WebGL2 fallback) from one code base.

## Layout

| Crate | Role |
|---|---|
| `fsg-geo` | WGS84 geodesy, local frames, floating origin, UTC |
| `fsg-atmosphere` | Atmosphere and cloud fields (ICAO standard atmosphere today) |
| `fsg-sim` | Integer master clock, the `Block` contract, multi-rate scheduling, vehicle state |
| `fsg-input` | Gamepad, joystick, and yoke mapping to pilot controls |
| `fsg-scenery` | Elevation, imagery, and photogrammetry source traits (imagery and photogrammetry are deferred stubs) |
| `fsg-render` | wgpu renderer: camera-relative WGS84 globe, reversed-Z depth |
| `fsg-app` | Desktop and browser executable |

## Run

Desktop:

```sh
cargo run -p fsg-app --release
```

Browser (needs [trunk](https://trunkrs.dev) and the `wasm32-unknown-unknown` target):

```sh
cd crates/fsg-app
trunk serve --release
```

Controls: arrow keys fly (Down is stick back), A/D rudder, W/S throttle. A
standard-layout gamepad flies with the left stick, rudder on the right stick,
and throttle on the right trigger. Other joysticks and yokes bind axis 0 to
roll, axis 1 to pitch, and axis 2 to the throttle lever.

## Checks

`scripts/ci.sh` runs every gate in order: formatting, Clippy (native and
wasm32), tests, docs, release build, the web bundle, and the structure checks.
