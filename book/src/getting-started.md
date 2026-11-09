# Getting Started

## Check out the sibling repositories

The workspace depends on several sibling repositories by relative path
(`../Aviate`, `../jsbsim-rs`, and so on). `Cargo.toml` lists them, each with
its git URL, pinned `rev`, and `path`, under
`[workspace.metadata.external_sources]`. Entries without a `path` are git
dependencies that Cargo fetches itself.

Run this from the repository root to clone every path dependency at its pinned
revision:

```sh
python3 - <<'PY'
import subprocess, tomllib
with open("Cargo.toml", "rb") as f:
    sources = tomllib.load(f)["workspace"]["metadata"]["external_sources"]
for name, src in sources.items():
    if "path" in src:
        subprocess.run(["git", "clone", src["git"], src["path"]], check=True)
        subprocess.run(["git", "-C", src["path"], "checkout", src["rev"]], check=True)
PY
```

Each `path` is relative to the checkout, so a `git worktree` has to live in the
same directory as the main checkout to find the siblings. A worktree nested
inside the checkout will not resolve them.

The toolchain comes from `rust-toolchain.toml`. `rustup` installs it on the
first `cargo` command.

## Run the desktop cockpit

```sh
cargo run -p simulate-app --release
```

The window opens in free flight in the default aircraft. [Flying the
Simulator](flying.md) covers the controls. Logging goes through `tracing`, and
`RUST_LOG` overrides the default filter.

## Run in the browser

```sh
rustup target add wasm32-unknown-unknown
cargo install trunk
cd apps/simulate-app
trunk serve --release
```

Open the address trunk prints. The browser build uses WebGPU and falls back to
WebGL2. Gamepads work through the browser Gamepad API.

## Scenery and environment variables

By default the maps stream real-world tiles: Esri World Imagery, AWS Terrain
Tiles (Terrarium elevation), and OpenFreeMap vectors.

| Variable | Platform | Effect |
|---|---|---|
| `SIMULATE_OFFLINE_SCENERY` | native | When set to any value, uses the built-in synthetic regional catalog instead of tile servers. No network is used. |
| `SIMULATE_TILE_CACHE_DIR` | native | Directory where downloaded tiles are kept across runs. Without it, tiles go to `target/simulate-tile-cache` in the workspace. |
| `SIMULATE_AIRPORT_PACK` | native | Path to a `SIMAPT01` airport pack that replaces the embedded airport database. If the file fails to load, the embedded database is used. |

The Flight Hub's flight tab also switches scenery mode while the app is running.

## Run headless

`ironbird` needs no GPU or window. This closes Aviate's fixed-wing controller
around the built-in C172 for 30 simulated seconds and logs a summary:

```sh
cargo run --release -p ironbird -- --mode tier0-c172 --duration-s 30
```

[Flying an External Autopilot](external-autopilots.md) builds on this. The
[CLI reference](cli-and-schema-reference.md) lists every mode.
