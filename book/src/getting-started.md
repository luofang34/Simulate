# Getting Started

## Sibling repositories

The workspace takes crates from sibling repositories (Navigate, Indicate,
Aviate, and so on) as git dependencies, each pinned to a full commit `rev` in
`Cargo.toml`. Cargo fetches them on the first build, so a plain clone builds
with nothing checked out beside it. `scripts/check-sibling-sources.sh` keeps
every entry for one repository on one `rev` and requires `Cargo.lock` to build
each repository from that one revision; to move a sibling, change its `rev` on
every line that names the repository and run any `cargo` command.

To build against local checkouts while changing a sibling, run

```sh
scripts/use-local-siblings.sh Navigate Indicate=~/Indicate-analog   # or --all
```

It writes a git-ignored Cargo config in this checkout (.cargo/config.toml, so
each worktree chooses for itself) that patches every crate taken from those
repositories to the directory given as `<Repo>=<dir>` (any checkout or
worktree of that repository), else to `../<Repo>`, or to `$SIBLINGS_DIR/<Repo>`
when `SIBLINGS_DIR` is set. It refuses a directory that does not track those
crates and prints where each repository now builds from. The patch rewrites
`Cargo.lock`, so `scripts/check-sibling-sources.sh` and every `--locked` CI
build fail until `scripts/use-local-siblings.sh --none` removes the override
and restores the pinned sources. To land a cross-repository change, merge the
sibling first, move the `rev` to that commit, then run `--none`.

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
