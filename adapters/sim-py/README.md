# `sim-py` — Python Bindings for Simulate

`sim-py` exposes `Simulate`'s Rust orbital dynamics, astronomical time-scale conversions, CCSDS Navigation Data Message (`OPM`, `OMM`, `OEM`, `APM`, `AEM`, `CDM`, `TDM`) parsing/round-tripping, NAIF SPICE binary (`DAF/SPK`, `DAF/CK`) inspection/evaluation/creation, geodesic H3 coverage grid analysis, and reproducible Monte Carlo dispersion sampling to Python via PyO3 `0.30.1`.

## Building with `maturin`

```bash
cd adapters/sim-py
maturin develop --release
```

## Building and Verifying Directly with Cargo (macOS)

When linking an extension `cdylib` manually on macOS outside `maturin`, pass Apple linker flags so CPython symbols resolve dynamically at `import sim_py` time:

```bash
RUSTFLAGS="-C link-arg=-undefined -C link-arg=dynamic_lookup" \
  cargo build --manifest-path adapters/sim-py/Cargo.toml

tmpdir="$(mktemp -d)"
ln -sf "${CARGO_TARGET_DIR:-target}/debug/libsim_py.dylib" "$tmpdir/sim_py.so"
PYTHONPATH="$tmpdir" python3 -B adapters/sim-py/tests/python_smoke.py
\rm -rf "$tmpdir"
```

## Running Rust Tests & Clippy

```bash
cargo test --manifest-path adapters/sim-py/Cargo.toml --no-default-features --features auto-initialize
```
