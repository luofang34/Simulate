# Authoring or Exporting a New Aircraft

There are two supported ways to create a fixed-wing `syspack` for `Simulate` and `ironbird`:

1. **Exporting from `ace-rs` (Conceptual / Parametric Aircraft Design)** — recommended when designing a new airframe from geometry, airfoil polars, and propulsion specs.
2. **Authoring a Data-Only `syspack` Directory** — recommended when wrapping an existing JSBSim `<fdm_config>` XML model or wind-tunnel dataset (like `vehicles/c172s` or `vehicles/c172p`).

---

## Workflow A: Exporting a Design from `ace-rs`

When an aircraft is defined in the sibling `ace-rs` repository, `ace-export` generates the entire `syspack` directory — including the component tree, inertia tensor, JSBSim aerodynamic/propulsion XML, and SHA-256 manifest digests — in one command. Run it from this repository's root, with `<ace-rs>` standing for your `ace-rs` checkout:

```sh
cargo run --manifest-path <ace-rs>/Cargo.toml -p ace-export --example export_syspack -- \
  <ace-rs>/examples/c172/vehicle.json \
  vehicles/c172s
```

`vehicles/c172s` was seeded this way before POH/TCDS calibration.

After exporting:

1. Run **design performance estimation** to inspect the synthesized V-speeds, climb ceiling, glide ratio, and range:
   ```sh
   cargo run --release -p ironbird -- \
     --mode estimate-dir \
     --package-dir vehicles/c172s \
     --report-out c172_design_estimate.md
   ```
2. If you have target POH tables or flight-test recordings, add `performance/airspeed_envelope.json`, `performance/pitot_static_calibration.json`, and `performance/poh_benchmarks.json`, refresh the `assets` hashes in `syspack.json`, and run closed-loop calibration (`--mode calibrate-dir`).

---

## Workflow B: Authoring a Data-Only `syspack` Directory

You can create a new aircraft package without writing a single line of Rust code (see `vehicles/c172s` and `vehicles/c172p` as reference templates).

### Step 1: Create the Folder and JSBSim FDM (`fdm/<model>.xml`)

Create `vehicles/<my-aircraft>/fdm/<model>.xml` conforming to `<fdm_config version="2.0">` with:

- `<metrics>`: wing `wingarea` ($\text{ft}^2$), `wingspan` ($\text{ft}$), `chord` ($\text{ft}$), tail arm/area, and `AERORP` / `EYEPOINT` / `VRP` stations.
- `<mass_balance>`: `Ixx`, `Iyy`, `Izz` ($\text{slug}\cdot\text{ft}^2$), `emptywt` ($\text{lbs}$), `CG` location, and `<pointmass name="Payload">` (`pointmass-weight-lbs[0]`) so `sim-pack` can set exact test gross masses during calibration and range sweeps.
- `<ground_reactions>`: nose/tail and main gear `<contact type="BOGEY">` units with brake group assignments (`LEFT`, `RIGHT`).
- `<propulsion>`: piston (`<piston_engine>`), turbine, or electric motor paired with `<thruster>` (`<propeller>` or direct thrust) and `<tank type="FUEL">`.
- `<flight_control>`: pitch (`fcs/elevator-cmd-norm`), roll (`fcs/aileron-cmd-norm`), yaw (`fcs/rudder-cmd-norm`), and flap (`fcs/flap-cmd-norm` → `fcs/flap-pos-norm`) channels.
- `<aerodynamics>`: lift (`LIFT`), drag (`DRAG`), side (`SIDE`), roll (`ROLL`), pitch (`PITCH`), and yaw (`YAW`) axes.

> **Important convention for closed-loop calibration**: Include a payload pointmass (`inertia/pointmass-weight-lbs[0]`) in `<mass_balance>`. Both `sim_pack::evaluate_package` and `sim_pack::estimate_design_performance` probe the non-payload weight (`inertia/weight-lbs - inertia/pointmass-weight-lbs[0]`) and adjust `inertia/pointmass-weight-lbs[0]` so every stall, climb, and cruise test flies at the exact requested `gross_mass_kg`.

### Step 2: Create `syspack.json` and Compute Asset Digests

Whenever you add or edit an asset under `vehicles/<my-aircraft>/` (`fdm/*.xml` or `performance/*.json`), update its `sha256` and `size_bytes` in `syspack.json`. You can refresh all asset digests in a package directory with a single Python command:

```sh
python3 -c '
import hashlib, json, pathlib, sys
pkg = pathlib.Path(sys.argv[1])
manifest_path = pkg / "syspack.json"
doc = json.loads(manifest_path.read_text())
for rel in list(doc["assets"].keys()):
    data = (pkg / rel).read_bytes()
    doc["assets"][rel] = {
        "sha256": hashlib.sha256(data).hexdigest(),
        "size_bytes": len(data),
    }
manifest_path.write_text(json.dumps(doc, indent=2) + "\n")
' vehicles/<my-aircraft>
```

### Step 3: Verify Package Loading and Estimate Baseline Performance

```sh
cargo run --release -p ironbird -- \
  --mode estimate-dir \
  --package-dir vehicles/<my-aircraft>
```

If `LoadedSyspack::load` succeeds, `ironbird` prints the full Markdown report containing the aircraft's stall speeds, $V_X$, $V_Y$, $V_G$, $(L/D)_{\max}$, service ceiling, cruise specific range, and payload–range table.
