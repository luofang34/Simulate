# The `syspack` Vehicle Contract

Every vehicle in `Simulate` and `ironbird` — whether embedded in the binary under `vehicles/` or loaded at runtime from disk or memory — is a **`syspack`**: a directory containing a `syspack.json` manifest and the asset files it enumerates.

Loading is **fail-closed**:

1. `syspack::VerifiedSyspack::load` verifies `schema_version == 1`, checks that every declared asset exists, and verifies both the exact **byte length** (`size_bytes`) and **SHA-256 digest** (`sha256`) of every file before any parser sees its bytes.
2. `sim_pack::LoadedSyspack::load` then validates the `mbd-core` component tree, instantiates every declared COTS sensor and camera mount, and parses any optional `performance/*.json` envelopes and benchmarks. If any check fails, a typed `PackError` is returned and no partial vehicle state is constructed.

## Directory Layout of a Fixed-Wing `syspack`

```text
vehicles/<aircraft-id>/
├── syspack.json                              # Required: SHA-256 manifest, component tree, dynamics & devices
├── fdm/
│   └── <model_name>.xml                      # Required (for JsbsimXml): JSBSim 6-DoF flight dynamics model
└── performance/
    ├── airspeed_envelope.json                # Optional: V-speeds & structural arcs for PFD/HUD speed tapes
    ├── pitot_static_calibration.json         # Optional: Flap-indexed KCAS <-> KIAS position error (PEC)
    └── poh_benchmarks.json                   # Optional: Declarative POH & empirical flight-test suite
```

## Anatomy of `syspack.json`

A fixed-wing `syspack.json` declares six top-level sections:

| Field | Type | Purpose |
|---|---|---|
| `schema_version` | `u32` | Always `1`. |
| `id` | `String` | Stable machine identifier (e.g. `"c172_class_reference"`, `"p38g"`). |
| `name` | `String` | Human-readable aircraft title. |
| `version` | `String` | Semantic version of the package. |
| `kind` | `SystemKind` | `"FixedWing"`, `"RotaryWing"`, or `"Spacecraft"`. |
| `assets` | `Map<String, AssetDigest>` | Relative asset path mapped to `{ "sha256": "<hex>", "size_bytes": <u64> }`. |
| `components` | `Vec<ComponentRecord>` | Hierarchical `mbd-core` rigid-body mass, inertia, fluid, and payload tree. |
| `dynamics` | `DynamicsSpec` | Flight dynamics binding (e.g. `JsbsimXml` virtual filesystem mapping). |
| `controls` | `ControlsSpec` | Cockpit system capabilities (`has_magnetos`, `has_mixture`, `has_flaps`, etc.). |
| `devices` | `Vec<MountedDeviceSpec>` | COTS sensors (`Icm42688` IMU, `Bmp390` barometer, `PinholeCamera`) attached to component sites. |

### 1. `components`: The `mbd-core` Mass & Site Tree

Each entry in `components` defines a physical station in body coordinates (`+X` forward, `+Y` right, `+Z` down in metres) relative to its parent component:

- **`airframe` (`parent: null`)**: Carries the structural `dry_mass_kg` and $3\times 3$ `dry_inertia_body` ($\text{kg}\cdot\text{m}^2$).
- **`fuel_tank`**: Carries `initial_fluid_mass_kg` (used by `sim_pack::estimate_design_performance` as the default usable fuel capacity when no `TailProfile` overrides it).
- **`payload_station`**: Carries `initial_payload_mass_kg` (crew + baggage mass).
- **Sensor & camera mount sites** (`imu_mount`, `baro_mount`, `fpv_camera_mount` / `nose_camera_mount`): Zero-mass kinematic attachment frames for `devices`.

### 2. `dynamics`: Virtual JSBSim Resolution

`Simulate` runs natively and in `wasm32-unknown-unknown` without touching the host filesystem during flight. The `JsbsimXml` block maps package assets into an in-memory `jsbsim::MemoryResolver`:

```json
"dynamics": {
  "JsbsimXml": {
    "aircraft_dir": "aircraft",
    "model_name": "c172_class_reference",
    "entry_asset": "fdm/c172_class_reference.xml",
    "virtual_path": "aircraft/c172_class_reference/c172_class_reference.xml",
    "additional_assets": []
  }
}
```

When `LoadedSyspack::build_jsbsim_resolver` runs, `entry_asset` (and any `additional_assets` such as external engine or propeller XML files) are mounted at their `virtual_path` inside `MemoryResolver`.

### 3. `controls`: Cockpit System Latching

`ControlsSpec` tells `sim-core` and `simulate-app` which auxiliary controls (`sim_core::SystemControls`) apply to the aircraft:

```json
"controls": {
  "has_magnetos": true,
  "has_starter": true,
  "has_mixture": true,
  "has_prop_pitch": false,
  "has_fuel_cutoff": true,
  "has_flaps": true,
  "has_retractable_gear": false,
  "has_wheel_brakes": true
}
```

### 4. `devices`: Mounted IMU, Barometer & Optical Camera

Every `MountedDeviceSpec` binds a deterministic sensor model to a named site in `components`:

- **`Icm42688`**: 6-axis MEMS IMU with deterministic PRNG `seed`.
- **`Bmp390`**: Digital pressure barometer on an I2C bus/address.
- **`PinholeCamera`**: Offscreen optical sensor with body lever arm `offset_body_m`, quaternion `body_to_camera_xyzw`, pinhole intrinsics (`width`, `height`, `fx`, `fy`, `cx`, `cy`), and frame rate (`rate_hz`, typically `30 Hz`).
