# Simulate Repository Instructions

## Documentation and Code Quality Gates

All code and documentation in `Simulate` must pass `./scripts/ci.sh`:

- Write clear, concise documentation comments on all public items (`missing_docs = "deny"`).
- Do not use `unsafe`, `unwrap`, `expect`, `panic`, `todo`, `unimplemented`, `unreachable`, `exit`, or indexing/slicing (`#forbid` / `#deny` in `Cargo.toml`).
- Follow `scripts/check-structure.sh`:
  - Do not create `mod.rs`, `utils.rs`, `helpers.rs`, or `common.rs`. Use `name.rs` + `name/` directory layout.
  - Keep every `.rs` file at or below 500 lines (`lib.rs` at or below 100 lines, starting with a `//!` crate doc comment).
  - Keep every function at or below 80 lines.
  - Do not write `+= 1` in non-test code; use `.wrapping_add(1)` for monotonic counters.

---

## Visual Navigation Validation & Closed-Loop Simulation Needs (`Navigate` Integration)

`Simulate` serves as the deterministic, closed-loop 6-DoF simulation testbed to validate and refine `Navigate`'s real-time visual positioning and sensor-fusion stack (`NVKTILE1` corridor landmark banks, `25-30 Hz` `ActiveTrackSet` surface tracking, `AmortizedMasker`, `VisualSession`, `VisualFixSource`, and `navigate-fusion`).

When building out `sim-scenery`, `sim-render`, `sim-core`, and `simulate-app` for visual navigation validation, implement the following four components:

### 1. Terrain Elevation and Orthoimagery Scenery Provider (`sim-scenery`)

`Navigate` requires real orthoimagery texture and terrain relief rather than the bare graticule ellipsoid (`EllipsoidSurface` / `Deferred::IMAGERY`).

- **Catalog-backed `ElevationSource` and `ImagerySource`**:
  - Add a scenery provider that loads `Navigate` reference tile catalogs (`catalog.json` with SHA-256-verified Web Mercator orthoimagery PNG/JPEG tiles and Mapzen Terrarium elevation PNG tiles, such as the `Delaware` or `Oryol` datasets).
  - Decode Terrarium elevation pixels (`(R * 256.0 + G + B / 256.0) - 32768.0` metres) and sample bilinear ground height in `ElevationSource::ground_height_m(&self, position: &Geodetic)`.
  - Serve RGBA8 orthoimagery rasters in `ImagerySource::imagery(&self, tile: TileId)` with explicit coverage masks so missing tiles return `Ok(None)` or `SceneryError::Missing` instead of zero-height sea-level artifacts.
- **Terrain Patch Displacement and Ortho Draping (`sim-render`)**:
  - Displace globe/corridor mesh vertices along the WGS84 ellipsoid normal (`sim_geo::LocalFrame::up`) using the resident DEM height and sample draped orthoimagery tiles in the fragment shader, while keeping camera-relative `f64` patch offsets (`patch_centre_ecef - eye_ecef`) so there is zero `f32` jitter at any location on Earth.

### 2. Offscreen Pinhole / Mounted Camera Sensor Plugin (`sim-render` & `sim-core`)

To feed synthetic camera frames into `Navigate` (`XFeat`, `LighterGlue`, `Segformer`, `PropellerDetector`, and `corridor-track`), `Simulate` needs a mounted camera sensor that renders offscreen at deterministic simulation ticks.

- **Configurable Camera Mount and Intrinsics**:
  - Define `CameraMount` with body-to-camera lever arm `offset_body_m: DVec3` and rotation `body_to_camera: DQuat` (supporting forward nose cameras, nadir belly cameras, and oblique side-window cameras with arbitrary roll/pitch/yaw offsets).
  - Define pinhole intrinsics `SensorIntrinsics { width: u32, height: u32, fx: f64, fy: f64, cx: f64, cy: f64 }` (for example, `800 x 600` or `1600 x 1200` matching CIPA DC-008 35 mm equivalents) and convert cleanly between `sim_render::CameraPose` (`+X` right, `+Y` up, `-Z` forward in ECEF) and `Navigate`'s local ENU camera convention (`eye_enu: [f64; 3]`, `world_to_cam: [f64; 9]` with `+X` right, `+Y` down, `+Z` forward).
- **Offscreen Render Target and Depth Readout**:
  - Add an offscreen `wgpu::Texture` colour target (`RGBA8UnormSrgb` with `RENDER_ATTACHMENT | COPY_SRC | TEXTURE_BINDING`) and `Depth32Float` reversed-Z depth buffer so `sim-render` can render camera frames headlessly (in CI or batch evaluation) or alongside the interactive desktop/web window.
  - Optionally render a synthetic airframe window bezel and harmonic propeller blade disk overlay in camera coordinates so `Navigate`'s `AmortizedMasker` (`Segformer` + attitude-warped sky mask + `PropellerDetector`) can be stress-tested against known ground-truth occlusion masks.
- **`CameraSensorBlock` (`sim_core::Block`)**:
  - Implement `sim_core::Block` at a master-clock-compatible rate (such as `30 Hz`, `TICKS_PER_SECOND / 30`) that captures `(SimInstant, VehicleState, CameraPose, SensorIntrinsics)` and emits frame payloads for visual navigation.

### 3. Deterministic Truth vs. Visual Estimate Harness (`sim-core`)

Real flight recordings lack sub-metre 6-DoF ground truth and cannot test closed-loop guidance. `Simulate` must expose an evaluation harness that compares `Navigate`'s outputs against exact simulation truth at every tick:

- **Exact Ground-Truth Telemetry**:
  - At each `SimInstant`, record exact `VehicleState` (`position_ecef`, `velocity_ecef`, `attitude`, `angular_rate_body`), exact camera ENU pose `(true_eye_enu, true_world_to_cam)` relative to the corridor catalog anchor, and true optical-axis terrain depth.
- **Two-Tier Latency and `MapCell` Integrity Verification**:
  - Model realistic processing delays on the `sim_core` master clock using `RateAdapter`:
    - **Fast warm-track loop (`30 Hz`, `~35 ms` delay)**: `surface_track_odometry` (`independent_ekf_fix = false`, `fusion_eligibility = "correlated_track"`).
    - **Async corridor anchor loop (`1-2 Hz`, `~150-450 ms` delay)**: `corridor_map_anchor` gated by `500 m x 500 m` `MapCell` deduplication (`independent_ekf_fix = true` only on the first fix in a new `MapCell`, `"shared_map_cell"` on revisit).
  - Verify that feeding `corridor-track` outputs into `navigate-visual-fusion` (`VisualFixSource`) and `navigate-fusion` keeps EKF Normalized Estimation Error Squared (NEES) consistent without overconfident covariance collapse when hovering, orbiting in the same `MapCell`, or flying across seasonal appearance shifts.

### 4. Flight Dynamics and Scenario Profiles

- Support both `sim_core::KinematicModel` and `jsbsim-rs` (`Fdm::snapshot_into` / `Fdm::restore`) as interchangeable `sim_core::Block` flight dynamics sources.
- Provide deterministic scripted flight profiles over catalog regions (such as the Delaware corridor):
  1. **Nominal corridor cruise (`30 Hz`)**: straight and gently banking flight across multiple `600 m` `NVKTILE1` tiles and `500 m` `MapCell` boundaries.
  2. **Off-seed lateral/altitude offset acquisition**: cold-start recovery `+650 m` to `+1,500 m` off the nominal track and `+200 m` altitude offset.
  3. **Aggressive bank / pitch / horizon-loss manoeuvres**: testing `AmortizedMasker` attitude sky-mask warping (`r_key * r_cur.transpose()`) between `Segformer` refreshes and `ActiveTrackSet` re-anchoring.
  4. **Same-cell loiter / orbit**: verifying that repeated anchors inside a single `MapCell` are marked `shared_map_cell` (`independent_ekf_fix = false`) and do not corrupt the EKF covariance.
