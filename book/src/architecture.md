# Architecture

This chapter covers the invariants the simulator is built around, and the crate
that owns each one. Gates in `scripts/` enforce them where code can.

## Time

Every simulated component steps on one integer master clock, `SimInstant`, at
`TICKS_PER_SECOND` ticks per second. Rates that share nothing, such as a 120 Hz
flight model and the AGC's 12/1,024,000 s memory cycle, interleave exactly, so
a run replays bit for bit at any frame rate.

`sim_spacetime::SimEpoch` is the one mapping from ticks to absolute time for a
run. Ticks advance TDB uniformly. UTC, UT1 and TT are read from one
`TimeScaleCatalog`:

- the built-in NAIF leap-second kernel;
- the IERS steered-rate table for UTC before 1972, so 1960s flights sit on the
  same time line as the ephemerides;
- a frozen `TAI - UTC` past the kernel.

Flight dynamics, orbital propagation, the sky, air data, the supervisor and the
cockpit displays all read time through the flight's `SimEpoch`. Scenarios take
theirs from `ScenarioProfile::start_utc_unix_s`, the only table of start
times. Free flight starts at the wall-clock second it begins, then follows sim
time through pause and rewind. `scripts/check-clock.sh` rejects hand-written
`utc_at` functions and epoch constants anywhere else.

### Clocks that move

A simulated oscillator counts proper time, not TDB.
`sim_spacetime::ClockPotential::proper_rate_offset` gives `dτ/dTT - 1` to
order `1/c²`, from Earth's monopole and `J2` and the IAU `L_G` convention.
With it:

- a clock on the geoid keeps TT;
- a GPS clock gains 38.6 µs a day;
- a clock in low orbit loses time.

`ProperTimeClock` integrates the rate in a block's snapshot.
`orbit_periodic_clock_term_s` is the eccentricity term from IS-GPS-200. Take
the integration steps from `SimEpoch::tt_j2000_seconds_at`, not from ticks.
TDB's annual swing against TT is about as large as the GPS effect.

## Vessels and fidelity tiers

`sim-core` flies a vehicle in its central body's rotating, body-fixed frame:
JSBSim, `RigidBody6DofBlock`, ground contact. `sim-orbital` propagates in ICRF
axes about a primary body, with a full solar-system force model. They meet in
`sim-universe`:

- `Universe` holds every vessel of a run, in id order, on the master clock.
- A vessel is `Local`, flown by a `LocalPlant` on its own period, or
  `Orbital`, held by the one `sim_orbital::OrbitalUniverse` the `Universe`
  owns. That scheduler integrates on a fixed quantum grid, orders events and
  stamped commands, and samples densely in between. Either way, the result
  depends only on the instants asked for, never on how a frame loop slices
  time.
- `promote_to_orbital` and `demote_to_local` hand a vessel over through
  `to_inertial` and `to_body_fixed`, the exact frame change, at any instant
  its local step has reached. Leaving orbit integrates the vessel to exactly
  that instant first. Body orientation comes from the orbital propagator's
  own catalog, so a vessel does not jump.
- Gravity is n-body everywhere, so an orbital vessel has no sphere of
  influence, only an origin its motion is integrated about. The scheduler
  moves that origin between Earth, the Moon, Mars and the solar-system
  barycentre as the vessel travels, and `Universe::central_body` reports it.
  A vessel comes back to flight dynamics about whichever body it then orbits.
- `Universe::command` stamps a burn law or another command on an orbital
  vessel at an instant. It takes effect at exactly that instant.
- `scripts/check-architecture.sh` refuses any `sim-core` dependency on the
  orbital stack, so `sim-universe` stays the only bridge.

An Earth-centred orbital propagation must include the Sun and the Moon. The
frame's own acceleration towards them is always applied, and only their pull on
the vessel cancels it. An Earth-only force model drifts by hundreds of metres in
minutes. The test `flight_and_orbital_dynamics_agree_across_a_handoff` flies
one orbit both ways. It allows the two tiers to differ only by the solar and
lunar tide, which `RigidBody6DofBlock` does not model.

## Threads

The simulation and the frame loop meet at `sim_core::SimHost`.

- The frame loop turns wall time into ticks with `RealTimeClock`, asks the
  host to simulate up to an instant (`request_until` / `request_by`), and draws
  the newest `DisplayFrame`. A `DisplayFrame` carries both plant outputs around
  the display instant and the phase between them.
- Wall time never reaches the host. Truth therefore depends only on the instants
  requested and on the stamped inputs.
- `InlineHost` steps the session on the caller's thread. It is used in the
  browser and in tests.
- `ThreadedHost` owns the session on a named native thread. It takes ordered
  commands over one channel and sends frames back over another. `latest()`
  never blocks, and sensor exposures from frames it skips are still delivered.
  Shutdown joins the thread or reports it stuck after a timeout.
- A test feeds both hosts the same jittery frame times and inputs and requires
  bit-identical frames. Choosing a host changes latency, never truth.

A host never refuses a request for running ahead. A real-time loop bounds its own
lag, as `Flight` does with its catch-up limit.

`simulate-app`'s `Flight` still steps inline. To move it onto `ThreadedHost`,
three things must change:

- Every per-frame read through `session.block()` must arrive in a cockpit
  snapshot captured on the simulation thread. These reads are specific force,
  engine status, articulation, armed state, system controls, faults, the panel
  observation and rewind checkpoints.
- System-control changes, rewind and reset must become ordered commands, so
  they land at deterministic instants.
- `PlantBlock` and its contents must be `Send`. `JsbsimFdmBlock` is. The Aviate
  rigs and `WorldSurfaceHandle` still need a compile-time check.

In the browser, a Worker would own the session. That needs COOP/COEP headers
and an atomics-enabled wasm build, so `InlineHost` stays the web default until
both are in place.

## Rendering

Three renderers draw the world:

- maplibre's truth scene draws the window and the 30 Hz sensor slot;
- `sim-render`'s `SensorAperturePass` draws simulated cameras and batch
  evaluation;
- `sim-render`'s fallback globe.

None of them decides alone what the sky or the air looks like.

Each frame, `SkyRig` evaluates the ephemeris at the flight's `SimEpoch` instant
and builds one `sim_render::EnvironmentFrame`. The frame is plain data in the
observer body's fixed axes. It holds:

- the Sun: direction, disc, eclipse fraction and illuminance;
- the twilight ramp;
- the body's atmosphere, `None` on an airless body;
- fog extinction;
- a cloud-layer handle;
- exposure.

Every renderer reads only this frame:

- The aperture and the fallback globe clear to `sky_linear_rgb_at(eye)` and
  fog terrain in `globe.wgsl`. That path is exact when fog is zero.
- The truth scene takes the Sun, twilight and atmosphere through
  `TruthSky::apply` and `sim-maplibre`'s public setters.

So a simulated camera and the pilot's window always see the same sky and air.
maplibre still shades its own sky from those inputs, so they match in input, not
yet pixel for pixel.

Weather plugs in at the frame:

- A weather block sets `fog.extinction_per_m` and publishes a cloud field
  whose handle goes in `cloud_layer`.
- Precipitation becomes extinction plus a streak overlay keyed off the same
  handle.
- Volumetric clouds raymarch in `sim-render` against each target's depth.

The window needs two more things from maplibre:

- fog and exposure setters on `TruthSceneMap`;
- per-slot access to its reversed-Z depth target and exact view-projection.

A physically based sky would replace `sky_linear_rgb_at`, using the
`rayleigh_per_m` and `scale_height_m` the frame already carries.
