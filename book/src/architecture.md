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

Body positions come from NASA NAIF kernels cut to the spans the simulator
flies: DE440 for 2026 to 2028, and the two weeks of the Apollo 11 mission in
July 1969 for its scenarios. Each embedded subset is checked against a pinned
SHA-256 before use, and a time outside them is an error, not an extrapolation.

Measured Mars terrain is embedded the same way: a 256 by 256 window of NASA PDS
MOLA MEGDR topography (`MEGT44N000HB`, 128 pixel/degree, about 17.4 to 19.4 N and
76.5 to 78.5 E, around Jezero crater), cut without resampling by
`crates/sim-analysis-contract/scripts/extract_mola_jezero.py` and checked against a
pinned SHA-256 by `DemElevationGrid::mars_jezero_mola`. MOLA heights are above the
areoid and its latitudes are planetocentric; the loader adds the mean
areoid-over-ellipsoid offset (about 25 m rms residual, 100 m worst case) and
converts the latitude bounds to geodetic.

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

Away from Earth the same question is asked against TDB.
`BarycentricClockModel::proper_rate_offset` gives `dτ/dTDB - 1` from the
potential of the Sun, Earth, the Moon and the planets out to Saturn and the
clock's barycentric speed, with `L_B` so that a geoid clock keeps TDB on
average. Master-clock ticks are uniform TDB, so that rate integrates over
ticks as it stands. Against the built-in ephemeris:

- a clock on the Moon gains 55.98 µs a day on a geoid clock (Ashby and Patla
  give 56.02);
- a clock on Mars gains 487.8 µs a day, averaged over a Martian year. Ashby
  and Patla's solar-tide treatment gives 477, 2 % less than this first-order
  rate;
- the two formulas agree on a GPS clock to `2e-14`, once
  `tdb_minus_tt_position_term_s` accounts for clocks that agree in TT
  standing apart in TDB by where they are.

`sim_devices::OscillatorBlock` is a frequency standard that counts with
either rate and carries its factory offset.

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
- A `TierPolicy` makes those handoffs by itself. It holds, per body, the
  height above which a local vessel goes orbital and a lower height below
  which an orbital one comes back, so a vessel skimming one threshold does
  not flip. It acts only at whole multiples of its check period, and a local
  plant in ground contact, in air or under power holds its tier.
  `Universe::take_tier_changes` reports what it did.
- A plant says whether orbital propagation can carry it at all. An aircraft
  cannot, so it stays in flight dynamics whatever a policy says.
- `Universe::checkpoint` and `restore` take the clock and every orbital
  vessel back to an earlier instant. A local plant's state stays with
  whoever built it: they reach it through `Universe::local_mut`, put it back
  first, then restore the universe.
- `Universe::command` stamps a burn law or another command on an orbital
  vessel at an instant. It takes effect at exactly that instant.
- `simulate-app`'s free flight is one such vessel. Each free flight has a
  `Universe` on its `SimEpoch`, and the flight's `SimSession` is the
  ownship's local plant in it, so stamped controls, sensor exposures and
  rewind checkpoints all sit on the clock other vessels share. An aircraft
  stays in the local tier. Scripted scenarios still step their own run.
- A spaceflight is a `Universe` with a tier policy and `sim_raketa`'s
  `RaketaPlant` as the ownship. It leaves the pad in flight dynamics, coasts
  in orbital propagation, and is handed back whenever the pilot flies it.
- `scripts/check-architecture.sh` refuses any `sim-core` dependency on the
  orbital stack, so `sim-universe` stays the only bridge.

An Earth-centred orbital propagation must include the Sun and the Moon. The
frame's own acceleration towards them is always applied, and only their pull on
the vessel cancels it. An Earth-only force model drifts by hundreds of metres in
minutes. The test `flight_and_orbital_dynamics_agree_across_a_handoff` flies
one orbit both ways. It allows the two tiers to differ only by the solar and
lunar tide, which `RigidBody6DofBlock` does not model.

## Libration-point orbits

Gravity is n-body throughout, so an orbit about Sun-Earth L2 needs no special
frame or patched sphere of influence: it is a trajectory the full ephemeris
and solar radiation pressure carry, and it is unstable. `sim-mission` plans a
first arc about the point (`plan_assisted_l2_bounded_arc`) and then holds it
(`fly_l2_station_keeping`): every few weeks a burn along the Sun-Earth line,
sized so that the spacecraft crosses the plane through that line with no
velocity along it a revolution later. A 6,200 kg spacecraft with a 160 m²
sunshield takes about 5 m/s to settle from the planner's arc and under 4 m/s
a year after that; left alone it is gone within seven months.

## Signals

A link between two ends is judged by `sim-analysis` either geometrically,
with both ends at the sample instant, or with the signal's travel time.
Under `SignalTimeModel::LightTime` the source is taken where it was when the
signal left, the target where it is when the signal arrives, and each
occulting body where it is as the signal passes. Between Mars and Earth the
ends are 3 to 22 minutes apart, so a window edge set by one planet's horizon
is seen a light time later at the other, and uplink and downlink windows
differ. Multi-hop relays are geometric only. Requests that do not name a
model are geometric.

## Sunlight

Everything that feels the Sun reads one model.
`sim_spacetime::SolarFluxModel::sample` gives, at a barycentric position and
an instant, the direction to the Sun, the irradiance by the inverse square of
its distance, and the fraction of the solar disc that Earth, the Moon or Mars
leave in view. From that one sample:

- `sim_orbital::SolarRadiationPressureLaw` adds the light's momentum to a
  vessel's environment force, wrapped around whatever law flies it;
- `sim_devices::SolarArrayBlock` turns it into electrical power, in orbit or
  on a surface;
- `AtmosphericTransmission` takes off what a clear atmosphere absorbs from
  the direct beam on the way down.

So force, power and eclipse timing cannot disagree. The renderers'
`EnvironmentFrame` still takes its Sun from the sky rig's own ephemeris.

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

The sky rig's ephemeris and the one the dynamics integrate in are separate
models, held together by a test: over Earth they place the Moon within
`0.001°` of each other and the Sun within `0.01°`, the Sun's share being the
aberration of light, which the sky shows and geometric positions leave out.

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
