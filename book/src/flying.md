# Flying the Simulator

The bindings below are the defaults. The flight keys, gamepad presets, and
stick assignments can all be changed in the Flight Hub's controls tab. The
code is authoritative: flight keys live in `apps/simulate-app/src/controls/keys.rs`,
cockpit keys in `apps/simulate-app/src/keyboard.rs`, hub keys in
`apps/simulate-app/src/esc_menu/keymap.rs`, and gamepad presets in
`apps/simulate-app/src/controls/preset.rs`.

## Vehicles

`V` cycles the free-flight vehicle, and the hub's hangar tab picks one
directly:

| Vehicle | Flight model | What the stick commands |
|---|---|---|
| C172 Skyhawk | JSBSim, `vehicles/c172` | Control surfaces and throttle directly |
| Lockheed P-38G Lightning | JSBSim, `vehicles/p38g` | Control surfaces and throttle directly; opens on its period panel with the HUD and glass dock hidden |
| X500 quadrotor | Rigid body flown by Aviate's multirotor controller in-process | Velocity: stick for horizontal speed, throttle above or below centre to climb or descend, rudder for yaw rate |
| Fixed-wing fly-by-wire | JSBSim C172 flown by Aviate's fixed-wing controller in-process | Bank and pitch attitude, throttle, rudder for heading |
| C172 on a Pilotage mission | The same JSBSim C172 and Aviate controller, commanded in MAVLink by Pilotage's host core, with Navigate's navigation sets beside it on the same sensors | Nothing: it starts in the air near Zaragoza on the first shareable link's route, Pilotage arms and flies it, and the window title shows the mission phase and each navigation set's error. Restart flies it again; it cannot be rewound |
| Kinematic | Kinematic 6-DoF model | Direct |

The hangar's Raketa starts a spaceflight instead. The rocket stands on Launch
Complex 39A and ascent guidance flies it to a 185 km parking orbit, where
guidance lets go and orbital propagation carries it. `M` takes it from
guidance at any time. In your hands the throttle is main-engine thrust up to
4 g, with unlimited propellant, and the stick swings the nose: forward and
back tilt it away from and towards the chase view, the rudder across it.
Touching the throttle or stick in orbit brings the vessel back to flight
dynamics, and it returns to orbital propagation once it coasts above the
atmosphere again. A spaceflight has no rewind history, and it ends with an
error if the rocket comes back down to the ground.

The two Aviate vehicles run a Tier 0 loop inside the app. See
[Flying an External Autopilot](external-autopilots.md#tier-0-aviate-in-process).

## Keyboard

Flight keys act while held:

| Key | Action |
|---|---|
| `Left` / `Right` | Roll |
| `Down` / `Up` | Pitch up (stick back) / pitch down |
| `A` / `D` | Rudder left / right |
| `W` / `S` | Throttle up / down |
| `B` | Wheel brakes |
| `E` | Starter |

Cockpit keys act once per press:

| Key | Action |
|---|---|
| `Esc` | Open or close the Flight Hub |
| `Space` | Pause or resume |
| `]` | Cycle time warp |
| `V` | Next free-flight vehicle |
| `K` | Next aerodrome; respawns there |
| `Tab` | Cycle the operator view: cockpit, chase, sensor aperture, chart map, volumetric scene |
| `C` | Cycle the virtual camera |
| `H` | Show or hide the glass PFD/MFD/HSI dock |
| `P` | Cycle the avionics preset: gauges, single moving map, glass cockpit |
| `U` | Show or hide the full-window conformal HUD |
| `J` | Expand or minimize the EFB moving map |
| `L` | Cycle sky time: auto, live UTC, dawn, noon, dusk, night |
| `N` | Cycle onboard sensor realism: realistic, degraded navigation, ideal |
| `F` / `G` | Flaps down / up one notch |
| `Y` or `[` | Landing gear up/down (retractable gear) |
| `O` | Landing light |
| `M` | In a scenario or a spaceflight, take manual control. In free flight, cycle the magnetos: both, left, right, off |
| `R` / `T` | Mixture richer / leaner |
| `Q` | P-38G flap hold valve |
| `I` | P-38G coolant and oil cooler flaps |
| `Z` / `X` | P-38G feather left / right propeller |
| `Cmd+Q` / `Super+Q` | Quit |

Aircraft system keys only act on aircraft that have the system. The package's
`controls` section declares which ones it has (see [The `syspack` Vehicle
Contract](syspack-contract.md)). A piston engine needs leaning at altitude just
as a real one does.

## Gamepads, joysticks, and yokes

Gamepads and HID devices go through gilrs: OS HID natively, the Gamepad API in
the browser. The preset follows the vehicle class unless you choose one in the
controls tab.

| Preset | Layout |
|---|---|
| Fixed-Wing Gamepad (default for aeroplanes) | Left stick yoke, `LT`/`RT` rudder, `LB`/`RB` throttle (latching), right stick free for camera look |
| Fixed-Wing Mode 2 Dual-Stick | Right stick yoke, left stick throttle and rudder |
| Quad Cam-Drone Mode 2 (default for multirotors) | Left stick climb (centred) and yaw, right stick roll and pitch |
| Quad FPV Acro Mode 2 | Left stick direct throttle and yaw, right stick roll and pitch |
| Flight Stick / Yoke - HOTAS | Axis X roll, Y pitch, slider throttle, Rz twist yaw |

Every preset also binds HID axis 0 to roll, axis 1 to pitch, and axis 2 to a
throttle lever, so a yoke works whichever gamepad preset is active.

## Scenarios

Scripted scenarios fly themselves and replay identically every run. A number
key starts one from the beginning, and `0` returns to free flight. The hub's
scenarios tab lists the same set.

| Key | Scenario |
|---|---|
| `1` | Delaware corridor cruise |
| `2` | Off-seed acquisition |
| `3` | Aggressive bank |
| `4` | Same-cell loiter |
| `5` | Apollo 11 lunar landing |
| `6` | Apollo 11 command module reentry |
| `7` | `47N` C172 traffic pattern (uncontrolled, CTAF) |
| `8` | `KEWR` C172 traffic pattern (towered) |
| `0` | Free flight |

## Flight Hub

`Esc`, or `Start` on a gamepad, opens the hub and holds the simulation. It has
five tabs, and `F1` to `F5` jump to them:

| Tab | Contents |
|---|---|
| Flight & Timeline | Resume, restart, quit, take manual control, the rewind timeline, simulation rate, sky time, avionics layout, scenery mode, pacing |
| Aircraft Hangar | Airframe and flight-dynamics choice, with a live 3D preview |
| World & Aerodromes | Ranked aerodrome list, search, 3D globe, spawn mode |
| Scenarios | The scripted scenarios |
| Controls & Presets | Gamepad preset, live axis readout, axis rebind/invert/deadzone/expo, key rebinding |

To navigate the hub, use the arrow keys or the D-pad. `Enter` or `South`
confirms, `East` backs out, and `Tab`/`Shift+Tab` or the bumpers change tabs.
The mouse works throughout, and typing goes into the aerodrome search.

### Rewind

The flight keeps checkpoints at a fixed interval of simulated time, about a
minute of history (`apps/simulate-app/src/flight/rewind.rs`). On the flight
tab, the timeline steps back or forward, jumps to any point on the bar, or
returns to live. Scrubbing does not discard anything, and you can go back to
the live frontier exactly. Resuming from a scrubbed point drops the checkpoints
after it.
