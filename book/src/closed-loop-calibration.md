# POH & Empirical Flight-Test Calibration

A flight dynamics model (`fdm/*.xml`) contains dozens of coupled aerodynamic, propulsion, and mass parameters: lift curve slope $C_{L\alpha}$, flap increments $\Delta C_L / \Delta C_D$, zero-lift drag $C_{D0}$, induced drag efficiency $e$, high-alpha flow-separation drag, propeller $C_T(J)$ and $C_P(J)$ polars, piston engine volumetric efficiency, BSFC, and mechanical friction horsepower.

Checking individual tables in isolation does not guarantee that the integrated aircraft stalls, climbs, glides, or cruises at its published Pilot's Operating Handbook (`POH`) numbers. Instead, **`sim_pack::calibration`** executes a **declarative, closed-loop 6-DoF flight-test campaign** against `performance/poh_benchmarks.json`.

---

## Five Closed-Loop Flight-Test Regimes

When `evaluate_package(&package)` (or `ironbird --mode calibrate-dir --package-dir <path>`) runs, `sim-pack` instantiates `jsbsim::Fdm` at `120 Hz` (`DT_S = 1/120 s`) in strict mode and flies five regimes:

1. **Static Full-Throttle Ground Runup (`static_runup`)**
   - Places the aircraft on the runway at `pressure_alt_ft` with wheel brakes locked (`fcs/left-brake-cmd-norm = 1.0`, `fcs/right-brake-cmd-norm = 1.0`) and wide-open throttle (`fcs/throttle-cmd-norm = 1.0`).
   - Integrates for `4.0 s` (`480` steps) until propeller RPM settles, verifying that static WOT RPM lies within the Type Certificate Data Sheet (`TCDS`) window `[min_rpm, max_rpm]`.
   - *Physical sensitivity*: isolates low-advance-ratio propeller power absorption $C_P(J \approx 0)$ and sea-level engine torque.

2. **Power-Off Stalls Across Flaps & Bank Angles (`stalls`)**
   - Configures exact `gross_mass_kg` via `inertia/pointmass-weight-lbs[0]`, sets `fcs/flap-pos-norm = flap_norm` and `fcs/throttle-cmd-norm = 0.0`, and sweeps angle of attack $\alpha \in [0.18, 0.36]\text{ rad}$ to find peak aerodynamic lift $L_{\max}$.
   - Computes calibrated stall speed at coordinated bank angle $\phi$ ($n_z = 1/\cos\phi$):
     $$V_{S,\text{KCAS}} = V_{\text{ref}} \sqrt{\frac{W \cdot n_z}{L_{\max}(V_{\text{ref}})}}$$
   - Maps $V_{S,\text{KCAS}} \to V_{S,\text{KIAS}}$ through `performance/pitot_static_calibration.json` (`PEC`) and grades both `KCAS` and `KIAS` residuals against `tolerances.stall_kt`.

3. **Full-Throttle Steady Climb Across Altitude (`climb_points`)**
   - Initializes the aircraft at `pressure_alt_ft` and `gross_mass_kg` with `fcs/flap-pos-norm = 0.0` and `fcs/throttle-cmd-norm = 1.0`.
   - Flies a closed-loop pitch-attitude airspeed hold (`PI+D` elevator controller with wings-level roll/yaw damping) tracking `climb_kcas` for `30 s` (`3,600` steps), measuring settled vertical speed over the `[24 s, 30 s]` window after phugoid transients decay.
   - *Mixture physics*: When `lean_best_power = true` (or above `3,000 ft`), mixture is leaned to peak power (`equivalence_ratio ≈ 1.34` on JSBSim's `FGPiston` combustion efficiency ridge).

4. **Steady Level Cruise & Fuel Flow (`cruise_points`)**
   - Flies closed-loop altitude-hold (`theta` outer loop) and propeller-RPM-hold (`throttle` PI loop when `target_rpm` is specified, or fixed throttle when `fixed_throttle` is specified) for `30 s` (`3,600` steps).
   - Leans mixture to POH recommended cruise lean (`50°F` rich of peak EGT, `equivalence_ratio ≈ 1.28`, `mixture_efficiency = 1.000`).
   - Grades settled true airspeed (`KTAS`), percentage of rated brake horsepower (`%BHP`), and fuel flow (`GPH`) over `[24 s, 30 s]` against `tolerances.{cruise_ktas, cruise_pct_bhp, cruise_gph}`.

5. **Empirical Recorded Flight-Segment Replay (`empirical_segments`)**
   - Replays real-world recorded flight segments (`LevelCruiseRpm`, `LevelCruiseThrottle`, or `FullThrottleClimbKcas`) at their recorded pressure altitude, estimated takeoff mass, and along-track wind component $V_{\text{wind}}$, comparing simulated groundspeed
     $$V_{\text{GS,sim}} = V_{\text{KTAS,sim}} + V_{\text{wind}}$$
     and rate of climb against GPS/ADAHRS observations.
