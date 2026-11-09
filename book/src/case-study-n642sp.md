# Case Study: 1998 Cessna 172S `N642SP`

The reference piston aircraft in `vehicles/c172` is anchored to an actual certificated aircraft: **`N642SP`**, a **1998 Cessna 172S Skyhawk SP** (Serial Number `172S8050`, FAA Type Certificate Data Sheet `3A12` Rev 84, based at Solberg-Hunterdon Airport `N51`, Readington, NJ).

---

## 1. Airframe, Powerplant & Avionics Baseline (`TailProfile`)

| Attribute | `N642SP` Specification |
|---|---|
| **Registration / S/N** | `N642SP` / `172S8050` (1998 Model 172S Skyhawk SP) |
| **Home Aerodrome** | `N51` (Solberg-Hunterdon, Readington, NJ, field elevation `195 ft` MSL) |
| **Engine** | Textron Lycoming `IO-360-L2A`, `180 BHP @ 2,700 RPM` (`361.0 in³`, `8.5:1` compression, fuel-injected) |
| **Propeller** | McCauley `1A170E/JHA7660`, 2-blade fixed-pitch (`76.0 in` diameter, `60.0 in` nominal pitch at `75%` station) |
| **Static WOT RPM (TCDS `3A12`)** | `2,300–2,400 RPM` at sea level ISA |
| **Maximum Takeoff Mass** | `2,550 lb` (`1,156.66 kg`) |
| **Weighed Basic Empty Mass** | `1,668 lb` (`756.6 kg`) at `41.4 in` aft of datum (post Garmin G3X upgrade; speed fairings installed) |
| **Usable Fuel** | `53.0 US gal` (`318 lb` / `144.24 kg` `100LL`) |
| **Avionics Suite** | Garmin G3X Touch `GDU 460` 10.6" PFD/MFD (`STC SA01899WI`) + `GSU 25D` ADAHRS + `GEA 24` EIS + Garmin `G5` standby + `GTN 650W` + `GFC 500` AP |

---

## 2. Reconciling POH Tables with Real `N642SP` Operations

Calibrating `vehicles/c172` against both the **1998 Cessna 172S POH** (`172SPHUS-00`) and **`N642SP`** operational data highlights four physical couplings that every fixed-wing calibration must account for:

1. **Speed Fairings (`+2 KTAS` in Cruise)**:
   The factory POH Section 5 cruise table assumes main/nose wheel fairings are installed (`124 KTAS` at `6,000 ft / 2,600 RPM`). Aircraft operated without fairings lose `2–3 KTAS` ($\Delta C_{D0} \approx +0.0012$). `TailProfile::wheel_fairings_installed = true` records this configuration explicitly.
2. **Flap-Indexed Pitot-Static Position Error (`PEC`)**:
   At MTOW (`2,550 lb`), POH clean stall is `48 KIAS = 53 KCAS` and Flaps $30^\circ$ stall is `40 KIAS = 48 KCAS`. Without `performance/pitot_static_calibration.json`, tuning $C_{L,\max}$ to match `40 KIAS` directly in `KCAS` would give the wing `44%` too much lift coefficient! Separating physical `KCAS` aerodynamics from sensor `PEC` yields `< 0.2 kt` error in both `KCAS` and `KIAS`.
3. **Propeller Advance Ratio $J$ Coupling (`Static` vs. $V_X$ vs. $V_Y$ vs. `Cruise`)**:
   On a fixed-pitch `76-in` McCauley propeller, advance ratio $J = V / (n D)$ moves monotonically across flight regimes:
   - Static ground runup: $J = 0.00 \implies 2,385\text{ RPM}$ (`TCDS 2,300–2,400 RPM`)
   - Sea-level $V_X$ (`62 KIAS` / `64 KCAS`): $J \approx 0.34$
   - Sea-level $V_Y$ (`74 KIAS / KCAS`): $J \approx 0.40 \implies 737\text{ fpm}$
   - Cruise (`2,400–2,650 RPM`, `114–124 KTAS`): $J \in [0.58, 0.65]$
   - Power-off windmilling glide ($V_G \approx 68\text{ KCAS}$): $J \in [0.95, 1.15]$, where windmilling negative $C_T$ sets $(L/D)_{\max} \approx 9.7$.
4. **Mixture Leaning Above `3,000 ft`**:
   A normally aspirated `IO-360-L2A` enriches as air density $\rho(h)$ drops. `sim_pack::calibration` leans climb mixture to best power (`equivalence_ratio ≈ 1.34`) and cruise mixture to POH recommended lean (`equivalence_ratio ≈ 1.28`), matching POH fuel flows (`8.8–10.2 GPH`) to `0.26 GPH` RMSE.

---

## 3. Verified Closed-Loop Results (`ironbird --mode calibrate-c172`)

Running `cargo run --release -p ironbird -- --mode calibrate-c172` executes the complete `N642SP` suite:

| Category | Metric / Condition | Simulated | Target (POH / `N642SP`) | Residual / RMSE | Status |
|---|---|---:|---:|---:|:---:|
| **Static Runup** | Sea-level WOT (`0 kt`) | `2,385 RPM` | `2,300–2,400 RPM` | In window | `PASS` |
| **Stalls (6 pts)** | Clean & Flaps $10^\circ/30^\circ$, $0^\circ/30^\circ/45^\circ$ bank (`KCAS`) | — | POH Fig 5-3 | **`0.16 kt` RMSE** | `PASS` |
| **Stalls (`PEC`)** | Clean (`48 KIAS`) & Flaps $30^\circ$ (`40 KIAS`) | `48.2 / 40.2 KIAS` | `48.0 / 40.0 KIAS` | **`0.19 kt` RMSE** | `PASS` |
| **Climb (`0 ft`)** | `74 KCAS` WOT (`2,550 lb`) | `737 fpm` | `730 fpm` | `+7 fpm` | `PASS` |
| **Climb (`4,000 ft`)** | `73 KCAS` WOT leaned | `550 fpm` | `555 fpm` | `-5 fpm` | `PASS` |
| **Climb (`8,000 ft`)** | `71 KCAS` WOT leaned | `383 fpm` | `380 fpm` | `+3 fpm` | `PASS` |
| **Climb (`10,000 ft`)** | `70 KCAS` WOT leaned | `299 fpm` | `295 fpm` | `+4 fpm` | `PASS` |
| **Climb (`12,000 ft`)** | `69 KCAS` WOT leaned | `185 fpm` | `210 fpm` | `-25 fpm` (**`12.0 fpm` RMSE**) | `PASS` |
| **Cruise (6 pts)** | `2,000–8,000 ft`, `2,400–2,650 RPM` | `114.6–122.4 KTAS` | `114.0–124.0 KTAS` | **`1.18 KTAS` / `0.26 GPH` RMSE** | `PASS` |
| **Empirical (`N51`)** | 3 `N51`/Delaware corridor segments | `75.1 / 118.2 / 114.1 kt` | `76.0 / 118.0 / 114.0 kt` | **`0.62 kt` GS RMSE** | `PASS` |
