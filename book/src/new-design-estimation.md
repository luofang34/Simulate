# Estimating Performance, V-Speeds & Range of New Designs

When designing a **new conceptual or preliminary aircraft** (for example, a new general-aviation configuration exported from `ace-rs`, a long-endurance fixed-wing UAV, or an electric/piston prototype in JSBSim XML), no Pilot's Operating Handbook (`POH`) or flight-test log exists yet. Instead of verifying the model against known numbers, the engineer needs the simulator to **discover the aircraft's flight envelope, climb ceiling, glide ratio, cruise specific range, endurance, and payload–range diagram**.

`sim_pack::estimate_design_performance` and `ironbird --mode estimate-*` provide this capability out of the box for any fixed-wing `syspack`.

```sh
# Estimate performance, V-speeds, ceilings, and range of any on-disk syspack directory
cargo run --release -p ironbird -- \
  --mode estimate-dir \
  --package-dir vehicles/<new-design> \
  --report-out <new-design>_performance.md

# Run on the built-in C172 package
cargo run --release -p ironbird -- --mode estimate-c172
```

---

## What `estimate_design_performance` Computes

Given only `syspack.json` (the `mbd-core` component tree and `JsbsimXml` FDM), the estimator executes five automated 6-DoF JSBSim sweeps:

### 1. Mass & Fuel Breakdown Extraction

Reads dry structural mass ($\sum m_{\text{dry}}$), usable fuel capacity ($\sum m_{\text{fluid}}$ on `fuel_tank` components, or `TailProfile::usable_fuel_kg`), and nominal payload ($\sum m_{\text{payload}}$ on `payload_station` components) directly from `syspack.json`.

### 2. Stall & Reference V-Speed Synthesis ($V_{S0}, V_{S1}, V_X, V_Y, V_G, V_A$)

- **Stall Sweep**: Sweeps angle of attack $\alpha \in [0.16, 0.36]\text{ rad}$ at clean (`Flaps 0`) and full landing flaps (`Flaps 1.0`) to find $V_{S1}$ and $V_{S0}$ (`KCAS`).
- **Sea-Level Climb Polar Sweep**: Flies 5 closed-loop full-throttle climbs spanning $[1.16\,V_{S1}, 1.48\,V_{S1}]$ at sea level (`0 ft` ISA):
  - Identifies **best angle-of-climb speed $V_X$** and maximum flight-path climb angle
    $$\gamma_{\max} = \arcsin\!\left(\frac{\mathrm{ROC}}{101.2686 \cdot V_{\mathrm{KTAS}}}\right)$$
  - Identifies **best rate-of-climb speed $V_Y$** and sea-level maximum climb rate $\mathrm{ROC}_{\max}$ (`fpm`).
- **Power-Off Glide Polar Sweep**: Flies 4 closed-loop power-off glides at `1,500 ft` ISA to identify **best glide speed $V_G$**, maximum lift-to-drag ratio $(L/D)_{\max}$, and still-air glide distance per `1,000 ft` altitude loss:
  $$d_{\text{glide / 1000 ft}} = (L/D)_{\max} \cdot \frac{1000\text{ ft}}{6076.115\text{ ft/NM}}$$
- **Auto-Synthesized `AirspeedEnvelope`**: Constructs a validated `sim_pack::AirspeedEnvelope` (`synthesized_envelope`) that can be serialized directly to `performance/airspeed_envelope.json` to drive `Indicate` PFD and HUD speed tapes.

### 3. Multi-Altitude Climb Profile & Ceiling Extrapolation

Flies full-throttle climbs at $V_Y(h)$ across `0 ft`, `4,000 ft`, `8,000 ft`, and `12,000 ft` ISA, recording `KCAS`, `KTAS`, `ROC (fpm)`, climb angle $\gamma$ (`deg`), and climb gradient (`ft/NM`). Fitting linear rate-of-climb decay $\mathrm{ROC}(h) = \mathrm{ROC}_0 - k \cdot h$ yields:

- **Service Ceiling** ($\mathrm{ROC} = 100\text{ fpm}$): $h_{\text{service}} = (\mathrm{ROC}_0 - 100) / k$
- **Absolute Ceiling** ($\mathrm{ROC} = 0\text{ fpm}$): $h_{\text{abs}} = \mathrm{ROC}_0 / k$

### 4. Cruise Polar, Specific Range & Endurance

Flies steady level cruise at three representative altitudes and power settings:

- **Economy Cruise (`4,000 ft`, `68%` throttle)**
- **Normal Cruise (`6,000 ft`, `78%` throttle)**
- **High-Speed Cruise (`8,000 ft`, `88%` throttle)**

At each point, the estimator measures stabilized `KTAS` and fuel flow $\dot{V}_f$ (`GPH`, directly from `propulsion/engine[0]/fuel-flow-rate-gph` when present, or via BSFC shaft-power fallback on simplified thruster models) and computes:

$$\text{Specific Range} = \frac{V_{\mathrm{KTAS}}}{\dot{V}_f}\;\left[\frac{\text{NM}}{\text{US gal}}\right], \qquad T_{\text{45min res}} = \max\!\left(0,\; \frac{V_{\text{fuel,usable}}}{\dot{V}_f} - 0.75\text{ hr}\right)$$

$$R_{\text{45min res}} = V_{\mathrm{KTAS}} \cdot T_{\text{45min res}}\;[\text{NM}], \qquad R_{\text{dry}} = V_{\mathrm{KTAS}} \cdot \frac{V_{\text{fuel,usable}}}{\dot{V}_f}\;[\text{NM}]$$

### 5. Three-Corner Payload–Range Profile

Synthesizes the three classic mission corners of the aircraft's payload–range diagram (with a `45-min` IFR reserve):

- **Point A — Max Payload / Reduced Fuel**: Maximum cabin/cargo payload within reference gross weight, trading fuel for payload.
- **Point B — Full Usable Fuel (Normal Cruise)**: Full fuel tanks with remaining allowable payload at `6,000 ft` normal cruise.
- **Point C — Zero-Payload Ferry (Economy Cruise)**: Full usable fuel with zero payload flown at economy specific range.

---

## Sample Output (`ironbird --mode estimate-c172`)

Running `cargo run --release -p ironbird -- --mode estimate-c172` produces the following Markdown engineering report directly from 6-DoF JSBSim sweeps:

```markdown
# New-Design Performance & Range Estimate — `c172_class_reference` (C172-Class Reference Aircraft)

- **Mass Breakdown**: Empty `767.0 kg` | Usable Fuel `144.2 kg` (`53.0 US gal`) | Nominal Payload `230.0 kg` | Ref Gross `1156.7 kg`
- **Synthesized V-Speeds (KCAS)**: Vs0 = 48.1 | Vs1 = 53.1 | Vx = 63.9 (gamma_max = 5.9 deg) | Vy = 74.2 (`758 fpm` SL) | Vg = 68.0 (L/D = 9.12, `1.50 NM / 1,000 ft`) | Va = 103.5 | Vmax,SL = 128.1 KTAS
- **Estimated Ceilings**: Service Ceiling (`100 fpm`) **`15217 ft`** | Absolute Ceiling **`17534 ft`**

## Climb Profile vs. Altitude
| Alt (ft) | KCAS | KTAS | ROC (fpm) | Climb Angle (deg) | Gradient (ft/NM) |
|---:|---:|---:|---:|---:|---:|
| 0 | 74.2 | 74.6 | 759 | 5.77 | 611 |
| 4000 | 73.1 | 77.8 | 586 | 4.27 | 452 |
| 8000 | 72.1 | 81.3 | 400 | 2.78 | 295 |
| 12000 | 71.0 | 85.2 | 246 | 1.63 | 173 |

## Cruise Polar, Specific Range & Endurance (`45-min` Reserve)
| Regime | Alt (ft) | KTAS | Fuel (GPH) | Spec. Range (NM/gal) | Endurance (hr) | Range w/ Res (NM) | Dry Range (NM) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Economy Cruise (4,000 ft) | 4000 | 97.2 | 5.11 | 19.02 | 9.63 | 935 | 1008 |
| Normal Cruise (6,000 ft) | 6000 | 106.9 | 6.95 | 15.39 | 6.88 | 736 | 816 |
| High-Speed Cruise (8,000 ft) | 8,000 | 117.0 | 9.12 | 12.82 | 5.06 | 592 | 679 |

## Payload–Range Envelope (`45-min` Reserve)
| Mission Corner | Payload (kg) | Fuel (kg) | TOW (kg) | Cruise (KTAS) | Still-Air Range (NM) |
|---|---:|---:|---:|---:|---:|
| Point A — Max Payload / Reduced Fuel | 295.9 | 93.8 | 1156.7 | 106.9 | 450 |
| Point B — Full Usable Fuel (Normal Cruise) | 245.4 | 144.2 | 1156.7 | 106.9 | 736 |
| Point C — Zero-Payload Ferry (Economy Cruise) | 0.0 | 144.2 | 911.2 | 98.6 | 963 |
```

> **Propulsion Modelling Tip for Ceiling & Range Estimation:**
> A `<piston_engine>` coupled to a `<propeller>` (as in `vehicles/c172/fdm/c172_class_reference.xml`) naturally models manifold-pressure density lapse with altitude and BSFC fuel burn (`propulsion/engine[0]/fuel-flow-rate-gph`), yielding realistic service ceilings (`~15,200 ft`) and altitude specific-range curves. By contrast, a simplified `<electric_engine>` driving a `<direct>` thruster produces density-independent thrust (`T = const`), so its climb rate does not lapse with altitude unless an atmospheric density table is included in the propulsion or drag model.
