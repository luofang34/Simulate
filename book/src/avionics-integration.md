# Avionics & Airspeed Tape Integration

A central design rule in `Simulate` is that **aircraft performance limitations and sensor calibrations belong inside the `syspack`** rather than hardcoded in UI widgets. When an aircraft package declares `performance/airspeed_envelope.json` and `performance/pitot_static_calibration.json`, those files automatically drive:

- The `Indicate` primary flight display (`PFD`) airspeed tape, colour arcs, and V-speed bugs;
- The full-window conformal Head-Up Display (`HUD`) speed tape in `simulate-app`;
- The onboard air-data computer (`Flight::onboard_sensors`) feeding standalone instruments and EFB panels.

```
┌─────────────────────────────────────────────────────────────────────────┐
│ vehicles/<aircraft>/                                                    │
│  ├─ performance/airspeed_envelope.json       (Vs0, Vs1, Vfe, Vno, Vne,  │
│  │                                            Vx, Vy, Vg, Va, Vr, Vref) │
│  └─ performance/pitot_static_calibration.json (Flap-indexed KCAS->KIAS) │
└───────────────────┬─────────────────────────────────────┬───────────────┘
                    │                                     │
                    ▼                                     ▼
     sim_pack::ResolvedAirspeedEnvelope      sim_pack::PitotStaticCalibration
      (scales Va(m) & Vg(m) by √(m/m_ref))    (cas_to_ias_at_flap_deg_m_s)
                    │                                     │
                    ▼                                     ▼
     sim_indicate::AirspeedEnvelope          sim_indicate::AirDataSample
      (PFD & HUD speed bands + V-bugs)        (true_airspeed, calibrated,
                                               indicated_airspeed_m_s)
```

## 1. Airspeed Limitations & V-Speed Bugs (`performance/airspeed_envelope.json`)

`sim_pack::AirspeedEnvelope` validates the certificated speed ladder fail-closed on load:

$$0 < V_{S0} \le V_{S1} < V_{FE} \le V_{NO} < V_{NE}$$

and ensures every optional reference bug ($V_R, V_X, V_Y, V_G, V_A, V_{REF}$) lies within $(V_{S0}, V_{NO}]$ with $V_X \le V_Y$ and $V_A > V_{S1}$.

### Example (`vehicles/c172/performance/airspeed_envelope.json`)

```json
{
  "type_designator": "C172",
  "reference_document": "1998 Cessna Model 172S Skyhawk SP NAV III / G3X POH (N642SP, S/N 172S8050, 2,550 lb / 1,157 kg MTOW, KIAS)",
  "reference_mass_kg": 1156.66,
  "positive_limit_g": 3.8,
  "negative_limit_g": -1.52,
  "vs0_kt": 40.0,
  "vs1_kt": 48.0,
  "vfe_kt": 85.0,
  "vfe_takeoff_kt": 110.0,
  "vr_kt": 55.0,
  "vx_kt": 62.0,
  "vy_kt": 74.0,
  "vg_kt": 68.0,
  "va_ref_kt": 99.0,
  "vref_kt": 61.0,
  "vno_kt": 129.0,
  "vne_kt": 163.0
}
```

### Live Gross-Mass Scaling of $V_A(m)$ and $V_G(m)$

While dial/tape colour arcs ($V_{S0}, V_{S1}, V_{FE}, V_{NO}, V_{NE}$) are fixed markings, maneuvering speed $V_A$ and best-glide speed $V_G$ vary with the square root of aircraft gross mass:

$$V_A(m) = V_{A,\text{ref}} \sqrt{\frac{m}{m_{\text{ref}}}}, \qquad V_G(m) = V_{G,\text{ref}} \sqrt{\frac{m}{m_{\text{ref}}}}$$

Calling `package.resolved_airspeed_envelope()` or `package.airspeed_envelope_at_mass(mass_kg)` returns a `ResolvedAirspeedEnvelope` with `va_kt` scaled to the live mass (for example, `99 KIAS` at `2,550 lb` MTOW vs. `92 KIAS` at `2,200 lb` training weight on the C172S).

## 2. Pitot-Static Position Error (`performance/pitot_static_calibration.json`)

At high angle of attack and with flaps deflected, local flow distortion around the wing and pitot-static ports causes indicated airspeed (`KIAS`) to read lower than calibrated airspeed (`KCAS`). For example, the 1998 C172S POH (Section 5, Figure 5-1) specifies:

- **Flaps $0^\circ$ stall**: $48\text{ KIAS} = 53\text{ KCAS}$ ($\Delta V_{\text{PEC}} = -5.0\text{ kt}$)
- **Flaps $30^\circ$ stall**: $40\text{ KIAS} = 48\text{ KCAS}$ ($\Delta V_{\text{PEC}} = -8.0\text{ kt}$)

`sim_pack::PitotStaticCalibration` stores piecewise-linear `kcas_kt` $\leftrightarrow$ `kias_kt` tables indexed by normalized flap setting `flap_norm` $\in [0, 1]$ and physical `flap_deg`. During simulation:

1. JSBSim computes aerodynamic forces from true/calibrated airspeed (`KCAS`).
2. `Flight::onboard_sensors` converts `calibrated_airspeed_m_s` to `indicated_airspeed_m_s` using `pec.cas_to_ias_at_flap_deg_m_s(articulation.flap_deg, cas_m_s)` and populates `AirDataSample::with_indicated_airspeed_m_s`.
3. `Indicate` PFD and HUD airspeed tapes render `indicated_airspeed_m_s` (`KIAS`) alongside `KIAS` V-speed bugs, matching the real cockpit instruments across the entire speed and flap envelope.
