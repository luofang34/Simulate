# Jacobian & Gauss-Newton Parameter Identification

When calibrating a new aircraft or refining an existing model after a flight-test campaign, manually adjusting engine and aerodynamic parameters by trial and error is slow because parameters cross-couple across climb, cruise speed, and fuel flow.

Passing `--sensitivity` to `ironbird` (or calling `sim_pack::identify_package_sensitivities(&package)`) runs an automated **finite-difference Jacobian and damped Gauss-Newton parameter identifier**.

```sh
cargo run --release -p ironbird -- \
  --mode calibrate-dir \
  --package-dir vehicles/c172s \
  --sensitivity \
  --report-out c172_sensitivity.md
```

---

## 1. Tunable Parameter Vector $\boldsymbol{\theta}$

The sensitivity engine probes four physically orthogonal parameters around the baseline package configuration:

| Parameter $\theta_j$ | Perturbation $\delta\theta_j$ | Primary Physical Mechanism |
|---|---:|---|
| `volumetric_efficiency_scale` | `+0.02` (`+2%`) | Scales air mass ingested per cylinder stroke (`propulsion/engine[0]/volumetric-efficiency`), shifting both WOT climb power and cruise fuel/power together. |
| `bsfc_scale` | `+0.02` (`+2%`) | Scales brake specific fuel consumption (`propulsion/engine[0]/bsfc-lbs_hphr`), shifting shaft power at fixed fuel flow without changing air mass flow. |
| `friction_hp_delta` | `+1.5 HP` | Adds parasitic mechanical friction (`propulsion/engine[0]/friction-hp`), shifting net shaft horsepower while leaving combustion fuel flow unchanged. |
| `equiv_mass_lbs` | `+25.0 lbs` | Perturbs aircraft gross weight (`inertia/pointmass-weight-lbs[0]`), shifting induced drag and climb rate $\mathrm{ROC} \propto -W$. |

---

## 2. Weighted Residual Vector & Damped Gauss-Newton Step

Let $\mathbf{r}(\boldsymbol{\theta}) \in \mathbb{R}^m$ be the vector of simulated minus target residuals across all climb points ($\text{fpm}$), cruise true airspeeds ($\text{KTAS}$), and cruise fuel flows ($\text{GPH}$), weighted by the inverse of their suite tolerances $\sigma_i$:

$$\tilde{r}_i(\boldsymbol{\theta}) = \frac{y_{i,\text{sim}}(\boldsymbol{\theta}) - y_{i,\text{target}}}{\sigma_i}, \qquad \mathcal{J}(\boldsymbol{\theta}) = \sum_{i=1}^m \tilde{r}_i(\boldsymbol{\theta})^2$$

For each parameter $j \in \{1,\dots,4\}$, `sim_pack::calibration::sensitivity` re-flies the suite at $\boldsymbol{\theta} + \delta\theta_j \mathbf{e}_j$ to form the normalized sensitivity matrix

$$\tilde{J}_{ij} = \frac{\tilde{r}_i(\boldsymbol{\theta} + \delta\theta_j \mathbf{e}_j) - \tilde{r}_i(\boldsymbol{\theta})}{\delta\theta_j}$$

and solves the $4 \times 4$ Levenberg–Marquardt / damped Gauss-Newton normal equations ($\lambda = 0.35$) via Gaussian elimination with partial pivoting:

$$\left(\tilde{\mathbf{J}}^\top \tilde{\mathbf{J}} + \lambda \mathbf{I}\right) \Delta\boldsymbol{\theta} = -\tilde{\mathbf{J}}^\top \tilde{\mathbf{r}}(\boldsymbol{\theta})$$

---

## 3. Interpreting the Identification Report

On `vehicles/c172s` (`N642SP`), the Jacobian table reports both the physical partial derivatives ($\partial\mathrm{ROC}/\partial\theta$, $\partial\mathrm{KTAS}/\partial\theta$, $\partial\mathrm{GPH}/\partial\theta$) and the recommended parameter step $\Delta\theta$:

| Parameter | Step $\delta\theta$ | $\partial\mathrm{ROC}/\partial\theta$ (`fpm`) | $\partial\mathrm{KTAS}/\partial\theta$ (`kt`) | $\partial\mathrm{GPH}/\partial\theta$ | Recommended $\Delta\theta$ |
|---|---:|---:|---:|---:|---:|
| `volumetric_efficiency_scale` | `+0.0200` | `+919.6` | `+29.96` | `+1.878` | `+0.01483` |
| `bsfc_scale` | `+0.0200` | `-871.1` | `-29.66` | `-0.892` | `-0.01564` |
| `friction_hp_delta` | `+1.5000` | `-9.5` | `-0.32` | `+0.009` | `-0.02116` |
| `equiv_mass_lbs` | `+25.0000` | `-0.4` | `-0.01` | `+0.000` | `-0.27383` |

When all recommended $|\Delta\theta|$ values are $< 2\%$ (`0.02`) on scale factors and $< 1\text{ HP}$ on friction horsepower, and `all_passed == true`, the package is converged.
