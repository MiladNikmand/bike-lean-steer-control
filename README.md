# bike-lean-steer-control

Linearized Whipple–Carvallo bicycle dynamics model with seven balance controllers, four disturbance types, actuator saturation and slew-rate limiting, an interactive GUI, and a self-contained HTML results report. Validated against the benchmark stability results of Meijaard et al. (2007). No MATLAB toolboxes required.

**[View the full results report →](https://miladnikmand.github.io/bike-lean-steer-control/report.html)**

---

## How it works

The project runs in two modes over the same underlying model and exports identical output either way.

**Interactive mode (`bike_launch`).** A five-tab scrollable `uifigure` panel lets you configure bike parameters, select controllers, tune gains, enable disturbances, and start a run. Results are saved to a numbered folder automatically.

**Batch mode (`run_all`).** Runs the full pipeline unattended from a CONFIG block at the top of the file. Useful for reproducible runs or running on a remote machine.

After either mode, generate the HTML report:

```matlab
>> bc_build_report       % writes report_data.js, opens report.html
```

---

## The model

State vector: `x = [phi; delta; phi_dot; delta_dot]` — roll angle, steer angle, and their rates. Four rigid bodies (rear frame + rider, fork + handlebar, rear wheel, front wheel) are assembled into the canonical Whipple–Carvallo equations:

```
M * q_ddot  +  v * C1 * q_dot  +  (g*K0 + v²*K2) * q  =  F
```

Converted to first-order state-space form at a chosen forward speed `v`:

```
x_dot = A(v) * x  +  B * u
```

where `A` and `B` are built by `bike_state_space.m` from the matrices computed by `compute_benchmark_matrices.m`. The mass matrix `M` matches the published benchmark values exactly — confirming the implementation against Meijaard et al. before any control is added.

### Stability regimes

| Speed | Regime | Open-loop behavior |
|---|---|---|
| v < ~4.3 m/s | **Unstable** | Weave and capsize modes both grow |
| 4.3 – 6.0 m/s | **Self-stable** | Gyroscopic + caster effect stabilises passively |
| v > ~6.0 m/s | **Capsize** | Slow non-oscillatory divergence; weave is stable |

`stability_analysis.m` maps the full 0–10 m/s range and prints the exact self-stable band for the current parameter set.

---

## Controllers

Seven feedback strategies, all selectable independently. No Control Systems Toolbox required — gains are found using plain linear algebra (DARE, grid search, root-locus sweep, Ackermann's formula).

| Controller | Feedback law | Gain method | Integral action |
|---|---|---|---|
| `RateOnly` | `T = k · phi_dot` | Root-locus sweep | No |
| `PD` | `T = k1·phi + k2·phi_dot` | Grid search | No |
| `PID` | `T = k1·phi + k2·phi_dot + k3·∫phi dt` | Grid search (augmented state) | **Yes** |
| `LQR` | `T = -K_lqr · x` | Algebraic Riccati (DARE) | No |
| `FullState` | `T = -K_ack · x` | Ackermann pole placement | No |
| `LQG` | `T = -K_lqr · x_hat` | LQR + Kalman filter (measures `phi_dot` only) | No |
| `GainSched` | `T = k(v) · phi_dot` | Speed-zone scheduling (3 zones) | No |

The PID integral term eliminates steady-state roll error under persistent disturbances (road camber). The LQG controller is the most realistic: only roll rate is measured (as from a gyroscope/IMU), and a Kalman filter estimates the full state from this noisy measurement.

---

## Physical constraints and actuator limits

Three distinct enforcement mechanisms, correctly separated:

| Parameter | Effect | Default |
|---|---|---|
| `phi_limit_deg` | **Hard stop** — bike has fallen. Simulation terminates, run marked FAILED | 90° |
| `delta_limit_deg` | **Actuator saturation** — steer is clamped, run continues, saturation logged | 60° |
| `steer_rate_limit_deg_s` | **Slew rate** — max steer rate per second; `Inf` = ideal actuator | Inf |

The steer saturation is a clamp, not a kill switch. A controller that drives the handlebar to its stop and then recovers is a meaningful real-world result. Saturation events are counted, timestamped, and highlighted on all output figures.

---

## Disturbances

Six disturbance types, each independently configurable:

| Type | Physical mechanism | Channel |
|---|---|---|
| Kick | Gaussian roll torque pulse (lateral impact) | φ |
| Gravel | Band-limited white noise (rough road) | φ, δ |
| Road camber | Constant gravity bias from banked surface | φ |
| Wind gust | von Kármán profile; `F = ½ρCdA v_w²` at CoM height | φ |
| Sinusoidal road | Periodic steer disturbance at configurable frequency | δ |
| Speed variation | `v(t) = v_nom + amp·sin(2π·freq·t)`; rebuilds `A(v)` each step | Parametric |

Disturbance events are marked with vertical lines and shaded regions on all figures and GIFs. Road camber specifically motivates integral action: PD and Rate-Only controllers settle at a nonzero lean angle, while PID, LQR, and LQG null it out.

---

## Requirements

**MATLAB R2020b or later.** No toolboxes required.

| Feature | Minimum version |
|---|---|
| `uifigure` scrollable panels (`Scrollable='on'`) | R2020b |
| `exportgraphics` PNG export | R2020a |
| `jsonencode` with `PrettyPrint` | R2021a |

All controllers use plain linear algebra. `care()` is used for LQR and LQG (available in base MATLAB since R2006a); a fallback iterative DARE solver is included if `care()` fails.

---

## Quick start

```matlab
>> cd path/to/bike-lean-steer-control
>> check_setup          % verify all files are present
>> bike_launch          % interactive GUI  (5 scrollable tabs)
```

Non-interactive:

```matlab
>> run_all              % batch mode — edit CONFIG block at top of file
```

After a run, generate the report:

```matlab
>> bc_build_report      % scans all run folders, writes report_data.js
```

Then open `report.html` in a browser. The report reads `report_data.js` as a JS bundle so it works on `file://` without CORS issues, and renders live on GitHub Pages.

To animate the most recent run:

```matlab
>> animate_bike_v2      % scans run folders, defaults to latest, asks about GIF export
```

---

## File layout

```
Entry points
  bike_launch.m              Interactive GUI (5-tab uifigure, R2020b+)
  run_all.m                  Non-interactive batch driver
  check_setup.m              Verifies all files are present

Pipeline
  bc_run_analysis.m          Simulation engine — all controllers, actuator limits
  bc_export_figures.m        PNG and GIF export for response figures
  bc_disturbance.m           Disturbance generator (6 types)
  bc_write_summary.m         Writes run_summary.json and README.txt
  bc_build_report.m          Generates report_data.js for the HTML report
  animate_bike_v2.m          3D stick-figure animation viewer + GIF export

Core model (Whipple-Carvallo benchmark)
  bike_params.m              Physical parameters (masses, inertia, geometry)
  compute_benchmark_matrices.m  Canonical M, C1, K0, K2 matrices
  bike_state_space.m         State-space A, B at a given forward speed

Standalone analysis scripts
  stability_analysis.m       Eigenvalue sweep + uncontrolled simulation
  balance_controller.m       Rate-feedback gain selection + speed-range sweep
  param_sensitivity.m        Geometry / mass sensitivity of the stable band
  response_analysis.m        Perturbation sweep + strategy comparison
  save_results.m             Batch .mat file generator for the animator

Report
  report.html                Self-contained HTML viewer (reads report_data.js)
  report_data.js             Auto-generated data bundle (not in repo — run bc_build_report)

Per-run output  (generated, not tracked in git)
  Bike_Control_Run_NNN/
    README.txt               Human-readable run summary
    run_summary.json         Machine-readable summary
    results.mat              Full simulation data
    figures/
      00_parameters.png      Parameter table + eigenvalue plot
      <Controller>_response.png    4-panel: phi, delta, rates, torque
      <Controller>_response.gif    Same, animated
      combined_comparison.png      All controllers overlaid
    animation/
      <Controller>_animation.gif   3D stick-figure side-by-side GIF
```

---

## Run folder schema

Each `run_summary.json` follows this structure:

```json
{
  "schema": "bike-lean-steer-control/1.0",
  "run_number": 1,
  "run_folder": "Bike_Control_Run_001",
  "generated": "2026-09-13 10:00:00",
  "speed_m_s": 1.0,
  "regime": "UNSTABLE",
  "max_re_ol": 3.527,
  "bike_params": { "mB": 85.0, "w": 1.02, "c": 0.08, "lambda_deg": 18.0, "rR": 0.3, "rF": 0.35 },
  "initial_conditions": { "phi0_deg": 5.0, "delta0_deg": -2.0 },
  "constraints": {
    "phi_limit_deg": 90,
    "delta_sat_deg": 60,
    "steer_rate_limit_deg_s": "Inf",
    "note": "phi_limit = hard stop. delta_sat = clamp only, run continues."
  },
  "disturbance": { "kick_enabled": false, "camber_enabled": false, ... },
  "runs": {
    "uncontrolled": { "failed": true,  "fail_time": 0.93, "saturated": false, ... },
    "RateOnly":     { "failed": false, "settle_time": 3.71, "sat_count": 0, ... },
    "LQR":          { "failed": false, "settle_time": 0.42, "sat_count": 0, ... },
    "LQG":          { "failed": false, "settle_time": 0.48, "sat_count": 2, "sat_first_t": 0.08, ... }
  }
}
```

---

## Configuration notes

**`run_all` vs `bike_launch`.** Identical output. `bike_launch` is for interactive exploration; `run_all` is for reproducible scripted runs. The CONFIG block at the top of `run_all.m` mirrors every option available in the GUI.

**Integration step `dt`.** Default `0.01 s` (100 Hz). Use `0.001` for LQG runs with high sensor noise or tight slew-rate limits where step accuracy affects the Kalman filter.

**Steer saturation vs hard stop.** `delta_limit_deg` is a clamp — the handlebar stops at 60° and the simulation continues. Only `phi_limit_deg` terminates a run (the bike has fallen over). This distinction is physically correct and is reflected in every output file.

**LQR Q and R tuning.** `Q = diag([100, 1, 10, 1])` penalises roll angle most. `R = 0.1` allows moderate torque. Increase `R` for softer, more energy-conservative control; decrease for faster convergence at the cost of larger actuator demands.

**LQG observer.** Observer poles are placed at `2 × pp_poles` so the estimator converges faster than the controller. Only `phi_dot` (roll rate) is treated as measured — as from a real gyroscope/IMU.

**GIF file size.** 120 frames at 10 fps ≈ 12 s of animation. Each GIF is typically 3–8 MB. Reduce `gif_frames` to cut size; playback speed is independent (controlled by `gif_fps`).

---

## Known limitations

This is a **linearized** model, valid for small angles near the upright equilibrium. It correctly captures lean-steer coupling, gyroscopic effects, and caster stabilization, but gives no information about tire slip, skidding, wheelspin, or any large-angle nonlinear behavior. The self-stable speed band and all eigenvalues are exact for the chosen parameter set. Adding tire models or longitudinal dynamics would require a nonlinear extension.

---

## License

Released under the MIT License — see [LICENSE](LICENSE).

The underlying bicycle model is derived from:

> Meijaard, J.P., Papadopoulos, J.M., Ruina, A., & Schwab, A.L. (2007). *Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review.* Proceedings of the Royal Society A, 463, 1955–1982.
