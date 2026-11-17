# Fuzzy-PID temperature control of a prototype salt spray dryer: simulation study

Reproducible MATLAB/Simulink code for the simulation study *"Fuzzy-PID Temperature
Control of a Prototype Spray Dryer for High-Quality Salt Production: A Simulation
Study Calibrated from Prototype Observations"* (Universitas Trunodjoyo Madura).

> **Every result produced by this code is a simulation result.** The plant
> model is calibrated against a few approximate operator-observed anchor points,
> not a logged dataset. See `model_description.md` §4.1 and the Limitations in
> `paper_support_notes.md`.

## Requirements

* MATLAB R2024a (24.1, Update 4), the version used to produce the results
* Simulink
* Fuzzy Logic Toolbox (`mamfis`, Fuzzy Logic Controller block)
* Control System Toolbox (`pidtune`, `tf`)
* Optimization Toolbox (`lsqnonlin`)
* Statistics and Machine Learning Toolbox (`boxchart` only)
* No C compiler is needed. The model uses only built-in Simulink blocks and the
  Fuzzy Logic Controller block.

## How to run

```
cd sim_fuzzy_pid
matlab -batch "run_all_scenarios"
```

A full run takes about 35–45 min, most of it the 150 Monte-Carlo simulations.
The console output is saved in `results_run_log.txt`.

## Files

| File | Purpose |
|---|---|
| `params_spray_dryer.m` | **Only** source of numerical parameters (blocks reference `P.<field>`) |
| `load_params.m` | Nominal parameters + calibrated plant + designed PID gains |
| `model_description.md` | Equations, assumptions, calibration and its limitations |
| `plant_lin.m`, `sim_lin.m`, `calib_run.m` | Linear two-node plant, exact solution, replay of the observed run |
| `calibrate_plant.m` | Multi-start bounded fit, identifiability spread, ±20 % sensitivity, FOPDT |
| `build_fis.m` | `fuzzy_inc.fis` (firmware), `fuzzy_pid_selftune.fis`, `fuzzy_pid_liu.fis` (ablation) |
| `build_model.m` | Builds `spray_dryer_fuzzy_pid.slx` programmatically |
| `design_pid.m` | pidtune / Ziegler–Nichols / Cohen–Coon / SIMC, evaluated on the full model |
| `scenario_P.m` | Definition of scenarios S1–S6 and the open-loop replay |
| `run_sim.m`, `compute_metrics.m` | Single simulation, performance metrics |
| `make_figures.m`, `fig_new.m`, `fig_save.m`, `ctrl_style.m` | Figures (PNG 300 dpi + PDF) |
| `write_tables.m` | CSV + LaTeX (booktabs) tables |
| `run_all_scenarios.m` | Top-level script |

## Simulink model

`spray_dryer_fuzzy_pid.slx` contains these subsystems: Setpoint, Spray disturbance,
Sensor RTD MAX31865, Controller (PID / Fuzzy / Fuzzy-PID / manual, selected by
`P.ctrl_mode`), Safety limiter, SSR actuator (`P.ssr_mode`: 1 duty-average,
2 detailed 2 s PWM; `P.ssr_fault_t` for an SSR short-circuit), Heater and chamber
plant, and Logging. Fixed-step `ode4`, 0.1 s (0.01 s in the PWM case),
single-tasking.

## Outputs (`results/`)

* `simulation_*.csv` / `.tex`: calibration, PID tuning, metrics S1–S5, safety S6,
  rule-base ablation
* `simulation_consistency_check.txt`: Simulink model vs. analytic solution
* `figures/*.png|pdf` and `figures/captions.md`: every caption states that the
  figure is simulated
* `calibrated_params.mat`, `pid_gains.mat`, `P_final.mat`, `S4_montecarlo.mat`, `all_runs.mat`
