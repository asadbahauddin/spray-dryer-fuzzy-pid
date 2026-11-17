# Model description — Simulation model of the prototype salt spray dryer

All results produced with this model are **simulated**. The only real
information is an operator observation used as approximate calibration
anchors (Section 4); no logged dataset exists.

## 1. Why a two-node model

The operator reported that, with the heater at 100 % duty, the chamber
temperature fell from 90 °C to 70 °C after the spray started, then rose
gradually and settled flat at 120 °C. A first-order lumped model
`C dT/dt = P − G (T − T_amb) − Q_evap` with constant inputs after the spray
step has a single exponential mode. The temperature therefore moves
monotonically towards its new equilibrium: it cannot first fall below its
start value and then exceed it. Reproducing a dip followed by a rise needs
at least two thermal time scales. The model therefore splits the
lumped capacity into:

* a **fast node** `T_a` – air in the duct and chamber, heater elements, thin
  internals (capacity `C_a`);
* a **slow node** `T_w` – stainless chamber wall, heater housing, duct metal
  (capacity `C_w`).

When spraying starts, the wall is still colder than the air and keeps
absorbing heat. Evaporation pulls the fast node down quickly, and the
air temperature then recovers as the wall warms up.

## 2. Equations

```
C_a dT_a/dt = η_h u(t − θ) P_max − G_air (T_a − T_amb) − H_aw (T_a − T_w) − Q_evap(t)
C_w dT_w/dt = H_aw (T_a − T_w) − UA_w (T_w − T_amb)
τ_s dT_m/dt = T_a − T_m                                    (RTD + thermowell lag)
T_meas[k]   = Q_adc( T_m(k·0.5 s) + v[k] ),  v ~ N(0, σ²), σ = 0.1 °C,  Q_adc step 0.03125 °C
Q_evap(t)   = η_evap · ṁ_liq · [ c_p,w (T_boil − T_feed) + h_fg ] · s(t)
P_max       = V² / R = 220² / 22 = 2200 W
```

* `u ∈ [0,1]` is the SSR duty. In *average* mode it is the controller
  output quantised to whole mains half-cycles inside the 2 s window
  (1/200 resolution). In *PWM* mode the heater is fully on for the first
  `u·2 s` of every window.
* `G_air = ṁ_air c_p,air` is the effective enthalpy flow of the fan air
  (upper bound 64 W/K from the 197 m³/h free-air rating).
* `s(t)` is the spray on/off signal. With 2.3 L/h feed at 30 °C the full
  evaporative load is ≈ 1.63 kW. `η_evap` lumps incomplete evaporation,
  wall deposition and the part of the latent load that is taken from the
  air after it has passed the sensor.
* Firmware safety rule: `u = 0` whenever `T_meas ≥ 200 °C`.
* Controller sample time 2 s. Sensor read 0.5 s. Solver ode4, fixed step 0.1 s
  (0.01 s for the detailed-PWM case).

## 3. Steady state

With the heater at constant duty, `G_total = G_air + H_aw·UA_w/(H_aw+UA_w)` and
`T_ss = T_amb + (η_h u P_max − Q_evap)/G_total`.
The observation that the temperature settled at 120 °C with the heater at
100 % and spray on means that, **within this model, 120 °C is the highest
chamber temperature achievable at 2.3 L/h feed.** Spray scenarios are therefore
run at a lower set-point (`P.sp_spray`).

## 4. Calibration (`calibrate_plant.m`)

Replayed run: it starts at the moment spraying begins, with the heater at 100 %, the
air node and sensor at 90 °C, and the wall node at an unknown temperature
`T_w,spray`. The preheat length and history were not recorded, so `T_w,spray` is a
fitted parameter. Residuals:

| residual | target | weight |
|---|---|---|
| minimum T_meas within 30 min | 70 °C | 1 |
| time of minimum | 7.5 min | 1 °C/min |
| T_meas at 60 min | 120 °C | 0.5 (time not observed, assumed) |
| T_meas at 120 min | 120 °C | 1 |
| change over last 10 min | 0 °C | 2 |
| prior: log(p/p_nom) | 0 | 0.3 |

Seven parameters are fitted (`C_a, C_w, G_air, H_aw, UA_w, η_evap, T_w,spray`)
by bounded `lsqnonlin` from 40 starts. `θ = 10 s`, `τ_s = 7 s` and `η_h = 1` are
fixed assumptions. With only five anchor numbers, the parameters are **not
uniquely identifiable**. The scripts report the spread of all parameter sets that fit
equally well, together with a ±20 % sensitivity table.

### 4.1 Structural limitation (reported, not hidden)

The best physically bounded fit reproduces the 70 °C minimum and the 120 °C
plateau, but the minimum occurs at ≈3.5 min instead of the reported 7.5 min
(anchor RMS ≈ 2 °C). Two alternatives were tested during model development
and rejected:

* **Three nodes (heater element + air + wall)**: anchor RMS 0.35 °C, but only
  with element-to-air conductance at its lower bound. That implies an element
  temperature above 800 °C, against a 200 °C element rating. Physically
  impossible.
* **Two nodes with a decaying spray load**: anchor RMS ≈ 1 °C, but only if the
  evaporative load falls by 90 % within about 15 min, which has no confirmed
  mechanism.

Analysis of the mode amplitudes explains this. After the dip, the 50 K rise
requires a slow heat store whose temperature swings by more than 100 K. A
stainless wall in contact with 90–120 °C air cannot do that. Plausible
explanations on the real prototype, which must be checked experimentally:
(i) the timing or minimum reported by the operator is only approximate; (ii) the
RTD near the inlet was wetted by spray at start-up (wet-bulb effect); (iii) the
spray rate decreased (partial nozzle clogging by salt). The physically bounded
two-node model was adopted and the misfit is reported.

Implications of the best fit: effective air flow ≈ 27 m³/h, against a free-air
rating of 197 m³/h. This is consistent with an axial fan throttled by the Ø51 mm
outlet, and should be verified with an anemometer. The dry steady state at 100 %
duty is ≈196 °C, close to the 200 °C element limit.

### 4.2 Set-point for spray scenarios

Within the calibrated model, the steady state at 100 % duty with spray is
≈121 °C. Spray scenarios therefore use 90 °C (the observed spray-start
temperature, ≈81 % steady duty), which leaves margin for disturbances.

## 5. FOPDT approximation

A +10 % duty step on the calibrated (linear) model gives `ΔT_meas(t)`.
`K` is the static gain. `τ` and `θ` come from a least-squares fit over the first
2 h, with `θ ≥` the heater delay. The two-point method (28.3 %/63.2 %) returns
`θ ≈ 0` because of the fast air mode, so it was not used. Because the model is
linear, the values do not depend on the operating point or on the spray load.
Result: K ≈ 1.66 °C/%, τ ≈ 1275 s, θ = 10 s (plus Ts/2 for tuning). The plant is
strongly lag-dominated (θ/τ ≈ 0.01). Ziegler–Nichols and Cohen–Coon therefore
give very high gains, which chatter with 0.1 °C sensor noise (see
`simulation_pid_tuning.csv`). The baseline PID is selected by minimum IAE,
subject to overshoot ≤ 10 % and steady-state duty standard deviation ≤ 5 %.
