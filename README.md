# Smart Salt Processor: prototype spray dryer with fuzzy temperature control

Prototype salt spray dryer developed at the Department of Mechatronics Engineering,
Universitas Trunojoyo Madura (applied research scheme, 2026). The repository contains the
controller firmware, the panel wiring, the hardware design files, prototype photographs and a
reproducible MATLAB/Simulink simulation study of PID, fuzzy and self-tuning Fuzzy-PID temperature control.

## Contents

| Path | Description |
|---|---|
| `ESP32_SPRAY_DRAYER/` | ESP32 firmware: PT100 RTD (MAX31865), time-proportioning SSR heater (2 s window), incremental Mamdani fuzzy controller, pump, flow meter, safety logic (200 °C trip, SSR-stuck alarm), web dashboard with CSV export |
| `UNO_HMI_SPRAY_DRAYER/` | Arduino Uno firmware for the 2.8" touch HMI (UART link to the ESP32) |
| `wiring_panel_spray_dryer_v1.txt` | Panel wiring (220 VAC, 12 VDC, grounding, safety notes) |
| `Gerber_PCB4_2026-10-30.zip` | Control PCB Gerber files |
| `tabung dwg rev.pdf` | Drying-chamber drawing (Ø350 × 900 mm) |
| `Dokumentasi/`, `poster.jpeg` | Prototype photographs and poster |
| `sim_fuzzy_pid/` | Simulation study: calibrated two-node thermal model, Simulink model built by script, PID / fuzzy / Fuzzy-PID comparison, Monte-Carlo robustness, safety scenarios. See [`sim_fuzzy_pid/README.md`](sim_fuzzy_pid/README.md) |

## Simulation study: quick start

Requires MATLAB R2024a with Simulink, Fuzzy Logic Toolbox, Control System Toolbox,
Optimization Toolbox and Statistics and Machine Learning Toolbox. No C compiler is needed.

```
cd sim_fuzzy_pid
matlab -batch "run_all_scenarios"
```

A full run takes about 45 min. Tables (CSV + LaTeX) and figures (PNG 300 dpi + PDF) are written to
`sim_fuzzy_pid/results/`.

> **Note on the results.** All temperature and control results in `sim_fuzzy_pid/results` are
> **simulation results**. The plant model was calibrated against a few approximate operator-observed
> points (90 → 70 → 120 °C with the heater at 100 %), not a logged dataset; see
> `sim_fuzzy_pid/model_description.md` for the assumptions and limitations. Experimental validation
> with logged temperature data is future work.

## Safety

The heater bank is 2.2 kW at 220 VAC, and the design has no hardware thermal cut-off (see the wiring notes).
Never leave the prototype unattended while the heater is on, and never run the heater without the blower.
The firmware's Wi-Fi access point uses a default password, which should be changed before deployment.
