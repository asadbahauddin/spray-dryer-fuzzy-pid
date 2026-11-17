function P = params_spray_dryer()
%PARAMS_SPRAY_DRYER  Single source of every numerical parameter of the study.
%   P = params_spray_dryer() returns a struct used by calibration, controller
%   design, the Simulink model (blocks reference P.<field> only) and plotting.
%
%   Values marked [CAL] are overwritten by calibrate_plant.m (stored in
%   results/calibrated_params.mat and merged by load_params.m).
%   Values marked [ASSUMED] are engineering assumptions, not measurements.

%% ---------------- Geometry (prototype drawing) -------------------------
P.D_chamber   = 0.350;     % m   inner diameter of drying chamber
P.H_chamber   = 0.900;     % m   chamber height
P.D_inlet     = 0.150;     % m   hot-air inlet duct diameter
P.D_outlet    = 0.051;     % m   outlet diameter
P.t_wall      = 1.5e-3;    % m   stainless wall thickness [ASSUMED]
P.rho_ss      = 7900;      % kg/m3 stainless steel density
P.cp_ss       = 500;       % J/(kg K) stainless steel specific heat
P.A_wall      = pi*P.D_chamber*P.H_chamber + 2*pi*(P.D_chamber/2)^2;   % m2
P.m_wall      = P.A_wall*P.t_wall*P.rho_ss;                            % kg

%% ---------------- Heater and actuator ----------------------------------
P.V_mains     = 220;       % V rms
P.R_heater    = 22;        % ohm (4 elements in parallel, total)
P.P_max       = P.V_mains^2/P.R_heater;   % W  (= 2200 W)
P.eta_h       = 1.0;       % -  fraction of electric power transferred to air
P.T_element_max = 200;     % degC element rating
P.ssr_window  = 2.0;       % s  time-proportioning window of zero-cross SSR
P.f_mains     = 50;        % Hz
P.ssr_q       = 1/(2*P.f_mains*P.ssr_window);   % duty resolution (half cycles)
P.ssr_mode    = 1;         % 1 = duty-cycle average, 2 = detailed PWM
P.ssr_fault_t = 1e9;       % s  time at which SSR fails short (1e9 = never)

%% ---------------- Air supply -------------------------------------------
P.Vdot_free   = 197/3600;  % m3/s free-air flow of axial fan
P.rho_air     = 1.16;      % kg/m3 at ~30 degC
P.cp_air      = 1007;      % J/(kg K)
P.G_air_free  = P.Vdot_free*P.rho_air*P.cp_air;   % W/K (upper bound, ~64 W/K)

%% ---------------- Spray (evaporative load) ------------------------------
P.Vdot_liq_Lph = 2.3;      % L/h feed rate [ASSUMED default, A5]
P.rho_liq     = 1000;      % kg/m3 (dilute brine approximated as water)
P.cp_w        = 4180;      % J/(kg K)
P.h_fg        = 2.257e6;   % J/kg
P.T_boil      = 100;       % degC
P.T_feed      = 30;        % degC
P.mdot_liq    = P.Vdot_liq_Lph/1000/3600*P.rho_liq;   % kg/s
P.Q_evap_full = P.mdot_liq*(P.cp_w*(P.T_boil-P.T_feed) + P.h_fg); % W (eta=1)
P.spray_scale = 1.0;       % multiplier for robustness studies

%% ---------------- Ambient ------------------------------------------------
P.T_amb       = 30;        % degC [ASSUMED default, A4]

%% ---------------- Lumped two-node plant ([CAL] = calibrated) -------------
P.C_a      = 3.0e3;        % J/K  fast node: air + heater elements + duct   [CAL]
P.C_w      = 1.0e4;        % J/K  slow node: chamber wall + housings        [CAL]
P.G_air    = 25;           % W/K  m_dot_air*cp_air (effective)              [CAL]
P.H_aw     = 20;           % W/K  air-to-wall conductance                   [CAL]
P.UA_w     = 10;           % W/K  wall-to-ambient loss                      [CAL]
P.eta_evap = 0.6;          % -    effective evaporation factor              [CAL]
P.Tw_spray = 50;           % degC wall node at spray start (run history unknown) [CAL]
P.theta_h  = 10;          % s    heater-to-air transport delay [ASSUMED]
P.C_scale  = 1.0;          % multiplier on C_a and C_w (robustness)
P.G_scale  = 1.0;          % multiplier on G_air (robustness)
P.theta_extra = 0;         % s    additional delay (robustness)

%% ---------------- Sensor: RTD PT100 3-wire + MAX31865 --------------------
P.tau_s     = 7;           % s   sensor (thermowell) lag
P.tau_s_off = 0.2;         % s   value used when "lag off"
P.Ts_sensor = 0.5;         % s   read interval
P.adc_q     = 0.03125;     % degC MAX31865 15-bit resolution (approx.)
P.noise_sd  = 0.1;         % degC
P.noise_on  = 1;           % 1/0
P.seed      = 1;           % noise seed

%% ---------------- Controllers -------------------------------------------
P.Ts        = 2.0;         % s   controller sample time (firmware)
P.u_min     = 0;  P.u_max = 100;   % % duty
P.ctrl_mode = 1;           % 1 PID, 2 Fuzzy, 3 Fuzzy-PID, 4 manual (open loop)
P.u_manual  = 100;         % % used when ctrl_mode = 4
% baseline PID (overwritten by design_pid.m)
P.Kp = 5;  P.Ki = 0.02;  P.Kd = 0;  P.N_d = 10;
% fuzzy scaling (identical to firmware)
P.e_range   = 20;          % degC  -> 1
P.de_range  = 2;           % degC per Ts -> 1
P.du_range  = 8;           % % per step  <- 1
% fuzzy-PID gain modulation bound
P.gain_bound = 0.5;        % +/-50 % of baseline gains

%% ---------------- Safety (firmware) --------------------------------------
P.T_trip    = 200;         % degC heater forced OFF if T_meas >= T_trip
P.sp_min    = 40;  P.sp_max = 180;   % degC firmware set-point limits
P.alarm_off_s = 60;        % s    firmware SSR-stuck rule: heater commanded OFF >= 60 s ...
P.alarm_dT  = 5;           % degC ... and temperature rose > 5 degC ...
P.alarm_Tmin = 60;         % degC ... and T > 60 degC  -> FAULT "SSR MACET - TEKAN E-STOP"

%% ---------------- Scenario / solver --------------------------------------
P.dt        = 0.1;         % s fixed solver step (divides Ts_sensor and Ts)
P.dt_pwm    = 0.01;        % s step used in detailed-PWM runs
P.t_stop    = 3*3600;      % s
P.log_Ts    = 2.0;         % s logging interval
P.sp_spray  = 90;          % degC set-point in spray scenarios S2/S4/S5 (= observed spray-start
                           %      temperature; leaves ~19 % duty margin in the calibrated model)
P.sp_time   = [0 1e9];     % s   setpoint schedule breakpoints
P.sp_val    = [120 120];   % degC
P.spray_t_on  = 1e9;       % s   spray start time (1e9 = never)
P.spray_t_off = 2e9;       % s   spray stop time
P.Ta0 = P.T_amb; P.Tw0 = P.T_amb; P.Tm0 = P.T_amb;   % initial states
P.u_init = 0;              % heater fraction assumed before t = 0 (delay buffer)

%% ---------------- Calibration anchors (operator observation) -------------
P.obs.T_start = 90;        % degC at spray start
P.obs.T_min   = 70;        % degC minimum after spray start
P.obs.t_min   = 7.5*60;    % s  (A2, operator estimate)
P.obs.T_plat  = 120;       % degC plateau
P.obs.t_reach = 60*60;     % s  [ASSUMED default, A2b not provided]
P.obs.t_end   = 120*60;    % s  run stopped, flat at 120 degC

%% ---------------- Monte-Carlo -------------------------------------------
P.mc_runs   = 50;
P.mc_seed   = 2026;
end
