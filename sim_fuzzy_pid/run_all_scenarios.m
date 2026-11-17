%RUN_ALL_SCENARIOS  Reproduce every simulated result of the study.
%   matlab -batch "run_all_scenarios"
%   Steps: calibration -> FIS -> Simulink model -> consistency check ->
%   PID design -> scenarios S1-S6 (+ PWM detail, rule-base ablation) ->
%   metrics tables (CSV + LaTeX) -> figures (PNG 300 dpi + PDF).
%   ALL outputs are simulation results.
clear; close all;
warning('off','MATLAB:print:ContentTypeImageSuggested');   % advisory only (vector PDF export)
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
cd(here); addpath(here);
rdir = fullfile(here,'results'); fdir = fullfile(rdir,'figures');
if ~exist(fdir,'dir'), mkdir(fdir); end
if exist(fullfile(fdir,'captions.md'),'file'), delete(fullfile(fdir,'captions.md')); end
tAll = tic;
St = ctrl_style();
fprintf('MATLAB %s\n', version);

%% 1. Calibration --------------------------------------------------------------
P0 = params_spray_dryer();
Cal = calibrate_plant(P0); %#ok<NASGU>
P = load_params();
[A_,B_] = plant_lin(P);
Qn = P.eta_evap*P.Q_evap_full;
xs = -A_\(B_*[1;Qn;1]); xd = -A_\(B_*[1;0;1]);
fprintf('Calibrated: T_ss(100%%, spray) = %.1f C, T_ss(100%%, dry) = %.1f C\n', xs(3), xd(3));
% steady duty needed at the spray set-point (linear model)
x0_ = -A_\(B_*[0;Qn;1]);
u_sp = (P.sp_spray - x0_(3))/(xs(3) - x0_(3));
fprintf('Spray-scenario set-point: %g C, steady duty %.1f %%\n', P.sp_spray, 100*u_sp);

%% 2. Fuzzy inference systems -----------------------------------------------------
F = build_fis(P, true);
fis_inc = F.fis_inc; fis_fpid = F.fis_fpid;

%% 3. Simulink model + consistency check against the analytic model -------------
build_model(true);
Q = scenario_P(P,'OL_replay');
o = run_sim(Q, fis_inc, fis_fpid);
R = calib_run(P);
tt = o.t - Q.spray_t_on; k = tt >= 0;
dev = max(abs(o.Tm(k) - interp1(R.t, R.Tm, tt(k))));
fprintf('Simulink vs analytic open-loop replay: max |dT| = %.3f C\n', dev);
fid = fopen(fullfile(rdir,'simulation_consistency_check.txt'),'w');
fprintf(fid,['Simulated open-loop replay (heater 100 %%, spray at 90 C) - Simulink model (ode4, 0.1 s, ' ...
    'SSR quantisation, %.0f s delay) vs exact linear solution used for calibration: max |dT_meas| = %.3f C\n'], ...
    P.theta_h, dev);
fclose(fid);

%% 4. Baseline PID design ------------------------------------------------------------
design_pid(P, fis_inc, fis_fpid);
sp_spray = P.sp_spray;
P = load_params(); P.sp_spray = sp_spray;
save(fullfile(rdir,'P_final.mat'),'P');

%% 5. Scenarios --------------------------------------------------------------------
ctrl = 1:3; nC = 3;
runs = struct();
scen = {'S1_120','S1_150','S2','S2_120','S3'};
for s = 1:numel(scen)
    for c = ctrl
        Q = scenario_P(P, scen{s}); Q.ctrl_mode = c;
        runs.(scen{s})(c) = run_sim(Q, fis_inc, fis_fpid);
    end
    fprintf('%s done (%.0f s)\n', scen{s}, toc(tAll));
end

% ---- metrics tables -------------------------------------------------------------
mt = @(m) [m.t_rise/60 m.t_settle/60 m.OS m.ess m.IAE m.ISE m.ITAE m.E_kWh m.TV m.u_max m.T_max];
stepNames = {'Rise_time_min','Settling_time_min','Overshoot_pct','SS_error_C','IAE_Cs','ISE_C2s', ...
    'ITAE_Cs2','Energy_kWh','Sum_abs_du_pct','Max_duty_pct','Max_T_C'};
distNames = {'Max_deviation_C','Recovery_time_min','SS_error_C','IAE_Cs','ISE_C2s','ITAE_Cs2', ...
    'Energy_kWh','Sum_abs_du_pct','Max_duty_pct'};
md = @(m) [m.dip m.t_rec/60 m.ess m.IAE m.ISE m.ITAE m.E_kWh m.TV m.u_max];
allM = struct();
% S1
for sn = {'S1_120','S1_150'}
    V = zeros(nC,numel(stepNames));
    for c = ctrl, r = runs.(sn{1})(c); m = compute_metrics(r,0,r.P.t_stop,'step'); V(c,:) = mt(m); allM.(sn{1})(c) = m; end
    T = addvars(array2table(V,'VariableNames',stepNames), St.names','Before',1,'NewVariableNames','Controller');
    write_tables(T, ['simulation_metrics_' sn{1}], sprintf(['Simulated start-up performance from %g' ...
        ' \\textdegree C to %g \\textdegree C without spray (scenario %s).'], P.T_amb, r.P.sp_val(1), strrep(sn{1},'_','\_')));
end
% S2 (start-up part and disturbance part)
for sn = {'S2','S2_120'}
    V1 = zeros(nC,numel(stepNames)); V2 = zeros(nC,numel(distNames));
    for c = ctrl
        r = runs.(sn{1})(c);
        m1 = compute_metrics(r,0,r.P.spray_t_on,'step'); m2 = compute_metrics(r,r.P.spray_t_on,r.P.t_stop,'disturbance');
        V1(c,:) = mt(m1); V2(c,:) = md(m2); allM.([sn{1} '_dist'])(c) = m2; allM.([sn{1} '_step'])(c) = m1;
    end
    T = addvars(array2table(V2,'VariableNames',distNames), St.names','Before',1,'NewVariableNames','Controller');
    write_tables(T, ['simulation_metrics_' sn{1} '_spray'], sprintf(['Simulated spray-start disturbance rejection ' ...
        'at set-point %g \\textdegree C (scenario %s; spray started at 90 min).'], r.P.sp_val(1), strrep(sn{1},'_','\_')));
    T = addvars(array2table(V1,'VariableNames',stepNames), St.names','Before',1,'NewVariableNames','Controller');
    write_tables(T, ['simulation_metrics_' sn{1} '_startup'], sprintf(['Simulated start-up performance before ' ...
        'spraying (scenario %s, set-point %g \\textdegree C).'], strrep(sn{1},'_','\_'), r.P.sp_val(1)));
end
% S3 (each set-point step separately)
segs = [0 60; 60 120; 120 180]*60; V = []; rn = {};
for c = ctrl
    r = runs.S3(c);
    for g = 1:3
        m = compute_metrics(r, segs(g,1), segs(g,2), 'step');
        V(end+1,:) = mt(m); rn{end+1,1} = sprintf('%s, step %d', St.names{c}, g); %#ok<SAGROW>
    end
end
T = addvars(array2table(V,'VariableNames',stepNames), rn,'Before',1,'NewVariableNames','Controller_step');
write_tables(T,'simulation_metrics_S3', ['Simulated set-point tracking 100 \textrightarrow{} 120 \textrightarrow{} ' ...
    '140 \textdegree C without spray (scenario S3); step 1 starts from ambient.']);

%% 6. S4 Monte-Carlo robustness ------------------------------------------------------
rng(P.mc_seed);
nR = P.mc_runs;
D = [0.8+0.4*rand(nR,1), 0.8+0.4*rand(nR,1), 0.7+0.6*rand(nR,1), 10*rand(nR,1)];  % C, G, spray, extra delay
mcT = cell(nC,1); mcM = zeros(nR,6,nC);
for c = ctrl
    for i = 1:nR
        Q = scenario_P(P,'S2'); Q.ctrl_mode = c;
        Q.C_scale = D(i,1); Q.G_scale = D(i,2); Q.spray_scale = D(i,3); Q.theta_extra = D(i,4); Q.seed = i;
        r = run_sim(Q, fis_inc, fis_fpid);
        m1 = compute_metrics(r,0,Q.spray_t_on,'step'); m2 = compute_metrics(r,Q.spray_t_on,Q.t_stop,'disturbance');
        mcT{c}(:,i) = r.Ta;
        mcM(i,:,c) = [m1.OS, m1.t_settle/60, m2.dip, m2.t_rec/60, m1.IAE+m2.IAE, m2.ess];
    end
    fprintf('S4 %s done (%.0f s)\n', St.names{c}, toc(tAll));
end
mcTime = r.t;
mcNames = {'Overshoot_pct','Settling_time_min','Max_dev_spray_C','Recovery_time_min','IAE_total_Cs','SS_error_C'};
V = []; rn = {};
for c = ctrl
    X = mcM(:,:,c);
    nanFrac = mean(isnan(X(:,4)));
    V(end+1,:) = [mean(X,1,'omitnan') nanFrac*100]; rn{end+1,1} = [St.names{c} ' mean']; %#ok<SAGROW>
    V(end+1,:) = [std(X,0,1,'omitnan') NaN];        rn{end+1,1} = [St.names{c} ' std'];  %#ok<SAGROW>
    V(end+1,:) = [max(X,[],1) NaN];                 rn{end+1,1} = [St.names{c} ' worst']; %#ok<SAGROW>
end
T = addvars(array2table(V,'VariableNames',[mcNames {'Not_recovered_pct'}]), rn,'Before',1,'NewVariableNames','Controller_stat');
write_tables(T,'simulation_metrics_S4_montecarlo', sprintf(['Simulated Monte-Carlo robustness (%d runs per controller): ' ...
    'C\\_a, C\\_w \\textpm 20 %%, air flow \\textpm 20 %%, spray rate \\textpm 30 %%, extra delay 0--10 s; set-point %g \\textdegree C.'], nR, P.sp_spray));
save(fullfile(rdir,'S4_montecarlo.mat'),'D','mcM','mcNames');

%% 7. S5 noise and sensor lag ----------------------------------------------------------
cases = {'noise off, lag off',0,P.tau_s_off; 'noise off, lag on',0,P.tau_s; ...
         'noise on, lag off',1,P.tau_s_off; 'noise on, lag on',1,P.tau_s};
V = []; rn = {}; s5 = struct();
for c = ctrl
    for k = 1:4
        Q = scenario_P(P,'S2'); Q.ctrl_mode = c; Q.noise_on = cases{k,2}; Q.tau_s = cases{k,3};
        r = run_sim(Q, fis_inc, fis_fpid); s5(c,k).r = r;
        m1 = compute_metrics(r,0,Q.spray_t_on,'step'); m2 = compute_metrics(r,Q.spray_t_on,Q.t_stop,'disturbance');
        % steady-state duty chatter: std of u over the last 30 min before spray
        w = r.t > Q.spray_t_on-1800 & r.t < Q.spray_t_on;
        V(end+1,:) = [m1.OS m1.t_settle/60 m2.dip m2.t_rec/60 m1.IAE+m2.IAE m1.TV+m2.TV std(r.u_cmd(w))]; %#ok<SAGROW>
        rn{end+1,1} = sprintf('%s, %s', St.names{c}, cases{k,1}); %#ok<SAGROW>
    end
end
T = addvars(array2table(V,'VariableNames',{'Overshoot_pct','Settling_time_min','Max_dev_spray_C', ...
    'Recovery_time_min','IAE_total_Cs','Sum_abs_du_pct','Duty_std_steady_pct'}), rn,'Before',1,'NewVariableNames','Case');
write_tables(T,'simulation_metrics_S5_noise_lag', sprintf(['Simulated effect of measurement noise ' ...
    '(\\textsigma = 0.1 \\textdegree C) and sensor lag (\\texttau\\_s = %g s) on scenario S2.'], P.tau_s));

%% 8. S6 safety -------------------------------------------------------------------------
s6f = run_sim(setfield(scenario_P(P,'S6_fault'),'ctrl_mode',1), fis_inc, fis_fpid); %#ok<SFLD>
s6t = run_sim(setfield(scenario_P(P,'S6_trip'),'ctrl_mode',1), fis_inc, fis_fpid);  %#ok<SFLD>
% firmware SSR-stuck rule: heater commanded OFF for >= 60 s while the
% temperature rose > 5 degC (relative to the start of the OFF period) and T > 60 degC
t_alarm = NaN; offStart = NaN;
for k = 1:numel(s6f.t)
    if s6f.u_cmd(k) <= 0
        if isnan(offStart), offStart = k; end
        if s6f.t(k) - s6f.t(offStart) >= P.alarm_off_s && ...
           s6f.Tm(k) - s6f.Tm(offStart) > P.alarm_dT && s6f.Tm(k) > P.alarm_Tmin
            t_alarm = s6f.t(k); break
        end
    else
        offStart = NaN;
    end
end
% maximum temperature/duty across all closed-loop runs
rowsS = {}; V = [];
for sn = scen
    for c = ctrl
        r = runs.(sn{1})(c);
        V(end+1,:) = [max(r.Ta) max(r.Tm) max(r.u_safe) any(r.Tm >= P.T_trip)]; %#ok<SAGROW>
        rowsS{end+1,1} = sprintf('%s, %s', strrep(sn{1},'_',' '), St.names{c}); %#ok<SAGROW>
    end
end
mcMax = cellfun(@(X) max(X(:)), mcT);
for c = ctrl, V(end+1,:) = [mcMax(c) NaN NaN 0]; rowsS{end+1,1} = ['S4 (all runs), ' St.names{c}]; end %#ok<SAGROW>
V(end+1,:) = [max(s6t.Ta) max(s6t.Tm) max(s6t.u_safe) any(s6t.Tm >= P.T_trip)]; rowsS{end+1,1} = sprintf('S6 SP %g C (firmware maximum), PID', P.sp_max);
V(end+1,:) = [max(s6f.Ta) max(s6f.Tm) max(s6f.u_frac)*100 any(s6f.Tm >= P.T_trip)]; rowsS{end+1,1} = 'S6 SSR short at 60 min, PID';
T = addvars(array2table(V,'VariableNames',{'Max_T_air_C','Max_T_meas_C','Max_duty_pct','Trip_reached'}), ...
    rowsS,'Before',1,'NewVariableNames','Run');
write_tables(T,'simulation_safety_S6', sprintf(['Simulated maximum temperatures and heater duty in all scenarios; ' ...
    'firmware trip at %g \\textdegree C. Dry steady state at 100 %% duty: %.0f \\textdegree C.'], P.T_trip, xd(3)));
fid = fopen(fullfile(rdir,'simulation_safety_notes.txt'),'w');
fprintf(fid,'Simulated dry steady state at 100 %% duty: %.1f C (sensor), spray steady state at 100 %%: %.1f C\n', xd(3), xs(3));
fprintf(fid,['SSR-short fault at 60 min: max T_air = %.1f C within %.0f min; firmware SSR-stuck alarm ' ...
    '(OFF >= %g s, rise > %g C, T > %g C) at t = %.1f min (%.1f min after the fault)
'], ...
    max(s6f.Ta), s6f.t(end)/60, P.alarm_off_s, P.alarm_dT, P.alarm_Tmin, t_alarm/60, (t_alarm-s6f.P.ssr_fault_t)/60);
fclose(fid);

%% 9. Detailed PWM (time-proportioning) vs duty average -----------------------------------
Q = scenario_P(P,'S2'); Q.ctrl_mode = 1; Q.t_stop = 100*60; Q.spray_t_on = 90*60;
Qp = Q; Qp.ssr_mode = 2; Qp.dt = P.dt_pwm; Qp.log_Ts = P.dt_pwm*10;
oAvg = run_sim(Q, fis_inc, fis_fpid); oPwm = run_sim(Qp, fis_inc, fis_fpid);
fprintf('PWM detail done (%.0f s)\n', toc(tAll));

%% 10. Ablation: Liu (2016) rule tables in the self-tuning FIS ------------------------------
Q = scenario_P(P,'S2'); Q.ctrl_mode = 3;
oLiu = run_sim(Q, fis_inc, F.fis_fpid_liu);
V = []; rn = {'Fuzzy-PID, symmetric rule base (this work)';'Fuzzy-PID, Liu (2016) tables';'PID baseline'};
for r = {runs.S2(3), oLiu, runs.S2(1)}
    m1 = compute_metrics(r{1},0,Q.spray_t_on,'step'); m2 = compute_metrics(r{1},Q.spray_t_on,Q.t_stop,'disturbance');
    V(end+1,:) = [m1.OS m1.t_settle/60 m1.IAE m2.dip m2.t_rec/60 m2.IAE]; %#ok<SAGROW>
end
T = addvars(array2table(V,'VariableNames',{'Overshoot_pct','Settling_time_min','IAE_startup_Cs', ...
    'Max_dev_spray_C','Recovery_time_min','IAE_spray_Cs'}), rn,'Before',1,'NewVariableNames','Variant');
write_tables(T,'simulation_ablation_rulebase','Simulated ablation of the self-tuning rule base (scenario S2).');

%% 11. Figures ---------------------------------------------------------------------------------
make_figures(P, runs, mcT, mcTime, mcM, mcNames, s5, cases, s6f, s6t, oAvg, oPwm, allM, t_alarm);

save(fullfile(rdir,'all_runs.mat'),'runs','allM','-v7.3');
fprintf('ALL DONE in %.1f min\n', toc(tAll)/60);
bdclose('all');
