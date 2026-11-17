function G = design_pid(P, fisInc, fisFpid)
%DESIGN_PID  Tune the baseline discrete PID on the FOPDT approximation with
%   pidtune, Ziegler-Nichols (reaction curve), Cohen-Coon and SIMC
%   (Skogestad 2003), evaluate each on the full Simulink model (start-up +
%   spray disturbance, with sensor noise) and keep the best: lowest total
%   IAE subject to overshoot <= 10 % and steady-state duty standard
%   deviation <= 5 % (SSR/heater wear, noise amplification).
%   Gains in % duty per degC (Ki per degC.s, Kd in %.s per degC).
F = P.fopdt; K = F.K; tau = F.tau;
th = F.theta + P.Ts/2;                 % effective dead time incl. sampling
Gp = tf(K,[tau 1],'InputDelay',th);
cand = struct('method',{},'Kp',{},'Ki',{},'Kd',{},'N',{});
% (1) pidtune on the continuous FOPDT (default 60 deg phase-margin target)
C = pidtune(Gp,'PIDF');
if C.Kd < 0 || C.Ki < 0, C = pidtune(Gp,'PI'); C.Kd = 0; end
Nn = 10; if C.Kd > 0, Nn = min(max(C.Kd/(C.Kp*max(C.Tf,1e-6)),2),20); end
cand(end+1) = struct('method','pidtune','Kp',C.Kp,'Ki',C.Ki,'Kd',C.Kd,'N',Nn);
% (2) Ziegler-Nichols open-loop (reaction curve)
Kp = 1.2*tau/(K*th); Ti = 2*th; Td = 0.5*th;
cand(end+1) = struct('method','Ziegler-Nichols','Kp',Kp,'Ki',Kp/Ti,'Kd',Kp*Td,'N',10);
% (3) Cohen-Coon
r = th/tau;
Kp = (1/K)*(1/r)*(4/3 + r/4); Ti = th*(32+6*r)/(13+8*r); Td = 4*th/(11+2*r);
cand(end+1) = struct('method','Cohen-Coon','Kp',Kp,'Ki',Kp/Ti,'Kd',Kp*Td,'N',10);
% (4) SIMC PI, tight tuning tau_c = theta, and smoother tau_c = 5 theta
for m = [1 5]
    tc = m*th; Kp = tau/(K*(tc+th)); Ti = min(tau, 4*(tc+th));
    cand(end+1) = struct('method',sprintf('SIMC PI (tau_c = %g theta)',m),'Kp',Kp,'Ki',Kp/Ti,'Kd',0,'N',10); %#ok<AGROW>
end

% evaluate on the Simulink model (design scenario = S2 settings)
rows = cell(numel(cand),1); vals = zeros(numel(cand),8);
for k = 1:numel(cand)
    Q = scenario_P(P,'S2'); Q.ctrl_mode = 1;
    Q.Kp = cand(k).Kp; Q.Ki = cand(k).Ki; Q.Kd = cand(k).Kd; Q.N_d = cand(k).N;
    o = run_sim(Q, fisInc, fisFpid);
    m1 = compute_metrics(o, 0, Q.spray_t_on, 'step');
    m2 = compute_metrics(o, Q.spray_t_on, Q.t_stop, 'disturbance');
    w = o.t > Q.spray_t_on-1800 & o.t < Q.spray_t_on;
    rows{k} = cand(k).method;
    vals(k,:) = [cand(k).Kp cand(k).Ki cand(k).Kd m1.OS m1.IAE+m2.IAE m2.dip m1.TV+m2.TV std(o.u_cmd(w))];
end
T = array2table(vals,'VariableNames',{'Kp_pct_per_C','Ki_pct_per_Cs','Kd_pct_s_per_C', ...
    'Overshoot_pct','IAE_total_Cs','Max_dev_spray_C','Sum_abs_du_pct','Duty_std_steady_pct'});
T = addvars(T, rows, 'Before',1,'NewVariableNames','Tuning_method');
ok = T.Overshoot_pct <= 10 & T.Duty_std_steady_pct <= 5;
score = T.IAE_total_Cs; score(~ok) = inf;
[~, ib] = min(score);
if ~isfinite(score(ib)), [~, ib] = min(T.Duty_std_steady_pct); end
G = cand(ib);
T.Selected = repmat({''},height(T),1); T.Selected{ib} = 'baseline';
write_tables(T,'simulation_pid_tuning', sprintf(['Simulated comparison of PID tuning rules on the calibrated model ' ...
    '(FOPDT: K = %.2f \\textdegree C/\\%%, \\texttau = %.0f s, \\texttheta = %.0f s; scenario S2 with noise). ' ...
    'Selection: minimum IAE with overshoot $\\le$ 10 \\%% and steady duty std $\\le$ 5 \\%%.'], K, tau, th));
save(fullfile(fileparts(mfilename('fullpath')),'results','pid_gains.mat'),'G');
fprintf('Baseline PID: %s  Kp=%.3g Ki=%.3g Kd=%.3g N=%.3g\n', G.method, G.Kp, G.Ki, G.Kd, G.N);
end
