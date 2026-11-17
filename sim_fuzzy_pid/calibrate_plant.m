function Cal = calibrate_plant(P)
%CALIBRATE_PLANT  Fit the two-node model to the operator-observed anchor
%   points (NOT a logged dataset). Multi-start bounded least squares with a
%   weak prior toward physically motivated nominal values; reports the
%   spread of equally good parameter sets (identifiability) and a +/-20 %
%   sensitivity table; derives a FOPDT approximation for PID tuning.
%   Saves results/calibrated_params.mat and tables, and figures.
here = fileparts(mfilename('fullpath'));
rdir = fullfile(here,'results'); if ~exist(rdir,'dir'), mkdir(rdir); end

names = {'C_a','C_w','G_air','H_aw','UA_w','eta_evap','Tw_spray'};
units = {'J/K','J/K','W/K','W/K','W/K','-','degC'};
np    = numel(names);
pnom  = cellfun(@(n) P.(n), names);
lb    = [1e2  1e3   1    1    0.5  0.05  P.T_amb];
ub    = [5e4  2e5   P.G_air_free  300  60  1.00  P.obs.T_start];
lam   = 0.3;                          % prior weight (degC per e-fold)

res = @(x) calib_residuals(x, P, names, pnom, lam);
opts = optimoptions('lsqnonlin','Display','off','MaxFunctionEvaluations',4000, ...
    'FunctionTolerance',1e-10,'StepTolerance',1e-10);
rng(P.mc_seed);
nStart = 40;
X0 = [log(pnom); log(lb) + rand(nStart-1,np).*(log(ub)-log(lb))];
sols = zeros(nStart,np); cost = inf(nStart,1); anchorRMS = inf(nStart,1);
for k = 1:nStart
    try
        x = lsqnonlin(res, X0(k,:), log(lb), log(ub), opts);
        r = res(x); sols(k,:) = exp(x); cost(k) = sum(r.^2);
        anchorRMS(k) = sqrt(mean(r(1:5).^2));
    catch
    end
end
[~, ib] = min(cost);
pbest = sols(ib,:);
fprintf('Calibration: best cost %.3f, anchor RMS %.3f degC\n', cost(ib), anchorRMS(ib));
atBound = names(pbest <= lb*1.01 | pbest >= ub*0.99);
if ~isempty(atBound), fprintf('  parameters at a bound: %s\n', strjoin(atBound,', ')); end

% ----- Apply best parameters -------------------------------------------
Pc = P; for i = 1:np, Pc.(names{i}) = pbest(i); end
R  = calib_run(Pc);

% ----- Identifiability: all solutions fitting the anchors about as well -
thr = max(1.0, 1.5*anchorRMS(ib));
good = anchorRMS <= thr & isfinite(cost);
S = sols(good,:);
spread = [min(S,[],1); max(S,[],1)];
Tid = table(names', units', pnom', lb', ub', pbest', spread(1,:)', spread(2,:)', ...
    'VariableNames',{'Parameter','Unit','Prior','Lower_bound','Upper_bound','Best_fit','Min_among_good_fits','Max_among_good_fits'});
writetable(Tid, fullfile(rdir,'simulation_calibration_parameters.csv'));
fprintf('%d of %d starts reached anchor RMS <= %.2f degC\n', nnz(good), nStart, thr);

% derived physically meaningful quantities for every good fit
dq = zeros(size(S,1),5);
for k = 1:size(S,1)
    Pk = P; for i = 1:np, Pk.(names{i}) = S(k,i); end
    dq(k,:) = steady_quantities(Pk);
end
d0 = steady_quantities(Pc);
qn = {'G_total (W/K)';'Q_evap (W)';'T_ss dry, u=100% (degC)';'T_ss spray, u=100% (degC)';'Implied air flow (m3/h)'};
Tder = table(qn, d0(:), min(dq,[],1)', max(dq,[],1)', ...
    'VariableNames',{'Quantity','Best_fit','Min_among_good_fits','Max_among_good_fits'});
writetable(Tder, fullfile(rdir,'simulation_calibration_derived.csv'));

% ----- Sensitivity +/-20 % ------------------------------------------------
rows = {}; vals = [];
for i = 1:np
    for s = [0.8 1.2]
        Ps = Pc; Ps.(names{i}) = pbest(i)*s;
        Rs = calib_run(Ps);
        rows{end+1,1} = sprintf('%s x%.1f', names{i}, s); %#ok<AGROW>
        vals(end+1,:) = [Rs.T_min, Rs.t_min/60, Rs.T_reach, Rs.T_end]; %#ok<AGROW>
    end
end
vn = {'T_min_degC','t_min_min','T_at_60min_degC','T_at_120min_degC'};
Tsens = array2table(vals,'VariableNames',vn);
Tsens = addvars(Tsens, rows, 'Before',1,'NewVariableNames','Perturbation');
Tsens = [table({'best fit'},R.T_min,R.t_min/60,R.T_reach,R.T_end,'VariableNames',Tsens.Properties.VariableNames); Tsens];
Tsens = [table({'observed anchor'},P.obs.T_min,P.obs.t_min/60,P.obs.T_plat,P.obs.T_plat,'VariableNames',Tsens.Properties.VariableNames); Tsens];
writetable(Tsens, fullfile(rdir,'simulation_calibration_sensitivity.csv'));

% ----- FOPDT approximation (u in %, T_meas in degC) ----------------------
F = fopdt_from_model(Pc);
fprintf('FOPDT: K = %.3f degC/%%, tau = %.1f s, theta = %.1f s\n', F.K, F.tau, F.theta);

% ----- Save -----------------------------------------------------------------
Cal = struct(); for i = 1:np, Cal.(names{i}) = pbest(i); end
Cal.fopdt = F; Cal.anchorRMS = anchorRMS(ib);
Cal.good_sets = S; Cal.names = names; Cal.R = R;
save(fullfile(rdir,'calibrated_params.mat'),'Cal');

% ----- Figures ----------------------------------------------------------------
plot_calibration(P, Pc, R, S, names, F);
end

% =============================================================================
function r = calib_residuals(x, P, names, pnom, lam)
p = exp(x);
for i = 1:numel(names), P.(names{i}) = p(i); end
R = calib_run(P);
if ~R.ok
    r = [50*ones(5,1); lam*(x(:)-log(pnom(:)))]; return
end
r = [ R.T_min - P.obs.T_min;                       % 70 degC minimum
     (R.t_min - P.obs.t_min)/60;                   % 1 degC per minute of timing error
      0.5*(R.T_reach - P.obs.T_plat);              % ~120 degC at ~60 min (approximate)
      R.T_end - P.obs.T_plat;                      % 120 degC at 120 min
      2*R.slope_end ];                             % flat at the end
r = [r; lam*(x(:)-log(pnom(:)))];
end

function d = steady_quantities(P)
[A,B] = plant_lin(P);
Q = P.eta_evap*P.Q_evap_full*P.spray_scale;
xd = -A\(B*[1;0;1]); xs = -A\(B*[1;Q;1]);
Gt = P.eta_h*P.P_max/(xd(3) - P.T_amb);
d = [Gt, Q, xd(3), xs(3), P.G_air/(P.rho_air*P.cp_air)*3600];
end

function F = fopdt_from_model(P)
% Step of +10 % duty around the spray operating point; linear model so the
% gain/time constants do not depend on the operating point. Two-point
% (28.3 % / 63.2 %) method on the sensor output, heater delay included.
dt = 1; N = 4*3600;
[A,B] = plant_lin(P);
Q = P.eta_evap*P.Q_evap_full;
u0 = 0.5;
x0 = -A\(B*[u0;Q;1]);
nd = round((P.theta_h+P.theta_extra)/dt);
W = repmat([u0;Q;1],1,N); W(1,nd+1:end) = u0+0.1;
X = sim_lin(P,x0,W,dt);
y = X(3,:)-X(3,1); t = (0:N)*dt;
K = y(end)/10;
z = y/y(end); k0 = find(z > 1e-6, 1);
[zu, iu] = unique(z(k0:end)); tu = t(k0-1+iu);
t28 = interp1(zu, tu, 0.283); t63 = interp1(zu, tu, 0.632);
tau2p = 1.5*(t63-t28);
% least-squares FOPDT fit over the first 2 h (the two-point rule returns
% theta ~ 0 because of the fast air mode); theta >= heater delay
w = t <= 2*3600;
fo = @(q) K*10*(1-exp(-max(t(w)-q(2),0)/q(1)));
J  = @(q) sum((y(w) - fo(abs(q))).^2);
q  = abs(fminsearch(J, [tau2p, P.theta_h+P.tau_s], optimset('Display','off')));
tau = q(1); theta = max(q(2), P.theta_h);
F = struct('K',K,'tau',tau,'theta',theta,'t',t,'y',y,'u_step',10,'tau_2pt',tau2p);
end

function plot_calibration(P, Pc, R, S, names, F)
St = ctrl_style();
f = fig_new(17,7.5);
tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
ax = nexttile(tl); hold(ax,'on');
% envelope of equally good fits
for k = 1:size(S,1)
    Pk = P; for i = 1:numel(names), Pk.(names{i}) = S(k,i); end
    Rk = calib_run(Pk);
    plot(ax, Rk.t/60, Rk.Tm, '-', 'Color',[0.75 0.75 0.75], 'LineWidth',0.6, 'HandleVisibility','off');
end
h1 = plot(ax, R.t/60, R.Tm, '-', 'Color',St.colors(1,:), 'LineWidth',1.5);
h2 = plot(ax, R.t/60, R.Tw, ':', 'Color',St.ol_color, 'LineWidth',1.2);
ta = [0, P.obs.t_min, P.obs.t_reach, P.obs.t_end]/60;
Ta = [P.obs.T_start, P.obs.T_min, P.obs.T_plat, P.obs.T_plat];
h3 = plot(ax, ta, Ta, 'ks', 'MarkerFaceColor','w','MarkerSize',7,'LineWidth',1.2);
h4 = plot(ax, NaN, NaN, '-', 'Color',[0.75 0.75 0.75]);
xlabel(ax,'Time after spray start (min)'); ylabel(ax,'Temperature (\circC)');
legend(ax,[h1 h2 h4 h3],{'Simulated T_{meas} (best fit)','Simulated wall T_w', ...
    'Equally good fits','Observed anchors (approx.)'}, ...
    'Location','southeast','FontSize',7.5);
ylim(ax,[0 140]);
title(ax,'(a) Open-loop calibration, heater 100 %'); xlim(ax,[0 P.obs.t_end/60]);
ax2 = nexttile(tl);
plot(ax2, F.t/60, F.y, '-', 'Color',St.colors(1,:)); hold(ax2,'on');
yf = F.K*F.u_step*(1-exp(-max(F.t-F.theta,0)/F.tau));
plot(ax2, F.t/60, yf, '--', 'Color',St.colors(2,:));
xlabel(ax2,'Time (min)'); ylabel(ax2,'\DeltaT_{meas} (\circC)');
legend(ax2,{'Simulated two-node model','FOPDT approximation'},'Location','southeast');
title(ax2,'(b) +10 % duty step and FOPDT fit');
text(ax2,0.96,0.40,sprintf('K = %.2f \\circC/%%\n\\tau = %.0f s\n\\theta = %.0f s',F.K,F.tau,F.theta), ...
    'Units','normalized','HorizontalAlignment','right','VerticalAlignment','bottom');
fig_save(f,'fig03_calibration', ['Simulated model response: model calibration against operator-observed ' ...
    'temperature profile (approximate anchor points), heater at 100 % duty and spray started at 90 \circC (a); ' ...
    'simulated step response of the calibrated model and its FOPDT approximation (b).']);
end
