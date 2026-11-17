function R = calib_run(P)
%CALIB_RUN  Open-loop replay of the observed run, starting at the spray start:
%   heater at 100 %, sensor (and air node) at P.obs.T_start, wall node at the
%   unknown P.Tw_spray (preheat length/history was not recorded, A4), spray on
%   for P.obs.t_end s. Exact modal solution of the linear model on a 1 s grid.
%   R.t  : time after spray start (s); R.Tm, R.Ta, R.Tw : temperatures (degC)
Q  = P.eta_evap*P.Q_evap_full*P.spray_scale;
[A,B] = plant_lin(P);
[V,Lam] = eig(A); lam = real(diag(Lam)); V = real(V);
x1 = [P.obs.T_start; P.Tw_spray; P.obs.T_start];
xss = -A\(B*[1;Q;1]);
c = V\(x1 - xss);
R.t = 0:1:P.obs.t_end;
X = xss + V*(c.*exp(lam*R.t));
R.Ta = X(1,:); R.Tw = X(2,:); R.Tm = X(3,:);
R.ok = all(isfinite(R.Tm));
% anchor features
idx = R.t <= 30*60;
[R.T_min, i] = min(R.Tm(idx)); R.t_min = R.t(i);
if i > 1 && i < nnz(idx)          % parabolic refinement (smooth for the optimiser)
    y = R.Tm(i-1:i+1); d = y(1) - 2*y(2) + y(3);
    if d > 0
        s = 0.5*(y(1)-y(3))/d; R.t_min = R.t(i) + s; R.T_min = y(2) - 0.25*(y(1)-y(3))*s;
    end
end
R.T_reach = interp1(R.t, R.Tm, P.obs.t_reach);
R.T_end   = R.Tm(end);
R.slope_end = R.Tm(end) - interp1(R.t, R.Tm, P.obs.t_end-600);  % degC per 10 min
end
