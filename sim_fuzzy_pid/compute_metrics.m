function M = compute_metrics(o, t0, t1, mode, band)
%COMPUTE_METRICS  Performance metrics of a simulated run on [t0, t1] (s).
%   mode = 'step'        : set-point response (rise, settling, overshoot ...)
%   mode = 'disturbance' : spray-start rejection (max deviation, recovery)
%   Metrics are evaluated on the true chamber-air temperature T_a.
if nargin < 5, band = 2; end
idx = o.t >= t0 & o.t <= t1;
t = o.t(idx) - t0; y = o.Ta(idx); r = o.SP(idx); u = o.u_cmd(idx);
e = r - y;
M.IAE  = trapz(t, abs(e));
M.ISE  = trapz(t, e.^2);
M.ITAE = trapz(t, t.*abs(e));
M.E_kWh = (interp1(o.t,o.E,t1) - interp1(o.t,o.E,t0))/3.6e6;
M.TV   = sum(abs(diff(u)));
M.u_max = max(o.u_safe(idx)); M.T_max = max(y);
tail = t >= t(end) - 600;
M.ess = abs(mean(e(tail)));
rf = r(end);
outside = abs(y - rf) > band;
switch mode
    case 'step'
        y0 = y(1); d = rf - y0;
        z = (y - y0)/d;
        i10 = find(z >= 0.1,1); i90 = find(z >= 0.9,1);
        if isempty(i10) || isempty(i90), M.t_rise = NaN; else, M.t_rise = t(i90)-t(i10); end
        M.OS = max(0, max(z) - 1)*100;
        k = find(outside,1,'last');
        if isempty(k), M.t_settle = 0; elseif k == numel(t), M.t_settle = NaN; else, M.t_settle = t(k+1); end
        M.dip = NaN; M.t_rec = NaN;
    case 'disturbance'
        M.t_rise = NaN; M.OS = NaN; M.t_settle = NaN;
        M.dip = max(abs(e));
        k = find(outside,1,'last');
        if isempty(k), M.t_rec = 0; elseif k == numel(t), M.t_rec = NaN; else, M.t_rec = t(k+1); end
end
end
