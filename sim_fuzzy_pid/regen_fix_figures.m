%REGEN_FIX_FIGURES  Re-create the calibration (fig03) and safety (fig11) figures
%   and the safety notes without re-running the Monte-Carlo study.
here = fileparts(mfilename('fullpath')); cd(here);
warning('off','MATLAB:print:ContentTypeImageSuggested');
warning('off','optim:lsqncommon:AlgorithmConflict');
if ~exist('skipCal','var'), calibrate_plant(params_spray_dryer()); end
S = load(fullfile('results','P_final.mat')); P = S.P;
F = build_fis(P,false);
load_system('spray_dryer_fuzzy_pid');
s6f = run_sim(setfield(scenario_P(P,'S6_fault'),'ctrl_mode',1), F.fis_inc, F.fis_fpid); %#ok<SFLD>
s6t = run_sim(setfield(scenario_P(P,'S6_trip'),'ctrl_mode',1), F.fis_inc, F.fis_fpid);  %#ok<SFLD>
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
plot_s6(P, s6f, s6t, t_alarm);
[A_,B_] = plant_lin(P); Qn = P.eta_evap*P.Q_evap_full;
xs = -A_\(B_*[1;Qn;1]); xd = -A_\(B_*[1;0;1]);
fid = fopen(fullfile('results','simulation_safety_notes.txt'),'w');
fprintf(fid,'Simulated dry steady state at 100 %% duty: %.1f C (sensor), spray steady state at 100 %%: %.1f C\n', xd(3), xs(3));
fprintf(fid,['SSR-short fault at 60 min: max T_air = %.1f C within %.0f min; firmware SSR-stuck alarm ' ...
    '(OFF >= %g s, rise > %g C, T > %g C) at t = %.1f min (%.1f min after the fault)\n'], ...
    max(s6f.Ta), s6f.t(end)/60, P.alarm_off_s, P.alarm_dT, P.alarm_Tmin, t_alarm/60, (t_alarm-s6f.P.ssr_fault_t)/60);
fclose(fid);
type(fullfile('results','simulation_safety_notes.txt'));
bdclose('all');
