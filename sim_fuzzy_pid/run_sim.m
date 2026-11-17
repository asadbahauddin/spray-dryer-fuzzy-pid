function o = run_sim(P, fisInc, fisFpid)
%RUN_SIM  Run spray_dryer_fuzzy_pid.slx for one parameter struct P.
%   Returns logged signals on the P.log_Ts grid.
mdl = 'spray_dryer_fuzzy_pid';
if ~bdIsLoaded(mdl), load_system(mdl); end
if P.Kd > 0 && P.Kp > 0
    Tf = P.Kd/(P.Kp*P.N_d); P.d_a = Tf/(Tf+P.Ts);
else
    P.d_a = 0;
end
in = Simulink.SimulationInput(mdl);
in = in.setVariable('P',P);
in = in.setVariable('fis_inc',fisInc);
in = in.setVariable('fis_fpid',fisFpid);
in = in.setModelParameter('FixedStep',num2str(P.dt),'StopTime',num2str(P.t_stop));
so = sim(in);
if ~isempty(so.ErrorMessage), error('run_sim:fail','%s',so.ErrorMessage); end
L = so.simlog; v = L.signals.values;
o.t = L.time(:);
o.SP = v(:,1); o.Ta = v(:,2); o.Tm = v(:,3); o.u_cmd = v(:,4); o.u_safe = v(:,5);
o.u_frac = v(:,6); o.Q = v(:,7); o.Ph = v(:,8); o.E = v(:,9); o.Tw = v(:,10);
o.Kp = v(:,11); o.Ki = v(:,12); o.Kd = v(:,13);
o.P = P;
end
