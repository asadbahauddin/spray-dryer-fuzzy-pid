function Q = scenario_P(P, name)
%SCENARIO_P  Parameter struct for each simulation scenario.
Q = P;
Q.Ta0 = P.T_amb; Q.Tw0 = P.T_amb; Q.Tm0 = P.T_amb;
Q.spray_t_on = 1e9; Q.spray_t_off = 2e9; Q.ssr_fault_t = 1e9;
switch name
    case 'S1_120'      % start-up, no spray
        Q.sp_time = [0 1e9]; Q.sp_val = [120 120]; Q.t_stop = 2*3600;
    case 'S1_150'
        Q.sp_time = [0 1e9]; Q.sp_val = [150 150]; Q.t_stop = 2*3600;
    case 'S2'          % start-up to spray set-point, spray starts at 90 min
        Q.sp_time = [0 1e9]; Q.sp_val = [P.sp_spray P.sp_spray];
        Q.spray_t_on = 90*60; Q.t_stop = 3*3600;
    case 'S2_120'      % same at 120 degC (heater capacity limit)
        Q.sp_time = [0 1e9]; Q.sp_val = [120 120];
        Q.spray_t_on = 90*60; Q.t_stop = 3*3600;
    case 'S3'          % set-point changes, no spray
        Q.sp_time = [0 60 120]*60; Q.sp_val = [100 120 140]; Q.t_stop = 3*3600;
    case 'S6_fault'    % SSR fails short at 60 min, no spray, SP 120
        Q.sp_time = [0 1e9]; Q.sp_val = [120 120]; Q.ssr_fault_t = 60*60; Q.t_stop = 4*3600;
    case 'S6_trip'     % highest set-point the firmware accepts (SP_MAX = 180 degC), no spray
        Q.sp_time = [0 1e9]; Q.sp_val = [P.sp_max P.sp_max]; Q.t_stop = 4*3600;
    case 'OL_replay'   % open-loop replay of the observed run (heater 100 %)
        Q.ctrl_mode = 4; Q.u_manual = 100; Q.noise_on = 0; Q.u_init = 1;
        Q.sp_time = [0 1e9]; Q.sp_val = [0 0];
        Q.Ta0 = P.obs.T_start; Q.Tm0 = P.obs.T_start; Q.Tw0 = P.Tw_spray;
        Q.spray_t_on = 0; Q.t_stop = P.obs.t_end; Q.log_Ts = 1;
    otherwise
        error('unknown scenario %s', name);
end
end
