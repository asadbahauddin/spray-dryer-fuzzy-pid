function P = load_params()
%LOAD_PARAMS  Nominal parameters + calibrated plant + designed PID gains.
P = params_spray_dryer();
rdir = fullfile(fileparts(mfilename('fullpath')),'results');
f = fullfile(rdir,'calibrated_params.mat');
if exist(f,'file')
    S = load(f,'Cal');
    for n = S.Cal.names, P.(n{1}) = S.Cal.(n{1}); end
    P.fopdt = S.Cal.fopdt;
end
f = fullfile(rdir,'pid_gains.mat');
if exist(f,'file')
    S = load(f,'G');
    P.Kp = S.G.Kp; P.Ki = S.G.Ki; P.Kd = S.G.Kd; P.N_d = S.G.N;
    P.pid_method = S.G.method;
end
P = derived(P);
end

function P = derived(P)
if P.Kd > 0 && P.Kp > 0
    Tf = P.Kd/(P.Kp*P.N_d); P.d_a = Tf/(Tf+P.Ts);
else
    P.d_a = 0;
end
end
