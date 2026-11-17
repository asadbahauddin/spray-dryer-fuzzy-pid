function mdl = build_model(doPrint)
%BUILD_MODEL  Programmatically build spray_dryer_fuzzy_pid.slx.
%   All block parameters reference fields of the struct P (base workspace or
%   Simulink.SimulationInput variable). Controller selection by P.ctrl_mode:
%   1 = PID, 2 = incremental Fuzzy, 3 = self-tuning Fuzzy-PID, 4 = manual.
%   FIS objects are read from variables fis_inc and fis_fpid.
if nargin < 1, doPrint = true; end
mdl  = 'spray_dryer_fuzzy_pid';
here = fileparts(mfilename('fullpath'));
if bdIsLoaded(mdl), close_system(mdl,0); end
f = fullfile(here,[mdl '.slx']); if exist(f,'file'), delete(f); end
new_system(mdl); load_system('simulink'); load_system('fuzblock');
FLC = sprintf('fuzblock/Fuzzy Logic \nController');

set_param(mdl,'Solver','ode4','SolverType','Fixed-step','FixedStep','P.dt', ...
    'StopTime','P.t_stop','EnableMultiTasking','off','SaveTime','off','SaveOutput','off', ...
    'SaveState','off','SignalLogging','off','ReturnWorkspaceOutputs','on', ...
    'AlgebraicLoopMsg','error','UnconnectedInputMsg','error','UnconnectedOutputMsg','none');

%% ===== Setpoint =========================================================
s = sub(mdl,'Setpoint',[40 40]);
B(s,'Clock','simulink/Sources/Clock');
B(s,'SPtable','simulink/Lookup Tables/1-D Lookup Table','Table','P.sp_val', ...
  'BreakpointsForDimension1','P.sp_time','InterpMethod','Flat','ExtrapMethod','Clip');
B(s,'SP','built-in/Outport');
L(s,'Clock/1','SPtable/1'); L(s,'SPtable/1','SP/1');

%% ===== Spray disturbance =================================================
s = sub(mdl,'Spray disturbance',[40 200]);
B(s,'Clock','simulink/Sources/Clock');
B(s,'On','simulink/Logic and Bit Operations/Compare To Constant','relop','>=','const','P.spray_t_on');
B(s,'Off','simulink/Logic and Bit Operations/Compare To Constant','relop','<','const','P.spray_t_off');
B(s,'And','simulink/Logic and Bit Operations/Logical Operator','Operator','AND');
B(s,'ToDouble','simulink/Signal Attributes/Data Type Conversion','OutDataTypeStr','double');
B(s,'Qevap','simulink/Math Operations/Gain','Gain','P.eta_evap*P.Q_evap_full*P.spray_scale');
B(s,'Q_evap','built-in/Outport');
L(s,'Clock/1','On/1'); L(s,'Clock/1','Off/1'); L(s,'On/1','And/1'); L(s,'Off/1','And/2');
L(s,'And/1','ToDouble/1'); L(s,'ToDouble/1','Qevap/1'); L(s,'Qevap/1','Q_evap/1');

%% ===== Sensor (RTD PT100 + MAX31865) ======================================
s = sub(mdl,'Sensor RTD MAX31865',[700 120]);
B(s,'T_air','built-in/Inport');
B(s,'Err','simulink/Math Operations/Sum','Inputs','+-');
B(s,'InvTau','simulink/Math Operations/Gain','Gain','1/P.tau_s');
B(s,'Lag','simulink/Continuous/Integrator','InitialCondition','P.Tm0');
B(s,'Read','simulink/Discrete/Zero-Order Hold','SampleTime','P.Ts_sensor');
B(s,'Noise','simulink/Sources/Random Number','Mean','0','Variance','1','Seed','P.seed','SampleTime','P.Ts_sensor');
B(s,'NoiseGain','simulink/Math Operations/Gain','Gain','P.noise_sd*P.noise_on');
B(s,'AddNoise','simulink/Math Operations/Sum','Inputs','++');
B(s,'ADC','simulink/Discontinuities/Quantizer','QuantizationInterval','P.adc_q');
B(s,'T_meas','built-in/Outport');
L(s,'T_air/1','Err/1'); L(s,'Err/1','InvTau/1'); L(s,'InvTau/1','Lag/1'); L(s,'Lag/1','Err/2');
L(s,'Lag/1','Read/1'); L(s,'Read/1','AddNoise/1'); L(s,'Noise/1','NoiseGain/1');
L(s,'NoiseGain/1','AddNoise/2'); L(s,'AddNoise/1','ADC/1'); L(s,'ADC/1','T_meas/1');

%% ===== Controller ===========================================================
s = sub(mdl,'Controller',[250 100]);
B(s,'SP','built-in/Inport'); B(s,'T_meas','built-in/Inport');
B(s,'HoldSP','simulink/Discrete/Zero-Order Hold','SampleTime','P.Ts');
B(s,'HoldT','simulink/Discrete/Zero-Order Hold','SampleTime','P.Ts');
B(s,'e','simulink/Math Operations/Sum','Inputs','+-');
L(s,'SP/1','HoldSP/1'); L(s,'T_meas/1','HoldT/1'); L(s,'HoldSP/1','e/1'); L(s,'HoldT/1','e/2');
% normalised fuzzy inputs (firmware scaling)
B(s,'eScale','simulink/Math Operations/Gain','Gain','1/P.e_range');
B(s,'eSat','simulink/Discontinuities/Saturation','UpperLimit','1','LowerLimit','-1');
% firmware: ePrev = e at the first sample -> first de = 0
B(s,'ePrev','simulink/Discrete/Unit Delay','InitialCondition','P.sp_val(1)-P.Tm0','SampleTime','P.Ts');
B(s,'de','simulink/Math Operations/Sum','Inputs','+-');
B(s,'deScale','simulink/Math Operations/Gain','Gain','1/P.de_range');
B(s,'deSat','simulink/Discontinuities/Saturation','UpperLimit','1','LowerLimit','-1');
B(s,'FuzzIn','simulink/Signal Routing/Mux','Inputs','2');
L(s,'e/1','eScale/1'); L(s,'eScale/1','eSat/1'); L(s,'e/1','ePrev/1'); L(s,'e/1','de/1');
L(s,'ePrev/1','de/2'); L(s,'de/1','deScale/1'); L(s,'deScale/1','deSat/1');
L(s,'eSat/1','FuzzIn/1'); L(s,'deSat/1','FuzzIn/2');

% --- (1) baseline PID -----------------------------------------------------
B(s,'Kp0','simulink/Sources/Constant','Value','P.Kp');
B(s,'Ki0','simulink/Sources/Constant','Value','P.Ki');
B(s,'Kd0','simulink/Sources/Constant','Value','P.Kd');
pidCore(s,'PID',FLC);
L(s,'e/1','PID/1'); L(s,'HoldT/1','PID/2'); L(s,'Kp0/1','PID/3'); L(s,'Ki0/1','PID/4'); L(s,'Kd0/1','PID/5');

% --- (2) incremental fuzzy (firmware) --------------------------------------
B(s,'FuzzyInc',FLC); set_param([s '/FuzzyInc'],'FIS','fis_inc');
B(s,'duScale','simulink/Math Operations/Gain','Gain','P.du_range');
B(s,'uAcc','simulink/Math Operations/Sum','Inputs','++');
B(s,'uClamp','simulink/Discontinuities/Saturation','UpperLimit','P.u_max','LowerLimit','P.u_min');
B(s,'uPrev','simulink/Discrete/Unit Delay','InitialCondition','P.u_min','SampleTime','P.Ts');
L(s,'FuzzIn/1','FuzzyInc/1'); L(s,'FuzzyInc/1','duScale/1'); L(s,'duScale/1','uAcc/1');
L(s,'uPrev/1','uAcc/2'); L(s,'uAcc/1','uClamp/1'); L(s,'uClamp/1','uPrev/1');

% --- (3) self-tuning Fuzzy-PID ------------------------------------------------
B(s,'SelfTune',FLC); set_param([s '/SelfTune'],'FIS','fis_fpid');
B(s,'dK','simulink/Signal Routing/Demux','Outputs','3');
L(s,'FuzzIn/1','SelfTune/1'); L(s,'SelfTune/1','dK/1');
gn = {'Kp','Ki','Kd'};
for i = 1:3
    B(s,['b' gn{i}],'simulink/Math Operations/Gain','Gain','P.gain_bound');
    B(s,['one' gn{i}],'simulink/Math Operations/Bias','Bias','1');
    B(s,[gn{i} 't'],'simulink/Math Operations/Product');
    L(s,sprintf('dK/%d',i),['b' gn{i} '/1']); L(s,['b' gn{i} '/1'],['one' gn{i} '/1']);
    L(s,['one' gn{i} '/1'],[gn{i} 't/1']); L(s,[gn{i} '0/1'],[gn{i} 't/2']);
end
pidCore(s,'FuzzyPID_core',FLC);
L(s,'e/1','FuzzyPID_core/1'); L(s,'HoldT/1','FuzzyPID_core/2');
L(s,'Kpt/1','FuzzyPID_core/3'); L(s,'Kit/1','FuzzyPID_core/4'); L(s,'Kdt/1','FuzzyPID_core/5');
% gains output for logging
B(s,'GainsMux','simulink/Signal Routing/Mux','Inputs','3');
L(s,'Kpt/1','GainsMux/1'); L(s,'Kit/1','GainsMux/2'); L(s,'Kdt/1','GainsMux/3');

% --- (4) manual and selector -------------------------------------------------
B(s,'Manual','simulink/Sources/Constant','Value','P.u_manual');
B(s,'Mode','simulink/Sources/Constant','Value','P.ctrl_mode');
B(s,'Select','simulink/Signal Routing/Multiport Switch','Inputs','4','DataPortOrder','One-based contiguous');
L(s,'Mode/1','Select/1'); L(s,'PID/1','Select/2'); L(s,'uClamp/1','Select/3');
L(s,'FuzzyPID_core/1','Select/4'); L(s,'Manual/1','Select/5');
B(s,'u_cmd','built-in/Outport'); B(s,'gains','built-in/Outport');
L(s,'Select/1','u_cmd/1'); L(s,'GainsMux/1','gains/1');

%% ===== Safety limiter (firmware) ============================================
s = sub(mdl,'Safety limiter',[420 100]);
B(s,'u_cmd','built-in/Inport'); B(s,'T_meas','built-in/Inport');
B(s,'Zero','simulink/Sources/Constant','Value','0');
B(s,'Trip','simulink/Signal Routing/Switch','Criteria','u2 >= Threshold','Threshold','P.T_trip');
B(s,'u_safe','built-in/Outport');
L(s,'Zero/1','Trip/1'); L(s,'T_meas/1','Trip/2'); L(s,'u_cmd/1','Trip/3'); L(s,'Trip/1','u_safe/1');

%% ===== SSR actuator (zero-cross, time-proportioning) ========================
s = sub(mdl,'SSR actuator',[560 100]);
B(s,'u_pct','built-in/Inport');
B(s,'ToFrac','simulink/Math Operations/Gain','Gain','1/100');
B(s,'HalfCycles','simulink/Discontinuities/Quantizer','QuantizationInterval','P.ssr_q');
B(s,'Clock','simulink/Sources/Clock');
B(s,'Win','simulink/Sources/Constant','Value','P.ssr_window');
B(s,'Rem','simulink/Math Operations/Math Function','Operator','rem');
B(s,'Phase','simulink/Math Operations/Gain','Gain','1/P.ssr_window');
B(s,'OnCmp','simulink/Logic and Bit Operations/Relational Operator','Operator','<');
B(s,'OnDbl','simulink/Signal Attributes/Data Type Conversion','OutDataTypeStr','double');
B(s,'SSRmode','simulink/Sources/Constant','Value','P.ssr_mode');
B(s,'ModeSel','simulink/Signal Routing/Multiport Switch','Inputs','2','DataPortOrder','One-based contiguous');
B(s,'Fault','simulink/Logic and Bit Operations/Compare To Constant','relop','>=','const','P.ssr_fault_t');
B(s,'One','simulink/Sources/Constant','Value','1');
B(s,'FaultSw','simulink/Signal Routing/Switch','Criteria','u2 ~= 0');
B(s,'u_frac','built-in/Outport');
L(s,'u_pct/1','ToFrac/1'); L(s,'ToFrac/1','HalfCycles/1');
L(s,'Clock/1','Rem/1'); L(s,'Win/1','Rem/2'); L(s,'Rem/1','Phase/1');
L(s,'Phase/1','OnCmp/1'); L(s,'HalfCycles/1','OnCmp/2'); L(s,'OnCmp/1','OnDbl/1');
L(s,'SSRmode/1','ModeSel/1'); L(s,'HalfCycles/1','ModeSel/2'); L(s,'OnDbl/1','ModeSel/3');
L(s,'Clock/1','Fault/1'); L(s,'One/1','FaultSw/1'); L(s,'Fault/1','FaultSw/2');
L(s,'ModeSel/1','FaultSw/3'); L(s,'FaultSw/1','u_frac/1');

%% ===== Heater + chamber plant (two-node) =====================================
s = sub(mdl,'Heater and chamber plant',[700 220]);
B(s,'u_frac','built-in/Inport'); B(s,'Q_evap','built-in/Inport');
B(s,'Pheat','simulink/Math Operations/Gain','Gain','P.eta_h*P.P_max');
B(s,'Delay','simulink/Continuous/Transport Delay','DelayTime','P.theta_h+P.theta_extra', ...
  'InitialOutput','P.eta_h*P.P_max*P.u_init','BufferSize','4096');
B(s,'TaAmb','simulink/Math Operations/Bias','Bias','-P.T_amb');
B(s,'Gair','simulink/Math Operations/Gain','Gain','P.G_air*P.G_scale');
B(s,'dTaw','simulink/Math Operations/Sum','Inputs','+-');
B(s,'Haw','simulink/Math Operations/Gain','Gain','P.H_aw');
B(s,'SumA','simulink/Math Operations/Sum','Inputs','+---');
B(s,'invCa','simulink/Math Operations/Gain','Gain','1/(P.C_a*P.C_scale)');
B(s,'Ta','simulink/Continuous/Integrator','InitialCondition','P.Ta0');
B(s,'TwAmb','simulink/Math Operations/Bias','Bias','-P.T_amb');
B(s,'UAw','simulink/Math Operations/Gain','Gain','P.UA_w');
B(s,'SumW','simulink/Math Operations/Sum','Inputs','+-');
B(s,'invCw','simulink/Math Operations/Gain','Gain','1/(P.C_w*P.C_scale)');
B(s,'Tw','simulink/Continuous/Integrator','InitialCondition','P.Tw0');
B(s,'Energy','simulink/Continuous/Integrator','InitialCondition','0');
B(s,'T_air','built-in/Outport'); B(s,'T_wall','built-in/Outport');
B(s,'P_heater','built-in/Outport'); B(s,'E_heater','built-in/Outport');
L(s,'u_frac/1','Pheat/1'); L(s,'Pheat/1','Delay/1'); L(s,'Delay/1','SumA/1');
L(s,'Ta/1','TaAmb/1'); L(s,'TaAmb/1','Gair/1'); L(s,'Gair/1','SumA/2');
L(s,'Ta/1','dTaw/1'); L(s,'Tw/1','dTaw/2'); L(s,'dTaw/1','Haw/1'); L(s,'Haw/1','SumA/3');
L(s,'Q_evap/1','SumA/4'); L(s,'SumA/1','invCa/1'); L(s,'invCa/1','Ta/1');
L(s,'Haw/1','SumW/1'); L(s,'Tw/1','TwAmb/1'); L(s,'TwAmb/1','UAw/1'); L(s,'UAw/1','SumW/2');
L(s,'SumW/1','invCw/1'); L(s,'invCw/1','Tw/1');
L(s,'Pheat/1','Energy/1');
L(s,'Ta/1','T_air/1'); L(s,'Tw/1','T_wall/1'); L(s,'Pheat/1','P_heater/1'); L(s,'Energy/1','E_heater/1');

%% ===== Logging ===================================================================
s = sub(mdl,'Logging',[900 100]);
nm = {'SP','T_air','T_meas','u_cmd','u_safe','u_frac','Q_evap','P_heater','E_heater','T_wall','gains'};
for i = 1:numel(nm), B(s,nm{i},'built-in/Inport'); end
B(s,'Mux','simulink/Signal Routing/Mux','Inputs',num2str(numel(nm)));
B(s,'ToWs','simulink/Sinks/To Workspace','VariableName','simlog', ...
  'SaveFormat','Structure With Time','SampleTime','P.log_Ts','MaxDataPoints','inf');
for i = 1:numel(nm), L(s,[nm{i} '/1'],sprintf('Mux/%d',i)); end
L(s,'Mux/1','ToWs/1');

%% ===== Top-level wiring ==============================================================
T = mdl;
L(T,'Setpoint/1','Controller/1');
L(T,'Sensor RTD MAX31865/1','Controller/2');
L(T,'Controller/1','Safety limiter/1'); L(T,'Sensor RTD MAX31865/1','Safety limiter/2');
L(T,'Safety limiter/1','SSR actuator/1');
L(T,'SSR actuator/1','Heater and chamber plant/1'); L(T,'Spray disturbance/1','Heater and chamber plant/2');
L(T,'Heater and chamber plant/1','Sensor RTD MAX31865/1');
L(T,'Setpoint/1','Logging/1'); L(T,'Heater and chamber plant/1','Logging/2');
L(T,'Sensor RTD MAX31865/1','Logging/3'); L(T,'Controller/1','Logging/4');
L(T,'Safety limiter/1','Logging/5'); L(T,'SSR actuator/1','Logging/6');
L(T,'Spray disturbance/1','Logging/7'); L(T,'Heater and chamber plant/3','Logging/8');
L(T,'Heater and chamber plant/4','Logging/9'); L(T,'Heater and chamber plant/2','Logging/10');
L(T,'Controller/2','Logging/11');

% tidy layout
subs = find_system(mdl,'SearchDepth',1,'BlockType','SubSystem');
for i = 1:numel(subs), try, Simulink.BlockDiagram.arrangeSystem(subs{i}); catch, end, end
for c = {'PID','FuzzyPID_core'}
    try, Simulink.BlockDiagram.arrangeSystem([mdl '/Controller/' c{1}]); catch, end
end
try, Simulink.BlockDiagram.arrangeSystem(mdl); catch, end
set_param(mdl,'ZoomFactor','FitSystem');
save_system(mdl, f);

if doPrint
    d = fullfile(here,'results','figures'); if ~exist(d,'dir'), mkdir(d); end
    try
        print(['-s' mdl],'-dpng','-r300',fullfile(d,'fig01_model_top.png'));
        print(['-s' mdl],'-dpdf',fullfile(d,'fig01_model_top.pdf'));
        print(['-s' mdl '/Controller'],'-dpng','-r300',fullfile(d,'fig02_model_controller.png'));
        print(['-s' mdl '/Controller'],'-dpdf',fullfile(d,'fig02_model_controller.pdf'));
        print(['-s' mdl '/Heater and chamber plant'],'-dpng','-r300',fullfile(d,'fig02b_model_plant.png'));
        fid = fopen(fullfile(d,'captions.md'),'a');
        fprintf(fid,['- **fig01_model_top**: Top level of the Simulink Simulation model (setpoint, controller, ' ...
            'firmware safety limiter, SSR actuator, two-node heater/chamber plant, spray disturbance, RTD sensor, logging).\n']);
        fprintf(fid,['- **fig02_model_controller**: Controller subsystem of the Simulation model: discrete PID, ' ...
            'incremental fuzzy (firmware) and self-tuning Fuzzy-PID, selected by P.ctrl_mode.\n']);
        fclose(fid);
    catch ME
        warning('build_model:print','Model diagram not printed: %s', ME.message);
    end
end
end

% =============================================================================
function pidCore(parent, name, ~)
% Discrete PID, Ts = P.Ts, inputs: e, y, Kp, Ki, Kd (time-varying allowed)
%   u = Kp e + I - Kd * D{y},  I(k) = I(k-1) + Ki Ts e  (clamping anti-windup)
%   D{y}: first-order filtered backward difference of the measurement.
s = [parent '/' name];
add_block('built-in/Subsystem', s);
for p = {'e','y','Kp','Ki','Kd'}, B(s,p{1},'built-in/Inport'); end
B(s,'Pterm','simulink/Math Operations/Product');
B(s,'KiE','simulink/Math Operations/Product');
B(s,'Ts','simulink/Math Operations/Gain','Gain','P.Ts');
B(s,'Zero','simulink/Sources/Constant','Value','0');
B(s,'Clamp','simulink/Signal Routing/Switch','Criteria','u2 ~= 0');
B(s,'Iacc','simulink/Math Operations/Sum','Inputs','++');
B(s,'Iprev','simulink/Discrete/Unit Delay','InitialCondition','0','SampleTime','P.Ts');
B(s,'yDev','simulink/Math Operations/Bias','Bias','-P.Tm0');
B(s,'Dfilt','simulink/Discrete/Discrete Transfer Fcn', ...
  'Numerator','[(1-P.d_a)/P.Ts, -(1-P.d_a)/P.Ts]','Denominator','[1, -P.d_a]','SampleTime','P.Ts');
B(s,'Dterm','simulink/Math Operations/Product');
B(s,'uSum','simulink/Math Operations/Sum','Inputs','++-');
B(s,'uPrev','simulink/Discrete/Unit Delay','InitialCondition','0','SampleTime','P.Ts');
B(s,'SatHi','simulink/Logic and Bit Operations/Compare To Constant','relop','>','const','P.u_max');
B(s,'SatLo','simulink/Logic and Bit Operations/Compare To Constant','relop','<','const','P.u_min');
B(s,'ePos','simulink/Logic and Bit Operations/Compare To Zero','relop','>');
B(s,'eNeg','simulink/Logic and Bit Operations/Compare To Zero','relop','<');
B(s,'AndHi','simulink/Logic and Bit Operations/Logical Operator','Operator','AND');
B(s,'AndLo','simulink/Logic and Bit Operations/Logical Operator','Operator','AND');
B(s,'Or','simulink/Logic and Bit Operations/Logical Operator','Operator','OR');
B(s,'uSat','simulink/Discontinuities/Saturation','UpperLimit','P.u_max','LowerLimit','P.u_min');
B(s,'u','built-in/Outport');
L(s,'Kp/1','Pterm/1'); L(s,'e/1','Pterm/2');
L(s,'Ki/1','KiE/1'); L(s,'e/1','KiE/2'); L(s,'KiE/1','Ts/1');
L(s,'Zero/1','Clamp/1'); L(s,'Or/1','Clamp/2'); L(s,'Ts/1','Clamp/3');
L(s,'Clamp/1','Iacc/1'); L(s,'Iprev/1','Iacc/2'); L(s,'Iacc/1','Iprev/1');
L(s,'y/1','yDev/1'); L(s,'yDev/1','Dfilt/1'); L(s,'Kd/1','Dterm/1'); L(s,'Dfilt/1','Dterm/2');
L(s,'Pterm/1','uSum/1'); L(s,'Iacc/1','uSum/2'); L(s,'Dterm/1','uSum/3');
L(s,'uSum/1','uPrev/1'); L(s,'uPrev/1','SatHi/1'); L(s,'uPrev/1','SatLo/1');
L(s,'e/1','ePos/1'); L(s,'e/1','eNeg/1');
L(s,'SatHi/1','AndHi/1'); L(s,'ePos/1','AndHi/2'); L(s,'SatLo/1','AndLo/1'); L(s,'eNeg/1','AndLo/2');
L(s,'AndHi/1','Or/1'); L(s,'AndLo/1','Or/2');
L(s,'uSum/1','uSat/1'); L(s,'uSat/1','u/1');
end

function s = sub(mdl, name, pos)
s = [mdl '/' name];
add_block('built-in/Subsystem', s, 'Position',[pos pos+[110 60]]);
end

function B(sys, name, lib, varargin)
add_block(lib, [sys '/' name], varargin{:});
end

function L(sys, a, b)
add_line(sys, a, b, 'autorouting','smart');
end
