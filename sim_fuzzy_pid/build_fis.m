function F = build_fis(P, doPlots)
%BUILD_FIS  Create the two Mamdani FIS used in the study and save them.
%   fis_inc  : incremental fuzzy controller identical to the ESP32 firmware
%              (5 triangular MFs, 5x5 rule table, min-max, centroid).
%   fis_fpid : self-tuning supervisor for the PID gains (7 MFs, outputs
%              dKp, dKi, dKd normalised to [-1,1]); rule base symmetric in
%              the sign of the error (see buildSelfTuneTables).
%   fis_fpid_liu : same structure with the widely reproduced 7x7 tables of
%              Liu (2016) - used only as an ablation.
if nargin < 2, doPlots = true; end
here = fileparts(mfilename('fullpath'));
rdir = fullfile(here,'results'); if ~exist(rdir,'dir'), mkdir(rdir); end

%% ---------------- Incremental fuzzy (firmware) --------------------------
L5 = {'NB','NS','Z','PS','PB'}; c5 = [-1 -0.5 0 0.5 1]; w5 = 0.5;
fis = mamfis('Name','fuzzy_inc','AndMethod','min','OrMethod','max', ...
    'ImplicationMethod','min','AggregationMethod','max','DefuzzificationMethod','centroid');
fis = addInput(fis,[-1 1],'Name','e');
fis = addInput(fis,[-1 1],'Name','de');
fis = addOutput(fis,[-1 1],'Name','du');
for v = {'e','de','du'}
    for k = 1:5
        fis = addMF(fis,v{1},'trimf',[c5(k)-w5 c5(k) c5(k)+w5],'Name',L5{k});
    end
end
% firmware rule table: rows e = NB..PB, columns de = NB..PB, entries 0..4
RT = [0 0 0 1 2;
      0 0 1 2 3;
      0 1 2 3 4;
      1 2 3 4 4;
      2 3 4 4 4];
rl = zeros(25,5); n = 0;
for i = 1:5, for j = 1:5, n = n+1; rl(n,:) = [i j RT(i,j)+1 1 1]; end, end
fis = addRule(fis, rl);
F.fis_inc = fis;
writeFIS(fis, fullfile(here,'fuzzy_inc.fis'));

%% ---------------- Self-tuning Fuzzy-PID ---------------------------------
[Tkp,Tki,Tkd] = buildSelfTuneTables();
F.fis_fpid = makeSelfTune('fuzzy_pid_selftune', Tkp, Tki, Tkd);
writeFIS(F.fis_fpid, fullfile(here,'fuzzy_pid_selftune.fis'));
[Lkp,Lki,Lkd] = liuTables();
F.fis_fpid_liu = makeSelfTune('fuzzy_pid_liu', Lkp, Lki, Lkd);
writeFIS(F.fis_fpid_liu, fullfile(here,'fuzzy_pid_liu.fis'));

% export rule tables
L7 = {'NB','NM','NS','ZO','PS','PM','PB'};
tbl = {Tkp,'dKp';Tki,'dKi';Tkd,'dKd'};
fid = fopen(fullfile(rdir,'simulation_fuzzy_pid_rule_tables.md'),'w');
for t = 1:3
    fprintf(fid,'\n%s (rows e = NB..PB, columns de = NB..PB)\n\n| e \\ de | %s |\n|%s\n', ...
        tbl{t,2}, strjoin(L7,' | '), repmat('---|',1,8));
    for i = 1:7
        fprintf(fid,'| %s | %s |\n', L7{i}, strjoin(L7(tbl{t,1}(i,:)),' | '));
    end
end
fclose(fid);

if doPlots, plotFis(F); end
end

% =========================================================================
function fis = makeSelfTune(name, Tkp, Tki, Tkd)
L7 = {'NB','NM','NS','ZO','PS','PM','PB'}; c7 = linspace(-1,1,7); w7 = 1/3;
fis = mamfis('Name',name,'AndMethod','min','OrMethod','max', ...
    'ImplicationMethod','min','AggregationMethod','max','DefuzzificationMethod','centroid');
fis = addInput(fis,[-1 1],'Name','e');
fis = addInput(fis,[-1 1],'Name','de');
fis = addOutput(fis,[-1 1],'Name','dKp');
fis = addOutput(fis,[-1 1],'Name','dKi');
fis = addOutput(fis,[-1 1],'Name','dKd');
for v = {'e','de','dKp','dKi','dKd'}
    for k = 1:7
        fis = addMF(fis,v{1},'trimf',[c7(k)-w7 c7(k) c7(k)+w7],'Name',L7{k});
    end
end
rl = zeros(49,7); n = 0;
for i = 1:7, for j = 1:7
    n = n+1; rl(n,:) = [i j Tkp(i,j) Tki(i,j) Tkd(i,j) 1 1];
end, end
fis = addRule(fis, rl);
end

function [Tkp,Tki,Tkd] = buildSelfTuneTables()
% Heuristics (Zhao, Tomizuka & Isaka 1993; Astrom & Hagglund 2006):
%  - large |e| (start-up, big set-point change): larger Kp for speed,
%    smaller Ki to limit integral build-up, smaller Kd;
%  - medium |e| approaching the set-point (e*de < 0): moderate Kp, larger
%    Kd to brake and prevent overshoot;
%  - small |e|: larger Ki to remove steady-state error, slightly lower Kp
%    for noise robustness; if the error is diverging (e*de > 0) Kp is raised.
% Levels -3..3 map to NB..PB. Tables are symmetric under (e,de)->(-e,-de).
Tkp = zeros(7); Tki = Tkp; Tkd = Tkp;
kp_e = [-1 0 1 3]; ki_e = [3 1 -1 -3]; kd_e = [0 1 1 -2];
for i = -3:3
    for j = -3:3
        a = abs(i); div = (i*j > 0); conv = (i*j < 0);
        kp = kp_e(a+1) + div + (i==0)*floor(abs(j)/2);
        ki = ki_e(a+1) - div - (i==0 && abs(j)>=2);
        kd = kd_e(a+1) + (conv && a>0 && a<3) + (i==0)*(abs(j)>=2);
        Tkp(i+4,j+4) = min(max(kp,-3),3) + 4;
        Tki(i+4,j+4) = min(max(ki,-3),3) + 4;
        Tkd(i+4,j+4) = min(max(kd,-3),3) + 4;
    end
end
end

function [Tkp,Tki,Tkd] = liuTables()
% Widely reproduced 7x7 self-tuning tables (Liu Jinkun, "Advanced PID
% Control and MATLAB Simulation") - VERIFY edition/page. 1=NB ... 7=PB.
Tkp = [7 7 6 6 5 4 4; 7 7 6 5 5 4 3; 6 6 6 5 4 3 3; 6 6 5 4 3 2 2;
       5 5 4 3 3 2 2; 5 4 3 2 2 2 1; 4 4 2 2 2 1 1];
Tki = [1 1 2 2 3 4 4; 1 1 2 3 3 4 4; 1 2 3 3 4 5 5; 2 2 3 4 5 6 6;
       2 3 4 5 5 6 7; 4 4 5 5 6 7 7; 4 4 5 6 6 7 7];
Tkd = [5 3 1 1 1 2 5; 5 3 1 2 2 3 4; 4 3 2 2 3 3 4; 4 3 3 3 3 3 4;
       4 4 4 4 4 4 4; 7 3 5 5 5 5 7; 7 6 6 6 5 5 7];
end

function plotFis(F)
% membership functions
f = fig_new(17,9);
tl = tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
pairs = {F.fis_inc,'input',1,'(a) e (incremental FIS)'; F.fis_inc,'input',2,'(b) \Deltae (incremental FIS)';
         F.fis_inc,'output',1,'(c) \Deltau (incremental FIS)'; F.fis_fpid,'input',1,'(d) e, \Deltae (self-tuning FIS)';
         F.fis_fpid,'output',1,'(e) \DeltaK_p, \DeltaK_i, \DeltaK_d'};
for k = 1:size(pairs,1)
    ax = nexttile(tl);
    [x,y] = plotmf(pairs{k,1},pairs{k,2},pairs{k,3});
    plot(ax,x,y,'LineWidth',1.1); ylim(ax,[0 1.3]); yticks(ax,[0 0.5 1]); xlim(ax,[-1 1]);
    mfs = pairs{k,1}.(ternary(strcmp(pairs{k,2},'input'),'Inputs','Outputs'))(pairs{k,3}).MembershipFunctions;
    for m = 1:numel(mfs)
        text(ax, mfs(m).Parameters(2), 1.0, mfs(m).Name,'HorizontalAlignment','center', ...
            'VerticalAlignment','bottom','FontSize',7.5);
    end
    title(ax,pairs{k,4}); xlabel(ax,'Normalised universe'); ylabel(ax,'\mu');
end
fig_save(f,'fig04_fis_membership', ['Membership functions of the incremental fuzzy controller (a-c) and ' ...
    'of the self-tuning Fuzzy-PID supervisor (d-e) used in the Simulation (normalised universes; ' ...
    'e: \pm20 \circC, \Deltae: \pm2 \circC per 2 s, \Deltau: \pm8 % per step).']);

% surfaces
f = fig_new(17,11);
tl = tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
opt = gensurfOptions('NumGridPoints',41);
items = {F.fis_inc,1,'(a) \Deltau, incremental fuzzy'; F.fis_fpid,1,'(b) \DeltaK_p, self-tuning';
         F.fis_fpid,2,'(c) \DeltaK_i, self-tuning'; F.fis_fpid,3,'(d) \DeltaK_d, self-tuning'};
for k = 1:4
    ax = nexttile(tl);
    opt.OutputIndex = items{k,2};
    [X,Y,Z] = gensurf(items{k,1},opt);
    surf(ax,X,Y,Z,'EdgeAlpha',0.25); view(ax,-37.5,30); colormap(f,parula);
    xlabel(ax,'e (norm.)'); ylabel(ax,'\Deltae (norm.)'); zlabel(ax,'output'); title(ax,items{k,3});
end
fig_save(f,'fig05_fis_surfaces', ['Control surfaces of the fuzzy inference systems used in the Simulation: ' ...
    'incremental fuzzy controller (a) and self-tuning gain supervisor (b-d).']);
end

function o = ternary(c,a,b), if c, o = a; else, o = b; end, end
