function make_figures(P, runs, mcT, mcTime, mcM, mcNames, s5, cases, s6f, s6t, oAvg, oPwm, allM, t_alarm)
%MAKE_FIGURES  All result figures (every caption states "Simulated").
St = ctrl_style(); nC = 3;

%% S1 start-up ---------------------------------------------------------------------
f = fig_new(17,11); tl = tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
sn = {'S1_120','S1_150'}; lab = {'(a)','(b)';'(c)','(d)'};
for j = 1:2
    ax = nexttile(tl,j); hold(ax,'on');
    for c = 1:nC, r = runs.(sn{j})(c); plot(ax,r.t/60,r.Ta,St.styles{c},'Color',St.colors(c,:)); end
    plot(ax,r.t/60,r.SP,':','Color',St.sp_color);
    ylabel(ax,'T_{air} (\circC)'); title(ax,sprintf('%s Start-up to %g \\circC',lab{1,j},r.SP(1)));
    if j==1, legend(ax,[St.names {'Set-point'}],'Location','southeast'); end
    ax = nexttile(tl,j+2); hold(ax,'on');
    for c = 1:nC, r = runs.(sn{j})(c); stairs(ax,r.t/60,r.u_safe,St.styles{c},'Color',St.colors(c,:)); end
    ylabel(ax,'Heater duty (%)'); xlabel(ax,'Time (min)'); ylim(ax,[-2 102]); title(ax,[lab{2,j} ' Heater duty']);
end
fig_save(f,'fig06_S1_startup',['Simulated start-up from ambient without spray (scenario S1): chamber air ' ...
    'temperature for set-points of 120 \circC (a) and 150 \circC (b) and the corresponding heater duty (c, d).']);

%% S2 spray disturbance -----------------------------------------------------------------
for sn = {'S2','S2_120'}
    f = fig_new(17,11); tl = tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
    rr = runs.(sn{1}); ts = rr(1).P.spray_t_on/60; sp = rr(1).SP(1);
    win = {[0 rr(1).P.t_stop/60], [ts-5 ts+60]}; ttl = {'(a) Full run','(b) Zoom on spray start'};
    for j = 1:2
        ax = nexttile(tl,j); hold(ax,'on');
        for c = 1:nC, plot(ax,rr(c).t/60,rr(c).Ta,St.styles{c},'Color',St.colors(c,:)); end
        plot(ax,rr(1).t/60,rr(1).SP,':','Color',St.sp_color);
        xline(ax,ts,'-','Spray on','Color',[.5 .5 .5],'LabelVerticalAlignment','bottom','FontSize',8);
        xlim(ax,win{j}); ylabel(ax,'T_{air} (\circC)'); title(ax,ttl{j});
        if j == 2, ylim(ax,[sp-15 sp+8]); yline(ax,sp+[-2 2],':','Color',[.6 .6 .6]); end
        if j == 1, legend(ax,[St.names {'Set-point'}],'Location','southeast'); end
        ax = nexttile(tl,j+2); hold(ax,'on');
        for c = 1:nC, stairs(ax,rr(c).t/60,rr(c).u_safe,St.styles{c},'Color',St.colors(c,:)); end
        xlim(ax,win{j}); ylim(ax,[-2 102]); ylabel(ax,'Heater duty (%)'); xlabel(ax,'Time (min)');
        title(ax,strrep(ttl{j},'(a)','(c)')); if j==2, title(ax,'(d) Heater duty, zoom'); end
    end
    if strcmp(sn{1},'S2')
        name = 'fig07_S2_spray'; cap = sprintf(['Simulated spray-start disturbance at a set-point of %g \\circC ' ...
            '(scenario S2): chamber air temperature (a, b) and heater duty (c, d); dotted lines in (b) mark the \\pm2 \\circC band.'], sp);
    else
        name = 'fig07b_S2_120'; cap = ['Simulated spray-start disturbance at a set-point of 120 \circC, ' ...
            'which equals the calibrated steady state at 100 % duty with spray: all controllers saturate.'];
    end
    fig_save(f,name,cap);
end

%% S3 set-point changes ----------------------------------------------------------------------
f = fig_new(17,10); tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
ax = nexttile(tl); hold(ax,'on'); rr = runs.S3;
for c = 1:nC, plot(ax,rr(c).t/60,rr(c).Ta,St.styles{c},'Color',St.colors(c,:)); end
stairs(ax,rr(1).t/60,rr(1).SP,':','Color',St.sp_color); ylabel(ax,'T_{air} (\circC)');
legend(ax,[St.names {'Set-point'}],'Location','southeast'); title(ax,'(a) Chamber air temperature');
ax = nexttile(tl); hold(ax,'on');
for c = 1:nC, stairs(ax,rr(c).t/60,rr(c).u_safe,St.styles{c},'Color',St.colors(c,:)); end
ylabel(ax,'Heater duty (%)'); xlabel(ax,'Time (min)'); ylim(ax,[-2 102]); title(ax,'(b) Heater duty');
fig_save(f,'fig08_S3_setpoint',['Simulated set-point tracking 100 \rightarrow 120 \rightarrow 140 \circC ' ...
    'without spray (scenario S3).']);

%% S4 robustness envelope ---------------------------------------------------------------------
f = fig_new(17,12); tl = tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
for c = 1:nC
    ax = nexttile(tl,c); hold(ax,'on'); X = mcT{c};
    lo = min(X,[],2); hi = max(X,[],2);
    fill(ax,[mcTime;flipud(mcTime)]/60,[lo;flipud(hi)],St.colors(c,:),'FaceAlpha',0.25,'EdgeColor','none');
    plot(ax,mcTime/60,mean(X,2),'-','Color',St.colors(c,:));
    plot(ax,mcTime/60,runs.S2(c).SP,':','Color',St.sp_color);
    xline(ax,runs.S2(c).P.spray_t_on/60,'-','Color',[.5 .5 .5]);
    ylim(ax,[20 P.sp_spray+25]); title(ax,sprintf('(%c) %s',char('a'+c-1),St.names{c}));
    xlabel(ax,'Time (min)'); if c==1, ylabel(ax,'T_{air} (\circC)'); end
    if c==1, legend(ax,{'Min-max envelope','Mean','Set-point'},'Location','southeast'); end
end
sel = [1 3 5]; yl = {'Overshoot (%)','Max. deviation after spray (\circC)','IAE (\circC s)'};
for k = 1:3
    ax = nexttile(tl,3+k);
    Y = squeeze(mcM(:,sel(k),:));
    boxchart(ax,Y,'MarkerStyle','.'); ax.XTickLabel = St.names; ylabel(ax,yl{k});
    title(ax,sprintf('(%c) %s',char('d'+k-1),strrep(mcNames{sel(k)},'_',' ')));
end
fig_save(f,'fig09_S4_robustness',sprintf(['Simulated Monte-Carlo robustness, %d runs per controller ' ...
    '(C \\pm20 %%, air flow \\pm20 %%, spray rate \\pm30 %%, extra delay 0-10 s): temperature envelopes (a-c) ' ...
    'and distributions of overshoot, spray-induced deviation and IAE (d-f).'], size(mcM,1)));

%% S5 noise / lag ------------------------------------------------------------------------------
f = fig_new(17,10); tl = tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
ts = s5(1,1).r.P.spray_t_on;
for k = [1 4]
    ax = nexttile(tl,(k==4)+1); hold(ax,'on');
    for c = 1:nC
        r = s5(c,k).r; w = r.t > ts-1800 & r.t < ts+1800;
        stairs(ax,r.t(w)/60,r.u_cmd(w),St.styles{c},'Color',St.colors(c,:));
    end
    ylim(ax,[0 100]); xlabel(ax,'Time (min)'); ylabel(ax,'Heater duty (%)');
    title(ax,sprintf('(%c) Duty, %s',char('a'+(k==4)),cases{k,1}));
    if k==1, legend(ax,St.names,'Location','northwest'); end
end
TV = zeros(4,nC); DV = zeros(4,nC);
for c = 1:nC, for k = 1:4
    r = s5(c,k).r; TV(k,c) = sum(abs(diff(r.u_cmd)));
    m2 = compute_metrics(r, ts, r.P.t_stop, 'disturbance'); DV(k,c) = m2.dip;
end, end
ax = nexttile(tl,3); b = bar(ax,TV); for c=1:nC, b(c).FaceColor = St.colors(c,:); end
ax.XTickLabel = strrep(cases(:,1),', ',newline); ylabel(ax,'\Sigma|\Deltau| (%)'); title(ax,'(c) Control effort variation');
ax = nexttile(tl,4); b = bar(ax,DV); for c=1:nC, b(c).FaceColor = St.colors(c,:); end
ax.XTickLabel = strrep(cases(:,1),', ',newline); ylabel(ax,'Max. deviation (\circC)'); title(ax,'(d) Spray-induced deviation');
legend(ax,St.names,'Location','northwest');
fig_save(f,'fig10_S5_noise_lag',['Simulated effect of RTD measurement noise and sensor lag (scenario S5): ' ...
    'heater duty around the spray start without (a) and with (b) noise and lag, total control-effort variation (c) ' ...
    'and maximum spray-induced deviation (d).']);

%% S6 safety --------------------------------------------------------------------------------------
plot_s6(P, s6f, s6t, t_alarm);

%% PWM ripple ---------------------------------------------------------------------------------------
f = fig_new(17,8); tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
w = [80 82]*60;
ax = nexttile(tl); hold(ax,'on');
k1 = oAvg.t>=w(1)&oAvg.t<=w(2); k2 = oPwm.t>=w(1)&oPwm.t<=w(2);
plot(ax,oAvg.t(k1)/60,oAvg.Ta(k1),'-','Color',St.colors(1,:));
plot(ax,oPwm.t(k2)/60,oPwm.Ta(k2),'-','Color',St.colors(2,:));
plot(ax,oPwm.t(k2)/60,oPwm.Tm(k2),'-','Color',St.colors(3,:));
ylabel(ax,'Temperature (\circC)'); legend(ax,{'T_{air}, duty-average SSR','T_{air}, time-proportioning PWM','T_{meas}, PWM'},'Location','best');
title(ax,'(a) Temperature ripple');
ax = nexttile(tl); hold(ax,'on');
stairs(ax,oPwm.t(k2)/60,oPwm.u_frac(k2)*100,'-','Color',St.colors(2,:));
stairs(ax,oAvg.t(k1)/60,oAvg.u_frac(k1)*100,'-','Color',St.colors(1,:),'LineWidth',1.5);
ylabel(ax,'Heater state / duty (%)'); xlabel(ax,'Time (min)'); ylim(ax,[-5 105]);
legend(ax,{'SSR output (PWM, 2 s window)','Duty average'},'Location','best'); title(ax,'(b) SSR output');
fig_save(f,'fig13_pwm_ripple',['Simulated comparison of the duty-average SSR model and the detailed 2 s ' ...
    'time-proportioning model (PID, set-point steady state).']);

%% Fuzzy-PID gain trajectories (S2) ------------------------------------------------------------------
f = fig_new(17,7); ax = axes(f); hold(ax,'on'); r = runs.S2(3);
plot(ax,r.t/60,r.Kp/P.Kp,'-'); plot(ax,r.t/60,r.Ki/P.Ki,'--');
if P.Kd > 0, plot(ax,r.t/60,r.Kd/P.Kd,'-.'); lg = {'K_p/K_{p0}','K_i/K_{i0}','K_d/K_{d0}'}; else, lg = {'K_p/K_{p0}','K_i/K_{i0}'}; end
xline(ax,r.P.spray_t_on/60,'-','Spray on','Color',[.5 .5 .5]);
ylim(ax,[0.4 1.6]); xlabel(ax,'Time (min)'); ylabel(ax,'Gain ratio (-)'); legend(ax,lg,'Location','best');
fig_save(f,'fig14_fpid_gains','Simulated evolution of the self-tuned PID gains relative to the baseline (scenario S2).');

%% Metrics bar chart -------------------------------------------------------------------------------------
f = fig_new(17,10); tl = tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
items = {'S1_120','OS','Overshoot S1-120 (%)'; 'S1_120','t_settle','Settling S1-120 (min)';
         'S1_120','IAE','IAE S1-120 (\circC s)'; 'S2_dist','dip','Max. dev. after spray (\circC)';
         'S2_dist','t_rec','Recovery after spray (min)'; 'S2_dist','E_kWh','Energy after spray (kWh)'};
for k = 1:6
    ax = nexttile(tl); v = arrayfun(@(m) m.(items{k,2}), allM.(items{k,1}));
    if strcmp(items{k,2},'t_settle') || strcmp(items{k,2},'t_rec'), v = v/60; end
    b = bar(ax,1:nC,v,'FaceColor','flat'); b.CData = St.colors;
    ax.XTick = 1:nC; ax.XTickLabel = St.names; title(ax,items{k,3});
end
fig_save(f,'fig12_metrics_bar','Simulated performance metrics of the three controllers (scenarios S1 at 120 \circC and S2).');
end
