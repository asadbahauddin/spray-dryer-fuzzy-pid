function plot_s6(P, s6f, s6t, t_alarm)
%PLOT_S6  Safety-scenario figure (SSR short-circuit, highest firmware set-point).
St = ctrl_style();
f = fig_new(17,7.5); tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
ax = nexttile(tl); hold(ax,'on');
h1 = plot(ax,s6f.t/60,s6f.Ta,'-','Color',St.colors(1,:));
h2 = plot(ax,s6f.t/60,s6f.SP,':','Color','k');
h3 = yline(ax,P.T_trip,'--r');
h4 = xline(ax,s6f.P.ssr_fault_t/60,'-','Color',[.5 .5 .5],'LineWidth',1.2);
hs = [h1 h2 h3 h4]; lg = {'T_{air}','Set-point','200 \circC limit','SSR short-circuit'};
if ~isnan(t_alarm)
    h5 = xline(ax,t_alarm/60,'-.','Color',[.85 .33 .1],'LineWidth',1.2);
    hs(end+1) = h5; lg{end+1} = sprintf('Firmware alarm (+%.1f min)',(t_alarm-s6f.P.ssr_fault_t)/60);
end
ylim(ax,[0 215]); xlabel(ax,'Time (min)'); ylabel(ax,'T_{air} (\circC)');
legend(ax,hs,lg,'Location','southeast');
title(ax,'(a) SSR short-circuit fault (PID, no spray)');
ax = nexttile(tl); hold(ax,'on');
yyaxis(ax,'left'); plot(ax,s6t.t/60,s6t.Ta,'-'); ylabel(ax,'T_{air} (\circC)'); ylim(ax,[0 215]);
yline(ax,P.T_trip,'--r','200 \circC limit','LabelHorizontalAlignment','right','LabelVerticalAlignment','top');
yyaxis(ax,'right'); stairs(ax,s6t.t/60,s6t.u_safe,'-'); ylabel(ax,'Heater duty (%)'); ylim(ax,[0 125]);
xlabel(ax,'Time (min)'); title(ax,['(b) Highest firmware set-point, ' num2str(P.sp_max) ' \circC (PID)']);
fig_save(f,'fig11_S6_safety',['Simulated safety scenarios (S6): SSR short-circuit fault at 60 min, where the firmware ' ...
    'cannot switch the heater off and only the SSR-stuck alarm and the E-stop remain (a); highest set-point accepted by the firmware, 180 \circC (b).']);
end
