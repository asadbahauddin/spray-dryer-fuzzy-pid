function S = ctrl_style()
%CTRL_STYLE  Consistent names and colours for the three controllers.
S.names  = {'PID','Fuzzy','Fuzzy-PID'};
S.colors = [0.000 0.447 0.741;    % PID       blue
            0.850 0.325 0.098;    % Fuzzy     orange
            0.466 0.674 0.188];   % Fuzzy-PID green
S.styles = {'-','--','-.'};
S.sp_color = [0 0 0];
S.ol_color = [0.45 0.45 0.45];
end
