function [A,B] = plant_lin(P)
%PLANT_LIN  State-space form of the two-node lumped thermal model.
%   x = [T_a; T_w; T_m]   (air node, wall node, sensor reading before noise)
%   w = [u (0..1); Q_evap (W); 1]
%   dx/dt = A*x + B*w
%
%   C_a dT_a/dt = eta_h u P_max - G_air (T_a-T_amb) - H_aw (T_a-T_w) - Q_evap
%   C_w dT_w/dt = H_aw (T_a-T_w) - UA_w (T_w-T_amb)
%   tau_s dT_m/dt = T_a - T_m
Ca = P.C_a*P.C_scale;  Cw = P.C_w*P.C_scale;
G  = P.G_air*P.G_scale; H = P.H_aw; U = P.UA_w; ts = P.tau_s;
A = [-(G+H)/Ca,  H/Ca,        0;
      H/Cw,     -(H+U)/Cw,    0;
      1/ts,      0,          -1/ts];
B = [P.eta_h*P.P_max/Ca, -1/Ca, G*P.T_amb/Ca;
     0,                   0,    U*P.T_amb/Cw;
     0,                   0,    0];
end
