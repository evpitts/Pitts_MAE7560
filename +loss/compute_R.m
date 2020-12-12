function [R] = compute_R(x, H, simpar)

% T_i2b = calc_attitude(x,simpar);
% T_b2s = calc_T_b2s(simpar);

sig_los = simpar.nav.params.sig_los;
% 
% G_m = [H*T_i2b*T_b2s 1];
% %I think this should be the standard deviation of the measurement error?
% R_total = diag([sig_los^2, sig_los^2]);
% 
% R = G_m*R_total*G_m';
R = diag([sig_los^2, sig_los^2]);
end