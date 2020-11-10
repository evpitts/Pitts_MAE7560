function xhatdot = navState_de(xhat,input)
%navState_de computes the derivative of the nav state
%
% Inputs:
%   xhat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   xhatdot = nav state derivative (mixed units)
%
% Example Usage
% xhatdot = navState_de(xhat,input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
a_tilde = input.ytilde;
% a_thrust = input.a_thrust;

T_i2b = calc_attitude(xhat, simpar);
T_b2i = T_i2b'; %Review why this is a transpose instead of an inverse
omega_mi_m = [0;0;simpar.general.omega_moon];

tau_r = simpar.general.tau_r;
tau_a = simpar.general.tau_a;
d_g = simpar.general.d_g;
d_h = simpar.general.d_h;

%% Unpack x
r_hat = xhat(simpar.states.ixf.pos);
v_hat = xhat(simpar.states.ixf.vel);
range_bias_hat = xhat(simpar.states.ixf.b_clock);
ghat_bias = xhat(simpar.states.ixf.gbias);
height_hat = xhat(simpar.states.ixf.h_t);
accl_bias_hat = xhat(simpar.states.ixf.b_accl);

%% Compute individual elements of x_dot
rhat_dot = v_hat;
a_grav = -simpar.general.MU/norm(r_hat)^3*r_hat;
vhat_dot = T_b2i*(a_tilde-accl_bias_hat)+a_grav+ghat_bias;

bhat_r_dot = -1/tau_r*range_bias_hat;

i_hat = r_hat/norm(r_hat);
v_perp_hat = v_hat-cross(omega_mi_m,r_hat)-dot(v_hat, i_hat)*i_hat;

ghat_bias_dot = -norm(v_perp_hat)/d_g*ghat_bias;

hhat_dot = -norm(v_perp_hat)/d_h*height_hat;

bhat_ax_dot = -accl_bias_hat/tau_a;
%% Assign to output
xhatdot = [rhat_dot;...
           vhat_dot;...
           bhat_r_dot;...
           ghat_bias_dot;...
           hhat_dot;...
           bhat_ax_dot];
end