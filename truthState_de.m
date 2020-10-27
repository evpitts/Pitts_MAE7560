function xdot = truthState_de(x, input)
%truthState_de computes the derivative of the truth state
%
% Inputs:
%   x = truth state (mixed units)
%   u = input (mixed units)
%
% Outputs
%   xdot = truth state derivative (mixed units)
%
% Example Usage
% xdot = truthState_de( x, input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
omega = input.u;
w_a = input.w_a;
w_r = input.w_r;
w_d = input.w_d;
w_h = input.w_h;
w_ax = input.w_ax;
omega_bi_m = [0;0;1];
mu = 4.9048695*10^12;
tau_r = simpar.general.tau_r;
tau_ax = simpar.general.tau_ax;
d_g = simpar.general.d_g;
d_h = simpar>general.d_h;

%% Unpack x
r = x(simpar.states.ix.pos);
v = x(simpar.states.ix.vel);
att = x(simpar.states.ix.q_moon);
q_im = x(simpar.states.ix.q_camera);
range_bias = x(simpar.states.ix.b_clock);
g_bias = x(simpar.states.ix.gbias);
height = x(simpar.states.ix.h_t);
accl_bias = x(simpar.states.ix.b_accl);
%% Compute individual elements of x_dot
r_dot = v;
accel_grav = -mu/norm(r)^3 * r;
%%%%%Fix this
accel_thrust = guidance();
%%%%%
v_dot = accel_grav + accel_thrust + g_bias + w_a;

q_im_dot = 1/2*qmult([0;omega_bi_m],q_im);
q_bc_dot = zeros(4,1);

b_r_dot = -1/tau_r*range_bias + w_r;

i_hat = r/norm(r);
v_perp = v - omega_bi_m*r - (v * i_hat)*i_hat;
gbias_dot = -norm(v_perp)/d_g*g_bias + w_d;

h_dot = norm(v_perp)/d_h*height + w_h;

b_ax_dot = -1/tau_ax*accl_bias + w;

%% Assign to output
xdot = [r_dot;...
        v_dot;...
        q_im_dot;...
        q_bc_dot;...
        b_r_dot;...
        gbias_dot;...
        h_dot;...
        b_ax_dot];
end