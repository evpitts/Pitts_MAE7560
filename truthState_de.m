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
% a_thrust = input.a_thrust;
% a_grav = input.a_grav;
omega_mi_m = [0;0;simpar.general.omega_moon];
tau_r = simpar.general.tau_r;
tau_a = simpar.general.tau_a;
d_g = simpar.general.d_g;
d_h = simpar.general.d_h;

%% Unpack x
r = x(simpar.states.ix.pos);
v = x(simpar.states.ix.vel);
att = x(simpar.states.ix.q_moon);
cam = x(simpar.states.ix.q_camera);
range_bias = x(simpar.states.ix.b_clock);
g_bias = x(simpar.states.ix.gbias);
height = x(simpar.states.ix.h_t);
accl_bias = x(simpar.states.ix.b_accl);

%% Compute individual elements of x_dot
r_dot = v;
a_grav = -simpar.general.mu/norm(r)^3*r;
v_dot = a_grav + input.a_thrust + g_bias + input.w_a;

q_im_dot = 1/2*qmult([0;omega_mi_m],att);
q_bc_dot = zeros(4,1);

b_r_dot = -1/tau_r*range_bias + input.w_r;

i_hat = r/norm(r);
v_perp = v - cross(omega_mi_m,r) - dot(v, i_hat)*i_hat;
gbias_dot = -norm(v_perp)/d_g*g_bias + input.w_g;

h_dot = norm(v_perp)/d_h*height + input.w_h;

b_ax_dot = -accl_bias/tau_a + input.w_accl;

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