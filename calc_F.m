function [ Fhat ] = calc_F( xhat, Ti2b, simpar )
%calc_F computes the dynamics coupling matrix
%
% Inputs:
%   xhat = state vector
%   ytilde = continuous measurements
%   simpar = simulation parameters
%
% Outputs
%   Fhat = state dynamics matrix
%
% Example Usage
% [ Fhat ] = calc_F( xhat, ytilde, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University

%% Unpack the inputs
tau_r = simpar.general.tau_r;
tau_a = simpar.general.tau_a;
d_g = simpar.general.d_g;
d_h = simpar.general.d_h;
mu = simpar.general.mu;

r_hat = xhat(simpar.states.ixfe.pos);
v_hat = xhat(simpar.states.ixfe.vel);
omega_mi_m = [0;0;simpar.general.omega_moon];
ihat_r = r_hat/norm(r_hat);
v_perp_hat = v_hat-cross(omega_mi_m,r_hat)-dot(v_hat, ihat_r)*ihat_r;


%% Compute F_v
F_v = -mu/norm(r_hat)^3*(eye(3)-3*(ihat_r*ihat_r'));

%% Compute F_br
F_br = -1/tau_r;

%% Compute F_grav
F_grav = -norm(v_perp_hat)/d_g*eye(3);

%% Compute F_h
F_h = -norm(v_perp_hat)/d_h;

%% Compute F_b
F_ba = -1/tau_a*eye(3);

%% Compute Fhat
%F_r is all zeros
Fhat = zeros(simpar.states.nxfe,simpar.states.nxfe);
Fhat(simpar.states.ixfe.pos, simpar.states.ixfe.vel) = eye(3); 
Fhat(simpar.states.ixfe.vel, simpar.states.ixfe.pos) = F_v; 
Fhat(simpar.states.ixfe.vel, simpar.states.ixfe.gbias) = eye(3);
Fhat(simpar.states.ixfe.vel, simpar.states.ixfe.b_accl) = -Ti2b';
Fhat(simpar.states.ixfe.b_clock, simpar.states.ixfe.b_clock) = F_br;
Fhat(simpar.states.ixfe.gbias, simpar.states.ixfe.gbias) = F_grav;
Fhat(simpar.states.ixfe.h_t, simpar.states.ixfe.h_t) = F_h;
Fhat(simpar.states.ixfe.b_accl, simpar.states.ixfe.b_accl) = F_ba;
end