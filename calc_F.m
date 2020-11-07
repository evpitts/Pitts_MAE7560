function [ Fhat ] = calc_F( xhat, x, simpar )
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
r_hat = xhat(simpar.states.ixf.pos);
v_hat = xhat(simpar.states.ixf.vel);
omega_mi_m = [0;0;simpar.general.omega_moon];
i_hat = r_hat/norm(r_hat);
v_perp_hat = v_hat-cross(omega_mi_m,r_hat)-dot(v_hat, i_hat)*i_hat;

tau_r = simpar.general.tau_r;
tau_a = simpar.general.tau_a;
d_g = simpar.general.d_g;
d_h = simpar.general.d_h;

%% Compute F_r
F_r = [zeros(3) eye(3) zeros(3,1) zeros(3) zeros(3,1) zeros(3)];

%% Compute F_v
F_v = [-simpar.general.MU/norm(r_hat)^3 * eye(3) - 3*simpar.general.MU*(i_hat*i_hat')/norm(r_hat)...
        zeros(3) zeros(3,1) eye(3) zeros(3,1) zeros(3)];

%% Compute F_br
F_br = [zeros(1,3) zeros(1,3) -1/tau_r zeros(1,3) zeros(1) zeros(1,3)];

%% Compute F_grav
F_grav = [zeros(3) zeros(3) zeros(3,1) -norm(v_perp_hat)/d_g * eye(3) zeros(3,1) zeros(3)];

%% Compute F_h
F_h = [zeros(1,3) zeros(1,3) zeros(1) zeros(1,3) -norm(v_perp_hat)/d_h zeros(1,3)];

%% Compute F_b
F_b = [zeros(3) zeros(3) zeros(3,1) zeros(3) zeros(3,1) -1/tau_a*eye(3)];

%% Compute Fhat
Fhat = [F_r; F_v; F_br; F_grav; F_h; F_b];
end