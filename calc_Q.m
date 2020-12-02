function [ Q ] = calc_Q( v_perp, simpar )
%calc_Q Calculates the process noise power spectral density
%
% Inputs:
%   xhat = state vector
%   simpar= simulation parameters
%
% Outputs
%   Q = process noise dynamic coupling matrix
%
% Example Usage
% [ Q ] = calc_Q( xhat, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University
%% Assign Q
Q_a = 2*simpar.nav.params.Q_nongrav*eye(3);
Q_rbias = 2*simpar.nav.params.sig_rbias_ss^2/simpar.general.tau_r;
Q_abias = 2*simpar.nav.params.sig_abias_ss^2/simpar.general.tau_a*eye(3);
Q_g = 2*norm(v_perp)*simpar.nav.params.sig_grav_ss^2/simpar.general.d_g*eye(3);
Q_h = 2*norm(v_perp)*simpar.nav.params.sig_h_ss^2/simpar.general.d_h;

Q = blkdiag(Q_a, Q_rbias, Q_g, Q_h, Q_abias);
%Q = blkdiag(Q_rbias, Q_g, Q_g, Q_g, Q_h, Q_abias, Q_abias, Q_abias);
end
