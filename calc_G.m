function [ G ] = calc_G( xhat, simpar )
%calc_G Calculates the process noise dynamic coupling matrix
%
% Inputs:
%   xhat = state vector
%   simpar= simulation parameters
%
% Outputs
%   G = process noise dynamic coupling matrix
%
% Example Usage
% [ G ] = calc_G( xhat, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University

%% Unpack the inputs

%% Compute G
Ti2b = calc_attitude(xhat, simpar);
Tb2i = Ti2b';
n = simpar.states.nxfe;
G = zeros(n,11);
G(simpar.states.ix.vel, simpar.states.ix.pos) = -Tb2i;
G(7:14,4:11) = eye(8);
end
