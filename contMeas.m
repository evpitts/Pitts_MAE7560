function [ ytilde ] = contMeas(x, a_thr,  w_a, simpar)
%contInertialMeas synthesizes noise measurements used to propagate the
%navigation state
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
% Example Usage
% [ output_args ] = contInertialMeas( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59
% Reference: 
% Copyright 2020 Utah State University

%% Prelim
b_a = x(simpar.states.ix.abias);

%% Calculate vehicle attitude assuming y = -n_v;
T_i2b = calc_attitude( x, simpar );

%% Compute accelorometer measurements
ytilde = T_i2b + (a_thr + w_a) + b_a + eta_a;
end