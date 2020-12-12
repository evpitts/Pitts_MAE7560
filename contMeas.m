function [ ytilde ] = contMeas(x, a_thr,  w_a, simpar, T_i2b)
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
b_a = x(simpar.states.ix.b_accl);

%% Calculate vehicle attitude assuming y = -n_v;
%T_i2b = calc_attitude( x, simpar );

%%Synthesized noise
eta_a = sqrt(simpar.truth.params.vrw^2/simpar.general.dt)*rand(3,1);

%% Compute accelorometer measurements
%MAYBE NEED TO CHANGE BACK TO ADDING T AND a_th
%The last term is the noise - we set it to zero
ytilde = T_i2b * (a_thr + w_a) + b_a + eta_a;
end
