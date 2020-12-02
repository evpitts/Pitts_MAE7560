function [ x ] = initialize_truth_state(simpar)
%initialize_truth_state initialize the truth state vector consistent with
%the initial covariance matrix
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
% [ output_args ] = initialize_truth_state( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59
% Reference: 
% Copyright 2020 Utah State University

% In the initialization of the truth and navigation states, you need only
% ensure that the estimation error is consistent with the initial
% covariance matrix.  One realistic way to do this is to set the true 
% vehicle states to the same thing every time, and randomize any sensor 
% parameters.

% disp(simpar.general.ic(1))
% for index = 1:length(simpar.general.ic)
%     x(index,1) = simpar.general.ic(index);
% end
% disp(x)
fnames = fieldnames(simpar.general.ic);
x = zeros(length(fnames),1);

for index=1:length(fnames)
    x(index) = simpar.general.ic.(fnames{index});
end

%Randomize every state except for the position and velocity states
x(simpar.states.ix.b_clock) = simpar.truth.ic.sig_br*randn;
x(simpar.states.ix.gbias(1)) = simpar.truth.ic.sig_epsx*randn;
x(simpar.states.ix.gbias(2)) = simpar.truth.ic.sig_epsy*randn;
x(simpar.states.ix.gbias(3)) = simpar.truth.ic.sig_epsz*randn;
x(simpar.states.ix.h_t) = simpar.truth.ic.sig_h*randn;
x(simpar.states.ix.b_accl(1)) = simpar.truth.ic.sig_ax*randn;
x(simpar.states.ix.b_accl(2)) = simpar.truth.ic.sig_ay*randn;
x(simpar.states.ix.b_accl(3)) = simpar.truth.ic.sig_az*randn;
% names = fieldnames(simpar.general.ic);
% x = zeros(length(names),1);
% 
% %Get all of the initial conditions
% for i=1:length(names)
%     x(i) = simpar.general.ic.(names{i});
% end
% x(simpar.states.ix.q_camera) = simpar.general.q_b2c_nominal;
% 
end
