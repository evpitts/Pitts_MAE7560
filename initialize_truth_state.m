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

% filename = 'config.xlsx';
% x = xlsread(filename,'initialConditions','B1:B26');

% disp(simpar.general.ic(1))
% for index = 1:length(simpar.general.ic)
%     x(index,1) = simpar.general.ic(index);
% end
% disp(x)

x = zeros(simpar.states.nx,1);
%Get the position
x(simpar.states.ix.pos) = [simpar.general.ic.ri_x,...
    simpar.general.ic.ri_y,simpar.general.ic.ri_z]';
%Get the velocity
x(simpar.states.ix.vel) = [simpar.general.ic.vi_x,...
    simpar.general.ic.vi_y,simpar.general.ic.vi_z]';
%Get the moon quaternion
x(simpar.states.ix.q_moon) = [simpar.general.ic.qm_i_a,...
    simpar.general.ic.qm_i_i,simpar.general.ic.qm_i_j,...
    simpar.general.ic.qm_i_k]';
%Get the camera quaternion
x(simpar.states.ix.q_camera) = [simpar.general.ic.qc_b_a,...
    simpar.general.ic.qc_b_i,simpar.general.ic.qc_b_j,...
    simpar.general.ic.qc_b_k]';
%Get the clock bias
x(simpar.states.ix.b_clock) = [simpar.general.ic.b_r];
%Get the gravitational bias
x(simpar.states.ix.gbias) = [simpar.general.ic.epsilong_x,...
    simpar.general.ic.epsilong_y,simpar.general.ic.epsilong_z];
%Get the feature position
x(simpar.states.ix.pos_f) = [simpar.general.ic.rf_x,...
    simpar.general.ic.rf_y,simpar.general.ic.rf_z];
%Get the terrain height
x(simpar.states.ix.h_t) = [simpar.general.ic.h_t];
%Get the acceleromter bias
x(simpar.states.ix.b_accl) = [simpar.general.ic.ba_x,...
    simpar.general.ic.ba_y,simpar.general.ic.ba_z];
%Get all of the vehicle ics
x(simpar.states.ix.vehicle) = [simpar.general.ic.ri_x,...
    simpar.general.ic.ri_y,simpar.general.ic.ri_z,...
    simpar.general.ic.vi_x,simpar.general.ic.vi_y,...
    simpar.general.ic.vi_z];
%Get all of the parameter ics
x(simpar.states.ix.parameter) = [simpar.general.ic.qm_i_a,...
    simpar.general.ic.qm_i_i,simpar.general.ic.qm_i_j,...
    simpar.general.ic.qm_i_k,simpar.general.ic.qc_b_a,...
    simpar.general.ic.qc_b_i,simpar.general.ic.qc_b_j,...
    simpar.general.ic.qc_b_k,simpar.general.ic.qc_b_k,...
    simpar.general.ic.b_r,simpar.general.ic.epsilong_x,...
    simpar.general.ic.epsilong_y,simpar.general.ic.epsilong_z,...
    simpar.general.ic.rf_x,simpar.general.ic.rf_y,...
    simpar.general.ic.rf_z,simpar.general.ic.ba_x,...
    simpar.general.ic.ba_y,simpar.general.ic.ba_z];
end
