function [ P0 ] = initialize_covariance( simpar )
%initialize_covariance computes the initial covariance matrix
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
% [ output_args ] = initialize_covariance( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59
% Reference: 
% Copyright 2020 Utah State University

%Ininitalize to zero
P0 = zeros(simpar.states.nxfe, simpar.states.nxfe);
%Populate the covariance matrix
fnames = fieldnames(simpar.nav.ic);
for i=1:length(fnames)
    P0(i,i) = simpar.nav.ic.(fnames{i})^2;
end

end
