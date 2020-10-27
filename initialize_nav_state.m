function [ xhat ] = initialize_nav_state( x, P, simpar)
%initialize_nav_state initializes the navigation state vector consistent
%with the initial covariance matrix
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
% [ output_args ] = initialize_nav_state( input_args )

% Author: 
% Date: 31-Aug-2020 15:46:59
% Reference: 
% Copyright 2020 Utah State University

% Consistent with the truth state initialization, you should randomize the
% vehicle states, and initialize any sensor parameters to zero.  An example
% of these calculations are shown below.
xhat = truth2nav(x);
end
