function [ xhat ] = initialize_nav_state( x, simpar)
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
%Inject errors into the position and velocity states 
%(all other states were randomized when the truth state was initialized)
xhat(simpar.states.ix.pos(1)) = xhat(simpar.states.ix.pos(1))+simpar.truth.ic.sig_rsx*randn;
xhat(simpar.states.ix.pos(2)) = xhat(simpar.states.ix.pos(2))+simpar.truth.ic.sig_rsy*randn;
xhat(simpar.states.ix.pos(3)) = xhat(simpar.states.ix.pos(3))+simpar.truth.ic.sig_rsz*randn;

xhat(simpar.states.ix.vel(1)) = xhat(simpar.states.ix.vel(1))+simpar.truth.ic.sig_vsx*randn;
xhat(simpar.states.ix.vel(2)) = xhat(simpar.states.ix.vel(2))+simpar.truth.ic.sig_vsy*randn;
xhat(simpar.states.ix.vel(3)) = xhat(simpar.states.ix.vel(3))+simpar.truth.ic.sig_vsz*randn;

xhat(simpar.states.ixf.parameter) = 0;
end
