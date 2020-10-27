function [ x ] = truth2nav( x_t )
%truth2nav maps the truth state vector to the navigation state vector
%
% Inputs:
%   x_t = truth state (mixed units)
%
% Outputs
%   x = navigation state (mixed units)
%
% Example Usage
% [ xhat ] = truth2nav( x )

% Author: Randy Christensen
% Date: 21-May-2019 14:17:45
% Reference: 
% Copyright 2019 Utah State University
x = [eye(6) zeros(6,8) zeros(6,4) zeros(6,3) zeros(6,4);
     zeros(4,6) zeros(4,8) eye(4) zeros(4,3) zeros(4,4);
     zeros(4,6) zeros(4,8) zeros(4,4) zeros(4,3) eye(4)] * x_t;
end
