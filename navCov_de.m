 function [ P_dot ] = navCov_de( P, input )
%navCov_de computes the derivative of the nav state covariance
%
% Inputs:
%   Phat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   Phat_dot = nav state derivative (mixed units)
%
% Example Usage
% [ Phat_dot ] = navCov_de( Phat, input )

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%Unpack the inputs for clarity
xhat = input.xhat;
simpar = input.simpar;
%ytilde = input.ytilde;
v_perp = input.v_perp;

%Compute state dynamics matrix
F = calc_F(xhat, input.Ti2b, simpar);

%Compute process noise coupling matrix
B = calc_G(simpar, input.Ti2b);

%Compute process noise PSD matrix
Q = calc_Q(v_perp, simpar);

%Compute Phat_dot - We have determined that S=0
P_dot = F*P+P*F'+B*Q*B';%+F_eta*S_eta*F_eta'

end
