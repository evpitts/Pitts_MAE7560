function loss_res = compute_residual(xhat, loss_ztilde, r_f_i, simpar, Ti2b)
%compute_residual_example calculates the measurement residual
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
% [ output_args ] = compute_residual_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:01:24
% Reference: 
% Copyright 2020 Utah State University

%Predict the measurement
loss_ztildehat = loss.pred_measurement(xhat,r_f_i,simpar, Ti2b);
%Get the prediction error
loss_res = loss_ztilde-loss_ztildehat;
end
