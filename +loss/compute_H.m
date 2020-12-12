function H = compute_H(x, xhat, simpar, Ti2b)
%compute_H_example calculates the measurement sensitivity matrix
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
% [ output_args ] = compute_H_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:04:33
% Reference: 
% Copyright 2020 Utah State University

%%Let z_tilde be our measurement. Then z_tilde = [lx/lz; ly/lz] + nu
                                            %% = h(l(x)) + nu
%%We find that del_z is equalt to H*del_x + nu, where H is the Jacobian of
%%h with respect to x. This Jacobian requires the chain rule, as the
%%relationship between h and x is implicit. Thus, del_z = (dh/dl *
%%dl/dx)*del_x + nu
%Get T_i_b and make it a quaternion
%T_i2b = calc_attitude(x, simpar);
qi2b = tmat2q(Ti2b);
%Get the position from the truth state (just for r_f_i)
r_b_i = x(simpar.states.ix.pos);
%Get the radius of the moon
R_moon = simpar.general.r_moon;
%Calculate r_f_i
[~,r_f_i,intersect,~] = gen_landmark(r_b_i, zeros(3,1), qi2b, R_moon, 'center', ...
                     simpar.general.q_b2c_nominal);

%Get r_b_i from the nav state
r_b_i_hat = xhat(simpar.states.ixfe.pos);

%Transform to a matrix
T_b_c = q2tmat(simpar.general.q_b2c_nominal);

%Calculate l_c
l_c = T_b_c*Ti2b*(r_f_i - r_b_i_hat);

%Get the partials
dh_dl = [1/l_c(3) 0 -l_c(1)/l_c(3)^2;...
         0 1/l_c(3) -l_c(2)/(l_c(3)^2)];
dl_dx = zeros(3,14);
dl_dx(1:3,1:3) = -T_b_c*Ti2b*eye(3,3);

%Calculate H
H = dh_dl*dl_dx;
end
