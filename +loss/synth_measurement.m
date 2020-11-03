function [z_tilde,r_f_i] = synth_measurement(x,simpar)
%Get the position
r_b_i = x(simpar.states.ix.pos);
%Get the radius of the moon
R_moon = simpar.general.R_M;

%Do the inertial to body transformation
Ti2b = calc_attitude(x,simpar);
qi2b = tmat2q(Ti2b);
%Do the inertial to moon transformation
qi2m = x(simpar.states.ix.q_moon);
Ti2m = q2tmat(qi2m);
%Do the body to camera transformation
% Tb2c = eye(3,3);
Tb2c = q2tmat(simpar.general.q_b2c_nominal);
%qb2c = x(simpar.states.ix.q_camera);

%TODO: replace the following line of code with a call to gen_landmark
% r_f_i = [200e2; 1730e3; 100e3];
r_f_i = gen_landmark(r_b_i, zeros(3,1), qi2b, R_moon, 'center', ...
    simpar.general.q_b2c_nominal);
lc = Tb2c*Ti2b*(r_f_i-r_b_i);
lx = lc(1);
ly = lc(2);
lz = lc(3);

%We assume the noise is zero at this point - 11/03/2020
nu_c = 0;
z_tilde = [lx/lz; ly/lz] + nu_c;
end