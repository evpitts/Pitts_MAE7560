function [z_tilde,r_f_i] = synth_measurement(x,simpar, Ti2b)
%Get the position
r_b_i = x(simpar.states.ix.pos);
%Get the radius of the moon
R_moon = simpar.general.r_moon;

%Do the inertial to body transformation
%Ti2b = calc_attitude(x,simpar);
qi2b = tmat2q(Ti2b);
%Do the inertial to moon transformation
qi2m = x(simpar.states.ix.q_moon);
%Ti2m = q2tmat(qi2m);
%Do the body to camera transformation
% Tb2c = eye(3,3);
qb2c = x(simpar.states.ix.q_camera);
Tb2c = q2tmat(qb2c);
%qb2c = x(simpar.states.ix.q_camera);

% r_f_i = [200e2; 1730e3; 100e3];
[~, r_f_i, intersect, ~] = gen_landmark(r_b_i, zeros(3,1), qi2b, R_moon, 'center', ...
    qb2c);
assert(intersect, 'Camera did not intersect the moon!')
lc = Tb2c*Ti2b*(r_f_i-r_b_i);
lx = lc(1);
ly = lc(2);
lz = lc(3);

%We assume the noise is zero at this point - 11/03/2020
%This is the camera noise - might need to change it
%nu_c = sqrt(1/simpar.general.dt)*randn(2,1);
nu_c = simpar.truth.params.sig_los*randn(2,1);
z_tilde = [lx/lz; ly/lz] + nu_c;
end