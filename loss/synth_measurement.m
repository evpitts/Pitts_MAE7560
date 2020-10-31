function [z_tilde,r_fi] = synth_measurement(x,simpar)
%Get the position
r_b_i = x(simpar.states.is.pos);
%Get the radius of the moon
R_moon = simpar.general.r_moon;

%Do the inertial to body transformation
Ti2b = calc_attitude(x,simpar);
qi2b = tmat2q(Ti2b);
%Do the inertial to moon transform
qi2m = x(simpar.states.ix.att);
Ti2m = q2tmat(qi2m);
%Do the body to camera transformation
Tb2c = q2tmat(simpar.general.q_b2c_nominal);
qb2c = x(simpar.states.ix.cam_att);

r_fi = [200e2; 1730e3; 100e3];
lc = Tb2c*Ti2b*(r_fi-r_bi);
lx = lc(1);
ly = lc(2);
lz = lc(3);

nu_c = 0;
z_tilde = [lx/lz; ly/lz] + nu_c;
end