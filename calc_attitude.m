function [ T_i2b ] = calc_attitude(x, simpar)
u_zi_i = [0,0,1]';
%Get the velocity vector
u_v_u = x(simpar.states.ix.vel);
u_v_u = u_v_u/norm(u_v_u);
%We point the thruster in the opposite direction of the velocity vector
u_yb_i = -u_v_u;
u_xb_i = cross(u_yb_i, u_zi_i);
u_xb_i = u_xb_i/norm(u_xb_i);
T_i2b = [u_xb_i';u_yb_i';u_zi_i'];
% T_i2b = T_i2b(1:2,1:2);
end