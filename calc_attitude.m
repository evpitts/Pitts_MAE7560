function [ T_i2b ] = calc_attitude(x, simpar)
% %TODO: Double check these calculations
u_zi_i = [0,0,1]';
v_i = x(simpar.states.ix.vel);
r_i = x(simpar.states.ix.pos);
u_v_i = v_i/norm(v_i);
u_yb_i = -u_v_i;

%Define the body z axis to be orthogonal to the orbital plane
assert(norm(cross(r_i,v_i)) > 1e-6,'Velocity is parallel or anti parallel with position!')
% fprintf('norm(cross(r_i,v_i)) = %d\n',norm(cross(r_i,v_i)));
u_h_i = -cross(r_i,v_i)/norm(cross(r_i,v_i));
u_zb_i = u_h_i;
%Define the body x axis via right hand rule
u_xb_i = cross(u_yb_i,u_zb_i);
%Compile the DCM from interial to body
T_i2b = [u_xb_i';u_yb_i';u_zb_i'];


% %Get the velocity vector
% u_v_u = x(simpar.states.ix.vel);
% u_v_u = u_v_u/norm(u_v_u);
% %We point the thruster in the opposite direction of the velocity vector
% u_yb_i = -u_v_u;
% u_xb_i = cross(u_yb_i, u_zi_i);
% u_xb_i = u_xb_i/norm(u_xb_i);
% T_i2b = [u_xb_i';u_yb_i';u_zi_i'];

assert(all(all(abs(T_i2b*T_i2b' - eye(3))<= 1e-12)),'T_i2b not orthonormal')

end