function [cov_residual] = compute_covariance_residual(H, P, R)

%G is the measurement noise coupling matrix, which is identity in our
%problem
G = eye(2);
% See pg 
cov_residual = H*P*H'+G*R*G';