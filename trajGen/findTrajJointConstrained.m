% 7/30/13
% findTrajJoint.m
% generate optimal trajectory for through a set of keyframes by solving a constrained
%   QP optimization problem analytically (as opposed to with quadprog)
% 	uses technique from "Polynominal Trajectory Planning for Quadrotor Flight", Richter et al.  
% Dependencies: findCostMatrix.m, findConstConstraints.m
%
% inputs: 
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   dim: integer, current dimension
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes, 
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
% outputs:
%   xT: (n+1) x m matrix, where row i contains the ith coefficient for the jth trajectory
%       xT is nondimensionalized in time



function [xT] = findTrajJointConstrained(r, n, m, dim, tDes, posDes)

% we want to find the optimal derivatives at each keyframe
% the original cost function is J = x'Qx
% where x = [c1,n c1,n-1 ... c1,1 c1,0 ... c2,n ... cm,n ... cm, 0]
% we are seeking the optimal position and derivative conditions at each
%   keyframe for each piece of the trajectory, so we define the new state 
%   d = [x1(t0) x'1(t0) ... x^(r-1)1(t0) ... x1(t1) x'1(t1) ... x^(r-1)1(t1) ...
%       x2(t1) x'2(t1) ... x^(r-1)2(t1) ... x2(t2) x'2(t2) ... x^(r-1)2(t2) ...
%       ...
%       xm(tm-1) x'm(tm-1) ... x^(r-1)m(tm-1) ... xm(tm) x'm(tm) ... x^(r-1)m(tm)]





%%%
% [2*Q A'; A 0]^(-1)*[0;b] to find [x;lambda], where lambda are lagrange
%       multipler constraint constants 
% to finds coefficients x = [c1,n c1,n-1 ... c1,1 c1,0 ... c2,n ... cm,n ... cm, 0]

% non-dimensionalize the problem to avoid large numbers in computations

t0 = 0;
t1 = 1;


Q_joint = [];
for i = 1:m,
    Q = findCostMatrix(n, r, t0, t1);
    
    % scale for time nondimensionalization
    Q = 1./((tDes(i+1, 1)-tDes(i, 1))^(2*r)).*Q;
    
    Q_joint = blkdiag(Q_joint, Q);
end


% note that here, since posDes is only one dimensional in k, dim is always
%   1 even though when the original posDes matrix is used, a dimension is
%   specified
% construct equality constraints 
[A_fixed, b_fixed] = findFixedConstraints(r, n, m, dim, posDes, t0, t1, tDes);
[A_cont, b_cont] = findContConstraints(r, n, m, dim, posDes, t0, t1, tDes);

% put each A_eq for each dimension into block diagonal matrix
A_eq = [A_fixed; A_cont];
b_eq = [b_fixed; b_cont];

[ARows, ~] = size(A_eq);

% note A_eq should have dimensions 2*n*m x (n+1)*m, since all derivatives are
%   constrained now 
% Q_joint has dimensions (n+1)*m x (n+1)*m
solution_temp = [2.*Q_joint A_eq'; A_eq zeros(ARows)]\[zeros((n+1)*m, 1); b_eq];

% x coefficients are the first (n+1)*m terms 
xT_all = solution_temp(1:(n+1)*m, 1);

xT_all'*Q_joint*xT_all




%%%
% explicitly break trajectory into its piecewise parts for output
xT = zeros((n+1), m);
for j = 1:m,
    xT(:, j) = xT_all((j-1)*(n+1)+1:j*(n+1));
end




end

