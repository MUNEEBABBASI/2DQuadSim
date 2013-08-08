% 7/30/13
% findTraj.m
% generate optimal trajectory (in one dimension)
%   note that this implies no coridor constraints
% Dependencies: findContConstraints.m, findFixedConstraints.m,
%   findDerivativeCoeff.m, findCostMatrix.m
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


function [xT] = findTraj(r, n, m, dim, tDes, posDes)

% use nondimensionalized time
t0 = 0;
t1 = 1;

% we seek trajectories x1(t) = c1,n*t^n + c1,n-1*t^(n-1) + ... c1,0 ...
% xm(t) = cm,n*t^n + cm,n-1*t^(n-1) + ... cm,0
% x = [c1,(n) c1,(n-1) ... c1,1 c1,0 c2,n c2,(n-1) ... c2,1 c2,0 .... cm,0 ]




%%%
% construct cost matrix Q

Q_joint = zeros(m*(n+1));
for i = 1:m,
    Q = findCostMatrix(n, r, t0, t1); % find cost matrix for each segment
    
    Q_joint(2*r*(i-1)+1:2*r*i, (n+1)*(i-1)+1:(n+1)*i) = Q; %put in block diagonal matrix
end



%%%
% construct equality constraints 
[A_fixed, b_fixed] = findFixedConstraints(r, n, m, dim, posDes, t0, t1, [], 1);
[A_cont, b_cont] = findContConstraints(r, n, m, dim, posDes, t0, t1);

% put each A_eq for each dimension into block diagonal matrix
A_eq = [A_fixed; A_cont];
b_eq = [b_fixed; b_cont];




%%%
% find optimal trajectory through quadratic programming
xT_all = quadprog(Q_joint,[],[],[],A_eq,b_eq);



%%%
% explicitly break tracjetory into its piecewise parts for output
xT = zeros((n+1), m);
for j = 1:m,
    xT(:, j) = xT_all((j-1)*(n+1)+1:j*(n+1));
end


end








% % finds matrix for endpoint constraints, assuming a polynominal basis
% % inputs:
% %   n: integer, order of desired polynominal trajectory
% %   r: integer, derivative to minimize in cost function
% %   t0: real value, begnning time of the trajectory
% %   t1: real value, end time of the trajectory
% % outputs:
% %   A: r x (n+1) matrix, A matrix for constraint function Ax=b
% function [A] = findEndpointConstraintMatrix(n, r, t)
% 
% % find A matrix in constriants Ax=b, for:
% % [x1(t) x'1(t) .... x^(k-1)(t)]
% 
% % find coefficients of derivatives
% derCoeff = findDerivativeCoeff(n, r);
% 
% % substitute in t
% A = zeros(r, n+1);
% for i = 0:r-1, %conditions for derivatives 0 to k-1
%     maxPower = nnz(derCoeff(i+1, :))-1;
%     for j = 0:n, %for terms 0 to n
%         if (j <= maxPower),
%         A(i+1, j+1) = t^(maxPower - j)*derCoeff(i+1, j+1);
%         end
%     end
% end
% 
% end
