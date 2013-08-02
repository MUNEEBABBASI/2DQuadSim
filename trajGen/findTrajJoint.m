% 7/30/13
% findTrajJoint.m
% generate optimal trajectory for through a set of keyframes by optimizing
%   derivatives at endpoints, for one dimension
% 	uses technique from"Polynominal Trajectory Planning for Quadrotor Flight", Richter et al.  
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



function [xT] = findTrajJoint(r, n, m, dim, tDes, posDes)

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
% construct Q_joint by making a block diagonal matrix from Q's of
%   each piece of the trajectory
%   note that we are not nondimensionalizing the time here
Q_joint = [];
for i = 1:m,
    Q = findCostMatrix(n, r, tDes(i, 1), tDes(i+1, 1));
      
    Q_joint = blkdiag(Q_joint, Q);
end


%%%
% construct A_joint
% here, d = Ax maps the x state vector to the endpoint derivative state vector d

A_joint = [];
for i = 1:m,
    
    % find coefficients of derivatives
    derCoeff = findDerivativeCoeff(n, r-1);

    % substitute in the time values
    A = zeros(2*r, n+1);
    
    for j = 0:r-1,
        maxPower = nnz(derCoeff(j+1, :))-1;
    
        for k = 0:maxPower,
                tinit = tDes(i, 1);
                tfinal = tDes(i+1, 1);
                
                A(j+1, k+1) = tinit^(maxPower-k)*derCoeff(j+1, k+1);
                A(j+1+r, k+1) = tfinal^(maxPower-k)*derCoeff(j+1, k+1);
        end
    end
    
    A_joint = blkdiag(A_joint, A);
end



%%%
% construct D_F and M
% D_F = [x1(t0) x1(t1) x2(t2) t3(t3) ... x_[m](t_[m]) ...
%       x1'(t0) x1'(t1) x2'(t2) t3'(t3) ... x_[m]'(t_[m]) ...
%       x1^(r-1) (t0) x1^(r-1)(t1) x2^(r-1)(t2) ... x_[m]^(r-1)(t_[m])]
% fixed values from these values

Dnum = 0;
D_F = [];
M = zeros(2*r*m, r*(m+1));

for i = 0:r-1,
    for j = 0:m,
        if (posDes(i+1, j+1) ~= Inf)
            D_F = [D_F; posDes(i+1, j+1)]; %add this constraint to D_F
            
            % add the appropriate M rows 
            if (j == 0),
                % if first row, add one condition
                M(i+1, :) = [zeros(1, Dnum) 1 zeros(1, r*(m+1)-Dnum-1)];
            elseif (j == m),
                % if last row, add one condition
                M((m-1)*2*r+r+i+1, :) = [zeros(1, Dnum) 1 zeros(1, r*(m+1)-Dnum-1)];
            else
                % if intermediate row, add two conditions to reflect continuity
                M((j-1)*2*r+r+i+1, :) = [zeros(1, Dnum) 1 zeros(1, r*(m+1)-Dnum-1)];
                M(j*2*r+i+1, :) = [zeros(1, Dnum) 1 zeros(1, r*(m+1)-Dnum-1)];
            end
            
            Dnum = Dnum + 1;
        end
    end
end

% construct M values for D_P, where 
% D_P = [x1(t0) x1(t1) x2(t2) t3(t3) ... x_[m](t_[m]) ...
%       x1'(t0) x1'(t1) x2'(t2) t3'(t3) ... x_[m]'(t_[m]) ...
%       x1^(r-1) (t0) x1^(r-1)(t1) x2^(r-1)(t2) ... x_[m]^(r-1)(t_[m])]
% unfixed values from these values

for i = 0:r-1,
    for j = 0:m,
        if (posDes(i+1, j+1) == Inf)
            
            % add the appropriate M rows 
            if (j == 0),
                % if first row, add one condition
                M(i+1, :) = [zeros(1, Dnum) 1 zeros(1, r*(m+1)-Dnum-1)];
            elseif (j == m),
                % if last row, add one condition
                M((m-1)*2*r+r+i+1, :) = [zeros(1, Dnum) 1 zeros(1, r*(m+1)-Dnum-1)];
            else
                % if intermediate row, add two conditions to reflect continuity
                M((j-1)*2*r+r+i+1, :) = [zeros(1, Dnum) 1 zeros(1, r*(m+1)-Dnum-1)];
                M(j*2*r+i+1, :) = [zeros(1, Dnum) 1 zeros(1, r*(m+1)-Dnum-1)];
            end
            
            Dnum = Dnum + 1;
        end
    end
end




%%%
% find optimal keyframe derivatives

% find R = M' inv(A)' Q inv(A) M
R = M'*inv(A_joint)'*Q_joint*inv(A_joint)*M;


% partition R matrix
numFree = length(D_F);
R_PP = R(numFree+1:(m+1)*r, numFree+1:(m+1)*r);
R_FP = R(1:numFree, numFree+1:(m+1)*r);



% find D_opt
D_opt = -inv(R_PP)'*R_FP'*D_F;





%%%
% find the minimum order polynominals with the optimal boundary conditions
% found using matrix inverses

% use D_opt to fill in missing boundary conditions
posDes_opt = zeros(r, m+1);
Dnum = 1;
for i = 1:r,
    for j = 1:m+1,
        if (posDes(i, j, dim) ~= Inf),
            posDes_opt(i, j) = posDes(i, j, dim);
        else
            posDes_opt(i, j) = D_opt(Dnum, 1);
            Dnum = Dnum+1;
        end
    end
end





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
      
    Q_joint = blkdiag(Q_joint, Q);
end


% note that here, since posDes is only one dimensional in k, dim is always
%   1 even though when the original posDes matrix is used, a dimension is
%   specified
[A_eq, b_eq] = findFixedConstraints(r, n, m, 1, posDes_opt, t0, t1, [], 1);


% note A_eq should have dimensions 2*n*m x (n+1)*m, since all derivatives are
%   constrained now 
% Q_joint has dimensions (n+1)*m x (n+1)*m
solution_temp = inv([2.*Q_joint A_eq'; A_eq zeros(2*r*m)]) * [zeros((n+1)*m, 1); b_eq];

% x coefficients are the first (n+1)*m terms 
xT_all = solution_temp(1:(n+1)*m, 1);



%%%
% explicitly break trajectory into its piecewise parts for output
xT = zeros((n+1), m);
for j = 1:m,
    xT(:, j) = xT_all((j-1)*(n+1)+1:j*(n+1));
end





end

