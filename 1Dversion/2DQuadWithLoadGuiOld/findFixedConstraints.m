% 7/30/13
% findFixedConstraints.m
% iterate through the desired position matrix and add a constraint for each fixed value
% constraints are of the form Ax = b
% note that constraints on intermediate keyframes need to be added twice -
%       at t1 of the trajectory before it and t1 of the trajectory after it
%
% inputs:
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired polynominal trajectory
%   dim: current dimensions
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes,
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
%   t0: real value, begnning time of the trajectory
%   t1: real value, end time of the trajectory
%   tDes: m+1 x 1 matrix, desired arrival times at each trajectory
%   nonDim: 0 or 1, 1 uses nondimensionalized times t0 and t1, 0 uses times in tDes
% outputs:
%   A_eq, b_eq: matrices for Ax=b formulation of constraints
function [A_eq, b_eq] = findFixedConstraints(r, n, m, dim, posDes, t0, t1, tDes, nonDim)

A_eq = [];
b_eq = [];


derCoeff = findDerivativeCoeff(n, r);

for j = 0:m, %for each keyframe
    for i = 0:r-1, %for all derivatives from 0 to r-1
        if posDes(i+1, j+1, dim) ~= Inf, %if fixed
            % construct and add constraint
            
            if (j == 0), % add one constraint to beginning of first piece
                A_temp = zeros(1, (n+1)*m);
                maxPower = nnz(derCoeff(i+1, :))-1;
                for k = 0:maxPower,
                    
                    if nonDim,
                        tinit = t0;
                    else
                        tinit = tDes(j+1, 1);
                    end
                    
                    A_temp(1, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(i+1, k+1);
                end
                
                b_temp = posDes(i+1, j+1, dim);
            elseif (j == m), % add one constraint to end of last piece
                A_temp = zeros(1, (n+1)*m);
                maxPower = nnz(derCoeff(i+1, :))-1;
                for k = 0:maxPower,
                    
                    if nonDim,
                        tfin = t1;
                    else
                        tfin = tDes(j+1, 1);
                    end
                    
                    A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(i+1, k+1);
                end
                
                b_temp = posDes(i+1, j+1, dim);
            else % else, add two constraints
                A_temp = zeros(2, (n+1)*m);
                maxPower = nnz(derCoeff(i+1,:))-1;
                for k = 0:maxPower,
                    
                    if nonDim,
                        tinit = t0;
                        tfin = t1;
                    else
                        tinit = tDes(j+1, 1);
                        tfin = tDes(j+1, 1);
                    end
                    
                    A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(i+1, k+1);
                    A_temp(2, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(i+1, k+1);
                end
                
                b_temp = posDes(i+1, j+1, dim)*ones(2, 1);
            end
            
            A_eq = [A_eq; A_temp];
            b_eq = [b_eq; b_temp];
        end
    end
end

end