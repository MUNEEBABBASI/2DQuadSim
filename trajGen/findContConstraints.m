% 7/30/13
% findContConstraints.m
% interate again through the desired position matrix
% construct constraints to ensure continuous connections between trajectories
%   where positions or dervatives aren't fixed
% these constraints are of the form x_j^(i)-x_j+1^(i) = 0, where i is any
%   derivative and j is any piece of the trajectory
%
% inputs:
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired polynominal trajectory
%   dim: current dimensions
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes,
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
%   t0: real value, begnning time of the trajectory
%   t1: real value, end time of the trajectory
%   tDes: m+1 x 1 matrix, desired arrival times at each trajectory
%   nonDim: 0 or 1, 1 uses nondimensionalized times t0 and t1
%       and scales conditions specified by posDes by times in tDes when calculaing b_eq, 
%       0 uses times in tDes times and doesn't scale endpoint conditions 
% outputs:
%   A_eq, b_eq: matrices for Ax=b formulation of constraints
function [A_eq, b_eq] = findContConstraints(r, n, m, dim, posDes, t0, t1, tDes, nonDim)

A_eq = [];
b_eq = [];


derCoeff = findDerivativeCoeff(n, r);

for j = 0:m, %for each keyframe
    for i = 0:r-1, %for all derivatives from 0 to r-1
        if posDes(i+1, j+1, dim) == Inf, %if unfixed
            % construct and add constraint
            A_temp = [];
            b_temp = []; 
            
            if (j > 0 && j < m) % if constraint is at an intermediate keyframe
                A_temp = zeros(1, (n+1)*m);
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
                    A_temp(1, j*(n+1)+k+1) = -tinit^(maxPower - k)*derCoeff(i+1, k+1);
                    
                    % if a derivative is nondimensionalized, scale the constraint
                    if nonDim,
                        A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1))^i)*A_temp(1, (j-1)*(n+1)+k+1);
                        A_temp(1, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1))^i)*A_temp(1, j*(n+1)+k+1);
                    end
                    
                end
                
                b_temp = 0;
            end
            
            A_eq = [A_eq; A_temp];
            b_eq = [b_eq; b_temp];
        end
    end
end


end