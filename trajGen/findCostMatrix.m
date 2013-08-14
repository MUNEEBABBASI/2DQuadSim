% 7/30/13
% findCostMatrix.m
% finds the cost matrix Q, assuming a polynominal basis
% inputs:
%   n: integer, order of desired polynominal trajectory
%   r: integer, derivative to minimize in cost function
%   t0: real value, begnning time of the trajectory
%   t1: real value, end time of the trajectory
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
% outputs:
%   Q: (n+1) x (n+1) matrix, Q matrix for the cost function J = x'Qx
function [Q] = findCostMatrix(n, r, t0, t1)

Q = zeros(n+1);
tempTerms = ones(n+1, 1);

for i = 0:n,
    tempTerm = 1;
    for m = 0:(r-1)
        tempTerm = tempTerm*(i-m);
    end
    tempTerms(i+1, 1) = tempTerm;
end

for i = 0:n,
    for j = 0:n,
        if ((i >= r) && (j >= r)),
        Q(i+1, j+1) = tempTerms(i+1, 1)*tempTerms(j+1, 1)*(t1^(i+j-2*r+1) - t0^(i+j-2*r+1))/(i+j-2*r+1);
        end
    end
end

% flip Q so that it'll correspond to the order of x coefficients
Q = rot90(rot90(Q));


end