% 7/30/13
% findTraj.m
% generate optimal trajectory (in one dimension)
% Dependencies: -
%
% inputs: 
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
%   posDes: r x m matrix, desired positions and/or derivatives at keyframes, 
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j 
% outputs:
%   xT: (n+1) x m matrix, where row i contains the ith coefficient for the jth trajectory
%       xT is nondimensionalized in time


function [xT] = findTraj(r, n, m, tDes, posDes)

% use nondimensionalized time
t0 = 0;
t1 = 1;

% we seek trajectories x1(t) = c1,n*t^n + c1,n-1*t^(n-1) + ... c1,0 ...
% xm(t) = cm,n*t^n + cm,n-1*t^(n-1) + ... cm,0
% x = [c1,(n) c1,(n-1) ... c1,1 c1,0 c2,n c2,(n-1) ... c2,1 c2,0 .... cm,0 ]




%%%
% construct cost matrix Q

% find cost matrix for each segment
Q_joint = zeros(m*(n+1));
for i = 1:m,
    Q = findCostMatrix(n, r, t0, t1);
    
    Q_joint(2*r*(i-1)+1:2*r*i, (n+1)*(i-1)+1:(n+1)*i) = Q;
end



%%%
% construct equality constraints 

% find coefficients of derivatives of the polynomial
derCoeff = findDerivativeCoeff(n, r);

A_eq = [];
b_eq = [];


% iterate through the desired position matrix and add a constraint for each fixed value
% constraints are of the form Ax = b
% note that constraints on intermediate keyframes need to be added twice -
%       at t1 of the trajectory before it and t1 of the trajectory after it
for j = 0:m, %for each keyframe 
    for i = 0:r-1, %for all derivatives from 0 to r-1
        if posDes(i+1, j+1) ~= Inf, %if fixed
            % construct and add constraint
            
            if (j == 0), % add one constraint to beginning of first piece
                A_temp = zeros(1, (n+1)*m);
                maxPower = nnz(derCoeff(i+1, :))-1;
                for k = 0:maxPower,
                    A_temp(1, j*(n+1)+k+1) = t0^(maxPower - k)*derCoeff(i+1, k+1);
                end
                
                b_temp = posDes(i+1, j+1);
            elseif (j == m), % add one constraint to end of last piece
                A_temp = zeros(1, (n+1)*m);
                maxPower = nnz(derCoeff(i+1, :))-1;
                for k = 0:maxPower,
                    A_temp(1, (j-1)*(n+1)+k+1) = t1^(maxPower - k)*derCoeff(i+1, k+1);
                end
                
                b_temp = posDes(i+1, j+1);
            else % else, add two constraints
                A_temp = zeros(2, (n+1)*m);
                maxPower = nnz(derCoeff(i+1,:))-1;
                for k = 0:maxPower,
                    A_temp(1, (j-1)*(n+1)+k+1) = t1^(maxPower - k)*derCoeff(i+1, k+1); 
                    A_temp(2, j*(n+1)+k+1) = t0^(maxPower - k)*derCoeff(i+1, k+1); 
                end      
                
                b_temp = posDes(i+1, j+1)*ones(2, 1);
            end
            
            A_eq = [A_eq; A_temp];
            b_eq = [b_eq; b_temp];
        end
    end
end


% interate again through the desired position matrix
% construct constraints to ensure continuous connections between trajectories
%   where positions or dervatives aren't fixed
% these constraints are of the form x_j^(i)-x_j+1^(i) = 0, where i is any
%   derivative and j is any piece of the trajectory
for j = 0:m, %for each keyframe 
    for i = 0:r-1, %for all derivatives from 0 to r-1
        if posDes(i+1, j+1) == Inf, %if unfixed
            % construct and add constraint
           
            if (j > 0 && j < m) % if constraint is at an intermediate keyframe
                A_temp = zeros(1, (n+1)*m);
                maxPower = nnz(derCoeff(i+1,:))-1;
                for k = 0:maxPower,
                    A_temp(1, (j-1)*(n+1)+k+1) = t1^(maxPower - k)*derCoeff(i+1, k+1); 
                    A_temp(1, j*(n+1)+k+1) = -t0^(maxPower - k)*derCoeff(i+1, k+1); 
                end      
                
                b_temp = 0;
            end
            
            A_eq = [A_eq; A_temp];
            b_eq = [b_eq; b_temp];
        end
    end
end



%%%
% construct any inequality constraints




%%%
% find optimal trajectory through quadratic programming
xT_all = quadprog(Q_joint,[],[],[],A_eq,b_eq);



%%%
% explicitly break trajetory into its piecewise parts for output
xT = zeros((n+1), m);
for j = 1:m,
    xT(:, j) = xT_all((j-1)*(n+1)+1:j*(n+1));
end


end




% finds the cost matrix Q, assuming a polynominal basis
% inputs:
%   n: integer, order of desired polynominal trajectory
%   k: integer, derivative to minimize in cost function
%   t0: real value, begnning time of the trajectory
%   t1: real value, end time of the trajectory
% outputs:
%   Q: (n+1) x (n+1) matrix, Q matrix for the cost function J = x'Qx
function [Q] = findCostMatrix(n, k, t0, t1)

Q = zeros(n+1);
tempTerms = ones(n+1, 1);

for i = 0:n,
    tempTerm = 1;
    for m = 0:(k-1)
        tempTerm = tempTerm*(i-m);
    end
    tempTerms(i+1, 1) = tempTerm;
end

for i = 0:n,
    for j = 0:n,
        if ((i >= k) && (j >= k)),
        Q(i+1, j+1) = tempTerms(i+1, 1)*tempTerms(j+1, 1)*(t1^(i+j-2*k+1) - t0^(i+j-2*k+1))/(i+j-2*k+1);
        end
    end
end

% flip Q so that it'll correspond to the order of x coefficients
Q = rot90(rot90(Q));

end



% finds coefficients for up to the kth derivative of polynominals of order n
% inputs:
%   n: integer, order of desired polynominal trajectory
%   r: integer, derivative to minimize in cost function
% outputs:
%   derCoeff: (r+1) x (n+1) matrix, row i contains coefficients for the
%       (i-1)th derivative of a polynominal of order n, assuming all
%       coefficients are 1
function derCoeff = findDerivativeCoeff(n, r)

derCoeff = zeros(r+1, n+1);
tempC = ones(1, n+1); % constant
derCoeff(1, :) = tempC;
for i = 1:r, %for derivatives 1 to k
    tempC = polyder(tempC);
    derCoeff(i+1, :) = [tempC zeros(1, n+1-length(tempC))];
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
