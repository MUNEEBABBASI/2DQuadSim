% 7/30/13
% findTraj.m
% generate optimal trajectory in many dimensions
%   this allows for specification of corridor constraints
% Dependencies: -
%
% inputs: 
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   d: integer, number of dimensions
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes, 
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
%   ineqConst: structure of sx1 arrays, for s constraints, with elements:
%       numConst: integer, number of constraints
%       start: sx1 matrix, keyframes where constraints begin
%       delta: sx1 matrix, maximum distance 
%       nc: sx1 matrix, number of intermediate points
%       dim: sxd matrix, the d dimensions that the constraint applies to
% outputs:
%   xT: (n+1) x m xd matrix, where row i contains the ith coefficient for
%       the jth trajectory in dimension k
%       xT is nondimensionalized in time


function [xT] = findTrajCorr(r, n, m, d, tDes, posDes, ineqConst)

% use nondimensionalized time
t0 = 0;
t1 = 1;

% we seek trajectories 
% x1(t) = cx1,n*t^n + cx1,n-1*t^(n-1) + ... cx1,0; 
% ...
% xm(t) = cxm,n*t^n + cxm,n-1*t^(n-1) + ... cxm,0; 
% ...
% y1(t) = cy1,n*t^n + cy1,n-1*t^(n-1) + ... cy1,0; 
% ... 
% z1(t) = cz1,n*t^n + cz1,n-1*t^(n-1) + ... cz1,0; 
% ... for d dimensions
% form the state vector x as: 
% x = [cx1,(n) cx1,(n-1) ... cx1,1 cx1,0 cx2,n cx2,(n-1) ... cx2,1 cx2,0 .... cxm,0 ....
%       cy1,(n) cy1,(n-1) ... cy1,1 cy1,0 cy2,n cy2,(n-1) ... cy2,1 cy2,0 .... cym,0 ....
%       cz1,(n) cz1,(n-1) ... cz1,1 cz1,0 cz2,n cz2,(n-1) ... cz2,1 cz2,0 .... czm,0]



%%%
% construct cost matrix Q


Q_opt = [];

for dim = 1:d,

Q_joint = zeros(m*(n+1));
for i = 1:m,
    Q = findCostMatrix(n, r, t0, t1); % find cost matrix for each segment
    
    Q_joint(2*r*(i-1)+1:2*r*i, (n+1)*(i-1)+1:(n+1)*i) = Q; %put in block diagonal matrix
end

% put each dimension's Q_joint into a block diagonal matrix
Q_opt = blkdiag(Q_opt, Q_joint);
end


%%%
% construct equality constraints 

A_opt = [];
b_opt = [];

for dim = 1:d,
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
        if posDes(i+1, j+1, dim) ~= Inf, %if fixed
            % construct and add constraint
            
            if (j == 0), % add one constraint to beginning of first piece
                A_temp = zeros(1, (n+1)*m);
                maxPower = nnz(derCoeff(i+1, :))-1;
                for k = 0:maxPower,
                    A_temp(1, j*(n+1)+k+1) = t0^(maxPower - k)*derCoeff(i+1, k+1);
                end
                
                b_temp = posDes(i+1, j+1, dim);
            elseif (j == m), % add one constraint to end of last piece
                A_temp = zeros(1, (n+1)*m);
                maxPower = nnz(derCoeff(i+1, :))-1;
                for k = 0:maxPower,
                    A_temp(1, (j-1)*(n+1)+k+1) = t1^(maxPower - k)*derCoeff(i+1, k+1);
                end
                
                b_temp = posDes(i+1, j+1, dim);
            else % else, add two constraints
                A_temp = zeros(2, (n+1)*m);
                maxPower = nnz(derCoeff(i+1,:))-1;
                for k = 0:maxPower,
                    A_temp(1, (j-1)*(n+1)+k+1) = t1^(maxPower - k)*derCoeff(i+1, k+1); 
                    A_temp(2, j*(n+1)+k+1) = t0^(maxPower - k)*derCoeff(i+1, k+1); 
                end      
                
                b_temp = posDes(i+1, j+1, dim)*ones(2, 1);
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
        if posDes(i+1, j+1, dim) == Inf, %if unfixed
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


% put each A_eq for each dimension into block diagonal matrix
A_opt = blkdiag(A_opt, A_eq);
b_opt = [b_opt; b_eq];

end



%%%
% construct any inequality constraints
A_ineq = [];
b_ineq = [];

for s = 1:ineqConst.numConst, % for each constraint 
    
    % find distance between keyframes
    pos1 = [];
    pos2 = [];
    for i = 1:length(ineqConst.dim(s, :))
        pos1 = [pos1 posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i))];
        pos2 = [pos2 posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, i))];
    end
    R = pdist([pos1; pos2]);
    
    % for each dimension to optimize 
    for i = 1:length(ineqConst.dim(s, :)), 
        
        % find constant to add to b term of inequality
        const = posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i))*...
            ((posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, i))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i)))^2/R^2-1);
        
        for j = 1:length(ineqConst.dim(s, :)),
            if i ~= j,
                const = const + ...
                    posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, j)) * ...
                    (posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, j))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, j))) * ...
                    (posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, i))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i)))/R^2;
            end
        end
        
        
        % find coefficients for constraint equation 
        coeff = zeros(length(ineqConst.dim(s, :)), 1);
        for j = 1:length(ineqConst.dim(s, :))
            if i == j,
                coeff(j, 1) = (1 - ...
                    (posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, j))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, j)))^2/R^2);
            else
                coeff(j, 1) = -(posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, j))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, j))) ...
                    * (posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, i))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i))) ...
                    /R^2;
            end
        end

        
        
        % find 2 constraints for each intermediate point
        A_temp = zeros(2*ineqConst.nc, (n+1)*m*d);
        b_temp = zeros(2*ineqConst.nc, 1);
       
        for j = 1:ineqConst.nc,
            intT = t0+ j/(1+ineqConst.nc)*(t1-t0);
            %find all non-zero coefficients in this row of A_temp
            terms = zeros(length(ineqConst.dim(s, :)), n+1);
            
            for k = 1:length(ineqConst.dim(s, :))
                for l = 0:n,
                    terms(k, l+1) = intT^(n-l)*coeff(k, 1);
                end
                
                
                A_temp((j-1)*2+1, ...
                    (ineqConst.dim(s, k)-1)*m*(n+1) + (ineqConst.start(s, 1)-1)*(n+1) + 1 ...
                    : (ineqConst.dim(s, k)-1)*m*(n+1) + (ineqConst.start(s, 1)-1)*(n+1) + 1+n) ... 
                    = terms(k, :);
                A_temp(j*2, ...
                    (ineqConst.dim(s, k)-1)*m*(n+1) + (ineqConst.start(s, 1)-1)*(n+1) + 1 ...
                    : (ineqConst.dim(s, k)-1)*m*(n+1) + (ineqConst.start(s, 1)-1)*(n+1) + 1+n) ... 
                    = -terms(k, :); 
                
            end
            

            
            b_temp((j-1)*2+1, 1) = ineqConst.delta-const;
            b_temp(j*2, 1) = ineqConst.delta+const;
        end
        
        A_ineq = [A_ineq; A_temp];
        b_ineq = [b_ineq; b_temp];
        
    end
        

end






%%%
% find optimal trajectory through quadratic programming
xT_all = quadprog(Q_opt,[],A_ineq, b_ineq,A_opt,b_opt);


%%%
% explicitly break trajectory into its piecewise parts and dimensions for output
xT = zeros((n+1), m, d);
for dim = 1:d,
    thisxT = xT_all((dim-1)*(n+1)*m+1:dim*(n+1)*m);
    for j = 1:m,
        xT(:, j, dim) = thisxT((j-1)*(n+1)+1:j*(n+1));
    end
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
