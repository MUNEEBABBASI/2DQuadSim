% 7/30/13
% findTrajLoad.m
% generate optimal trajectory in many dimensions
%   this allows for specification of corridor constraints
%   also allows for nonlinear constraints
% Dependencies: findContConstraints.m, findFixedConstraints.m,
%   findDerivativeCoeff.m, findCostMatrix.m
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


function [xT] = findTrajLoad(r, n, m, d, tDes, posDes, ineqConst)

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




global options

if (isempty(options))
    options.r = r;
    options.n = n;
    options.m = m;
    options.d = d;
    options.tDes = tDes;
    options.posDes = posDes;
    options.t0 = t0;
    options.t1 = t1;
    
    options.gamma = 1e-5;
end



%%%
% construct equality constraints
A_opt = [];
b_opt = [];

for dim = 1:d,
    
    % construct fixed value constraints
    [A_fixed, b_fixed] = findFixedConstraints(r, n, m, dim, posDes, t0, t1, [], 1);
    [A_cont, b_cont] = findContConstraints(r, n, m, dim, posDes, t0, t1);
    
    % put each A_eq for each dimension into block diagonal matrix
    A_opt = blkdiag(A_opt, [A_fixed; A_cont]);
    b_opt = [b_opt; [b_fixed; b_cont]];
    
end





%%%
% construct any inequality constraints
[A_ineq, b_ineq] = constructCorrConstraints(n, m, d, posDes, ineqConst, t0, t1);




%%%
% find optimal trajectory through quadratic programming
%xT_all = quadprog(Q_opt,[],A_ineq, b_ineq,A_opt,b_opt);
xT_all = fmincon(@costfunction,zeros((n+1)*m*d, 1),A_ineq, b_ineq, A_opt,b_opt, [], [], 'tensionConst');



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








function f = costfunction(x)

global options;

d = options.d;
m = options.m;
n = options.n;
r = options.r;
t0 = options.t0;
t1 = options.t1;


persistent Q

if (isempty(Q))
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
    
    Q.Q_opt = Q_opt;
end

f = x'*Q.Q_opt*x;

end









% make corridor constraints into inequality constraints
%
% inputs: 
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   d: integer, number of dimensions
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
%   t0: real value, begnning time of the trajectory
%   t1: real value, end time of the trajectory
% outputs: 
%   A_ineq, b_ineq: matrices in formulation A_ineq x <= b_ineq
function [A_ineq, b_ineq] = constructCorrConstraints(n, m, d, posDes, ineqConst, t0, t1)

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
end





