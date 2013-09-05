% 8/6/13
% findTrajLoad1D.m
% generate optimal trajectory, assuming only a 1D load
%
% Dependencies: findContConstraints.m, findFixedConstraints.m,
%   findDerivativeCoeff.m, findCostMatrix.m, translateLoadConst.m
%   findQuadEqConstraints.m, findQuadIneqConstraints.m
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
%   TDes: desired tensions at keyframes
%   g: constant, gravity
%   len, mL, mQ: constants, length of cable
% outputs:
%   xTL: (n+1) x mNew x d matrix, where row i contains the ith coefficient for
%       the jth trajectory in dimension k
%       xTL is nondimensionalized in time
%       trajectory for load
%   xTQ: (n+1) x mNew x d matrix, where row i contains the ith coefficient for
%       the jth trajectory in dimension k
%       trajectory for quad, only exists when system is in mode 2
%       all coefficients 0 otherwise
%   modes: a x 3 vector, logs mode switches
%       column 1 indicates keyframe switch occurs, column 2 is last mode,
%           column 3 is new mode (redundant, but just to be explicit)
%       1 indicates mode where cable is taut, trajectory is for load
%       2 indicates mode where cable is slack, trajectory is for quadrotor



function [xTL, xTQ, modes, A_eq, b_eq] = findTrajLoad1D(r, n, m, d, tDes, posDes, TDes, g, len, mL, mQ)


% check that we are dealing with a 1D problem
if d ~= 1,
    error('not a 1D problem!')
end



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

% these represent the quadrotor trajectory




 
%%%
% construct cost matrix Q
Q_joint = [];
for i = 1:m,
    
    if (TDes(i, 1) ~=0)
        Q = findCostMatrix(n, r, t0, t1); % optimize 6th derivative
        
        % multiply by time factor to nondimensionalize
        %Q = 1./((tDes(i+1, 1)-tDes(i, 1))^(2*r)).*Q;
    elseif (TDes(i, 1) == 0)
        Q = findCostMatrix(n, 4, t0, t1); % optimize 4th derivative
        
        % multiply by time factor to nondimensionalize
        %Q = 1./((tDes(i+1, 1)-tDes(i, 1))^(2*4)).*Q;
    end
    
    
    Q_joint = blkdiag(Q_joint, Q); %put in block diagonal matrix
end



%%%
% construct equality constraints
[A_eq, b_eq, modes] = findQuadEqConstraints(r, n, m, d, posDes, TDes, t0, t1, tDes, 1, g, len, mL, mQ);


size(A_eq)


%%%
% construct inequality constraints
[A_ineq, b_ineq] = findQuadIneqConstraints(r, n, m, d, posDes, modes, TDes, t0, t1, tDes, 1, g, len, mL, mQ);

%size(A_ineq)
A_ineq = [];
b_ineq = [];

%%%
% optimize trajectory
xT_all = quadprog(Q_joint,[],A_ineq, b_ineq,A_eq, b_eq);



%%% 
% break trajectory into parts
xTQ = zeros((n+1), m);
for j = 1:m,
    xTQ(:, j) = xT_all((j-1)*(n+1)+1:j*(n+1));
end




%%%
% translate into load trajectory
xTL = [];
for j = 0:m-1

    % if no switch occurs
    if isempty(modes) ||  (isempty(find(modes(:, 1) ==j)) || mode(find(modes(:, 1) == j), 2) == 2)
        xTL = [xTL [xTQ(1:n, j+1);xTQ(n+1, j+1)-len]];
   
    % if switching from mode 1
    else %mode(find(modes(:, 1) == j), 2) == 1,
        [stateEnd, ~] = evaluateTraj(tDes(j+1, 1), n, 1, d, xTL(:, j), tDes, r-1, []);

        xTL = [xTL [zeros(n-2, 1); -1/2*g*(tDes(j+2, 1)-tDes(j+1, 1))^2; stateEnd(2, 1)*(tDes(j+2, 1)-tDes(j+1, 1)); stateEnd(1, 1)]];
    end
 
    
end





end







