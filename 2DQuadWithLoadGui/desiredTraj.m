% 7/29/13
% desiredTraj.m
% desired quadrotor trajectory input
% Dependencies: findTraj.m, plotTraj.m, findTrajCorr.m, evaluateTraj.m
%   findContConstraints.m, findFixedConstraints.m, findDerivativeCoeff.m, findCostMatrix.m
%
% inputs: 
%   t: real value, current time
%   g, mQ, JQ: real values, constants
% outputs:
%   xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT: each a 2x1 vector, position and higher derivatives of trajectory at current time

%%%%%
% Specify the position and derivatives of the desired trajectory
function [xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT] = desiredTraj(t, g, mQ, JQ)


global traj


%%%
% design an optimal trajectory


%%%
% set up problem
r = 6; %derivative to minimize in cost function
n = 11; %order of desired trajectory
m = 3; %number of pieces in trajectory
d = 2; %dimensions

% specify the m+1 keyframes
tDes = [0; 2;4;6]; %specify desired arrival times at keyframes
%tDes = [0; 2;4;6]; %specify desired arrival times at keyframes
% specify desired positions and/or derivatives at keyframes, 
% Inf represents unconstrained values
% r x (m+1) x d, where each row i is the value the (i-1)th derivative of keyframe j for dimensions k 
posDes = zeros(r, m+1, d);
posDes(:, :, 1) = [0 1 1 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0]; 
posDes(:, :, 2) = [0 3 2 2; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];

%posDes(:, :, 1) = [0 1 1 0; 0 0 Inf 0; 0 0 Inf 0; 0 0 Inf 0; 0 0 Inf 0; 0 0 Inf 0]; 
%posDes(:, :, 2) = [0 0 2 2; 0 0 Inf 0; 0 0 Inf 0; 0 0 Inf 0; 0 0 Inf 0; 0 0 Inf 0];
[i, j, k] = size(posDes);
l = length(tDes);

% specify s corridor constraints
ineqConst.numConst = 1; %integer, number of constraints 
ineqConst.start = 2; %sx1 matrix of keyframes where constraints begin
ineqConst.nc = 20; %sx1 matrix of numbers of intermediate points
ineqConst.delta = 0.05; %sx1 matrix of maximum distnaces
ineqConst.dim = [1 2]; %sxd matrix of dimensions that each constraint applies to




%%%
% verify that the problem is well-formed

% polynominal trajectories must be at least of order 2r-1 to have all derivatives lower than r defined
if (n < (2*r-1)) 
    error('trajectory is not of high enough order for derivative optimized')
end

if (i < r),
    error('not enough contraints specified: to minimize kth derivative, constraints must go up to the (k-1)th derivative');
end

if (j < m+1 || l < m+1), % must specify m+1 keyframes for m pieces of trajectory
    error('minimum number of keyframes not specified');
end

if (ismember(Inf, posDes(:, 1, :)) || ismember(Inf, posDes(:, m+1, :)) )
    error('endpoints must be fully constrained');
end

if (k < d)
    error('not enough dimensions specified');
end





%%%
% find trajectories for each dimension, nondimensionalized in time

if(isempty(traj))
    
    % xT holds all coefficents for all trajectories
    % row i is the ith coefficient for the column jth trajectory in dimension k
    traj.xT = zeros(n+1, m, d);
    for i = 1:d,
        %traj.xT(:, :, i) = findTraj(r, n, m, i, tDes, posDes);
        %xT(:, :, i) = findTrajJoint(r, n, m, i, tDes, posDes);
    end
    
    traj.xT = findTrajCorr(r, n, m, d, tDes, posDes, ineqConst);
    %traj.xT = findTrajLoad(r, n, m, d, tDes, posDes, ineqConst);
    
    
    % plot trajectory
    dimLabels{1} = 'y (m)';
    dimLabels{2} = 'z (m)';
    plotDim = [1 2]; %if you want to plot two dimensions against each other, specify here
    % nxm matrix, creates n plots of column 1 vs. column 2
    
    plotTraj(traj.xT, n, m, d, tDes, posDes, 0.01, dimLabels, plotDim);
    
    [~, traj.derivativesX] = evaluateTraj(t, n, m, d, traj.xT, tDes, r, []);
end


% evaluate trajectory at the desired time
[xEval, ~] = evaluateTraj(t, n, m, d, traj.xT, tDes, 6, traj.derivativesX);

% construct return values
xT = xEval(1, :)';
dxT = xEval(2, :)';
d2xT = xEval(3, :)';
d3xT = xEval(4, :)';
d4xT = xEval(5, :)';
d5xT = xEval(6, :)';
d6xT = xEval(7, :)';

% save important properties
traj.posDes = posDes;
traj.tDes = tDes;
traj.ineqConst = ineqConst;
traj.r = r;
traj.n = n;
traj.m = m;
traj.d = d;








%%%
% alternatively, manually specify trajectory
% traj.posDes = [];
% traj.tDes = [];

%     T=5;
%     th_max = 2*pi/180 ;
%     L = 1 ;
%     Gx = L*sin(th_max) ; Gy = 0*-(L-L*cos(th_max)) ;
%     xT = [0 + Gx*sin(2*pi*1/T*t) ; -Gy-L + Gy*cos(2*pi*1/T*t)] ;
%     vT = (2*pi*1/T)*[Gx*cos(2*pi*1/T*t) ; Gy*-sin(2*pi*1/T*t)] ;
%     aT = (2*pi*1/T)^2*[Gx*-sin(2*pi*1/T*t) ; Gy*-cos(2*pi*1/T*t)] ;
%     jT = (2*pi*1/T)^3*[Gx*-cos(2*pi*1/T*t) ; Gy*sin(2*pi*1/T*t)] ;
%     sT = (2*pi*1/T)^4*[Gx*sin(2*pi*1/T*t) ; Gy*cos(2*pi*1/T*t)] ;

% 
%     xT = [ 5*cos(t); 5*sin(t)];
%     dxT = [ -5*sin(t) ; 5*cos(t) ];
%     d2xT = [ -5*cos(t) ; -5*sin(t) ];
%     d3xT = [ 5*sin(t) ; -5*cos(t)];
%     d4xT = [ 5*cos(t) ; 5*sin(t)];
%     d5xT = [-5*sin(t) ; 5*cos(t)];
%     d6xT = [-5*cos(t) ; -5*sin(t)];

% xT = [t; t];
% dxT = [1; 1];
% d2xT = [0; 0];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];
% 
% xT = [t; 0];
% dxT = [1; 0];
% d2xT = [0; 0];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];

% xT = [0; t];
% dxT = [0; 1];
% d2xT = [0; 0];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];

% 
% xT = [t^2; t];
% dxT = [2*t; 1];
% d2xT = [2; 0];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];

% xT = [t; t^2];
% dxT = [1; 2*t];
% d2xT = [0; 2];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];

% xT = [t; t^3];
% dxT = [1; 3*t];
% d2xT = [0; 3];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];


end


