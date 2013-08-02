% 7/31/13
% desiredTraj.m
% generates an optimal trajectory through a set of keyframes
% an implementation of techniques described in "Minimum Snap Trajectory Generation 
% and Control for Quadrotors", Mellinger and Kumar 
%
% indices convention: 
% for a polynominal of order n, its coefficients are:
%       x(t) = c_n t^n + c_[n-1] t^(n-1) + ... + c_1 t + c_0
% for m keyframes, the times of arrival at keyframes are t0, t1, ..., tm
% the polynominal segment between keyframe 0 and 1 is x1, 1 and 2 is x2,
%   ... m-1 and m is xm
%
% Dependencies: findTraj.m, plotTraj.m, findTrajCorr.m, evaluateTraj.m
%   findContConstraints.m, findFixedConstraints.m, findDerivativeCoeff.m, findCostMatrix.m




close all
clear all
clc


%%%
% set up problem
r = 4; %derivative to minimize in cost function
n = 7; %order of desired trajectory
m = 3; %number of pieces in trajectory
d = 2; %dimensions

% specify the m+1 keyframes
tDes = [0; 2; 4; 6]; %specify desired arrival times at keyframes
% specify desired positions and/or derivatives at keyframes, 
% Inf represents unconstrained values
% r x (m+1) x d, where each row i is the value the (i-1)th derivative of keyframe j for dimensions k 
posDes = zeros(r, m+1, d);
posDes(:, :, 1) = [0 1 1 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0]; 
posDes(:, :, 2) = [0 0 2 2; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
posDes(:, :, 3) = [1 2 3 4; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
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

% xT holds all coefficents for all trajectories
% row i is the ith coefficient for the column jth trajectory in dimension k
xT = zeros(n+1, m, d); 
xT2 = zeros(n+1, m, d); 
for i = 1:d,
    xT(:, :, i) = findTraj(r, n, m, i, tDes, posDes);
    xT2(:, :, i) = findTrajJoint(r, n, m, i, tDes, posDes);
end


xT3 = findTrajCorr(r, n, m, d, tDes, posDes, ineqConst);
%xT3 = findTrajLoad(r, n, m, d, tDes, posDes, ineqConst);



%%% 
% plot the trajectory

% create legend labels for dimensions, must correspond to order of m
dimLabels{1} = 'x (m)';
dimLabels{2} = 'y (m)'; 
dimLabels{3} = 'z (m)'; 
plotDim = [1 2]; %if you want to plot two dimensions against each other, specify here 
    % nxm matrix, creates n plots of column 1 vs. column 2
    
plotTraj(xT, n, m, d, tDes, posDes, 0.01, dimLabels, plotDim);
plotTraj(xT2, n, m, d, tDes, posDes, 0.01, dimLabels, plotDim);
plotTraj(xT3, n, m, d, tDes, posDes, 0.01, dimLabels, plotDim);





