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

% constants
g = 9.81; %m/s/s
mQ = 0.5; %mass of quadrotor, kg
mL = 0.08; %mass of load, kg
IQ = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3] ;
JQ = IQ(2,2) ;
l = 1; %length of cable, m

%%%
% set up problem
% r = 6; %derivative to minimize in cost function
% n = 11; %order of desired trajectory
% m = 3; %number of pieces in trajectory
% d = 2; %dimensions
% 
% % specify the m+1 keyframes
% tDes = [0; 2; 4; 6]; %specify desired arrival times at keyframes
% % specify desired positions and/or derivatives at keyframes, 
% % Inf represents unconstrained values
% % r x (m+1) x d, where each row i is the value the (i-1)th derivative of keyframe j for dimensions k 
% posDes = zeros(r, m+1, d);
% posDes(:, :, 1) = [0 1 1 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0]; 
% posDes(:, :, 2) = [0 3 2 2; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
% posDes(:, :, 3) = [1 2 3 4; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
% [i, j, k] = size(posDes);
% l = length(tDes);



r = 6; %derivative to minimize in cost function
n = 11; %order of desired trajectory
m = 3; %number of pieces in trajectory
d = 1; %dimensions

% specify the m+1 keyframes
tDes = [0; 1; 2; 3];%[0;1.2; 3; 5]; % %specify desired arrival times at keyframes
TDes = [Inf; 0; Inf; Inf];
% specify desired positions and/or derivatives at keyframes, 
% Inf represents unconstrained values
% r x (m+1) x d, where each row i is the value the (i-1)th derivative of keyframe j for dimensions k 
% posDes = zeros(r, m+1, d);
% posDes(:, :, 1) = [0 1 1 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0]; 
% posDes(:, :, 2) = [0 3 2 2; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
% posDes(:, :, 3) = [1 2 3 4; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];

posDes(:, :, 1) = [0 0 Inf 0.5*-9.81*(tDes(3)-tDes(1)); 0 -1 Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
[i, j, k] = size(posDes);
p = length(tDes);



% specify s corridor constraints
ineqConst.numConst = 0; %integer, number of constraints 
ineqConst.start = 2; %sx1 matrix of keyframes where constraints begin
ineqConst.nc = 20; %sx1 matrix of numbers of intermediate points
ineqConst.delta = 0.05; %sx1 matrix of maximum distnaces
ineqConst.dim = [1 2]; %sxd matrix of dimensions that each constraint applies to


%%%
% verify that the problem is well-formed

% polynominal trajectories must be at least of order 2r-1 to have all derivatives lower than r defined
if (n < (2*r-p)) 
    error('trajectory is not of high enough order for derivative optimized')
end

if (i < r),
    error('not enough contraints specified: to minimize kth derivative, constraints must go up to the (k-1)th derivative');
end

if (j < m+1 || p < m+1), % must specify m+1 keyframes for m pieces of trajectory
    error('minimum number of keyframes not specified');
end

% if (ismember(Inf, posDes(:, 1, :)) || ismember(Inf, posDes(:, m+1, :)) )
%     error('endpoints must be fully constrained');
% end

if (k < d)
    error('not enough dimensions specified');
end



%%% 
% find trajectories for each dimension, nondimensionalized in time

% xT holds all coefficents for all trajectories
% row i is the ith coefficient for the column jth trajectory in dimension k
xT = zeros(n+1, m, d); 
xT2 = zeros(n+1, m, d); 
%for i = 1:d,
   %xT(:, :, i) = findTraj(r, n, m, i, tDes, posDes);
   %xT2(:, :, i) = findTrajJoint(r, n, m, i, tDes, posDes);
%end


%xT3 = findTrajCorr(r, n, m, d, tDes, posDes, ineqConst);
[xTL, xTQ, mode] = findTrajLoad1D(r, n, m, d, tDes, posDes, TDes, g, l, mL, mQ);


% look at l
t = 0:0.001:tDes(m+1); %tDes(m+1); %construct t vector 
len = zeros(1, length(t));
der2 = zeros(1, length(t));
    [dxTL, derivativesXL] = evaluateTraj(t(i), n, m, d, xTL, tDes, 2, []);
    [dxTQ, derivativesXQ] = evaluateTraj(t(i), n, m, d, xTQ, tDes, 2, []);
for i = 1:length(t),
    [dxTL, ~] = evaluateTraj(t(i), n, m, d, xTL, tDes, 2, derivativesXL);
    [dxTQ, ~] = evaluateTraj(t(i), n, m, d, xTQ, tDes, 2, derivativesXQ);
    len(1, i) = abs(dxTQ(1, 1) - dxTL(1, 1));
    
    der2(1, i) = dxTL(3, 1);
end

figure()
plot(t, len);
title('distance between quad and load');
ylabel('len (m)');
xlabel('time');

figure()
plot(t, mL*(der2+g));
title('tension');
ylabel('force (N)');
xlabel('time');


% % 
% % disp('continuity checks')
% % %check for continuity
% % for i = 0:m
% %     i
% %     tDes(i+1)
% % [contL, ~] = evaluateTraj(tDes(i+1, 1), n, m, d, xTL, tDes, 5, [])
% % 
% % [contQ, ~] = evaluateTraj(tDes(i+1, 1), n, m, d, xTQ, tDes, 5, [])
% % end
% 
% 


%%% 
% plot the trajectory

% create legend labels for dimensions, must correspond to order of m
dimLabels{3} = 'z (m)'; 
plotDim = [];
%plotDim = [1 2]; %if you want to plot two dimensions against each other, specify here 
    % nxm matrix, creates n plots of column 1 vs. column 2
    
plotTraj(0, tDes(m+1), xTL, n, m, d, tDes, posDes, 0.01, dimLabels, plotDim, 1);
plotTraj(0, tDes(m+1), xTQ, n, m, d, tDes, posDes, 0.01, dimLabels, plotDim, 2*r);





