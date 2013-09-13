% 9/12/13
% trajTimeTests.m
% time tests for different trajectory generation techniques
% assume 1 dimensional trajectories
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


% %%%
% % small test example
% r = 6; %derivative to minimize in cost function
% n = 11; %order of desired trajectory
% m = 3; %number of pieces in trajectory
% d = 1; %dimensions
% 
% % specify the m+1 keyframes
% tDes = [0;1.2; 3; 5]; % %specify desired arrival times at keyframes
% % specify desired positions and/or derivatives at keyframes, 
% % Inf represents unconstrained values
% % r x (m+1) x d, where each row i is the value the (i-1)th derivative of keyframe j for dimensions k 
% posDes(:, :, 1) = [-0.1653 0.2194 0.3734 1; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
% [i, j, k] = size(posDes);
% l = length(tDes);



% %%%
% % large test example
% r = 6; %derivative to minimize in cost function
% n = 11; %order of desired trajectory
% m = 50; %number of pieces in trajectory
% d = 1; %dimensions
% 
% % specify the m+1 keyframes
% %tRand = rand(1, m).*25;
% %tRand = sort(tRand);
% tRand = load('tRand');
% tDes = [0 tRand.tRand]; % %specify desired arrival times at keyframes
% tDes = tDes';
% % specify desired positions and/or derivatives at keyframes, 
% % Inf represents unconstrained values
% % r x (m+1) x d, where each row i is the value the (i-1)th derivative of keyframe j for dimensions k 
% 
% temp = Inf*ones(r-1, m-1);
% temp = [zeros(r-1, 1) temp zeros(r-1, 1)];
% %posRand = rand(1, m).*100;
% %posRand = sort(posRand);
% posRand = load('posRand');
% posDes = zeros(r, m+1, d);
% posDes(:, :, 1) = [0 posRand.posRand; temp];
% [i, j, k] = size(posDes);
% l = length(tDes);



% % specify s corridor constraints
% ineqConst.numConst = 0; %integer, number of constraints 
% ineqConst.start = 2; %sx1 matrix of keyframes where constraints begin
% ineqConst.nc = 20; %sx1 matrix of numbers of intermediate points
% ineqConst.delta = 0.05; %sx1 matrix of maximum distnaces
% ineqConst.dim = [1 2]; %sxd matrix of dimensions that each constraint applies to




%%%
% test runtimes with different size trajectories
%testVals = 50*ones(1, 50);%0:2:50;
%testVals = [1 5 7];
testVals(1, 1) = 70;
numTests = length(testVals);







timeFindTraj = zeros(1, numTests);
timeFindTrajJoint = zeros(1, numTests);
timeFindTrajConstrained = zeros(1, numTests);

allTimes = cell(numTests, 1);
allPosDes = cell(numTests, 1);

% save trajectories
allxT = cell(numTests, 1); 
allxT2 = cell(numTests, 1); 
allxT3 = cell(numTests, 1);
allPosDesOpt = cell(numTests, 1);




for test = 1:numTests,

r = 6; %derivative to minimize in cost function
n = 11; %order of desired trajectory
d = 1; %dimensions

    
m = testVals(1, test); %number of pieces in trajectory
% specify the m+1 keyframes
%tRand = rand(1, m).*100; %rand(1, m).*50;
%tRand = sort(tRand);
%tRand = load('tRand');
%tRand = tRand.tRand;
%tDes = [0 tRand]; % %specify desired arrival times at keyframes

tRand = rand(1, m)*0.5;
tDes = 0:1:m;

for i = 2:m+1,
    if (round(rand(1))) == 0,
    tDes(1, i) = tDes(1, i)-tRand(1, i-1);
    else
       tDes(1, i) = tDes(1, i)+tRand(1, i-1);
    end
end

tDes = tDes';
% specify desired positions and/or derivatives at keyframes, 
% Inf represents unconstrained values
% r x (m+1) x d, where each row i is the value the (i-1)th derivative of keyframe j for dimensions k 

temp = Inf*ones(r-1, m-1);
temp = [zeros(r-1, 1) temp zeros(r-1, 1)];
posRand = rand(1, m).*50; %rand(1, m).*100;
posRand = sort(posRand);
%posRand = load('posRand');
%posRand = posRand.posRand;
posDes = zeros(r, m+1, d);
posDes(:, :, 1) = [0 posRand; temp];
[i, j, k] = size(posDes);
l = length(tDes);

allTimes{test} = tDes;
allPosDes{test} = posDes;



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
% 
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
posDes_opt = zeros(r, m+1, d); 
xT2 = zeros(n+1, m, d); 
xT3 = zeros(n+1, m, d); 
for i = 1:d,
    
   tic;
   xT(:, :, i) = findTraj(r, n, m, i, tDes, posDes);
   tElapsed1 = toc;
   
   tic;
   [xT2(:, :, i), posDes_opt(:, :, i)] = findTrajJoint(r, n, m, i, tDes, posDes);
   tElapsed2 = toc;
   
   tic;
   [xT3(:, :, i)] = findTrajJointConstrained(r, n, m, i, tDes, posDes);
   tElapsed3 = toc;   
end





timeFindTraj(1, test) = tElapsed1;
timeFindTrajJoint(1, test) = tElapsed2;
timeFindTrajConstrained(1, test) = tElapsed3;

allxT{test} = xT;
allxT2{test} = xT2;
allxT3{test} = xT3;
allPosDesOpt{test} = posDes_opt;


%xT3 = findTrajCorr(r, n, m, d, tDes, posDes, ineqConst);
%xT3 = findTrajLoad(r, n, m, d, tDes, posDes, ineqConst);


% %%%
% % plot QP traj
% dimLabels{1} = 'x (m)';
% dimLabels{2} = 'y (m)'; 
% dimLabels{3} = 'z (m)'; 
% plotTraj(0, tDes(m+1), xT2, n, m, d, tDes, posDes, 0.01, dimLabels, [], 2*r);
% plotTraj(0, tDes(m+1), xT, n, m, d, tDes, posDes, 0.01, dimLabels, [], 2*r);


%xT
%xT2



end



% 
% figure()
% hold on;
% plot(testVals, timeFindTraj);
% plot(testVals, timeFindTrajJoint, 'r');
% plot(testVals, timeFindTrajConstrained, 'r--');
% xlabel('number trajectory segments')
% ylabel('time (s)');
% legend('quadprog', 'analaytic unconstrained', 'analytic constrained');
% 
