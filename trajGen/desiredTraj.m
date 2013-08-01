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
% Dependencies: findTraj.m, plotTraj.m, findTrajCorr.m
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
postDes = zeros(r, m+1, d);
posDes(:, :, 1) = [0 1 1 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0]; 
posDes(:, :, 2) = [0 0 2 2; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
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
for i = 1:d,
    xT(:, :, i) = findTraj(r, n, m, i, tDes, posDes);
end

xT2 = findTrajCorr(r, n, m, d, tDes, posDes, ineqConst);



%%% 
% plot the trajectory

% create legend labels for dimensions, must correspond to order of m
dimLabels{1} = 'y (m)';
dimLabels{2} = 'z (m)'; 
plotDim = [1 2]; %if you want to plot two dimensions against each other, specify here 
    % nxm matrix, creates n plots of column 1 vs. column 2
    
plotTraj(xT, n, m, d, tDes, posDes, 0.01, dimLabels, plotDim);
%plotTraj(xT2, n, m, d, tDes, posDes, 0.01, dimLabels, plotDim);






% ineqConst.start = 2;
% ineqConst.nc = 1;
% ineqConst.delta = 0.05;
% 
% % optimize a trajectory between these waypoints
% 
% if(isempty(traj))
% 
%     % joint optimization if more than one waypoint
%     if (numWaypoints > 2 && smooth == 1),
%         [yCoeff, zCoeff] = optTrajCorr(posT, posY, posZ, ineqConst);
%         %yCoeff = optTrajJoint(posT, posY);
%         %zCoeff = optTrajJoint(posT, posZ);
%     
%         for i = 1:numWaypoints -1,
%         traj.y{i} = yCoeff(:, i);
%         traj.z{i} = zCoeff(:, i);
%         end
%     
%     % otherwise, just fix the ends
%     else
%         for i = 1:numWaypoints-1,
%         traj.y{i} = optTraj([posY(1 ,i),0,0,0,0,0,0,0]',[posY(1, i+1),0,0,0,0,0,0,0]');
%         traj.z{i} = optTraj([posZ(1, i),0,0,0,0,0,0,0]',[posZ(1, i+1),0,0,0,0,0,0,0]');
%         end
%     end
%     
%     for i = 1:numWaypoints-1,
%        % traj.y{i} = optTraj([posY(i, 1),0,0,0,0,0,0,0]',[posY(i+1, 1),0,0,0,0,0,0,0]');
%       %  traj.z{i} = optTraj([posZ(i, 1),0,0,0,0,0,0,0]',[posZ(i+1, 1),0,0,0,0,0,0,0]');
%         
%         traj.dy{i} = polyder(traj.y{i});
%         traj.d2y{i} = polyder(traj.dy{i});
%         traj.d3y{i} = polyder(traj.d2y{i});
%         traj.d4y{i} = polyder(traj.d3y{i});
%         traj.d5y{i} = polyder(traj.d4y{i});
%         traj.d6y{i} = polyder(traj.d5y{i});
%         
%         traj.dz{i} = polyder(traj.z{i});
%         traj.d2z{i} = polyder(traj.dz{i});
%         traj.d3z{i} = polyder(traj.d2z{i});
%         traj.d4z{i} = polyder(traj.d3z{i});
%         traj.d5z{i} = polyder(traj.d4z{i});
%         traj.d6z{i} = polyder(traj.d5z{i});
%         
%         traj.posT = posT;
%         traj.posY = posY;
%         traj.posZ = posZ;
% 
%     end
% 
%     
% end
% 
% if(t<posT(1, 1))
%     xT = [posY(1, 1); posZ(1, 1)];
%     dxT = [ 0;0 ];
%     d2xT = [ 0;0 ];
%     d3xT = [ 0;0 ];
%     d4xT = [ 0;0 ];
%     d5xT = [ 0;0 ];
%     d6xT = [ 0;0 ];
%     
% elseif(t<(posT(numWaypoints, 1)))
%     for i = 1:numWaypoints-1,
%         if (t < posT(i+1, 1))
%             scaledt = (t-posT(i, 1))/(posT(i+1, 1)-posT(i, 1));
%             xT = [polyval(traj.y{i},scaledt);polyval(traj.z{i},scaledt)];
%             dxT = 1/(posT(i+1, 1)-posT(i, 1))*[polyval(traj.dy{i},scaledt);polyval(traj.dz{i},scaledt)];
%             d2xT = 1/(posT(i+1, 1)-posT(i, 1))^2*[polyval(traj.d2y{i},scaledt);polyval(traj.d2z{i},scaledt)];
%             d3xT = 1/(posT(i+1, 1)-posT(i, 1))^3*[polyval(traj.d3y{i},scaledt);polyval(traj.d3z{i},scaledt)];
%             d4xT = 1/(posT(i+1, 1)-posT(i, 1))^4*[polyval(traj.d4y{i},scaledt);polyval(traj.d4z{i},scaledt)];
%             d5xT = 1/(posT(i+1, 1)-posT(i, 1))^5*[polyval(traj.d5y{i},scaledt);polyval(traj.d5z{i},scaledt)];
%             d6xT = 1/(posT(i+1, 1)-posT(i, 1))^6*[polyval(traj.d6y{i},scaledt);polyval(traj.d6z{i},scaledt)];
% 
% %             scaledt = t;
% %             xT = [polyval(traj.y{i},scaledt);polyval(traj.z{i},scaledt)];
% %             dxT = [polyval(traj.dy{i},scaledt);polyval(traj.dz{i},scaledt)];
% %             d2xT = [polyval(traj.d2y{i},scaledt);polyval(traj.d2z{i},scaledt)];
% %             d3xT = [polyval(traj.d3y{i},scaledt);polyval(traj.d3z{i},scaledt)];
% %             d4xT = [polyval(traj.d4y{i},scaledt);polyval(traj.d4z{i},scaledt)];
% %             d5xT = [polyval(traj.d5y{i},scaledt);polyval(traj.d5z{i},scaledt)];
% %             d6xT = [polyval(traj.d6y{i},scaledt);polyval(traj.d6z{i},scaledt)];
% % 
% %             
%             return
%         end
%     end  
% else
%     xT = [posY(1, numWaypoints); posZ(1, numWaypoints)];
%     dxT = [ 0;0 ];
%     d2xT = [ 0;0 ];
%     d3xT = [ 0;0 ];
%     d4xT = [ 0;0 ];
%     d5xT = [ 0;0 ];
%     d6xT = [ 0;0 ];
% 
% end


