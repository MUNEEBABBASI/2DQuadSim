% optTrajTime.m


%function optTrajTime



clear all
close all
clc





%%%
% constants
g = 9.81; %m/s/s
mQ = 0.5; %mass of quadrotor, kg
mL = 0.08; %mass of load, kg
IQ = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3] ;
JQ = IQ(2,2) ;
l = 1; %length of cable, m


%%%
% set up problem
r = 6; %derivative to minimize in cost function
n = 11; %order of desired trajectory
d = 1; %dimensions


%%%
% desired trajectory
%%%
% 1. tension = 0 from going upwards
% specify the m+1 keyframes
m = 3; %number of pieces in trajectory
TDes = [Inf; 0; Inf; Inf; Inf];
% specify desired positions and/or derivatives at keyframes,
% Inf represents unconstrained values
% r x (m+1) x d, where each row i is the value the (i-1)th derivative of keyframe j for dimensions k
posDes = zeros(r, m+1, d);
% posDes(:, :, 1) = [0 1 1 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
% posDes(:, :, 2) = [0 3 2 2; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
% posDes(:, :, 3) = [1 2 3 4; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];

posDes = [0 Inf Inf 5; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0; 0 Inf Inf 0];
[i, j, k] = size(posDes);



%%%
% verify that the problem is well-formed


if (i < r),
    error('not enough contraints specified: to minimize kth derivative, constraints must go up to the (k-1)th derivative');
end

if (ismember(Inf, posDes(:, 1, :)) || ismember(Inf, posDes(:, m+1, :)) )
    error('endpoints must be fully constrained');
end

if (k < d)
    error('not enough dimensions specified');
end





tstart = 0 ;
tfinish = 2;

% %ordered sampling
% tbegins = tstart:0.1:tfinish;
% tends = tstart:0.1:tfinish;
% %tbegins = [0.5 0.8];
% %tends = [1 1.4];
% numBegs = length(tbegins);
% numEnds = length(tends);

%random sampling in the right halfplane
numSamples = 5000;
numEnds = round(sqrt(numSamples));
numBegs = round(sqrt(numSamples));
tbegins = sort(rand(1,numEnds)*2);
tends = sort(rand(1,numBegs)*2);


% record results
fvals = zeros(numEnds,numBegs);
xTL_all = cell(numEnds,numBegs);
xTQ_all = cell(numEnds,numBegs);


for beginIndex = 1:numBegs,
    for endIndex = 1:numEnds,
        
        tDes = [tstart; tbegins(1, beginIndex); tends(1, endIndex); tfinish];%[0;1.2; 3; 5]; % %specify desired arrival times at keyframes
        
        
        %%%
        % check that the solution is well formed
        p = length(tDes);
        
        
        % polynominal trajectories must be at least of order 2r-1 to have all derivatives lower than r defined
        if (n < (2*r-p))
            error('trajectory is not of high enough order for derivative optimized')
        end
        
        
        if (j < m+1 || p < m+1), % must specify m+1 keyframes for m pieces of trajectory
            error('minimum number of keyframes not specified');
        end
        
        if (issorted(tDes) && tends(1, endIndex)-tbegins(1, beginIndex)>0.1), %times are in increasing order, at least 0.1 seconds of free fall
        [xTL_all{endIndex, beginIndex}, xTQ_all{endIndex, beginIndex}, ~, fvals(endIndex,beginIndex)] = findTrajLoad1D(r, n, m, d, tDes, posDes, TDes, g, l, mL, mQ);
        else
            xTL_all = [];
            xTQ_all{endIndex, beginIndex} = [];
            fvals(endIndex,beginIndex) = NaN;
        end
    end
end


putvar(tstart, tfinish, tbegins, tends, fvals);

figure()
hold on;
grid on; 
surf(tbegins, tends, fvals);
xlabel('begin times')
ylabel('end times');
zlabel('cost');


% plot row containing the lowest cost
[~, col] = find(fvals == min(min(fvals)));
figure()
hold on;
for i = 1:length(col),
plot(tends, fvals(:, col(i)));
end
xlabel('ending time');
ylabel('cost');
legend(num2str(tbegins(col)));

%end