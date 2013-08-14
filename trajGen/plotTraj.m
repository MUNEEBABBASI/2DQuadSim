% 7/30/13
% plotTraj.m
% plots a given trajectory
% Dependencies: evaluateTraj.m
%
% inputs: 
%   t0, t1: integers, beginning and end time of plot
%   xT: (n+1) x m x d matrix, where row i contains the ith coefficient for the
%       jth trajectory in dimension k, nondimensionalized in time 
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   d: integer, number of dimensions
%   tDes: (m+1)x1 vector, desired time of arrival at each keyframe
%   posDes: r x (m+1) x d matrix, 
%       where each row i is the value the (i-1)th derivative of keyframe j for dimensions k 
%   dt: real value, desired time intervals to plot with
%   dimLabels: mx1 cell, create legend labels for dimensions,
%   plotDim: nxm matrix, creates n plots of column 1 vs. column 2
%   numDer: integer, derivative to plot up to (and including), default is 0
% outputs:
%   figure plotting trajectory


function [] = plotTraj(t0, t1, xT, n, m, d, tDes, posDes, dt, dimLabels, plotDim, varargin)

numDer = 0; 

if nargin > 11
    numDer = varargin{1};
end


t = 0:dt:tDes(m+1); %construct t vector 
t = t';
pos = zeros(numDer+1, d, length(t)); %holds position at time i for dimension d


% evaluate the piece-wise polynominal at each point in time
for i = 1:length(t),  
    for k = 0:numDer,
        pos(:, :, i) = evaluateTraj(t(i, 1), n, m, d, xT, tDes, numDer, []);
    end
end


[kmax, ~, ~] = size(posDes);

for i = 1:d,
    for k = 0:numDer,
        vals(:, 1) = pos(k+1, i, :);
        figure()
        hold on;
        plot(t, vals);
        
        if (k+1<kmax)
        plot(tDes, posDes(k+1, :, i), 'k^');
        end
        
        xlabel('time (s)');
        ylabel(dimLabels{i});
        title(['derivative ' num2str(k)])
    end
end


[extraPlots, ~] = size(plotDim);
for i = 1:extraPlots,
figure()
hold on;
grid on
if (nnz(plotDim(i, :)) == 2)
    vals(:, 1) = pos(1, plotDim(i, 1), :);
    vals(:, 2) = pos(1, plotDim(i, 2), :);
    plot(vals(:, 1), vals(:, 2));
    plot(posDes(1, :, 1), posDes(1, :, 2), 'k^');
	xlabel(dimLabels{plotDim(i, 1)});
    ylabel(dimLabels{plotDim(i, 2)});
elseif (nnz(plotDim(i, :)) == 3)
    vals(:, 1) = pos(1, plotDim(i, 1), :);
    vals(:, 2) = pos(1, plotDim(i, 2), :);
    vals(:, 3) = pos(1, plotDim(i, 3), :);
    plot3(vals(:, 1), vals(:, 2), vals(:, 3));
    plot3(posDes(1, :, 1), posDes(1, :, 2), posDes(1, :, 3), 'k^');
	xlabel(dimLabels{plotDim(i, 1)});
    ylabel(dimLabels{plotDim(i, 2)});
    zlabel(dimLabels{plotDim(i, 3)});
end
end




end


