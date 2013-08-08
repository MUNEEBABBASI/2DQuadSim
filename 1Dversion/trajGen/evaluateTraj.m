% 7/31/13
% evaluateTraj.m
% evaluates position and higher derivatives of a given polynominal
%   trajectory at a given time
% Dependencies: -
%
% inputs: 
%   t: real value, time to evaluate trajectory at
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   d: integer, number of dimensions
%   xT: (n+1) x m x d matrix, where row i contains the ith coefficient for the
%       jth trajectory in dimension k, nondimensionalized in time 
%   tDes: (m+1)x1 vector, desired time of arrival at each keyframe
%   r: integer, derivative number to evaluate up to (and including)
%   derivativesX: optional, r+1 cell of (n)xmxd matrices containing
%       coefficients of derivatives of the trajectory, calculated if input
%       is []
%   mode [optional]: a x 3 vector, logs mode switches
%       column 1 indicates keyframe switch occurs, column 2 is last mode,
%           column 3 is new mode (redundant, but just to be explicit)
%       1 indicates mode where cable is taut, trajectory is for load
%       2 indicates mode where cable is slack, trajectory is for quadrotor 
%       default is empty
%   currentMode [optional]: integer, 1 or 2 indicating current mode
%       default is 1
% outputs:
%   dxT: (r+1) x d vector, row i contains value of (i-1)th derivative in dimension j of xT at t


%%%%%
% Specify the position and derivatives of the desired trajectory
function [dxT, derivativesX] = evaluateTraj(t, n, m, d, xT, tDes, r, derivativesX, varargin)

modes = [];
currentMode = 1;

if (nargin > 8)
    modes = varargin{1};
end
if (nargin> 9)
    currentMode = varargin{2}; %assume default mode
end




dxT = zeros(r+1, d);

if (isempty(derivativesX)) ,
derivativesX = cell(r+1, 1);
derivativesX{1} = xT;


for l = 1:r,
    thisDer = zeros(n-l+1, m, d);
    for j = 1:m
        for k = 1:d,
            
            [a, b, c] = size(thisDer); 
            [a2, b2, c2] = size(polyder(derivativesX{l}(:, j, k)));
            thisDer(:, j, k) = [zeros(1, a-b2) polyder(derivativesX{l}(:, j, k))];

        end
    end
    
    derivativesX{l+1} = thisDer;
end
end


% nondimensionalized time
t0 = 0;
t1 = 1;

% evaluate trajectory at given time
if (t < tDes(1, 1)),
    disp('one')
    % evaluate in each dimension
    for k = 1:d
        for l = 0:r %evaluate each derivative at the first trajectory's inital time
            %scaledt = t0;
            %term = 1/((tDes(2, 1)-tDes(1, 1))^(l));
            term = 1;
            scaledt = t;
            dxT(l+1, k) = term*polyval(derivativesX{l+1}(:, 1, k), scaledt);
        end
    end
    
    % find which piece of the trajectory we're on based on time and
    %   evaluate there
elseif (t < tDes(m+1, 1));

    for j = 1:m,
        if (t < tDes(j+1, 1));
                disp('two')
    j
%             % check if the point we switched from was the boundary of a mode switch
%             if (~isempty(modes) && ismember(j-1, modes(:, 1)))
%                 index = find((j-1) == modes(:, 1));
%                 % if so, check which side of the switch we're on
%                 % if still in last mode for integration, go back to last
%                 %   segement
%                 if currentMode == modes(index, 2),
%                     j = j-1;
%                 end
%             end
            
            %scaledt = (t-tDes(j, 1))/(tDes(j+1, 1)-tDes(j, 1)); % find the nondimensionalized time
            %term = 1/((tDes(j+1, 1)-tDes(j, 1))^(l));
            term = 1;
            scaledt = t;
            
            % evaluate in each dimension
            for k = 1:d,
                for l = 0:r,
                    dxT(l+1, k) = term * polyval(derivativesX{l+1}(:, j, k), scaledt);
                end
            end
            
            break;
        end
    end
    
    % if after the final keyframe time, assume hover at final keyframe position
else
    disp('three')
    % evaluate in each dimension
    for k = 1:d
        for l = 0:r, %evaluate each derivative 
            %scaledt = (t-tDes(j, 1))/(tDes(j+1, 1)-tDes(j, 1)); % find the nondimensionalized time
            %term = 1/((tDes(m+1, 1)-tDes(m, 1))^(l));
            term = 1;
            scaledt = t1;
            
        	dxT(l+1, k) = term * polyval(derivativesX{l+1}(:, m, k), scaledt);
        end
    end
end
r
t
dxT


end


