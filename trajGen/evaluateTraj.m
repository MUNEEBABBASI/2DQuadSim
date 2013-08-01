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
% outputs:
%   dxT: (r+1) x d vector, row i contains value of (r-1)th derivative in dimension k of xT at t


%%%%%
% Specify the position and derivatives of the desired trajectory
function [dxT] = evaluateTraj(t, n, m, d, xT, tDes, r)

dxT = zeros(r+1, d);

derivativesX = cell(r+1, 1);
derivativesX{1} = xT;

for l = 1:r,
    thisDer = zeros(n-l+1, m, d);
    for j = 1:m
        for k = 1:d,
            thisDer(:, j, k) = polyder(derivativesX{l}(:, j, k));

        end
    end
    
    derivativesX{l+1} = thisDer;
end


% nondimensionalized time
t0 = 0;
t1 = 1;

% evaluate trajectory at given time
if (t < tDes(1, 1)),
    % evaluate in each dimension
    for k = 1:d
        for l = 0:r %evaluate each derivative at the first trajectory's inital time
            dxT(l+1, k) = polyval(derivativesX{l+1}(:, 1, k), t0);
        end
    end
    
    % find which piece of the trajectory we're on based on time and
    %   evaluate there
elseif (t < tDes(m+1, 1));
    
    for j = 1:m,
        if (t < tDes(j+1, 1));
            scaledt = (t-tDes(j, 1))/(tDes(j+1, 1)-tDes(j, 1)); % find the nondimensionalized time
            
            % evaluate in each dimension
            for k = 1:d,
                for l = 0:r,
                    dxT(l+1, k) = 1/((tDes(j+1, 1)-tDes(j, 1))^(l)) * polyval(derivativesX{l+1}(:, j, k), scaledt);
                end
            end
            
            break;
        end
    end
    
    % if after the final keyframe time, assume hover at final keyframe position
else
    
    % evaluate in each dimension
    for k = 1:d
        for l = 0:r, %evaluate each derivative 
        	dxT(l+1, k) = polyval(derivativesX{l+1}(:, m, k), t1);
        end
    end
end






end


