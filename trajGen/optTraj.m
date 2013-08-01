% 7/29/13
% optTraj.m
% generate optimal trajectory for load between two waypoints
% using equations from "Polynominal Trajectory Planning for Quadrotor Flight", Richter et al.  
% Dependencies: -
%
% inputs: 
%   t: real value, current time
%   g, mQ, JQ: real values, constants
% outputs:
%   xT: nx1 vector, coefficients of polynomial optimal trajectory

%%%%%
% Specify the position and derivatives of the desired trajectory
function [xT] = optTraj(x0, xf)

%nondimensionalized time and distance, from 0 to 1 for both
t0 = 0;
t1 = 1;
r = length(x0);

% c = [c_(2r-1) c_(2r-2) ... c1 c0]

% construct cost matrix
Q = zeros(2*r);
tempTerms = ones(2*r, 1);

for i = 0:(2*r-1),
    tempTerm = 1;
    for m = 0:(r-1)
        tempTerm = tempTerm*(i-m);
    end
    tempTerms(i+1, 1) = tempTerm;
end

for i = 0:(2*r-1),
    for j = 0:(2*r-1),
        if ((i >= r) && (j >= r)),
        Q(i+1, j+1) = tempTerms(i+1, 1)*tempTerms(j+1, 1)*(t1^(i+j-2*r+1) - t0^(i+j-2*r+1))/(i+j-2*r+1);
        end
    end
end

% flip Q so that it'll correspond to the order of c coefficients
Q = rot90(rot90(Q));



% construct constraints 

% find coefficients of derivatives
tempCoeff = zeros(r, 2*r);
tempC = ones(1, 2*r);
tempCoeff(1, :) = tempC;
for i = 2:r,
    tempC = polyder(tempC);
    tempCoeff(i, :) = [tempC zeros(1, 2*r - length(tempC))];
end

% substitute in t0 and tf
A = zeros(2*r);
for i = 1:r,
    maxPower = nnz(tempCoeff(i, :))-1;
    for j = 1:2*r,
        if (j <= maxPower+1),
        A(i, j) = t0^(maxPower - (j-1))*tempCoeff(i, j);
        A(i+r, j) = t1^(maxPower-(j-1))*tempCoeff(i, j);
        end
    end
end

b = [x0; xf];

fixed = ones(2*r, 1) ~= 0;

% find optimal trajectory through quadratic programming
xT = quadprog(Q,[],[],[],A(fixed, :),b(fixed, :));

end

