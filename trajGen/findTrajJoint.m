% 7/30/13
% findTrajJoint.m
% generate optimal trajectory for through a set of keyframes by optimizing
%   derivatives at endpoints 
% 	uses technique from"Polynominal Trajectory Planning for Quadrotor Flight", Richter et al.  
% Dependencies: -
%
% inputs: 
%   t: 1 x m vector, representing time to reach each waypoint  
%   x: r x m vector, coordinates and higher derivatives of each waypoint,
%       each column is a waypoint, each row is a derivative value, -1 if
%       unfixed
% outputs:
%   xT: 2r x (m-1) vector, coefficients of piecewise polynomial trajectory, where
%       columns are polynominals and rows are coefficients

%%%%%
% Specify the position and derivatives of the desired trajectory
function [xT] = findTrajJoint(t, xConst)

%nondimensionalized time and distance, from 0 to 1 for both
[r, m] = size(xConst);

% c = [c_(2r-1) c_(2r-2) ... c1 c0]

% construct Q_joint
Q_joint = zeros(2*r*(m-1));
for w = 1:m-1,
    Q = zeros(2*r);
    tempTerms = ones(2*r, 1);
    
    for i = 0:(2*r-1),
        tempTerm = 1;
        for j = 0:(r-1)
            tempTerm = tempTerm*(i-j);
        end
        tempTerms(i+1, 1) = tempTerm;
    end
    
    for i = 0:(2*r-1),
        for j = 0:(2*r-1),
            if ((i >= r) && (j >= r)),
                tinit = t(w, 1);
                tfinal = t(w+1, 1);
                % substitute in the proper t at this step
                Q(i+1, j+1) = tempTerms(i+1, 1)*tempTerms(j+1, 1)*(tfinal^(i+j-2*r+1) - tinit^(i+j-2*r+1))/(i+j-2*r+1);
            end
        end
    end
    
    % flip Q so that it'll correspond to the order of c coefficients
    Q = rot90(rot90(Q));
      
    Q_joint(2*r*(w-1)+1:2*r*w, 2*r*(w-1)+1:2*r*w) = Q;
end



% construct A_joint

A_joint = zeros(2*r*(m-1));
for w = 1:(m-1),
    % find coefficients of derivatives
    tempCoeff = zeros(r, 2*r);
    tempC = ones(1, 2*r);
    tempCoeff(1, :) = tempC;
    for i = 2:r,
        tempC = polyder(tempC);
        tempCoeff(i, :) = [tempC zeros(1, 2*r - length(tempC))];
    end
    
    % substitute in the proper t at this step
    A = zeros(2*r);
    for i = 1:r,
        maxPower = nnz(tempCoeff(i, :))-1;
        for j = 1:2*r,
            if (j <= maxPower+1),
                tinit = t(w, 1);
                tfinal = t(w+1, 1);
                A(i, j) = tinit^(maxPower - (j-1))*tempCoeff(i, j);
                A(i+r, j) = tfinal^(maxPower-(j-1))*tempCoeff(i, j);
            end
        end
    end
    
    A_joint(2*r*(w-1)+1:2*r*w, 2*r*(w-1)+1:2*r*w) = A;
end

% construct D_F
% D_F = [x1(t0) x1(t1) x2(t2) t3(t3) ... x_[m-1](t_[m-1]) x1'(t0) x1''(t0)
%   ... x1^(n)(t0) x[m-1]'(t_[m-1]) x[m-1]''(t_[m-1]) ...
%   x[m-1]^(n)(t_[m-1])]
% D_P = [x1'(t1) ... x1^(n)(t1) x2'(t2) ... x2^(n)(t2) ... x[m-1]'(t_[m-1]) ...
%   x[m-1]^(n)(t_[m-1])]

D_F = zeros(2*r-2+m, 1);

% fix positions at each waypoint
for i = 1:m,
    D_F(i, 1) = xConst(1, i);
end

% fix derivatives at t0 and t_[m-1]
for i = 2:r,
    D_F(m+i-1, 1) = xConst(i, 1);
end

for i = 2:r,
    D_F(m+r+i-2, 1) = xConst(i, m);
end

% construct M

% x1(t0) terms
M = [1 zeros(1, r*m-1)];
M = [M; zeros(r-1, m) eye(r-1) zeros(r-1, m*r-m-r+1)];

% terms for trajectories at times t1 to t_[m-2]
for i = 1:m-2,
    Mtemp= [zeros(1, i) 1 zeros(1, r*m-i-1)];
    Mtemp = [Mtemp; zeros(r-1, m+(2+i-1)*(r-1)) eye(r-1) zeros(r-1, m*r-(r-1)-(m+(2+i-1)*(r-1)))];
    M = [M; Mtemp; Mtemp];
end

% x_[m-1](t_[m-1]) terms
Mtemp = [zeros(1, m-1) 1 zeros(1, r*m-m)];
Mtemp = [Mtemp; zeros(r-1, m+r-1) eye(r-1) zeros(r-1, m*r-(r-1)-(m+r-1))];
M = [M; Mtemp];

% R = M' * inv(A_joint)' * Q_joint * inv(A_joint) * M;
% R has dimensions r*m x r*m
R = M'*inv(A_joint)'*Q_joint*inv(A_joint)*M;

% find R_FP and R_PP
R_FF = R(1:m+2*(r-1), 1:m+2*(r-1));
R_FP = R(1:m+2*(r-1), m+2*(r-1)+1:r*m);
R_PF = R(m+2*(r-1)+1:r*m, 1:m+2*(r-1));
R_PP = R(m+2*(r-1)+1:r*m, m+2*(r-1)+1:r*m);

% D_opt = -inv(R_PP)'*R_FP'*D_F
% D_opt = [x1'(t1) ... x1^(n-1)(t1) x2'(t2) ... x_[m-1]'(t_[m-1]) ... x_[m-1]^(n-1)(t_[m-1])]
D_opt = -inv(R_PP)'*R_FP'*D_F;

% find optimal trajectory through quadratic programming
% find trajectory for each segment 

xT = zeros(2*r, m-1);

for i = 1:m-1,
    % construct beginning and ending constraints, using values from D_opt
    % for intermediate derivative values
    if i == 1,
        x0 = xConst(:, 1);
        
        xf = zeros(r, 1);
        xf(1, 1) = xConst(1, i+1);
        xf(2:r, 1) = D_opt((i-1)*(r-1)+1:(i)*(r-1), 1);    
    elseif i < m-1,
        x0 = zeros(r, 1);
        x0(1, 1) = xConst(1, i);
        x0(2:r, 1) = D_opt((i-2)*(r-1)+1:(i-1)*(r-1), 1);
        
        xf = zeros(r, 1);
        xf(1, 1) = xConst(1, i+1);
        xf(2:r, 1) = D_opt((i-1)*(r-1)+1:(i)*(r-1), 1);        
    else
        x0 = zeros(r, 1);
        x0(1, 1) = xConst(1, i);
        x0(2:r, 1) = D_opt((i-2)*(r-1)+1:(i-1)*(r-1), 1);
        
        xf = xConst(:, m);
    end

    xTemp = optTraj(x0, xf);
  
    xT(:, i) = xTemp;
end

end

