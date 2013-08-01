% 7/31/13
% desiredTraj.m
% desired quadrotor trajectory input
% Dependencies: optTraj.m
%
% inputs: 
%   t: real value, current time
% outputs:
%   xT: (n+1) x d x k x m, where m of the (n+1) x d x k  matrices represent properties of
%       each piece of the piecewise trajectory, the k of the (n+1) x d
%       matrices represent the properties of each derivative of the
%       trajectory, the n+1 rows each contain a coefficient, and the d
%       columns represent a dimension that is being optimized (for example,
%       x, y, z position)





% 7/29/13
% optTrajCorr.m
% generate optimal trajectory for load going between m waypoints,
%   where derivatives at intermediate waypoints are optimized, 
%   with the option for corridor constraints
% using equations from "Polynominal Trajectory Planning for Quadrotor Flight", Richter et al.  
% Dependencies: -
%
% inputs: 
%   t: 1 x m vector, representing time to reach each waypoint  
%   yConst: r x m vector, y coordinates and higher derivatives of each waypoint,
%       each column is a waypoint, each row is a derivative value, -1 if
%       unfixed
%   zConst: r x m vector, z coordinates and higher derivatives of each waypoint,
%       each column is a waypoint, each row is a derivative value, -1 if
%       unfixed
%   ineqConst: structure of mx1 arrays, with elements:
%       start: starting waypoint of constraint 
%       delta:
%       nc:
% outputs:
%   yT: 2r x (m-1) vector, coefficients of y-component piecewise polynomial trajectory, where
%       columns are polynominals and rows are coefficients
%   zT: 2r x (m-1) vector, coefficients of z-component piecewise polynomial trajectory, where
%       columns are polynominals and rows are coefficients

%%%%%
% Specify the position and derivatives of the desired trajectory
function [yT zT] = optTrajCorr(t, yConst, zConst, ineqConst)

%nondimensionalized time and distance, from 0 to 1 for both
[r, m] = size(yConst);


numDim = 2;

Q_joint = cell(numDim, 1);
A_joint = cell(numDim, 1);
D_opt = cell(numDim, 1);

for dim = 1:numDim,

    if dim == 1,
        xConst = yConst;
    elseif dim == 2,
        xConst = zConst;
    end
    
% c = [c_(2r-1) c_(2r-2) ... c1 c0]

% construct Q_joint 
Q_joint{dim, 1} = zeros(2*r*(m-1));
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
      
    Q_joint{dim, 1}(2*r*(w-1)+1:2*r*w, 2*r*(w-1)+1:2*r*w) = Q;
end



% construct A_joint

A_joint{dim, 1} = zeros(2*r*(m-1));
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
    
    A_joint{dim, 1}(2*r*(w-1)+1:2*r*w, 2*r*(w-1)+1:2*r*w) = A;
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
R = M'*inv(A_joint{dim, 1})'*Q_joint{dim, 1}*inv(A_joint{dim, 1})*M;

% find R_FP and R_PP
R_FF = R(1:m+2*(r-1), 1:m+2*(r-1));
R_FP = R(1:m+2*(r-1), m+2*(r-1)+1:r*m);
R_PF = R(m+2*(r-1)+1:r*m, 1:m+2*(r-1));
R_PP = R(m+2*(r-1)+1:r*m, m+2*(r-1)+1:r*m);

% D_opt = -inv(R_PP)'*R_FP'*D_F
% D_opt = [x1'(t1) ... x1^(n-1)(t1) x2'(t2) ... x_[m-1]'(t_[m-1]) ... x_[m-1]^(n-1)(t_[m-1])]
D_opt{dim, 1} = -inv(R_PP)'*R_FP'*D_F;



end



% find optimal trajectory through quadratic programming
% find trajectory for each segment 

yT = zeros(2*r, m-1);
zT = zeros(2*r, m-1);

for w = 1:(m-1),
    %%%
    % construct beginning and ending constraints, using values from D_opt
    % for intermediate derivative values
    b = [];
    A = cell(numDim, 1);
    Q = cell(numDim, 1);
    
    % stack y and z components together for initial conditions 
    for dim = 1:numDim,
        
        if dim == 1,
            xConst = yConst;
        elseif dim == 2,
            xConst = zConst;
        end
    
    if w == 1,
        x0 = xConst(:, 1);
        
        xf = zeros(r, 1);
        xf(1, 1) = xConst(1, w+1);
        xf(2:r, 1) = D_opt{dim, 1}((w-1)*(r-1)+1:(w)*(r-1), 1);    
    elseif w < m-1,
        x0 = zeros(r, 1);
        x0(1, 1) = xConst(1, w);
        x0(2:r, 1) = D_opt{dim, 1}((w-2)*(r-1)+1:(w-1)*(r-1), 1);
        
        xf = zeros(r, 1);
        xf(1, 1) = xConst(1, w+1);
        xf(2:r, 1) = D_opt{dim, 1}((w-1)*(r-1)+1:(w)*(r-1), 1);        
    else
        x0 = zeros(r, 1);
        x0(1, 1) = xConst(1, w);
        x0(2:r, 1) = D_opt{dim, 1}((w-2)*(r-1)+1:(w-1)*(r-1), 1);
        
        xf = xConst(:, m);
    end
    
    b = [b; x0; xf];
    
    
    
    
    % nondimensionalized time
    t0 = 0;
    t1 = 1;
    
    %%%
    % construct cost matrix
    Q{dim, 1} = zeros(2*r);
    tempTerms = ones(2*r, 1);

    for i = 0:(2*r-1),
        tempTerm = 1;
        for k = 0:(r-1)
            tempTerm = tempTerm*(i-k);
        end
        tempTerms(i+1, 1) = tempTerm;
    end
    
    for i = 0:(2*r-1),
        for j = 0:(2*r-1),
            if ((i >= r) && (j >= r)),
                Q{dim, 1}(i+1, j+1) = tempTerms(i+1, 1)*tempTerms(j+1, 1)*(t1^(i+j-2*r+1) - t0^(i+j-2*r+1))/(i+j-2*r+1);
            end
        end
    end

    % flip Q so that it'll correspond to the order of c coefficients
    Q{dim, 1} = rot90(rot90(Q{dim, 1}));


    
    
    %%%
    % construct equality constraint
    % find coefficients of derivatives
    tempCoeff = zeros(r, 2*r);
    tempC = ones(1, 2*r);
    tempCoeff(1, :) = tempC;
    for i = 2:r,
        tempC = polyder(tempC);
        tempCoeff(i, :) = [tempC zeros(1, 2*r - length(tempC))];
    end
    
    % substitute in t0 and tf
    A{dim, 1} = zeros(2*r);
    for i = 1:r,
        maxPower = nnz(tempCoeff(i, :))-1;
        for j = 1:2*r,
            if (j <= maxPower+1),
                A{dim, 1}(i, j) = t0^(maxPower - (j-1))*tempCoeff(i, j);
                A{dim, 1}(i+r, j) = t1^(maxPower-(j-1))*tempCoeff(i, j);
            end
        end
    end

    end
    
    % combine A and Q matrices in block diagonal matrices
    A_opt = blkdiag(A{1, 1}, A{2, 1});
    Q_opt = blkdiag(Q{1, 1}, Q{2, 1});
    
    
    
    
    %%%
    A_ineq = [];
    b_ineq = [];
        
    % check for a possible corrdior constraint
    if (sum(ismember(ineqConst.start, w)) >= 1)

        R = ((yConst(1, w+1)-yConst(1, w))^2+(zConst(1, w+1)-zConst(1, w))^2)^(0.5);
        b_vals = [ ...
            ineqConst.delta - yConst(1, w)*((yConst(1, w+1)-yConst(1, w))^2/R^2-1) - zConst(1, w)*(zConst(1, w+1)-zConst(1, w))*(yConst(1, w+1)-yConst(1, w))/R^2 ; ...
            ineqConst.delta + yConst(1, w)*((yConst(1, w+1)-yConst(1, w))^2/R^2-1) + zConst(1, w)*(zConst(1, w+1)-zConst(1, w))*(yConst(1, w+1)-yConst(1, w))/R^2 ; ...
            ineqConst.delta - zConst(1, w)*((zConst(1, w+1)-zConst(1, w))^2/R^2-1) - yConst(1, w)*(yConst(1, w+1)-yConst(1, w))*(zConst(1, w+1)-zConst(1, w))/R^2 ; ...
            ineqConst.delta + zConst(1, w)*((zConst(1, w+1)-zConst(1, w))^2/R^2-1) + yConst(1, w)*(yConst(1, w+1)-yConst(1, w))*(zConst(1, w+1)-zConst(1, w))/R^2];
        
        for j = 1:ineqConst.nc,
            %thisT = t(w, 1)+j/(1+ineqConst.nc)*(t(w+1,1)-t(w, 1));
            % recall that for nondimensionalized time, t0 = 0, t1 = 1
            thisT = t0+j/(1+ineqConst.nc)*(t1-t0);
            coeff1 = 1-(yConst(1, w+1)-yConst(1, w))^2/R^2;
            coeff2 = -(zConst(1, w+1)-zConst(1, w))*(yConst(1, w+1)-yConst(1, w))/R^2;
            A1 = zeros(1, 2*r);
            A2 = zeros(1, 2*r);
            for i = 1:2*r,
                A1(1, i) = coeff1*thisT^(2*r-i); %y constraint
                A1(1, 2*r+i) = coeff2*thisT^(2*r-i); %z constraint
                
                A2(1, i) = -coeff1*thisT^(2*r-i);%y constraint
                A2(1, 2*r+i) = -coeff2*thisT^(2*r-i); %z constraint
            end
            
            coeff1 = -(yConst(1, w+1)-yConst(1, w))*(zConst(1, w+1)-zConst(1, w))/R^2;
            coeff2 = 1-(zConst(1, w+1)-zConst(1, w))^2/R^2;
            A3 = zeros(1, 2*r);
            A4 = zeros(1, 2*r);
            for i = 1:2*r,
                A3(1, i) = coeff1*thisT^(2*r-i); %y constraint
                A3(1, 2*r+i) = coeff2*thisT^(2*r-i); %z constraint
                
                A4(1, i) = -coeff1*thisT^(2*r-i);%y constraint
                A4(1, 2*r+i) = -coeff2*thisT^(2*r-i); %z constraint                
            end
            
            % add this set of constraints
            A_ineq = [A_ineq; A1;A2;A3;A4];
            
            if (sum(A1) ~= 0) 
                b_ineq = [b_ineq; b_vals(1, 1)];
            else
                b_ineq = [b_ineq; 0];
            end
            
            if (sum(A2) ~= 0) 
                b_ineq = [b_ineq; b_vals(2, 1)];
            else
                b_ineq = [b_ineq; 0];
            end
            
            if (sum(A3) ~= 0) 
                b_ineq = [b_ineq; b_vals(3, 1)];
            else
                b_ineq = [b_ineq; 0];
            end
            
            if (sum(A4) ~= 0) 
                b_ineq = [b_ineq; b_vals(4, 1)];
            else
                b_ineq = [b_ineq; 0];
            end

        end

    end
    
A_ineq
b_ineq

    fixed = [1 1 1 1 1 0 0 1 1 1 1 1 0 0]' ~= 0;
    options=optimset('Algorithm', 'interior-point-convex');
    xT = quadprog(Q_opt,[],A_ineq,b_ineq,A_opt(fixed, :),b(fixed, :),[],[],[],options);


    % separate y and z components
    yT(:, w) = xT(1:2*r, 1);
    zT(:, w) = xT(2*r+1:4*r, 1);
    
    
end


end

