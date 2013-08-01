% 7/30/13
% findTrajJoint.m
% generate optimal trajectory for through a set of keyframes by optimizing
%   derivatives at endpoints, for one dimension
% 	uses technique from"Polynominal Trajectory Planning for Quadrotor Flight", Richter et al.  
% Dependencies: findCostMatrix.m, findConstConstraints.m
%
% inputs: 
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   dim: integer, current dimension
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes, 
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
% outputs:
%   xT: (n+1) x m matrix, where row i contains the ith coefficient for the jth trajectory
%       xT is nondimensionalized in time



function [xT] = findTrajJoint(r, n, m, dim, tDes, posDes)

% we want to find the optimal derivatives at each keyframe
% the original cost function is J = x'Qx
% where x = [c1,n c1,n-1 ... c1,1 c1,0 ... c2,n ... cm,n ... cm, 0]
% we are seeking the optimal position and derivative conditions at each
%   keyframe for each piece of the trajectory, so we define the new state 
%   d = [x1(t0) x'1(t0) ... x^(r-1)1(t0) ... x1(t1) x'1(t1) ... x^(r-1)1(t1) ...
%       x2(t1) x'2(t1) ... x^(r-1)2(t1) ... x2(t2) x'2(t2) ... x^(r-1)2(t2) ...
%       ...
%       xm(tm-1) x'm(tm-1) ... x^(r-1)m(tm-1) ... xm(tm) x'm(tm) ... x^(r-1)m(tm)]





%%%
% construct Q_joint by making a block diagonal matrix from Q's of
%   each piece of the trajectory
%   note that we are not nondimensionalizing the time here
Q_joint = [];
for i = 1:m,
    Q = findCostMatrix(n, r, tDes(i, 1), tDes(i+1, 1));
      
    Q_joint = blkdiag(Q_joint, Q);
end


%%%
% construct A_joint
% here, d = Ax maps the x state vector to the endpoint derivative state vector d

A_joint = [];
for i = 1:m,
    
    % find coefficients of derivatives
    derCoeff = findDerivativeCoeff(n, r-1);
    derCoeff
    % substitute in the time values
    A = zeros(2*r, n+1);
    
    for j = 0:r-1,
        maxPower = nnz(derCoeff(j+1, :))-1;
        maxPower
        for k = 0:maxPower,
                tinit = tDes(i, 1);
                tfinal = tDes(i+1, 1);
                k
                A(j+1, k+1) = tinit^(maxPower-k)*derCoeff(j+1, k+1);
                A(j+1+r, k+1) = tfinal^(maxPower-k)*derCoeff(j+1, k+1);
        end
    end
    
    A_joint = blkdiag(A_joint, A);
end



% % construct D_F
% % D_F = [x1(t0) x1(t1) x2(t2) t3(t3) ... x_[m-1](t_[m-1]) x1'(t0) x1''(t0)
% %   ... x1^(n)(t0) x[m-1]'(t_[m-1]) x[m-1]''(t_[m-1]) ...
% %   x[m-1]^(n)(t_[m-1])]
% % D_P = [x1'(t1) ... x1^(n)(t1) x2'(t2) ... x2^(n)(t2) ... x[m-1]'(t_[m-1]) ...
% %   x[m-1]^(n)(t_[m-1])]
% 
% D_F = zeros(2*r-2+m, 1);
% 
% % fix positions at each waypoint
% for i = 1:m,
%     D_F(i, 1) = xConst(1, i);
% end
% 
% % fix derivatives at t0 and t_[m-1]
% for i = 2:r,
%     D_F(m+i-1, 1) = xConst(i, 1);
% end
% 
% for i = 2:r,
%     D_F(m+r+i-2, 1) = xConst(i, m);
% end
% 
% % construct M
% 
% % x1(t0) terms
% M = [1 zeros(1, r*m-1)];
% M = [M; zeros(r-1, m) eye(r-1) zeros(r-1, m*r-m-r+1)];
% 
% % terms for trajectories at times t1 to t_[m-2]
% for i = 1:m-2,
%     Mtemp= [zeros(1, i) 1 zeros(1, r*m-i-1)];
%     Mtemp = [Mtemp; zeros(r-1, m+(2+i-1)*(r-1)) eye(r-1) zeros(r-1, m*r-(r-1)-(m+(2+i-1)*(r-1)))];
%     M = [M; Mtemp; Mtemp];
% end
% 
% % x_[m-1](t_[m-1]) terms
% Mtemp = [zeros(1, m-1) 1 zeros(1, r*m-m)];
% Mtemp = [Mtemp; zeros(r-1, m+r-1) eye(r-1) zeros(r-1, m*r-(r-1)-(m+r-1))];
% M = [M; Mtemp];
% 
% % R = M' * inv(A_joint)' * Q_joint * inv(A_joint) * M;
% % R has dimensions r*m x r*m
% R = M'*inv(A_joint)'*Q_joint*inv(A_joint)*M;
% 
% % find R_FP and R_PP
% R_FF = R(1:m+2*(r-1), 1:m+2*(r-1));
% R_FP = R(1:m+2*(r-1), m+2*(r-1)+1:r*m);
% R_PF = R(m+2*(r-1)+1:r*m, 1:m+2*(r-1));
% R_PP = R(m+2*(r-1)+1:r*m, m+2*(r-1)+1:r*m);
% 
% % D_opt = -inv(R_PP)'*R_FP'*D_F
% % D_opt = [x1'(t1) ... x1^(n-1)(t1) x2'(t2) ... x_[m-1]'(t_[m-1]) ... x_[m-1]^(n-1)(t_[m-1])]
% D_opt = -inv(R_PP)'*R_FP'*D_F;



% 
% % find optimal trajectory through quadratic programming
% % find trajectory for each segment 
% 
% xT = zeros(2*r, m-1);
% 
% for i = 1:m-1,
%     % construct beginning and ending constraints, using values from D_opt
%     % for intermediate derivative values
%     if i == 1,
%         x0 = xConst(:, 1);
%         
%         xf = zeros(r, 1);
%         xf(1, 1) = xConst(1, i+1);
%         xf(2:r, 1) = D_opt((i-1)*(r-1)+1:(i)*(r-1), 1);    
%     elseif i < m-1,
%         x0 = zeros(r, 1);
%         x0(1, 1) = xConst(1, i);
%         x0(2:r, 1) = D_opt((i-2)*(r-1)+1:(i-1)*(r-1), 1);
%         
%         xf = zeros(r, 1);
%         xf(1, 1) = xConst(1, i+1);
%         xf(2:r, 1) = D_opt((i-1)*(r-1)+1:(i)*(r-1), 1);        
%     else
%         x0 = zeros(r, 1);
%         x0(1, 1) = xConst(1, i);
%         x0(2:r, 1) = D_opt((i-2)*(r-1)+1:(i-1)*(r-1), 1);
%         
%         xf = xConst(:, m);
%     end
% 
%     xTemp = optTraj(x0, xf);
%   
%     xT(:, i) = xTemp;
% end

end

