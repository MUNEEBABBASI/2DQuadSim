% 8/16/13
% findQuadIneqConstraints.m
% given the time, position/derivative, and tension constraints for a
%   load, generate constraints for quadrotor
%
% Dependencies: findContConstraints.m, findFixedConstraints.m,
%   findDerivativeCoeff.m, findCostMatrix.m
%
% inputs:
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   d: integer, number of dimensions
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes,
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
%   TDes: desired tensions at keyframes
%   t0: real value, begnning time of the trajectory
%   t1: real value, end time of the trajectory
%   tDes: m+1 x 1 matrix, desired arrival times at each trajectory
%   nonDim: 0 or 1, 1 uses nondimensionalized times t0 and t1
%       and scales conditions specified by posDes by times in tDes when calculaing b_eq,
%       0 uses times in tDes times and doesn't scale endpoint conditions
%   g: constant, gravity
%   len, mL, mQ: constants, length of cable
% outputs:
%   modes: a x 3 vector, logs mode switches
%       column 1 indicates keyframe switch occurs, column 2 is last mode,
%           column 3 is new mode (redundant, but just to be explicit)
%       1 indicates mode where cable is taut, trajectory is for load
%       2 indicates mode where cable is slack, trajectory is for quadrotor
%   A_eq, b_eq: matrices for Ax=b formulation of constraints
function [A_ineq, b_ineq] = findQuadIneqConstraints(r, n, m, d, posDes, modes, TDes, t0, t1, tDes, nonDim, g, len, mL, mQ)

A_ineq = [];
b_ineq = [];

derCoeff = findDerivativeCoeff(n, r);

for j = 0:m-1,
    
    % if the beginning of this segement isn't at 0 tension (not free-fall)
    if TDes(j+1, 1) ~= 0,
        
        % constraint tension force to be greater than 0
        
        epilson = 1e-3; % error
        Nc = 20; %number of intermediate points
        
        % at each sample point, the tension is greater than or
        %   equal to 0
        
        A_temp = zeros(Nc, (n+1)*m);
        b_temp = zeros(Nc, 1);
        
        % we're looking at acceleration here
        maxPower = nnz(derCoeff(3, :))-1;
        
        for p = 1:Nc,
                             
            if nonDim,
                tinit = t0;
                tfin = t1;
            else
                tinit = tDes(j+1, 1);
                tfin = tDes(j+1, 1);
            end
            
            tEval = tinit+p/(Nc+1)*(tfin-tinit);
            
            for k = 0:maxPower,
                
                A_temp(p, j*(n+1)+k+1) = -tEval^(maxPower - k)*derCoeff(3, k+1);
                
            end
            
            b_temp(p, 1) = (g-epilson/mL)*(tDes(j+2, 1)-tDes(j+1, 1))^2;
        end
        
        
        A_ineq = [A_ineq; A_temp];
        b_ineq = [b_ineq; b_temp];
   
        
        % otherwise, if tension is 0 at beginning point (this implies free fall traj for load)
    elseif TDes(j+1, 1)==0,
        
        % constraint position of quad to be always greater than the load
        %   (no collisions) and less than the length of the rope
        epilson1 = 0.1;
        epilson2 = 1e-3;
        Nc = 20;
        
        
        A_temp = zeros(2*Nc, (n+1)*m);
        b_temp = zeros(2*Nc, 1);
        
        
        % we're looking at position here
        
        for p = 1:Nc,
            
            if nonDim,
                tinit = t0;
                tfin = t1;
            else
                tinit = tDes(j+1, 1);
                tfin = tDes(j+1, 1);
            end
            
            tEval = tinit+p/(Nc+1)*(tfin-tinit);
            
            % position terms
            maxPower = nnz(derCoeff(1, :))-1;
            for k = 0:maxPower,
                
                A_temp(p, j*(n+1)+k+1) = -derCoeff(1, k+1)*tEval^(maxPower - k)+derCoeff(1, k+1)*tinit^(maxPower - k); % position of quad greater than load
                A_temp(Nc+p, j*(n+1)+k+1) = derCoeff(1, k+1)*tEval^(maxPower - k)-derCoeff(1, k+1)*tinit^(maxPower - k); % quad-load is less than length of rope
                
            end
            
            % velocity terms
            maxPower = nnz(derCoeff(2, :))-1;
            for k = 0:maxPower,
                
                A_temp(p, j*(n+1)+k+1) = A_temp(p, j*(n+1)+k+1)+derCoeff(2, k+1)*tinit^(maxPower - k)*tEval; % position of quad greater than load
                A_temp(Nc+p, j*(n+1)+k+1) = A_temp(p, j*(n+1)+k+1)-derCoeff(2, k+1)*tinit^(maxPower - k)*tEval; % quad-load is less than length of rope
                
            end
            
            
            b_temp(p, 1) = len-epilson1+g/2*tEval^2*(tDes(j+2, 1)-tDes(j+1, 1))^2;
            b_temp(Nc+p, 1) = g/2*tEval^2*(tDes(j+2, 1)-tDes(j+1, 1))^2-epilson2;
            
        end
        
        A_ineq = [A_ineq; A_temp];
        b_ineq = [b_ineq; b_temp];
        
    end
    
    
    
end


end