% 8/6/13
% findTrajLoad.m
% generate optimal trajectory, assuming only a 1D load
%
% Dependencies: findContConstraints.m, findFixedConstraints.m,
%   findDerivativeCoeff.m, findCostMatrix.m
%
% inputs:
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   d: integer, number of dimensions
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes,
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
%   ineqConst: structure of sx1 arrays, for s constraints, with elements:
%       numConst: integer, number of constraints
%       start: sx1 matrix, keyframes where constraints begin
%       delta: sx1 matrix, maximum distance
%       nc: sx1 matrix, number of intermediate points
%       dim: sxd matrix, the d dimensions that the constraint applies to
%   g: constant integer, gravity
%   l: constant integer, length of cable
% outputs:
%   xTL: (n+1) x mNew x d matrix, where row i contains the ith coefficient for
%       the jth trajectory in dimension k
%       xTL is nondimensionalized in time
%       trajectory for load
%   xTQ: (n+1) x mNew x d matrix, where row i contains the ith coefficient for
%       the jth trajectory in dimension k
%       trajectory for quad, only exists when system is in mode 2
%       all coefficients 0 otherwise
%   mode: a x 3 vector, logs mode switches
%       column 1 indicates keyframe switch occurs, column 2 is last mode,
%           column 3 is new mode (redundant, but just to be explicit)
%       1 indicates mode where cable is taut, trajectory is for load
%       2 indicates mode where cable is slack, trajectory is for quadrotor
%   tDesN: (m+1)x1 vector, time arrival at keyframes, could have changed if position of T =
%       0 didn't match time 
%   posDesN: r x m x d matrix, endpoint conditions at keyframes after
%       optimization


function [xTL, xTQ, mode, tDesN, posDesN] = findTrajLoad1D(r, n, m, d, tDes, posDes, TDes, g, len, mL, mQ)


% check that we are dealing with a 1D problem
if d ~= 1,
    error('not a 1D problem!')
end



% use nondimensionalized time
t0 = 0;
t1 = 1;

% we seek trajectories
% x1(t) = cx1,n*t^n + cx1,n-1*t^(n-1) + ... cx1,0;
% ...
% xm(t) = cxm,n*t^n + cxm,n-1*t^(n-1) + ... cxm,0;
% ...
% y1(t) = cy1,n*t^n + cy1,n-1*t^(n-1) + ... cy1,0;
% ...
% z1(t) = cz1,n*t^n + cz1,n-1*t^(n-1) + ... cz1,0;
% ... for d dimensions
% form the state vector x as:
% x = [cx1,(n) cx1,(n-1) ... cx1,1 cx1,0 cx2,n cx2,(n-1) ... cx2,1 cx2,0 .... cxm,0 ....
%       cy1,(n) cy1,(n-1) ... cy1,1 cy1,0 cy2,n cy2,(n-1) ... cy2,1 cy2,0 .... cym,0 ....
%       cz1,(n) cz1,(n-1) ... cz1,1 cz1,0 cz2,n cz2,(n-1) ... cz2,1 cz2,0 .... czm,0]




global options

if (isempty(options))
    options.r = r;
    options.n = n;
    options.m = m;
    options.d = d;
    options.tDes = tDes;
    options.posDes = posDes;
    %options.t0 = t0;
    %options.t1 = t1;
    
    options.gamma = 1e-5;
end




lastStart = 0; %keyframe that trajectory should begin at
mode = [];
xTL = [];
xTQ = [];
currentMode = 1; %assume for now system always starts with taut rope


% track new endpoint constraints
posDesN = posDes;

% track new time values if any 
tDesN = tDes;


%%%
% check for any keyframes where tension is 0
for i = 0:m,
    
    
    % if cable is taut
    if currentMode == 1,

        % if tension at a keyframe is 0, design a trajectory for all the ones
        %   before it leading to this point
        if TDes(i+1, 1) == 0 || i==m,
            
            
            % we are now seeking an x, where
            % p = lastStart+1
            % x = [cp,(n) cp,(n-1) ... cp,1 cp,0 ...
            %      c(p+1),n c(p+1),(n-1) ... c(p+1),1 c(p+1),0 ....
            %      ...
            %      ci,(n) ci,(n-1) ... ci,1 ci,0]^T;
            
            
            
            % construct Q matrix 
            Q_joint = [];
            % lastStart is the keyframe we want to start at
            % lastStart+1 is the trajectory piece we want to start at
            % i is the keyframe we want to end at
            % i is also trajectory piece we want to end at
            for j = (lastStart+1):i,                
                Q = findCostMatrix(n, r, t0, t1);
                
                Q_joint = blkdiag(Q_joint, Q);
            end
            
            
            
            % construct A matrix
            
            % make a new desired position matrix with nondimensionalized
            %   endpoint constraints 
            for j = (lastStart+1):(i+1),
                
                %%%%% ADD IN ENDPOINT SPECIAL CASES! 
                
                for k = 1:r
                    if j > 1,
                    posDesN(k, j) = posDesN(k, j)*(tDesN(j, 1)-tDesN(j-1, 1))^(k-1);
                    end
                end

            end
            
            % enforce constraints if tension goes to 0
            % assuming 1D trajectory, tension = 0 when mL*(d2xT+g) = 0
            if TDes(i+1, 1) == 0,
                posDesT = zeros(r, 1);
                posDesT(1:2, 1) = posDesN(1:2, i+1);
                posDesT(3, 1) = -g*(tDesN(i+1, 1)-tDesN(i, 1))^2;
                %posDesT(4:r, 1) = 0;
                
                if (posDesT(:, 1) ~= posDesN(:, i+1)),
                    disp('warning: constraints changed to accomodate tension')
                end
                posDesN(:, i+1) = posDesT;
            end

            
            
            posDesN(:, lastStart+1:i+1)
            
            % construct fixed value constraints and continuity constraints
            [A_fixed, b_fixed] = findFixedConstraints(r, n, i-lastStart, 1, posDesN(:, lastStart+1:i+1), t0, t1, [], 1);
            [A_cont, b_cont] = findContConstraints(r, n, i-lastStart, 1, posDesN(:, lastStart+1:i+1), t0, t1);
            
            % put in one matrix - recall there is only one dimension here
            A_eq = [A_fixed; A_cont];
            b_eq = [b_fixed; b_cont];
            
            
            
            %%% construct inequality constraints
            A_ineq = [];
            b_ineq = [];
%             % constrain the velocity to be greater than 0 (implying no
%             %   overshooting)
%             A_ineq = zeros(1, n+1);
%             derCoeff = findDerivativeCoeff(n, 1);
%             A_ineq(1, :) = -derCoeff(2, :); %pick out velocity vector
%             % note we skip evaluating at time t1 beacuse time is
%             % nondimensionalized and t1 =1
%             b_ineq = 0;


            % constraint tension force to be greater than 0
            epilson = 1e-5;
            Nc = 20;
            derrCoeff = findDerivativeCoeff(n, 2);
            derrDes = 2;
            
            for j = (lastStart+1):i, % for each segment
                
                % at each sample point, the tension is greater than or
                %   equal to 0
                A_ineqTemp = zeros(Nc, n+1);
                b_ineqTemp = zeros(Nc, 1);
                for k = 1:Nc,
                    
                    for p = derrDes+1:n+1,
                        tEval = t0+k/(Nc+1)*(t1-t0);
                        A_ineqTemp(k, p) = -derrCoeff(derrDes+1, p-derrDes)*tEval^(n-p+1);
                    end
                    b_ineqTemp(k, 1) = g*(tDes(j+1, 1)-tDes(j, 1))^2-epilson/mL;
                end
                A_ineq = blkdiag(A_ineq, A_ineqTemp);
                b_ineq = [b_ineq; b_ineqTemp];
            end

            A_ineq = [];
            b_ineq = [];

            
            
            % find this trajectory
            xT_all = quadprog(Q_joint,[],A_ineq,b_ineq,A_eq,b_eq);
  
            
            
            %%%
            % explicitly break tracjetory into its piecewise parts for output
            xT_this = zeros((n+1), i-lastStart);
            for j = 1:i-lastStart,
                xT_this(:, j) = xT_all((j-1)*(n+1)+1:j*(n+1));
            end
            xTL = [xTL xT_this]
            
            
            % add corresponding trajectories to quad
            for j = (lastStart+1):i
                xTQ = [xTQ [xTL(1:n, j);xTL(n+1, j)+len]]
            end
            
            % log the change in modes 
            mode = [mode; [i 1 2]];

            % update variables 
            lastStart = i; %keyframe next trajectory designed begins at
            currentMode = 2; %update mode to 2 
                   
        end
        
    elseif currentMode == 2,

        % lastStart is the keyframe we want to start at
        % lastStart+1 is the trajectory piece we want to start at
        % i is the keyframe we want to end at
        % i is also trajectory piece we want to end at
        
        % note that here, i = lastStart+1 and we're always designing for
        %   one trajectory piece
        
        
        %%% 
        % construct conditions for finding quadrotor trajectory
        posDesQ = zeros(r, 2); % we want to optimize snap of quadrotor between 2 points
            % we still specify constraints for up to rth derivative for continuity of trajectory
                
        % find states at moment of T = 0
        [stateEnd, ~] = evaluateTraj(tDesN(lastStart+1, 1), n, 1, 1, xTL(:, lastStart), tDesN, r-1, []);

        
        
        
        %%% 
        % find free-fall properties 
        % these calculations happen in real time to find end-point
        %   conditions
        
        posDesN
        % find end position of quad
        % if a displacement is specified, find the time it takes to reach it
        if (posDesN(1, i+1) ~= Inf), 
            % find displacement
            stateEnd
            i
            d = posDesN(1, i+1) - stateEnd(1, 1)
            
            
            % find time it takes to reach beginning of free fall to end
            % take the larger time - assume this is positive
            t_temp = roots([-g*1/2 stateEnd(2, 1) -d]) % solve for -1/2gt^2+vit - d = 0
            if (t_temp(1, 1) > t_temp(2, 1)),
                tFall = t_temp(1, 1);
            else
                tFall = t_temp(2, 1);
            end
     
            posDesQ(1, 2) = posDesN(1, i+1)+len; % xQ = xL+l
            
            if (tDesN(i+1, 1)-tDesN(i, 1) ~= tFall)
                tDesN(i+1, 1) = tDes(i, 1)+tFall;
                disp('updating time to accomodate freefall')
            end
            
        % otherwise, find where the load will be at the specified time
        else
            tFall = tDesN(i+1, 1)-tDesN(i, 1);
            
            % find position at end of free fall
            % d = -1/2*g*t^2+vit+xi
            posDesN(1, i+1) = (-1/2*g*tFall^2 + stateEnd(2, 1)*tFall + stateEnd(1, 1));
            posDesQ(1, 2) =  posDesN(1, i+1) + len; %xQ = xL+l
            
        end

        
        % find end velocity of quad 
        % vminus is moment right before rope goes taut
        % vplus is moment right after rope goes taut
        vLminus = stateEnd(2, 1) - g*tFall; % solve for vf = vi-gt
        vLplus = posDesN(2, i+1);
        
        % solve for vQ final
        vQminus = ((mL+mQ)*vLplus-mL*vLminus)/mQ;
        
        
        
        %%%
        % find quad initial states 
        
        % beginning quadrotor position, velocity can be derived from state at
        %   keyframe i
        posDesQ(1, 1) = stateEnd(1, 1)+len; %xQ = xL+l
        
        % nondimensionalize all higher derivatives
        for k = 2:r
            posDesQ(k, 1) = stateEnd(k, 1)*(tDesN(i+1, 1)-tDesN(i, 1))^(k-1); % all higher derivatives equal
        end
        
        %%% 
        % find quad final states
        posDesQ(2, 2) = vQminus*(tDesN(i+1, 1)-tDesN(i, 1)); % look for nondimensionalizd constraint
        
        % all other constraints match the load constraints
        for k = 3:r,
            posDesQ(k, 2) = posDesN(k, i+1)*(tDesN(i+1, 1)-tDesN(i, 1))^(k-1);
        end

        posDesQ
        
        
        
        %%% 
        % construct QP problem - note that there is always only 1 segment
        %   we want to minimize the snap 
        
        % construct Q matrix
        Q = findCostMatrix(n, 4, t0, t1);
              
        % find A matrix
        [A_fixed, b_fixed] = findFixedConstraints(r, n, 1, 1, posDesQ, t0, t1, [], 1);
        
        
        A_ineq = [];
        b_ineq = [];
        
        
        % constraint position of quad to be always greater than the load
        %   (no collisions) and less than the length of the rope

        
        epilson1 = 0.3;
        epilson2 = 1e-5;
        Nc = 20;
        derrCoeff = findDerivativeCoeff(n, 0);
        A_ineq = zeros(2*Nc, n+1);
        b_ineq = zeros(2*Nc, 1);
        for k = 1:Nc,
            for p = 1:n+1,
                tEval = t0+k/(Nc+1)*(t1-t0);
                A_ineq(k, p) = -derrCoeff(1, p)*tEval^(n-p+1);
                A_ineq(Nc+k, p) = derrCoeff(1, p)*tEval^(n-p+1);
            end

            b_ineq(k, 1) = -(-g/2*(tDesN(i+1, 1)-tDesN(i, 1))^2*tEval^2+stateEnd(2, 1)*(tDesN(i+1, 1)-tDesN(i, 1))*tEval+stateEnd(1, 1))-epilson1;
            b_ineq(Nc+k, 1) = (-g/2*(tDesN(i+1, 1)-tDesN(i, 1))^2*tEval^2+stateEnd(2, 1)*(tDesN(i+1, 1)-tDesN(i, 1))*tEval+stateEnd(1, 1))+len-epilson2;
        end

%         
%         A_ineq = [];
%         b_ineq = [];

        
        % find trajectory
        xT_all = quadprog(Q,[],A_ineq, b_ineq,A_fixed, b_fixed);
        
        
        
        %%%
        % add trajectory
        
        % add to quad trajectory
        xTQ = [xTQ [zeros(n-length(xT_all)+1, 1); xT_all]];
        
        % add free fall equations of motion for the load
        % note these equations are scaled for nondimensionalized time
        
        % x = -1/2*g*t^2 + vi*t + xi
        % v = -a*t + vi
        % a = -g;
        % all higher derivatives = 0;
        
        % first n-2 terms are 0
        xTL = [xTL [zeros(n-2, 1); -1/2*g*(tDesN(i+1, 1)-tDesN(i, 1))^2; stateEnd(2, 1)*(tDesN(i+1, 1)-tDesN(i, 1)); stateEnd(1, 1)]];
        
        % log mode switch
        mode = [mode; [i 2 1]];
        
        % calculate load initial conditions after the switch
        [stateEnd, ~] = evaluateTraj(tDesN(i+1, 1), n, i, 1, xTQ, tDesN, r-1, []);

        
        posDesT = zeros(r, 1);
        posDesT(1, 1) = stateEnd(1, 1)-len;
        posDesT(2, 1) = (stateEnd(2, 1)*mQ+vLminus*mL)/(mQ+mL);
        posDesT(3:r, 1) = stateEnd(3:r, 1);
        
        if (posDesT(:, 1) ~= posDesN(:, i+1)),
            disp('warning: updating constraints at end of collision')
        end
        
        posDesN(:, i+1) = posDesT;
        

        
        % update variables
        currentMode = 1;
        lastStart = i;
        
        
        
        
    end
end





end







