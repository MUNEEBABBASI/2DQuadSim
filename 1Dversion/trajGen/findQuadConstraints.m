% 8/6/13
% findQuadConstraints.m
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
function [A_eq, b_eq, modes] = findQuadConstraints(r, n, m, dim, posDes, TDes, t0, t1, tDes, nonDim, g, len, mL, mQ)

A_eq = [];
b_eq = [];

modes = [];
tDesN = tDes;


derCoeff = findDerivativeCoeff(n, r);

for j = 0:m, %for each keyframe
    
    % if the tension is 0
    % assume this will never be the first or last keyframe
    if j > 0 && j < m && TDes(j+1, 1) == 0,
        
        % log the switch
        thisMode = [j 1 2];
        modes = [modes; thisMode];
        
        %%%
        % calculate conditions for when tension goes to 0
        
        
        % add position if it's specified
        if posDes(1, j+1) ~= Inf,
            A_temp = zeros(2, (n+1)*m);
            maxPower = nnz(derCoeff(1, :))-1;
            
            for k = 0:maxPower,
                
                if nonDim,
                    tinit = t0;
                    tfin = t1;
                else
                    tinit = tDes(j+1, 1);
                    tfin = tDes(j+1, 1);
                end
                
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(1, k+1);
                A_temp(2, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(1, k+1);
                
                
            end
            
            b_temp = (posDes(1, j+1)+len)*ones(2, 1); %xQ = xL+len;
            
            % otherwise, add a continuity constraint
        else
            A_temp = zeros(1, (n+1)*m);
            maxPower = nnz(derCoeff(1,:))-1;
            
            for k = 0:maxPower,
                
                if nonDim,
                    tinit = t0;
                    tfin = t1;
                else
                    tinit = tDes(j+1, 1);
                    tfin = tDes(j+1, 1);
                end
                
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(1, k+1);
                A_temp(1, j*(n+1)+k+1) = -tinit^(maxPower - k)*derCoeff(1, k+1);
                
            end
            
            b_temp = 0;
        end
        
        A_temp
        b_temp
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp];
        
        
        
        
        % add velocity if it's specified
        if posDes(2, j+1) ~= Inf,
            A_temp = zeros(2, (n+1)*m);
            maxPower = nnz(derCoeff(2, :))-1;
            
            for k = 0:maxPower,
                
                if nonDim,
                    tinit = t0;
                    tfin = t1;
                else
                    tinit = tDes(j+1, 1);
                    tfin = tDes(j+1, 1);
                end
                
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(2, k+1);
                A_temp(2, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(2, k+1);
                
                
                %scale by time for nondimensionalization if nondimensionalized
                if nonDim,
                    A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1)))*A_temp(1, (j-1)*(n+1)+k+1);
                    A_temp(2, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1)))*A_temp(2, j*(n+1)+k+1);
                end
            end
            
            b_temp = posDes(2, j+1)*ones(2, 1); %vQ = vL
            
            
            % otherwise, add a continuity constraint
        else
            A_temp = zeros(1, (n+1)*m);
            maxPower = nnz(derCoeff(2,:))-1;
            
            for k = 0:maxPower,
                
                if nonDim,
                    tinit = t0;
                    tfin = t1;
                else
                    tinit = tDes(j+1, 1);
                    tfin = tDes(j+1, 1);
                end
                
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(2, k+1);
                A_temp(1, j*(n+1)+k+1) = -tinit^(maxPower - k)*derCoeff(2, k+1);
                
                % if a derivative is nondimensionalized, scale the constraint
                if nonDim,
                    A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1)))*A_temp(1, (j-1)*(n+1)+k+1);
                    A_temp(1, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1)))*A_temp(1, j*(n+1)+k+1);
                end
                
            end
            
            b_temp = 0;
        end
        
        A_temp
        b_temp
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp];
        
        
        
        
        % constrain the acceleration
        if posDes(3, j+1) ~= Inf && posDes(3, j+1) ~= g,
            disp(['changing acceleration constraint to match tension for keyframe ' int2str(j)])
        end
        A_temp = zeros(2, (n+1)*m);
        maxPower = nnz(derCoeff(3, :))-1;
        
        for k = 0:maxPower,
            
            if nonDim,
                tinit = t0;
                tfin = t1;
            else
                tinit = tDes(j+1, 1);
                tfin = tDes(j+1, 1);
            end
            
            A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(3, k+1);
            A_temp(2, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(3, k+1);
            
            
            %scale by time for nondimensionalization if nondimensionalized
            if nonDim,
                A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1))^2)*A_temp(1, (j-1)*(n+1)+k+1);
                A_temp(2, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1))^2)*A_temp(2, j*(n+1)+k+1);
            end
        end
        
        b_temp = -g*ones(2, 1); % mL(aL+g) = 0, aL = -g
        
        A_temp
        b_temp
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp];
        
        
        
        
        % all higher derivatives must be 0 for continuity
        for i = 3:r-1,
            % constrain the acceleration
            if posDes(i+1, j+1) ~= Inf && posDes(i+1, j+1) ~= 0,
                disp(['changing derivative constraint to match tension for keyframe ' int2str(j)])
            end
            
            A_temp = zeros(2, (n+1)*m);
            maxPower = nnz(derCoeff(i+1, :))-1;
            
            for k = 0:maxPower,
                
                if nonDim,
                    tinit = t0;
                    tfin = t1;
                else
                    tinit = tDes(j+1, 1);
                    tfin = tDes(j+1, 1);
                end
                
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(i+1, k+1);
                A_temp(2, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(i+1, k+1);
                
                
                %scale by time for nondimensionalization if nondimensionalized
                if nonDim,
                    A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1))^i)*A_temp(1, (j-1)*(n+1)+k+1);
                    A_temp(2, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1))^i)*A_temp(2, j*(n+1)+k+1);
                end
            end
            
            b_temp = zeros(2, 1);
            
            A_eq = [A_eq; A_temp];
            b_eq = [b_eq; b_temp];
        end
        
        
        
        
        
    % if end of free-fall period
    % this can also never be 0
    elseif j > 0 && j < m && TDes(j, 1) == 0,
        
        % log the switch back
        thisMode = [j 2 1];
        modes = [modes; thisMode];

        
        
        
        % conditions right before the switch
        
        % find desired quad position at end of free-fall based on load
        %   position
        A_temp = zeros(1, (n+1)*m);
        maxPower = nnz(derCoeff(1, :))-1;
        maxPower2 = nnz(derCoeff(2, :))-1;
        
        % x(tfinal)_Q = -g/2 (tfinal-tbegin)^2 + xdot(tinitial)_Q
        %   (tfinal-tbegin) + (x(tbegin)_Q-l) + l
        for k = 0:maxPower,
            
            if nonDim,
                tinit = t0;
                tfin = t1;
            else
                tinit = tDes(j, 1);
                tfin = tDes(j+1, 1);
            end

            
            %scale by time for nondimensionalization if nondimensionalized
            if nonDim,

                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(1, k+1) - tinit^(maxPower-k)*derCoeff(1, k+1);  
            else
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(1, k+1) - tinit^(maxPower-k)*derCoeff(1, k+1)+len;
            end


        end
        
        maxPower2 = nnz(derCoeff(2, :))-1;
        for k = 0:maxPower2,
            
            if nonDim,
                tinit = t0;
       
            else
                tinit = tDes(j, 1);
 
            end

            
            %scale by time for nondimensionalization if nondimensionalized
            if nonDim,
                A_temp(1, (j-1)*(n+1)+k+1) = A_temp(1, (j-1)*(n+1)+k+1) - tinit^(maxPower2-k)*derCoeff(2, k+1)*(t1-t0);
            
            else
                A_temp(1, (j-1)*(n+1)+k+1) = A_temp(1, (j-1)*(n+1)+k+1)- tinit^(maxPower2-k)*derCoeff(2, k+1)*(tDes(j+1, 1)-tDes(j, 1));
            end

        end
        
        
         b_temp = -g/2*(tDes(j+1, 1)-tDes(j, 1))^2;
        
        if nonDim,
            b_temp = b_temp*(t1-t0)^2;
        end
        
        A_temp
        b_temp
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp];
        
               
        
        
        
        % find desired quad velocity right before the switch
        % vL*mL + vQ*mQ = (mL+mQ)*v
        % vQ = ((mL+mQ)*v-vL*mL)/mQ;
        disp('huh')
        A_temp = zeros(1, (n+1)*m);
        maxPower = nnz(derCoeff(2, :))-1;

        for k = 0:maxPower,
            
            if nonDim,
                tinit = t0;
                tfin = t1;
            else
                tinit = tDes(j, 1);
                tfin = tDes(j+1, 1);
            end

            
            %scale by time for nondimensionalization if nondimensionalized
            if nonDim,
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(2, k+1)*1/(tDes(j+1, 1)-tDes(j, 1)) ...
                    + mL/mQ*tinit*(maxPower-k)*derCoeff(2, k+1)*1/(tDes(j+1, 1)-tDes(j, 1));
                A_temp(1, j*(n+1)+k+1) = -(mQ+mL)/mQ*tinit^(maxPower-k)*derCoeff(2, k+1)*1/(tDes(j+2, 1)-tDes(j+1, 1));
            else
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(2, k+1) ...
                    + mL/mQ*tinit*(maxPower-k)*derCoeff(2, k+1);
                A_temp(1, j*(n+1)+k+1) = -(mQ+mL)/mQ*tinit^(maxPower-k)*derCoeff(2, k+1);
            end



%               A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(2, k+1)/(tDes(j+1, 1)-tDes(j, 1));
%               b_temp = 1.24;
        end     
        
        
        b_temp = mL/mQ*g*(tDes(j+1, 1)-tDes(j, 1));
        
        if nonDim,
            b_temp = b_temp*(t1-t0);
        end
            
        A_temp
        b_temp
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp];        
        
        
        
        

        % acceleration is -g
        A_temp = zeros(1, (n+1)*m);
        maxPower = nnz(derCoeff(3, :))-1;
        
        for k = 0:maxPower,
            
            if nonDim,
                tfin = t1;
            else
                tfin = tDes(j+1, 1);
            end
            
            A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(3, k+1);
            
            
            %scale by time for nondimensionalization if nondimensionalized
            if nonDim,
                A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1))^2)*A_temp(1, (j-1)*(n+1)+k+1);
            end
        end
        
        b_temp = -g; % mL(aL+g) = 0, aL = -g
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp];
        
        
        
        
        % all higher derivatives must be 0 for continuity
        for i = 3:r-1,

            A_temp = zeros(1, (n+1)*m);
            maxPower = nnz(derCoeff(i+1, :))-1;
            
            for k = 0:maxPower,
                
                if nonDim,
                    tfin = t1;
                else
                    tfin = tDes(j+1, 1);
                end
                
                A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(i+1, k+1);
     
                
                
                %scale by time for nondimensionalization if nondimensionalized
                if nonDim,
                    A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1))^i)*A_temp(1, (j-1)*(n+1)+k+1);
                end
            end
            
            b_temp = 0;
            
            A_eq = [A_eq; A_temp];
            b_eq = [b_eq; b_temp];
        end        
        
        
        
        % conditions right after the switch
        % applies only if it isn't the last segment m
        
        
        % position 
        % enforce continuity condition
        
        if posDes(1, j+1) ~= Inf,
            disp(['changing position constraint at keyframe ' str2int(i)])
        end
        A_temp = zeros(1, (n+1)*m);
        maxPower = nnz(derCoeff(1,:))-1;
        
        for k = 0:maxPower,
            
            if nonDim,
                tinit = t0;
                tfin = t1;
            else
                tinit = tDes(j+1, 1);
                tfin = tDes(j+1, 1);
            end
            
             A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(1, k+1);
             A_temp(1, j*(n+1)+k+1) = -tinit^(maxPower - k)*derCoeff(1, k+1);


       %       A_temp(1, (j)*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(1, k+1);
        %      b_temp = -1.9;
        end
        
        b_temp = 0;
        
        A_temp
        b_temp
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp]; 
        
        
        
        
        % velocity constraint 
        % can be constrained or optimized
        % if optimized, there will be no continuity constraint beacuse
        %   velocity is not a continuous change
        
        % fixed constraint
        if posDes(2, j+1) ~= Inf,
            A_temp = zeros(1, (n+1)*m);
            maxPower = nnz(derCoeff(2, :))-1;
            for k = 0:maxPower,
                
                if nonDim,
                    tinit = t0;
                else
                    tinit = tDes(j+1, 1);
                end
                
                A_temp(1, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(2, k+1);
                
                %scale by time for nondimensionalization if nondimensionalized
                if nonDim,
                    A_temp(1, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1)))*A_temp(1, j*(n+1)+k+1);
                end

            end
            
            
            b_temp = posDes(2, j+1, dim); 

        end
        
        A_temp
        b_temp
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp]; 
        
        
        
        
        
        
        % constrain acceleration
        if posDes(3, j+1) ~= Inf && posDes(3, j+1) ~= g,
            disp(['changing acceleration constraint for keyframe ' int2str(j)])
        end
        A_temp = zeros(1, (n+1)*m);
        maxPower = nnz(derCoeff(3, :))-1;
        
        for k = 0:maxPower,
            
            if nonDim,
                tinit = t0;
            else
                tinit = tDes(j+1, 1);
            end
            
            A_temp(1, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(3, k+1);
            
            
            %scale by time for nondimensionalization if nondimensionalized
            if nonDim,
                A_temp(1, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1))^2)*A_temp(1, j*(n+1)+k+1);
            end
        end
        
        b_temp = -g; % mL(aL+g) = 0, aL = -g
        
        A_eq = [A_eq; A_temp];
        b_eq = [b_eq; b_temp];
        
        
        
        
        % all higher derivatives must be 0 for continuity
        for i = 3:r-1,
            % constrain t
            if posDes(i+1, j+1) ~= Inf && posDes(i+1, j+1) ~= 0,
                disp(['changing derivative constraint for keyframe ' int2str(j)])
            end
            
            A_temp = zeros(1, (n+1)*m);
            maxPower = nnz(derCoeff(i+1, :))-1;
            
            for k = 0:maxPower,
                
                if nonDim,
                    tinit = t0;
                else
                    tinit = tDes(j+1, 1);
                end
                
        
                A_temp(1, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(i+1, k+1);
                
                
                %scale by time for nondimensionalization if nondimensionalized
                if nonDim,
                    A_temp(1, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1))^i)*A_temp(1, j*(n+1)+k+1);
                end
            end
            
            b_temp = 0;
            
            A_eq = [A_eq; A_temp];
            b_eq = [b_eq; b_temp];
        end
        
 
        
        
    % otherwise, treat it as a normal point
    else
        
        for i = 0:r-1, %for all derivatives from 0 to r-1
            
            
            %if normal fixed constraint
            if (posDes(i+1, j+1, dim) ~= Inf),
                
                % construct and add constraint
                
                if (j == 0), % add one constraint to beginning of first piece
                    A_temp = zeros(1, (n+1)*m);
                    maxPower = nnz(derCoeff(i+1, :))-1;
                    for k = 0:maxPower,
                        
                        if nonDim,
                            tinit = t0;
                        else
                            tinit = tDes(j+1, 1);
                        end
                        
                        A_temp(1, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(i+1, k+1);
                        
                        %scale by time for nondimensionalization if nondimensionalized
                        if nonDim,
                            A_temp(1, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1))^i)*A_temp(1, j*(n+1)+k+1);
                        end
                    end
                    
                    
                    if i ==0,
                        b_temp = posDes(i+1, j+1, dim)+len; %xQ = xL+len
                    else
                        b_temp = posDes(i+1, j+1, dim); %all higher derivatives equal
                    end
                    
                    
                elseif (j == m), % add one constraint to end of last piece
                    A_temp = zeros(1, (n+1)*m);
                    maxPower = nnz(derCoeff(i+1, :))-1;
                    for k = 0:maxPower,
                        
                        if nonDim,
                            tfin = t1;
                        else
                            tfin = tDes(j+1, 1);
                        end
                        
                        A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(i+1, k+1);
                        
                        
                        %scale by time for nondimensionalization if nondimensionalized
                        if nonDim,
                            A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1))^i)*A_temp(1, (j-1)*(n+1)+k+1);
                        end
                        
                    end
                    
                    if i ==0,
                        b_temp = posDes(i+1, j+1, dim)+len; %xQ = xL+len
                    else
                        b_temp = posDes(i+1, j+1, dim); %all higher derivatives equl
                    end
                    
                    
                    
                else % else, add two constraints
                    A_temp = zeros(2, (n+1)*m);
                    maxPower = nnz(derCoeff(i+1,:))-1;
                    for k = 0:maxPower,
                        
                        if nonDim,
                            tinit = t0;
                            tfin = t1;
                        else
                            tinit = tDes(j+1, 1);
                            tfin = tDes(j+1, 1);
                        end
                        
                        A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(i+1, k+1);
                        A_temp(2, j*(n+1)+k+1) = tinit^(maxPower - k)*derCoeff(i+1, k+1);
                        
                        
                        %scale by time for nondimensionalization if nondimensionalized
                        if nonDim,
                            A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1))^i)*A_temp(1, (j-1)*(n+1)+k+1);
                            A_temp(2, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1))^i)*A_temp(2, j*(n+1)+k+1);
                        end
                        
                    end
                    
                    
                    if i ==0,
                        b_temp = (posDes(i+1, j+1, dim)+len)*ones(2, 1); %xQ = xL+len
                    else
                        b_temp = posDes(i+1, j+1, dim)*ones(2, 1); %all higher derivatives equl
                    end
                    
                end
                
                A_eq = [A_eq; A_temp];
                b_eq = [b_eq; b_temp];
                
                
                
            % otherwise, add continuity constraint if at intermediate
            %   keyframe
            elseif (j > 0 && j < m) % if constraint is at an intermediate keyframe
                A_temp = zeros(1, (n+1)*m);
                maxPower = nnz(derCoeff(i+1,:))-1;
                
                for k = 0:maxPower,
                    
                    if nonDim,
                        tinit = t0;
                        tfin = t1;
                    else
                        tinit = tDes(j+1, 1);
                        tfin = tDes(j+1, 1);
                    end
                    
                    A_temp(1, (j-1)*(n+1)+k+1) = tfin^(maxPower - k)*derCoeff(i+1, k+1);
                    A_temp(1, j*(n+1)+k+1) = -tinit^(maxPower - k)*derCoeff(i+1, k+1);
                    
                    % if a derivative is nondimensionalized, scale the constraint
                    if nonDim && i > 0,
                        A_temp(1, (j-1)*(n+1)+k+1) = 1/((tDes(j+1, 1)-tDes(j, 1))^i)*A_temp(1, (j-1)*(n+1)+k+1);
                        A_temp(1, j*(n+1)+k+1) = 1/((tDes(j+2, 1)-tDes(j+1, 1))^i)*A_temp(1, j*(n+1)+k+1);
                    end
                    
                end
                
                b_temp = 0;
                
                A_eq = [A_eq; A_temp];
                b_eq = [b_eq; b_temp];
            end
            
            
            
            
            
        end
    end
   
end


A_eq
b_eq

rank(A_eq)
size(A_eq)



end


