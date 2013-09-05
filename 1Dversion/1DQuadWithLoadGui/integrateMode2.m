% 7/26/13
% intergrateSys.m
% nested function to intergrate quadrotor with cable-suspended load, when cable is slack
% 1D case
% no controller
% Dependancies: desiredTraj.m, calculateInputs2.m
%
% inputs: 
%   tspan: 1x2 vector, [timeBegin timeEnd]
%   x20: 1xn vector, initial conditions
%   g, mL, mQ, JQ, l: real numbers, constants
%   kpx, kdx, kpL, kdL, kpQ, kdQ: real numbers, gain values
% outputs:
%   t: tx1 vector, time vector for t time steps
%   x2: txn vector, state vector for t time steps, n states

function  [t, x2, te, ye, ie] = integrateMode2(tspan, options2, x20, g, mQ, JQ)
    % call ode45
    [t, x2, te, ye, ie] = ode45(@mode2, tspan, x20, options2);
    
    % system dynamics
    %
    % inputs: 
    %   t: real number, current time
    %   x2: nx1 vector, state x at time t
    % outputs:
    %   x2dot: nx1 vector, state dx/dt at time t
    function x2dot = mode2(t, x2) 
        
        % x2 = [xL vL xQ vQ]'
        xL = x2(1, 1); vL = x2(2, 1); xQ = x2(3, 1); vQ = x2(4, 1); 
        
        [xT, dxT, d2xT] = desiredTraj(t, g, mQ, JQ, 2);

        f = mQ*(d2xT+g);
        
        %dynamic equations of system
        x2dot = zeros(4,1);
        x2dot(1,1) = vL; 
        x2dot(2,1) = -g; 
        x2dot(3,1) = vQ;
        x2dot(4, 1) = - g + f/mQ;
    end

end