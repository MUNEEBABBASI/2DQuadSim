% 7/26/13
% intergrateSys.m
% nested function to intergrate quadrotor with cable-suspended load, when cable is slack
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

function  [t, x2, te, ye, ie] = integrateMode2(tspan, options2, x20, g, mQ, JQ, kp, kd, kp_phi, kd_phi)
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
        
        % x2 = [xL vL xQ vQ phiQ phidotQ]', note x, v are vectors in R^2
        yL = x2(1, 1); zL = x2(2, 1); vyL = x2(3, 1); vzL = x2(4, 1); 
        yQ = x2(5, 1); zQ = x2(6, 1); vyQ = x2(7, 1); vzQ = x2(8, 1); 
        phiQ = x2(9, 1); phidotQ = x2(10, 1);

        % calculate control inputs
        [f, M] = calculateInputs2(t, x2, g, mQ, JQ, kp, kd, kp_phi, kd_phi);
        
        %dynamic equations of system
        x2dot = zeros(10,1);
        x2dot(1,1) = vyL; 
        x2dot(2,1) = vzL; 
        x2dot(3,1) = 0;
        x2dot(4, 1) = - g;
        x2dot(5, 1) = vyQ; 
        x2dot(6, 1) = vzQ;
        x2dot(7, 1) = -f*sin(phiQ)/mQ;
        x2dot(8, 1) = f*cos(phiQ)/mQ - g;
        x2dot(9, 1) = phidotQ;
        x2dot(10, 1) = M/JQ;
    end

end