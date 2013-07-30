% 7/16/13
% intergrateQuad.m
% nested function to intergrate quadrotor
% Dependancies: desiredTraj.m, calculateInputs.m
%
% inputs: 
%   tspan: 1x2 vector, [timeBegin timeEnd]
%   x0: 1xn vector, initial conditions
%   g, mQ, JQ: real numbers, constants
%   kp, kd, kp_phi, kd_phi: real numbers, gain values
% outputs:
%   t: tx1 vector, time vector for t time steps
%   x: txn vector, state vector for t time steps, n states


function [t, x] = integrateQuad(tspan, x0, g, mQ, JQ, kp, kd, kp_phi, kd_phi)

    % call ode45
    [t, x] = ode45(@quad, tspan, x0, []);
    
    % system dynamics
    %
    % inputs: 
    %   t: real number, current time
    %   x: nx1 vector, state x at time t
    % outputs:
    %   xdot: nx1 vector, state dx/dt at time t
    function xdot = quad(t, x) 
        % x = [xQ zQ vxQ vzQ phiQ phidotQ]'
        yQ = x(1, 1); zQ = x(2, 1); vyQ = x(3, 1); vzQ = x(4, 1); phiQ = x(5, 1); phidotQ = x(6, 1);
        
        % calculate control inputs
        [f, M] = calculateInputs(t, x, g, mQ, JQ, kp, kd, kp_phi, kd_phi);

        
        xdot = zeros(6,1);
        xdot(1,1) = vyQ; 
        xdot(2,1) = vzQ; 
        xdot(3, 1) = -f/mQ * sin(phiQ);
        xdot(4, 1) = f/mQ * cos(phiQ) - g;
        xdot(5, 1) = phidotQ; 
        xdot(6, 1) = M/JQ;
    end

    % takes difference of two angles, bounding output between -pi and pi
    function diff = angleDiff(x1, x2)
        diff = x1-x2;
        
        while diff > pi
            diff = diff - 2*pi;
        end
        
        while diff <= -pi
            diff = diff + 2*pi;
        end
    end
end

