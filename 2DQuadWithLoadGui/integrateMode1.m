% 7/26/13
% intergrateMode1.m
% nested function to intergrate quadrotor with cable-suspended load
% Dependancies: desiredTraj.m, calculateDerivatives.m, calculateInputs1.m
%
% inputs: 
%   tspan: 1x2 vector, [timeBegin timeEnd]
%   x10: 1xn vector, initial conditions
%   g, mL, mQ, JQ, l: real numbers, constants
%   kpx, kdx, kpL, kdL, kpQ, kdQ: real numbers, gain values
% outputs:
%   t: tx1 vector, time vector for t time steps
%   x1: txn vector, state vector for t time steps, n states


function [t, x1, te, ye, ie] = integrateMode1(tspan, options1, x10, g, mL, mQ, JQ, l, kpx, kdx, kpL, kdL, kpQ, kdQ)

    % call ode45
    [t, x1, te, ye, ie] = ode45(@mode1, tspan, x10, options1);
    
    % system dynamics
    %
    % inputs: 
    %   t: real number, current time
    %   x1: nx1 vector, state x at time t
    % outputs:
    %   x1dot: nx1 vector, state dx/dt at time t
    function x1dot = mode1(t, x1) 
        
        % x1 = [xL vL phiL phidotL phiQ phidotQ]', note xL and vL are vectors in R^2
        yL = x1(1, 1); zL = x1(2, 1); vyL = x1(3, 1); vzL = x1(4, 1); phiL = x1(5, 1); phidotL = x1(6, 1); ...
            phiQ = x1(7, 1); phidotQ = x1(8, 1);
        
        %find desired trajectory and its higher derivatives 
        [xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT] = desiredTraj(t, g, mQ, JQ); 

        %calculate desired control inputs
        [f, M, ~, ~, ~] = calculateInputs1(t, x1, g, mL, mQ, JQ, l, kpx, kdx, kpL, kdL, kpQ, kdQ);
        
        %%%
        %dynamic equations of the system
        x1dot = zeros(8,1);
        x1dot(1,1) = vyL; 
        x1dot(2,1) = vzL; 
        x1dot(3, 1) = (-f*cos(angleDiff(phiQ, phiL)) - mQ*l*phidotL^2)/(mQ+mL)*sin(phiL);
        x1dot(4, 1) = (-f*cos(angleDiff(phiQ, phiL)) - mQ*l*phidotL^2)/(mQ+mL)*(-cos(phiL)) - g;
        x1dot(5, 1) = phidotL; 
        x1dot(6, 1) = f*sin(angleDiff(phiQ, phiL))/mQ/l;
        x1dot(7, 1) = phidotQ;
        x1dot(8, 1) = M/JQ;
    end

    % takes difference of two angles, keeping it between -pi and pi
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

