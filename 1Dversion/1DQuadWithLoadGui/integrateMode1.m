% 7/26/13
% intergrateMode1.m
% nested function to intergrate quadrotor with cable-suspended load
% no controller
% 1D case
% Dependancies: desiredTraj.m
%
% inputs: 
%   tspan: 1x2 vector, [timeBegin timeEnd]
%   x10: 1xn vector, initial conditions
%   g, mL, mQ, JQ, l: real numbers, constants
%   kpx, kdx, kpL, kdL, kpQ, kdQ: real numbers, gain values
% outputs:
%   t: tx1 vector, time vector for t time steps
%   x1: txn vector, state vector for t time steps, n states


function [t, x1, te, ye, ie] = integrateMode1(tspan, options1, x10, g, mL, mQ, JQ, l)

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
        
        % x1 = [xL vL]'
        xL = x1(1, 1); vL = x1(2, 1);
        
        %find desired trajectory and its higher derivatives 
        [xT, dxT, d2xT] = desiredTraj(t, g, mQ, JQ, 1); 

        
        f = (mL+mQ)*(d2xT+g);
       

        
        

        %%%
        %dynamic equations of the system
        x1dot = zeros(2,1);
        x1dot(1,1) = vL; 
        x1dot(2,1) = -g+f/(mL+mQ);

        
    end


end

